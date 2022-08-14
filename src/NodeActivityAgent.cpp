/*
 * Copyright (c) 2015 - 2022, Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "config.h"
#include "NodeActivityAgent.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "geopm/Agg.hpp"
#include "geopm/Exception.hpp"
#include "geopm/Helper.hpp"
#include "geopm/PlatformIO.hpp"
#include "geopm/PlatformTopo.hpp"
#include "geopm/PluginFactory.hpp"
#include "geopm_debug.hpp"

#include "PlatformIOProf.hpp"
#include "ActivityPerformanceModel.hpp"

namespace geopm
{

    NodeActivityAgent::NodeActivityAgent()
        : NodeActivityAgent(PlatformIOProf::platform_io(), platform_topo(), activity_perf_model())
    {

    }

    NodeActivityAgent::NodeActivityAgent(PlatformIO &plat_io, const PlatformTopo &topo, ActivityPerformanceModel &activity_perf_model)
        : m_platform_io(plat_io)
        , m_platform_topo(topo)
        , m_activity_perf_model(activity_perf_model)
        , m_last_wait{{0, 0}}
        , M_WAIT_SEC(0.010) // 10ms wait default
        , M_POLICY_PHI_DEFAULT(0.5)
        , M_GPU_ACTIVITY_CUTOFF(0.05)
        , M_NUM_GPU(m_platform_topo.num_domain(
                    GEOPM_DOMAIN_GPU))
        , M_NUM_GPU_CHIP(m_platform_topo.num_domain(
                    GEOPM_DOMAIN_GPU_CHIP))
        , M_NUM_CHIP_PER_GPU(M_NUM_GPU_CHIP/M_NUM_GPU)
        , M_NUM_PACKAGE(m_platform_topo.num_domain(GEOPM_DOMAIN_PACKAGE))
        , M_NUM_CORE(m_platform_topo.num_domain(GEOPM_DOMAIN_CORE))
        , m_do_write_batch(false)
        , m_update_qm_max_rate(true)
    {
        geopm_time(&m_last_wait);
    }

    // Push signals and controls for future batch read/write
    void NodeActivityAgent::init(int level, const std::vector<int> &fan_in, bool is_level_root)
    {
        m_core_frequency_requests = 0;
        m_uncore_frequency_requests = 0;
        m_resolved_f_uncore_efficient = 0;
        m_resolved_f_uncore_max = 0;
        m_resolved_f_core_efficient = 0;
        m_resolved_f_core_max = 0;
        m_gpu_frequency_requests = 0;
        m_f_max = 0;
        m_f_efficient = 0;
        m_f_range = 0;

        for (int domain_idx = 0; domain_idx < M_NUM_GPU; ++domain_idx) {
            m_gpu_active_region_start.push_back(0.0);
            m_gpu_active_region_stop.push_back(0.0);
            m_gpu_active_energy_start.push_back(0.0);
            m_gpu_active_energy_stop.push_back(0.0);
        }

        m_freq_uncore_min = m_platform_io.read_signal("CPU_UNCORE_FREQUENCY_MIN_CONTROL", GEOPM_DOMAIN_BOARD, 0);
        m_freq_uncore_max = m_platform_io.read_signal("CPU_UNCORE_FREQUENCY_MAX_CONTROL", GEOPM_DOMAIN_BOARD, 0);

        if (level == 0) {
            init_platform_io();
        }
    }

    void NodeActivityAgent::init_platform_io(void)
    {

        std::vector<int> control_domains;
        control_domains.push_back(m_platform_io.control_domain_type("GPU_CORE_FREQUENCY_CONTROL"));
        //control_domains.push_back(m_platform_io.control_domain_type("GPU_CORE_FREQUENCY_MIN_CONTROL"));
        //control_domains.push_back(m_platform_io.control_domain_type("GPU_CORE_FREQUENCY_MAX_CONTROL"));

        std::vector<int> signal_domains;
        signal_domains.push_back(m_platform_io.signal_domain_type("GPU_CORE_FREQUENCY_STATUS"));
        signal_domains.push_back(m_platform_io.signal_domain_type("GPU_CORE_ACTIVITY"));
        signal_domains.push_back(m_platform_io.signal_domain_type("GPU_UTILIZATION"));

        // We'll use the coarsest granularity supported by any of the controls or signals except Energy
        // i.e. GPU if one control supports GPU and another supports GPU_CHIP
        int agent_domain = std::min(*std::min_element(std::begin(control_domains), std::end(control_domains)),
                                    *std::min_element(std::begin(signal_domains), std::end(signal_domains)));

#ifdef GEOPM_DEBUG
        int max_agent_domain = std::max(*std::max_element(std::begin(control_domains), std::end(control_domains)),
                                    *std::max_element(std::begin(signal_domains), std::end(signal_domains)));

        if (agent_domain != max_agent_domain) {
            throw Exception("NodeActivityAgent::" + std::string(__func__) +
                            "(): Required signals and controls do not all exist at the same domain.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
#endif

        if (agent_domain != GEOPM_DOMAIN_GPU && agent_domain != GEOPM_DOMAIN_GPU_CHIP) {
            throw Exception("NodeActivityAgent::" + std::string(__func__) +
                            "(): Required signals and controls do not exist at the " +
                            "GPU or GPU_CHIP domain!", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        m_agent_domain_count = m_platform_topo.num_domain(agent_domain);

        for (int domain_idx = 0; domain_idx < m_agent_domain_count; ++domain_idx) {
            m_gpu_freq_status.push_back({m_platform_io.push_signal("GPU_CORE_FREQUENCY_STATUS",
                                         agent_domain,
                                         domain_idx), NAN});
            m_gpu_core_activity.push_back({m_platform_io.push_signal("GPU_CORE_ACTIVITY",
                                           agent_domain,
                                           domain_idx), NAN});
            m_gpu_utilization.push_back({m_platform_io.push_signal("GPU_UTILIZATION",
                                         agent_domain,
                                         domain_idx), NAN});
        }

        for (int domain_idx = 0; domain_idx < m_agent_domain_count; ++domain_idx) {
            m_gpu_freq_min_control.push_back(control{m_platform_io.push_control("GPU_CORE_FREQUENCY_CONTROL",
                                                     agent_domain,
                                                     domain_idx), NAN});
            m_gpu_freq_max_control.push_back(control{m_platform_io.push_control("GPU_CORE_FREQUENCY_CONTROL",
                                                     agent_domain,
                                                     domain_idx), NAN});
        }

        // We treat energy & time as special cases and only use them at a specific domain.
        // This is because energy & time are used for for tracking agent behavior/reporting
        // and do not impact the agent algorithm
        m_time = {m_platform_io.push_signal("TIME", GEOPM_DOMAIN_BOARD, 0), NAN};

        for (int domain_idx = 0; domain_idx < M_NUM_GPU; ++domain_idx) {
            m_gpu_energy.push_back({m_platform_io.push_signal("GPU_ENERGY",
                                    m_platform_io.signal_domain_type("GPU_ENERGY"),
                                    domain_idx), NAN});
        }

        // Core & Uncore
        for (int domain_idx = 0; domain_idx < M_NUM_CORE; ++domain_idx) {
            m_core_scal.push_back({m_platform_io.push_signal("MSR::CPU_SCALABILITY_RATIO",
                                                             GEOPM_DOMAIN_CORE,
                                                             domain_idx), NAN});
            m_core_freq_control.push_back({m_platform_io.push_control("CPU_FREQUENCY_CONTROL",
                                                                      GEOPM_DOMAIN_CORE,
                                                                      domain_idx), -1});
        }

        for (int domain_idx = 0; domain_idx < M_NUM_PACKAGE; ++domain_idx) {
            m_qm_rate.push_back({m_platform_io.push_signal("MSR::QM_CTR_SCALED_RATE",
                                                           GEOPM_DOMAIN_PACKAGE,
                                                           domain_idx), NAN});

            m_uncore_freq_status.push_back({m_platform_io.push_signal("CPU_UNCORE_FREQUENCY_STATUS",
                                                                      GEOPM_DOMAIN_PACKAGE,
                                                                      domain_idx), NAN});
        }

        for (int domain_idx = 0; domain_idx < M_NUM_PACKAGE; ++domain_idx) {
            m_uncore_freq_min_control.push_back({m_platform_io.push_control("CPU_UNCORE_FREQUENCY_MIN_CONTROL",
                                              GEOPM_DOMAIN_PACKAGE,
                                              domain_idx), -1});
            m_uncore_freq_max_control.push_back({m_platform_io.push_control("CPU_UNCORE_FREQUENCY_MAX_CONTROL",
                                              GEOPM_DOMAIN_PACKAGE,
                                              domain_idx), -1});
        }

        //Configuration of QM_CTR must match QM_CTR config used for training data
        m_platform_io.write_control("MSR::PQR_ASSOC:RMID", GEOPM_DOMAIN_BOARD, 0, 0);
        m_platform_io.write_control("MSR::QM_EVTSEL:RMID", GEOPM_DOMAIN_BOARD, 0, 0);
        m_platform_io.write_control("MSR::QM_EVTSEL:EVENT_ID", GEOPM_DOMAIN_BOARD, 0, 2);
    }

    // Validate incoming policy and configure default policy requests.
    void NodeActivityAgent::validate_policy(std::vector<double> &in_policy) const
    {
        //assert(in_policy.size() == M_NUM_POLICY);
        double gpu_min_freq = m_platform_io.read_signal("LEVELZERO::GPU_CORE_FREQUENCY_MIN_AVAIL", GEOPM_DOMAIN_BOARD, 0);
        double gpu_max_freq = m_platform_io.read_signal("LEVELZERO::GPU_CORE_FREQUENCY_MAX_AVAIL", GEOPM_DOMAIN_BOARD, 0);

        // Check for NAN to set default values for policy
        if (std::isnan(in_policy[M_POLICY_GPU_FREQ_MAX])) {
            in_policy[M_POLICY_GPU_FREQ_MAX] = gpu_max_freq;
        }

        if (in_policy[M_POLICY_GPU_FREQ_MAX] > gpu_max_freq ||
            in_policy[M_POLICY_GPU_FREQ_MAX] < gpu_min_freq ) {
            throw Exception("NodeActivityAgent::" + std::string(__func__) +
                            "(): GPU_FREQ_MAX out of range: " +
                            std::to_string(in_policy[M_POLICY_GPU_FREQ_MAX]) +
                            ".", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        // Not all gpus provide an 'efficient' frequency signal, and the
        // value provided by the policy may not be valid.  In this case approximating
        // f_efficient as midway between F_min and F_max is reasonable.
        if (std::isnan(in_policy[M_POLICY_GPU_FREQ_EFFICIENT])) {
            in_policy[M_POLICY_GPU_FREQ_EFFICIENT] = (in_policy[M_POLICY_GPU_FREQ_MAX]
                                                      + gpu_min_freq) / 2;
        }

        if (in_policy[M_POLICY_GPU_FREQ_EFFICIENT] > gpu_max_freq ||
            in_policy[M_POLICY_GPU_FREQ_EFFICIENT] < gpu_min_freq ) {
            throw Exception("NodeActivityAgent::" + std::string(__func__) +
                            "(): GPU_FREQ_EFFICIENT out of range: " +
                            std::to_string(in_policy[M_POLICY_GPU_FREQ_EFFICIENT]) +
                            ".", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        if (in_policy[M_POLICY_GPU_FREQ_EFFICIENT] > in_policy[M_POLICY_GPU_FREQ_MAX]) {
            throw Exception("NodeActivityAgent::" + std::string(__func__) +
                            "(): GPU_FREQ_EFFICIENT (" +
                            std::to_string(in_policy[M_POLICY_GPU_FREQ_EFFICIENT]) +
                            ") value exceeds GPU_FREQ_MAX (" +
                            std::to_string(in_policy[M_POLICY_GPU_FREQ_MAX]) +
                            ").", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        // If no phi value is provided assume the default behavior.
        if (std::isnan(in_policy[M_POLICY_GPU_PHI])) {
            in_policy[M_POLICY_GPU_PHI] = M_POLICY_PHI_DEFAULT;
        }

        if (in_policy[M_POLICY_GPU_PHI] < 0.0 ||
            in_policy[M_POLICY_GPU_PHI] > 1.0) {
            throw Exception("NodeActivityAgent::" + std::string(__func__) +
                            "(): POLICY_GPU_PHI value out of range: " +
                            std::to_string(in_policy[M_POLICY_GPU_PHI]) + ".",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        // Policy provided initial values
        double f_max = in_policy[M_POLICY_GPU_FREQ_MAX];
        double f_efficient = in_policy[M_POLICY_GPU_FREQ_EFFICIENT];
        double gpu_phi = in_policy[M_POLICY_GPU_PHI];

        // initial range is needed to apply phi
        double f_range = f_max - f_efficient;

        // If phi is not 0.5 we move into the energy or performance biased regions
        if (gpu_phi > 0.5) {
            // Energy Biased.  Scale F_max down to F_efficient based upon phi value
            // Active region phi usage
            f_max = std::max(f_efficient, f_max - f_range * (gpu_phi-0.5) / 0.5);
        }
        else if (gpu_phi < 0.5) {
            // Perf Biased.  Scale F_efficient up to F_max based upon phi value
            // Active region phi usage
            f_efficient = std::min(f_max, f_efficient + f_range * (0.5-gpu_phi) / 0.5);
        }

        //Update Policy
        in_policy[M_POLICY_GPU_FREQ_MAX] = f_max;
        in_policy[M_POLICY_GPU_FREQ_EFFICIENT] = f_efficient;

        GEOPM_DEBUG_ASSERT(in_policy.size() == M_NUM_POLICY,
                           "NodeActivityAgent::" + std::string(__func__) +
                           "(): policy vector not correctly sized.  Expected  " +
                            std::to_string(M_NUM_POLICY) + ", actual: " +
                            std::to_string(in_policy.size()));

        ///////////////////////
        //CPU POLICY CHECKING//
        ///////////////////////
        double freq_core_min = m_platform_io.read_signal("CPU_FREQUENCY_MIN_AVAIL", GEOPM_DOMAIN_BOARD, 0);
        double freq_core_max = m_platform_io.read_signal("CPU_FREQUENCY_MAX_AVAIL", GEOPM_DOMAIN_BOARD, 0);

        // Check for NAN to set default values for policy
        if (std::isnan(in_policy[M_POLICY_CPU_FREQ_MAX])) {
            in_policy[M_POLICY_CPU_FREQ_MAX] = freq_core_max;
        }

        if (in_policy[M_POLICY_CPU_FREQ_MAX] > freq_core_max ||
            in_policy[M_POLICY_CPU_FREQ_MAX] < freq_core_min ) {
            throw Exception("NodeActivityAgent::" + std::string(__func__) +
                            "():CPU_FREQ_MAX out of range: " +
                            std::to_string(in_policy[M_POLICY_CPU_FREQ_MAX]) +
                            ".", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        // Check for NAN to set default values for policy
        if (std::isnan(in_policy[M_POLICY_CPU_FREQ_EFFICIENT])) {
            in_policy[M_POLICY_CPU_FREQ_EFFICIENT] = freq_core_min;
        }

        if (in_policy[M_POLICY_CPU_FREQ_EFFICIENT] > freq_core_max ||
            in_policy[M_POLICY_CPU_FREQ_EFFICIENT] < freq_core_min ) {
            throw Exception("NodeActivityAgent::" + std::string(__func__) +
                            "():CPU_FREQ_EFFICIENT out of range: " +
                            std::to_string(in_policy[M_POLICY_CPU_FREQ_EFFICIENT]) +
                            ".", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        if (in_policy[M_POLICY_CPU_FREQ_EFFICIENT] > in_policy[M_POLICY_CPU_FREQ_MAX]) {
            throw Exception("NodeActivityAgent::" + std::string(__func__) +
                            "():CPU_FREQ_EFFICIENT (" +
                            std::to_string(in_policy[M_POLICY_CPU_FREQ_EFFICIENT]) +
                            ") value exceeds CPU_FREQ_MAX (" +
                            std::to_string(in_policy[M_POLICY_CPU_FREQ_MAX]) +
                            ").", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        //////////////////////////
        //UNCORE POLICY CHECKING//
        //////////////////////////
        if (std::isnan(in_policy[M_POLICY_UNCORE_FREQ_MAX])) {
            in_policy[M_POLICY_UNCORE_FREQ_MAX] = m_freq_uncore_max;
        }
        if (std::isnan(in_policy[M_POLICY_UNCORE_FREQ_EFFICIENT])) {
            in_policy[M_POLICY_UNCORE_FREQ_EFFICIENT] = m_freq_uncore_min;
        }

        if (in_policy[M_POLICY_UNCORE_FREQ_EFFICIENT] > in_policy[M_POLICY_UNCORE_FREQ_MAX]) {
            throw Exception("NodeActivityAgent::" + std::string(__func__) +
                            "():CPU_UNCORE_FREQ_EFFICIENT (" +
                            std::to_string(in_policy[M_POLICY_UNCORE_FREQ_EFFICIENT]) +
                            ") value exceeds CPU_UNCORE_FREQ_MAX (" +
                            std::to_string(in_policy[M_POLICY_UNCORE_FREQ_MAX]) +
                            ").", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        // If no phi value is provided assume the default behavior.
        if (std::isnan(in_policy[M_POLICY_CPU_PHI])) {
            in_policy[M_POLICY_CPU_PHI] = M_POLICY_PHI_DEFAULT;
        }

        if (in_policy[M_POLICY_CPU_PHI] < 0.0 ||
            in_policy[M_POLICY_CPU_PHI] > 1.0) {
            throw Exception("NodeActivityAgent::" + std::string(__func__) +
                                   "(): POLICY_CPU_PHI value out of range: " +
                                   std::to_string(in_policy[M_POLICY_CPU_PHI]) + ".",
                                   GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        double f_core_max = in_policy[M_POLICY_CPU_FREQ_MAX];
        double f_core_efficient = in_policy[M_POLICY_CPU_FREQ_EFFICIENT];
        double f_core_range = f_core_max - f_core_efficient;

        double f_uncore_max = in_policy[M_POLICY_UNCORE_FREQ_MAX];
        double f_uncore_efficient = in_policy[M_POLICY_UNCORE_FREQ_EFFICIENT];
        double f_uncore_range = f_uncore_max - f_uncore_efficient;

        double cpu_phi = in_policy[M_POLICY_CPU_PHI];

        // If phi is not 0.5 we move into the energy or performance biased behavior
        if (cpu_phi > 0.5) {
            // Energy Biased.  Scale F_max down to F_efficient based upon phi value
            // Active region phi usage
            f_core_max = std::max(f_core_efficient, f_core_max -
                                                    f_core_range * (cpu_phi-0.5) / 0.5);
            f_uncore_max = std::max(f_uncore_efficient, f_uncore_max -
                                                        f_uncore_range * (cpu_phi-0.5) / 0.5);
        }
        else if (cpu_phi < 0.5) {
            // Perf Biased.  Scale F_efficient up to F_max based upon phi value
            // Active region phi usage
            f_core_efficient = std::min(f_core_max, f_core_efficient +
                                                    f_core_range * (0.5-cpu_phi) / 0.5);

            f_uncore_efficient = std::min(f_uncore_max, f_uncore_efficient +
                                                        f_uncore_range * (0.5-cpu_phi) / 0.5);
        }
        //Update Policy
        in_policy[M_POLICY_CPU_FREQ_MAX] = f_core_max;
        in_policy[M_POLICY_CPU_FREQ_EFFICIENT] = f_core_efficient;
        in_policy[M_POLICY_UNCORE_FREQ_MAX] = f_uncore_max;
        in_policy[M_POLICY_UNCORE_FREQ_EFFICIENT] = f_uncore_efficient;

        // Validate all (uncore frequency, max memory bandwidth) pairs
        std::set<double> policy_uncore_freqs;
        for (auto it = in_policy.begin() + M_POLICY_FIRST_UNCORE_FREQ;
             it != in_policy.end() && std::next(it) != in_policy.end(); std::advance(it, 2)) {
            auto mapped_mem_bw = *(it + 1);
            if (!std::isnan(*it)) {
                // We are using a static cast rather than reinterpreting the
                // memory so that regions can be input to this policy in the
                // same form they are output from a report.
                auto uncore_freq = static_cast<uint64_t>(*it);
                if (std::isnan(mapped_mem_bw)) {
                    throw Exception("NodeActivityAgent::" + std::string(__func__) +
                                    "(): mapped CPU_UNCORE_FREQUENCY with no max memory bandwidth.",
                                    GEOPM_ERROR_INVALID, __FILE__, __LINE__);
                }
                // Just make sure the frequency does not have multiple definitions.
                if (!policy_uncore_freqs.insert(uncore_freq).second) {
                    throw Exception("NodeActivityAgent::" + std::string(__func__) +
                                    " policy has multiple entries for CPU_UNCORE_FREQUENCY " +
                                    std::to_string(uncore_freq),
                                    GEOPM_ERROR_INVALID, __FILE__, __LINE__);
                }
            }
            else if (!std::isnan(mapped_mem_bw)) {
                throw Exception("NodeActivityAgent::" + std::string(__func__) +
                                " policy maps a NaN CPU_UNCORE_FREQUENCY with max memory bandwidth: " +
                                std::to_string(mapped_mem_bw),
                                GEOPM_ERROR_INVALID, __FILE__, __LINE__);

            }
        }
    }

    // Distribute incoming policy to children
    void NodeActivityAgent::split_policy(const std::vector<double>& in_policy,
                                        std::vector<std::vector<double> >& out_policy)
    {
        //assert(in_policy.size() == M_NUM_POLICY);
        for (auto &child_pol : out_policy) {
            child_pol = in_policy;
        }
    }

    // Indicate whether to send the policy down to children
    bool NodeActivityAgent::do_send_policy(void) const
    {
        return true;
    }

    void NodeActivityAgent::aggregate_sample(const std::vector<std::vector<double> > &in_sample,
                                        std::vector<double>& out_sample)
    {

    }

    // Indicate whether to send samples up to the parent
    bool NodeActivityAgent::do_send_sample(void) const
    {
        return false;
    }

    void NodeActivityAgent::adjust_platform(const std::vector<double>& in_policy)
    {
        //assert(in_policy.size() == M_NUM_POLICY);

        m_do_write_batch = false;

        if (m_update_qm_max_rate) {
            for (auto it = in_policy.begin() + M_POLICY_FIRST_UNCORE_FREQ;
                 it != in_policy.end() && std::next(it) != in_policy.end();
                 std::advance(it, 2)) {

                if (!std::isnan(*it)) {
                    auto uncore_freq = static_cast<uint64_t>(*it);
                    auto max_mem_bw = *(it + 1);
                    // Not valid to have NAN max mem bw for uncore freq.
                    GEOPM_DEBUG_ASSERT(!std::isnan(max_mem_bw),
                                       "mapped CPU_UNCORE_FREQUENCY with no max memory bandwidth assigned.");
                    m_qm_max_rate[uncore_freq] = max_mem_bw;
                }
            }
            // Warn the user if they have not entered a policy with memory bandwidth characterization.
            // In this case the maximum uncore frequency will be used.  This can easily be converted
            // to an assertion if it should be considered a failure case.
            if (m_qm_max_rate.size() == 0) {
                std::cerr << "Warning: <geopm> NodeActivityAgent did not receive a policy containing memory " <<
                             "bandwidth characterization.  This may negatively impact agent performance." << std::endl;
            }
            m_update_qm_max_rate = false;
        }

        // Per package freq
        std::vector<double> uncore_freq_request;
        m_resolved_f_uncore_efficient = in_policy[M_POLICY_UNCORE_FREQ_EFFICIENT];
        m_resolved_f_uncore_max = in_policy[M_POLICY_UNCORE_FREQ_MAX];
        double f_uncore_range = in_policy[M_POLICY_UNCORE_FREQ_MAX] - in_policy[M_POLICY_UNCORE_FREQ_EFFICIENT];

        // Setup Uncore Activity model
        if (m_qm_max_rate.size() != 0) {
            m_activity_perf_model.update_uncore_bandwidth_map(m_qm_max_rate);
        }
        m_activity_perf_model.set_frequency_bounds(ActivityPerformanceModel::M_DOMAIN_CPU_UNCORE,
                                                   in_policy[M_POLICY_UNCORE_FREQ_EFFICIENT],
                                                   in_policy[M_POLICY_UNCORE_FREQ_MAX]);

        // Per core freq
        std::vector<double> core_freq_request;
        m_resolved_f_core_efficient = in_policy[M_POLICY_CPU_FREQ_EFFICIENT];
        m_resolved_f_core_max = in_policy[M_POLICY_CPU_FREQ_MAX];
        double f_core_range = in_policy[M_POLICY_CPU_FREQ_MAX] - in_policy[M_POLICY_CPU_FREQ_EFFICIENT];

        // Setup Core Activity model
        m_activity_perf_model.set_frequency_bounds(ActivityPerformanceModel::M_DOMAIN_CPU_CORE,
                                                   in_policy[M_POLICY_CPU_FREQ_EFFICIENT],
                                                   in_policy[M_POLICY_CPU_FREQ_MAX]);

        // Per GPU freq
        std::vector<double> gpu_freq_request;
        m_f_max = in_policy[M_POLICY_GPU_FREQ_MAX];
        m_f_efficient = in_policy[M_POLICY_GPU_FREQ_EFFICIENT];
        m_f_range = m_f_max - m_f_efficient;

        // Setup GPU Activity model
        m_activity_perf_model.set_frequency_bounds(ActivityPerformanceModel::M_DOMAIN_GPU_CORE,
                                                   in_policy[M_POLICY_GPU_FREQ_EFFICIENT],
                                                   in_policy[M_POLICY_GPU_FREQ_MAX]);

        for (int domain_idx = 0; domain_idx < M_NUM_PACKAGE; ++domain_idx) {
            double uncore_freq = (double) m_uncore_freq_status.at(domain_idx).value;

            /////////////////////////////////////////////
            // L3 Total External Bandwidth Measurement //
            /////////////////////////////////////////////
            auto qm_max_itr = m_qm_max_rate.lower_bound(uncore_freq);
            if(qm_max_itr != m_qm_max_rate.begin()) {
                qm_max_itr = std::prev(qm_max_itr, 1);
            }

            double scalability_uncore = 1.0;

            // Handle divided by zero, either numerator or
            // denominator being NAN, and the un-characterized case
            if (!std::isnan(m_qm_rate.at(domain_idx).value) &&
                !std::isnan(qm_max_itr->second) &&
                qm_max_itr->second != 0 &&
                m_qm_max_rate.size() != 0) {
                scalability_uncore = (double) m_qm_rate.at(domain_idx).value /
                                              qm_max_itr->second;
            }
            double uncore_req = m_activity_perf_model.get_frequency_recommendation(ActivityPerformanceModel::M_DOMAIN_CPU_UNCORE,
                                                                                   scalability_uncore);
            uncore_freq_request.push_back(uncore_req);
        }

        for (int domain_idx = 0; domain_idx < M_NUM_CORE; ++domain_idx) {
            //////////////////////////////////
            // Core Scalability Measurement //
            //////////////////////////////////
            double scalability = (double) m_core_scal.at(domain_idx).value;
            if (std::isnan(scalability)) {
                scalability = 1.0;
            }
            double core_req = m_activity_perf_model.get_frequency_recommendation(ActivityPerformanceModel::M_DOMAIN_CPU_CORE,
                                                                                 scalability);
            core_freq_request.push_back(core_req);
        }

        // Per GPU Frequency Selection
        for (int domain_idx = 0; domain_idx < m_agent_domain_count; ++domain_idx) {
            // gpu Compute Activity - Primary signal used for frequency recommendation
            double gpu_core_activity = m_gpu_core_activity.at(domain_idx).value;
            // gpu Utilization - Used to scale activity for short GPU phases
            double gpu_utilization = m_gpu_utilization.at(domain_idx).value;

            // Default to F_max
            double f_request = m_f_max;
            double gpu_scalability = 1;

            if (!std::isnan(gpu_core_activity)) {
                // Boundary Checking
                gpu_core_activity = std::min(gpu_core_activity, 1.0);

                // Frequency selection is based upon the gpu compute activity.
                // For active regions this means that we scale with the amount of work
                // being done (such as SM_ACTIVE for NVIDIA GPUs).
                //
                // The compute activity is scaled by the GPU Utilization, to help
                // address the issues that come from workloads have short phases that are
                // frequency sensitive.  If a workload has a compute activity of 0.5, and
                // is resident on the GPU for 50% of cycles (0.5) it is treated as having
                // a compute activity value of 1.0 (100%)
                if (!std::isnan(gpu_utilization) &&
                    gpu_utilization > 0) {
                    gpu_utilization = std::min(gpu_utilization, 1.0);
                    gpu_scalability = gpu_core_activity / gpu_utilization;
                }
                else {
                    gpu_scalability = gpu_core_activity;
                }
                f_request = m_activity_perf_model.get_frequency_recommendation(ActivityPerformanceModel::M_DOMAIN_GPU_CORE,
                                                                               gpu_scalability);

                // Doing energy reading per GPU, not per CHIP
                if (domain_idx % (M_NUM_CHIP_PER_GPU) == 0) {
                    int gpu_idx = domain_idx / M_NUM_CHIP_PER_GPU;
                    // Tracking logic.  This is not needed for any performance reason,
                    // but does provide useful metrics for tracking agent behavior
                    if (gpu_core_activity >= M_GPU_ACTIVITY_CUTOFF) {
                        m_gpu_active_region_stop.at(gpu_idx) = 0;
                        if (m_gpu_active_region_start.at(gpu_idx) == 0) {
                            m_gpu_active_region_start.at(gpu_idx) = m_time.value;
                            m_gpu_active_energy_start.at(gpu_idx) = m_gpu_energy.at(gpu_idx).value;
                        }
                    }
                    else {
                        if (m_gpu_active_region_stop.at(gpu_idx) == 0) {
                            m_gpu_active_region_stop.at(gpu_idx) = m_time.value;
                            m_gpu_active_energy_stop.at(gpu_idx) = m_gpu_energy.at(gpu_idx).value;
                        }
                    }
                }
            }

            // Store frequency request
            gpu_freq_request.push_back(f_request);
        }

        if (!gpu_freq_request.empty()) {
            // set frequency control per gpu
            for (int domain_idx = 0; domain_idx < m_agent_domain_count; ++domain_idx) {
                if (gpu_freq_request.at(domain_idx) !=
                    m_gpu_freq_min_control.at(domain_idx).last_setting ||
                    gpu_freq_request.at(domain_idx) !=
                    m_gpu_freq_max_control.at(domain_idx).last_setting) {

                    m_platform_io.adjust(m_gpu_freq_min_control.at(domain_idx).batch_idx,
                                         gpu_freq_request.at(domain_idx));
                    m_gpu_freq_min_control.at(domain_idx).last_setting =
                                         gpu_freq_request.at(domain_idx);

                    m_platform_io.adjust(m_gpu_freq_max_control.at(domain_idx).batch_idx,
                                         gpu_freq_request.at(domain_idx));
                    m_gpu_freq_max_control.at(domain_idx).last_setting =
                                         gpu_freq_request.at(domain_idx);
                    ++m_gpu_frequency_requests;
                }
            }
            m_do_write_batch = true;
        }

        // set per core controls
        for (int domain_idx = 0; domain_idx < M_NUM_CORE; ++domain_idx) {
            if(std::isnan(core_freq_request.at(domain_idx))) {
                core_freq_request.at(domain_idx) = in_policy[M_POLICY_CPU_FREQ_MAX];
            }
            if (core_freq_request.at(domain_idx) !=
                m_core_freq_control.at(domain_idx).last_setting) {

                m_platform_io.adjust(m_core_freq_control.at(domain_idx).batch_idx,
                                     core_freq_request.at(domain_idx));

                m_core_freq_control.at(domain_idx).last_setting = core_freq_request.at(domain_idx);
                ++m_core_frequency_requests;
                m_do_write_batch = true;
            }
        }

        // set per package controls
        for (int domain_idx = 0; domain_idx < M_NUM_PACKAGE; ++domain_idx) {
            if(std::isnan(uncore_freq_request.at(domain_idx))) {
                uncore_freq_request.at(domain_idx) = in_policy[M_POLICY_UNCORE_FREQ_MAX];
            }

            if (uncore_freq_request.at(domain_idx) !=
                m_uncore_freq_min_control.at(domain_idx).last_setting ||
                uncore_freq_request.at(domain_idx) !=
                m_uncore_freq_max_control.at(domain_idx).last_setting) {
                //Adjust
                m_platform_io.adjust(m_uncore_freq_min_control.at(domain_idx).batch_idx,
                                    uncore_freq_request.at(domain_idx));

                m_platform_io.adjust(m_uncore_freq_max_control.at(domain_idx).batch_idx,
                                    uncore_freq_request.at(domain_idx));

                //save the value for future comparison
                m_uncore_freq_min_control.at(domain_idx).last_setting = uncore_freq_request.at(domain_idx);
                m_uncore_freq_max_control.at(domain_idx).last_setting = uncore_freq_request.at(domain_idx);
                ++m_uncore_frequency_requests;

                m_do_write_batch = true;
            }
        }
    }

    // If controls have a valid updated value write them.
    bool NodeActivityAgent::do_write_batch(void) const
    {
        return m_do_write_batch;
    }

    // Read signals from the platform and calculate samples to be sent up
    void NodeActivityAgent::sample_platform(std::vector<double> &out_sample)
    {
        assert(out_sample.size() == M_NUM_SAMPLE);

        // Collect latest GPU signal values
        for (int domain_idx = 0; domain_idx < M_NUM_GPU_CHIP; ++domain_idx) {
            m_gpu_freq_status.at(domain_idx).value = m_platform_io.sample(m_gpu_freq_status.at(
                                                                          domain_idx).batch_idx);
            m_gpu_core_activity.at(domain_idx).value = m_platform_io.sample(m_gpu_core_activity.at(
                                                                               domain_idx).batch_idx);
            m_gpu_utilization.at(domain_idx).value = m_platform_io.sample(m_gpu_utilization.at(
                                                                          domain_idx).batch_idx);
        }

        for (int domain_idx = 0; domain_idx < M_NUM_GPU; ++domain_idx) {
            m_gpu_energy.at(domain_idx).value = m_platform_io.sample(m_gpu_energy.at(
                                                                     domain_idx).batch_idx);
        }

        m_time.value = m_platform_io.sample(m_time.batch_idx);

        // Collect latest Core signal values
        for (int domain_idx = 0; domain_idx < M_NUM_PACKAGE; ++domain_idx) {
            // Frequency signals
            m_uncore_freq_status.at(domain_idx).value = m_platform_io.sample(m_uncore_freq_status.at(domain_idx).batch_idx);

            // Uncore steering signals
            m_qm_rate.at(domain_idx).value = m_platform_io.sample(m_qm_rate.at(domain_idx).batch_idx);
        }

        for (int domain_idx = 0; domain_idx < M_NUM_CORE; ++domain_idx) {
            // Core steering signals
            m_core_scal.at(domain_idx).value = m_platform_io.sample(m_core_scal.at(domain_idx).batch_idx);
        }


    }

    // Wait for the remaining cycle time to keep Controller loop cadence
    void NodeActivityAgent::wait(void)
    {
        geopm_time_s current_time;
        do {
            geopm_time(&current_time);
        }
        while(geopm_time_diff(&m_last_wait, &current_time) < M_WAIT_SEC);
        geopm_time(&m_last_wait);
    }

    // Adds the wait time to the top of the report
    std::vector<std::pair<std::string, std::string> > NodeActivityAgent::report_header(void) const
    {
        return {{"Wait time (sec)", std::to_string(M_WAIT_SEC)}};
    }

    // Adds number of frquency requests to the per-node section of the report
    std::vector<std::pair<std::string, std::string> > NodeActivityAgent::report_host(void) const
    {
        std::vector<std::pair<std::string, std::string> > result;

        result.push_back({"Core Frequency Requests", std::to_string(m_core_frequency_requests)});
        result.push_back({"Uncore Frequency Requests", std::to_string(m_uncore_frequency_requests)});
        result.push_back({"Resolved Maximum Core Frequency", std::to_string(m_resolved_f_core_max)});
        result.push_back({"Resolved Efficient Core Frequency", std::to_string(m_resolved_f_core_efficient)});
        result.push_back({"Resolved Core Frequency Range", std::to_string(m_resolved_f_core_max - m_resolved_f_core_efficient)});
        result.push_back({"Resolved Maximum Uncore Frequency", std::to_string(m_resolved_f_uncore_max)});
        result.push_back({"Resolved Efficient Uncore Frequency", std::to_string(m_resolved_f_uncore_efficient)});
        result.push_back({"Resolved Uncore Frequency Range", std::to_string(m_resolved_f_uncore_max - m_resolved_f_uncore_efficient)});


        result.push_back({"GPU Frequency Requests", std::to_string(m_gpu_frequency_requests)});
        result.push_back({"Resolved Max Frequency", std::to_string(m_f_max)});
        result.push_back({"Resolved Efficient Frequency", std::to_string(m_f_efficient)});
        result.push_back({"Resolved Frequency Range", std::to_string(m_f_range)});

        for (int domain_idx = 0; domain_idx < M_NUM_GPU; ++domain_idx) {
            double energy_stop = m_gpu_active_energy_stop.at(domain_idx);
            double energy_start = m_gpu_active_energy_start.at(domain_idx);
            double region_stop = m_gpu_active_region_stop.at(domain_idx);
            double region_start =  m_gpu_active_region_start.at(domain_idx);
            result.push_back({"GPU " + std::to_string(domain_idx) +
                              " Active Region Energy", std::to_string(energy_stop - energy_start)});
            result.push_back({"GPU " + std::to_string(domain_idx) +
                              " Active Region Time", std::to_string(region_stop - region_start)});
        }

        return result;
    }

    // This Agent does not add any per-region details
    std::map<uint64_t, std::vector<std::pair<std::string, std::string> > > NodeActivityAgent::report_region(void) const
    {
        return {};
    }

    // Adds trace columns signals of interest
    std::vector<std::string> NodeActivityAgent::trace_names(void) const
    {
        return {};
    }

    // Updates the trace with values for signals from this Agent
    void NodeActivityAgent::trace_values(std::vector<double> &values)
    {
    }

    void NodeActivityAgent::enforce_policy(const std::vector<double> &policy) const
    {
    }

    std::vector<std::function<std::string(double)> > NodeActivityAgent::trace_formats(void) const
    {
        return {};
    }

    // Name used for registration with the Agent factory
    std::string NodeActivityAgent::plugin_name(void)
    {
        return "node_activity";
    }

    // Used by the factory to create objects of this type
    std::unique_ptr<Agent> NodeActivityAgent::make_plugin(void)
    {
        return geopm::make_unique<NodeActivityAgent>();
    }

    // Describes expected policies to be provided by the resource manager or user
    std::vector<std::string> NodeActivityAgent::policy_names(void)
    {
        std::vector<std::string> names{"GPU_FREQ_MAX", "GPU_FREQ_EFFICIENT", "GPU_PHI",
                                       "CPU_FREQ_MAX", "CPU_FREQ_EFFICIENT", "CPU_UNCORE_FREQ_MAX",
                                       "CPU_UNCORE_FREQ_EFFICIENT", "CPU_PHI"};
        names.reserve(M_NUM_POLICY);

        for (size_t i = 0; names.size() < M_NUM_POLICY; ++i) {
            names.emplace_back("CPU_UNCORE_FREQ_" + std::to_string(i));
            names.emplace_back("MAX_MEMORY_BANDWIDTH_" + std::to_string(i));
        }
        return names;
    }

    // Describes samples to be provided to the resource manager or user
    std::vector<std::string> NodeActivityAgent::sample_names(void)
    {
        return {};
    }
}
