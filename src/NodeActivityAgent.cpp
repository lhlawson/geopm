/*
 * Copyright (c) 2015 - 2022, Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "config.h"
#include "NodeActivityAgent.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "geopm/Agg.hpp"
#include "geopm/Exception.hpp"
#include "geopm/Helper.hpp"
#include "geopm/PlatformIO.hpp"
#include "geopm/PlatformTopo.hpp"
#include "geopm/PluginFactory.hpp"
#include "geopm_debug.hpp"

#include "PlatformIOProf.hpp"
#include "FrequencyGovernor.hpp"

namespace geopm
{

    NodeActivityAgent::NodeActivityAgent()
        : NodeActivityAgent(PlatformIOProf::platform_io(), platform_topo(),
                            FrequencyGovernor::make_shared())
    {

    }

    NodeActivityAgent::NodeActivityAgent(PlatformIO &plat_io,
                                         const PlatformTopo &topo,
                                         std::shared_ptr<FrequencyGovernor> gov)
        : m_platform_io(plat_io)
        , m_platform_topo(topo)
        , m_last_wait{{0, 0}}
        , M_WAIT_SEC(0.020) // 20ms wait default
        // GPU centric entries
        , M_POLICY_PHI_DEFAULT(0.5)
        , M_GPU_ACTIVITY_CUTOFF(0.20)
        , M_NUM_GPU(m_platform_topo.num_domain(
                    GEOPM_DOMAIN_GPU))
        , M_NUM_GPU_CHIP(m_platform_topo.num_domain(
                         GEOPM_DOMAIN_GPU_CHIP))
        , M_NUM_CHIP_PER_GPU(M_NUM_GPU_CHIP / M_NUM_GPU)
        , m_do_write_batch(false)
        , m_do_send_policy(true)
        // CPU centric entries
        , M_NUM_PACKAGE(m_platform_topo.num_domain(GEOPM_DOMAIN_PACKAGE))
        , m_freq_governor(gov)
        , m_freq_ctl_domain_type(m_freq_governor->frequency_domain_type())
        , m_num_freq_ctl_domain(m_platform_topo.num_domain(m_freq_ctl_domain_type))
        , m_core_batch_writes(0)
        , m_uncore_frequency_requests(0)
        , m_uncore_frequency_clamped(0)
        , m_resolved_f_uncore_efficient(0)
        , m_resolved_f_uncore_max(0)
        , m_resolved_f_core_efficient(0)
        , m_resolved_f_core_max(0)
    {
        geopm_time(&m_last_wait);
    }

    void NodeActivityAgent::init(int level, const std::vector<int> &fan_in, bool is_level_root)
    {
        m_gpu_frequency_requests = 0;
        m_gpu_frequency_clipped = 0;
        m_resolved_f_gpu_max = 0;
        m_resolved_f_gpu_efficient = 0;
        m_f_range = 0;

        m_freq_uncore_min = m_platform_io.read_signal("CPU_UNCORE_FREQUENCY_MIN_CONTROL", GEOPM_DOMAIN_BOARD, 0);
        m_freq_uncore_max = m_platform_io.read_signal("CPU_UNCORE_FREQUENCY_MAX_CONTROL", GEOPM_DOMAIN_BOARD, 0);
        m_resolved_f_uncore_max = m_freq_uncore_max;

        for (int domain_idx = 0; domain_idx < M_NUM_GPU; ++domain_idx) {
            m_gpu_active_region_start.push_back(0.0);
            m_gpu_active_region_stop.push_back(0.0);
            m_gpu_active_energy_start.push_back(0.0);
            m_gpu_active_energy_stop.push_back(0.0);

            m_gpu_on_time.push_back(0.0);

            m_gpu_prev_energy.push_back(0.0);
            m_gpu_on_energy.push_back(0.0);
        }
        m_cpu_active_energy_start = 0.0;
        m_cpu_active_energy_stop = 0.0;
        m_cpu_on_energy = 0.0;

        if (level == 0) {
            init_gpu_platform_io();
            init_cpu_platform_io();
            init_constconfig_io();
        }
    }

    void NodeActivityAgent::init_cpu_platform_io(void)
    {
        int scalability_signal_domain = m_platform_io.signal_domain_type("MSR::CPU_SCALABILITY_RATIO");

        // If the frequency control domain does not contain the scalabilty domain
        // (i.e. the scalability domain is coarser than the freq domain) use the
        // scalability domain for frequency control.
        if (!m_platform_topo.is_nested_domain(scalability_signal_domain,
                                              m_freq_ctl_domain_type)) {

#ifdef GEOPM_DEBUG
            std::cerr << "CPUActivityAgent::" + std::string(__func__) +
                          "():MSR::CPU_SCALABILITY_RATIO domain (" +
                          std::to_string(scalability_signal_domain) +
                          ") is a coarser granularity than the CPU frequency control domain (" +
                          std::to_string(m_freq_ctl_domain_type) + ").";
#endif

            // Set Freq gov domain.
            m_freq_governor->set_domain_type(scalability_signal_domain);

            // update member vars
            m_freq_ctl_domain_type = m_freq_governor->frequency_domain_type();
            m_num_freq_ctl_domain = m_platform_topo.num_domain(m_freq_ctl_domain_type);
        }

        m_freq_governor->init_platform_io();

        m_freq_core_min = m_freq_governor->get_frequency_min();
        m_freq_core_max = m_freq_governor->get_frequency_max();

        for (int domain_idx = 0; domain_idx < m_num_freq_ctl_domain; ++domain_idx) {
            m_core_scal.push_back({m_platform_io.push_signal("MSR::CPU_SCALABILITY_RATIO",
                                                             m_freq_ctl_domain_type,
                                                             domain_idx), NAN});
        }

        for (int domain_idx = 0; domain_idx < M_NUM_PACKAGE; ++domain_idx) {
            m_qm_rate.push_back({m_platform_io.push_signal("MSR::QM_CTR_SCALED_RATE",
                                                           GEOPM_DOMAIN_PACKAGE,
                                                           domain_idx), NAN});

            m_uncore_freq_status.push_back({m_platform_io.push_signal("CPU_UNCORE_FREQUENCY_STATUS",
                                                                      GEOPM_DOMAIN_PACKAGE,
                                                                      domain_idx), NAN});

            m_uncore_freq_max_control.push_back({m_platform_io.push_control("CPU_UNCORE_FREQUENCY_MAX_CONTROL",
                                                                            GEOPM_DOMAIN_PACKAGE,
                                                                            domain_idx), -1});
        }

        // We treat energy as a special case and only use it at a specific domain.
        // It is only used for tracking agent behavior/reporting
        // and does not impact the agent algorithm
        m_cpu_energy = {m_platform_io.push_signal("CPU_ENERGY", GEOPM_DOMAIN_BOARD, 0), NAN};

        // Trust the FW during periods of idle
        m_platform_io.write_control("CPU_UNCORE_FREQUENCY_MIN_CONTROL", GEOPM_DOMAIN_BOARD, 0, m_freq_uncore_min);

        // Configuration of QM_CTR must match QM_CTR config used for tuning/training data.
        // Assign all cores to resource monitoring association ID 0.  This allows for
        // monitoring the resource usage of all cores.
        m_platform_io.write_control("MSR::PQR_ASSOC:RMID", GEOPM_DOMAIN_BOARD, 0, 0);
        // Assign the resource monitoring ID for QM Events to match the per core resource
        // association ID above (0)
        m_platform_io.write_control("MSR::QM_EVTSEL:RMID", GEOPM_DOMAIN_BOARD, 0, 0);
        // Select monitoring event ID 0x2 - Total Memory Bandwidth Monitoring.  This
        // is used to determine the Xeon Uncore utilization.
        m_platform_io.write_control("MSR::QM_EVTSEL:EVENT_ID", GEOPM_DOMAIN_BOARD, 0, 2);
    }

    // Push signals and controls for future batch read/write
    void NodeActivityAgent::init_gpu_platform_io(void)
    {

        std::vector<int> control_domains;
        control_domains.push_back(m_platform_io.control_domain_type("GPU_CORE_FREQUENCY_MIN_CONTROL"));
        control_domains.push_back(m_platform_io.control_domain_type("GPU_CORE_FREQUENCY_MAX_CONTROL"));

        std::vector<int> signal_domains;
        signal_domains.push_back(m_platform_io.signal_domain_type("GPU_CORE_FREQUENCY_STATUS"));
        signal_domains.push_back(m_platform_io.signal_domain_type("GPU_CORE_ACTIVITY"));
        signal_domains.push_back(m_platform_io.signal_domain_type("GPU_UTILIZATION"));

        // We'll use the coarsest granularity supported by any of the controls or signals except Energy
        // i.e. If one control supports domain GPU and another supports domain GPU_CHIP
        m_agent_domain = std::min(*std::min_element(std::begin(control_domains), std::end(control_domains)),
                                  *std::min_element(std::begin(signal_domains), std::end(signal_domains)));

#ifdef GEOPM_DEBUG
        {
            int max_agent_domain = std::max(*std::max_element(std::begin(control_domains), std::end(control_domains)),
                                            *std::max_element(std::begin(signal_domains), std::end(signal_domains)));

            if (m_agent_domain != max_agent_domain) {
                throw Exception("NodeActivityAgent::" + std::string(__func__) +
                                "(): Required signals and controls do not all exist at the same domain.",
                                GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }
        }
#endif

        if (m_agent_domain != GEOPM_DOMAIN_GPU &&
            m_agent_domain != GEOPM_DOMAIN_GPU_CHIP) {
            throw Exception("NodeActivityAgent::" + std::string(__func__) +
                            "(): Required signals and controls do not exist at the " +
                            "GPU or GPU_CHIP domain!", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        m_agent_domain_count = m_platform_topo.num_domain(m_agent_domain);

        for (int domain_idx = 0; domain_idx < m_agent_domain_count; ++domain_idx) {
            // Signals
            m_gpu_core_activity.push_back({m_platform_io.push_signal("GPU_CORE_ACTIVITY",
                                           m_agent_domain,
                                           domain_idx), NAN});
            m_gpu_utilization.push_back({m_platform_io.push_signal("GPU_UTILIZATION",
                                         m_agent_domain,
                                         domain_idx), NAN});

            // Controls
            m_gpu_freq_min_control.push_back(m_control{m_platform_io.push_control("GPU_CORE_FREQUENCY_MIN_CONTROL",
                                                       m_agent_domain,
                                                       domain_idx), NAN});
            m_gpu_freq_max_control.push_back(m_control{m_platform_io.push_control("GPU_CORE_FREQUENCY_MAX_CONTROL",
                                                       m_agent_domain,
                                                       domain_idx), NAN});
            m_gpu_idle_timer.push_back(10);
            m_gpu_idle_samples.push_back(0);
        }

        // We treat energy & time as special cases and only use them at a specific domain.
        // This is because energy & time are used for for tracking agent behavior/reporting
        // and do not impact the agent algorithm
        for (int domain_idx = 0; domain_idx < M_NUM_GPU; ++domain_idx) {
            m_gpu_energy.push_back({m_platform_io.push_signal("GPU_ENERGY",
                                    m_platform_io.signal_domain_type("GPU_ENERGY"),
                                    domain_idx), NAN});
        }

        m_freq_gpu_min = m_platform_io.read_signal("GPU_CORE_FREQUENCY_MIN_AVAIL", GEOPM_DOMAIN_BOARD, 0);
        m_freq_gpu_max = m_platform_io.read_signal("GPU_CORE_FREQUENCY_MAX_AVAIL", GEOPM_DOMAIN_BOARD, 0);

        //m_platform_io.write_control("GPU_CORE_FREQUENCY_MIN_CONTROL", GEOPM_DOMAIN_BOARD, 0, m_freq_gpu_min);

        const auto ALL_NAMES = m_platform_io.signal_names();
        // F efficient values
        const std::string FE_CONSTCONFIG = "CONST_CONFIG::GPU_FREQUENCY_EFFICIENT_HIGH_INTENSITY";
        const std::string FE_SIG_NAME = "LEVELZERO::GPU_CORE_FREQUENCY_EFFICIENT";
        if (ALL_NAMES.count(FE_CONSTCONFIG) != 0) {
            m_freq_gpu_efficient = m_platform_io.read_signal(FE_CONSTCONFIG, GEOPM_DOMAIN_BOARD, 0);
        }
        else if (ALL_NAMES.count(FE_SIG_NAME) != 0) {
            m_freq_gpu_efficient = m_platform_io.read_signal(FE_SIG_NAME, GEOPM_DOMAIN_BOARD, 0);
        }
        else {
            m_freq_gpu_efficient = (m_freq_gpu_max + m_freq_gpu_min) / 2;
        }

        if (m_freq_gpu_efficient > m_freq_gpu_max ||
            m_freq_gpu_efficient < m_freq_gpu_min ) {
            throw Exception("NodeActivityAgent::" + std::string(__func__) +
                            "(): GPU efficient frequency out of range: " +
                            std::to_string(m_freq_gpu_efficient) +
                            ".", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
    }

    void NodeActivityAgent::init_constconfig_io()
    {
        m_qm_max_rate = {};
        const auto ALL_NAMES = m_platform_io.signal_names();

        // F efficient values
        std::string fe_constconfig = "CONST_CONFIG::CPU_FREQUENCY_EFFICIENT_HIGH_INTENSITY";
        if (ALL_NAMES.count(fe_constconfig) != 0) {
            m_freq_core_efficient = m_platform_io.read_signal(fe_constconfig, GEOPM_DOMAIN_BOARD, 0);
        }
        else {
            m_freq_core_efficient = m_freq_core_min;
        }

        fe_constconfig = "CONST_CONFIG::CPU_UNCORE_FREQUENCY_EFFICIENT_HIGH_INTENSITY";
        if (ALL_NAMES.count(fe_constconfig) != 0) {
            m_freq_uncore_efficient = m_platform_io.read_signal(fe_constconfig, GEOPM_DOMAIN_BOARD, 0);
        }
        else {
            m_freq_uncore_efficient = m_freq_uncore_min;
        }

        if (m_freq_core_efficient > m_freq_core_max ||
            m_freq_core_efficient < m_freq_core_min ) {
            throw Exception("CPUActivityAgent::" + std::string(__func__) +
                            "(): Core efficient frequency out of range: " +
                            std::to_string(m_freq_core_efficient) +
                            ".", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        if (m_freq_uncore_efficient > m_freq_uncore_max ||
            m_freq_uncore_efficient < m_freq_uncore_min ) {
            throw Exception("CPUActivityAgent::" + std::string(__func__) +
                            "(): Uncore efficient frequency out of range: " +
                            std::to_string(m_freq_uncore_efficient) +
                            ".", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        // Grab all (uncore frequency, max memory bandwidth) pairs
        for (unsigned int entry_idx = 0; entry_idx < ALL_NAMES.size(); ++entry_idx) {
            const std::string KEY_NAME = "CONST_CONFIG::CPU_UNCORE_FREQUENCY_" +
                                          std::to_string(entry_idx);
            const std::string VAL_NAME = "CONST_CONFIG::CPU_UNCORE_MAX_MEMORY_BANDWIDTH_" +
                                          std::to_string(entry_idx);
            if (ALL_NAMES.count(KEY_NAME) != 0 &&
                ALL_NAMES.count(VAL_NAME) != 0) {
                double uncore_freq = m_platform_io.read_signal(KEY_NAME, GEOPM_DOMAIN_BOARD, 0);
                double max_mem_bw = m_platform_io.read_signal(VAL_NAME, GEOPM_DOMAIN_BOARD, 0);
                if (!std::isnan(uncore_freq) && !std::isnan(max_mem_bw) &&
                    uncore_freq != 0 && max_mem_bw != 0) {
                    m_qm_max_rate[uncore_freq] = max_mem_bw;
                }
            }
        }

        if (m_qm_max_rate.empty()) {
            throw Exception("CPUActivityAgent::" + std::string(__func__) +
                            "(): ConstConfigIOGroup configuration file does not contain" +
                            " memory bandwidth information.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
    }

    // Validate incoming policy and configure default policy requests.
    void NodeActivityAgent::validate_policy(std::vector<double> &in_policy) const
    {
        GEOPM_DEBUG_ASSERT(in_policy.size() == M_NUM_POLICY,
                           "NodeActivityAgent::validate_policy(): policy vector incorrectly sized");

        // If no phi value is provided assume the default behavior.
        if (std::isnan(in_policy[M_POLICY_PHI])) {
            in_policy[M_POLICY_PHI] = M_POLICY_PHI_DEFAULT;
        }

        if (in_policy[M_POLICY_PHI] < 0.0 ||
            in_policy[M_POLICY_PHI] > 1.0) {
            throw Exception("NodeActivityAgent::" + std::string(__func__) +
                            "(): POLICY_PHI value out of range: " +
                            std::to_string(in_policy[M_POLICY_PHI]) + ".",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
    }

    // Distribute incoming policy to children
    void NodeActivityAgent::split_policy(const std::vector<double>& in_policy,
                                        std::vector<std::vector<double> >& out_policy)
    {
        GEOPM_DEBUG_ASSERT(in_policy.size() == M_NUM_POLICY,
                           "NodeActivityAgent::split_policy(): policy vector incorrectly sized");
        for (auto &child_pol : out_policy) {
            child_pol = in_policy;
        }
    }

    // Indicate whether to send the policy down to children
    bool NodeActivityAgent::do_send_policy(void) const
    {
        return m_do_send_policy;
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
        bool cpu_write_batch = cpu_adjust_platform(in_policy);
        bool gpu_write_batch = gpu_adjust_platform(in_policy);
        m_do_write_batch = cpu_write_batch | gpu_write_batch;
    }

    bool NodeActivityAgent::cpu_adjust_platform(const std::vector<double>& in_policy)
    {
        m_do_send_policy = false;
        bool do_write_batch = false;

        // Calculate new frequency range values
        double f_core_range = m_freq_core_max - m_freq_core_efficient;
        double f_uncore_range = m_freq_uncore_max - m_freq_uncore_efficient;

        double phi = in_policy[M_POLICY_PHI];

        // Default phi = 0.5 case is full fe to fmax range
        // Core
        m_resolved_f_core_max = m_freq_core_max;
        m_resolved_f_core_efficient = m_freq_core_efficient;
        // Uncore
        m_resolved_f_uncore_max = m_freq_uncore_max;
        m_resolved_f_uncore_efficient = m_freq_uncore_efficient;

        // If phi is not 0.5 we move into the energy or performance biased behavior
        if (phi > 0.5) {
            // Energy Biased.  Scale F_max down to F_efficient based upon phi value
            // Active region phi usage
            m_resolved_f_core_max = std::max(m_freq_core_efficient, m_freq_core_max -
                                                                    f_core_range *
                                                                    (phi-0.5) / 0.5);

            m_resolved_f_uncore_max = std::max(m_freq_uncore_efficient, m_freq_uncore_max -
                                                                        f_uncore_range *
                                                                        (phi-0.5) / 0.5);
        }
        else if (phi < 0.5) {
            // Perf Biased.  Scale F_efficient up to F_max based upon phi value
            // Active region phi usage
            m_resolved_f_core_efficient = std::min(m_freq_core_max, m_freq_core_efficient +
                                                                    f_core_range *
                                                                    (0.5-phi) / 0.5);

            m_resolved_f_uncore_efficient = std::min(m_freq_uncore_max, m_freq_uncore_efficient +
                                                                        f_uncore_range *
                                                                        (0.5-phi) / 0.5);
        }

        //Update Policy
        m_freq_governor->validate_policy(m_resolved_f_core_efficient,
                                         m_resolved_f_core_max);
        m_freq_governor->set_frequency_bounds(m_resolved_f_core_efficient,
                                              m_resolved_f_core_max);

        f_core_range = m_resolved_f_core_max - m_resolved_f_core_efficient;
        f_uncore_range = m_resolved_f_uncore_max - m_resolved_f_uncore_efficient;

        // Per package freq
        std::vector<double> uncore_freq_request;

        for (int domain_idx = 0; domain_idx < M_NUM_PACKAGE; ++domain_idx) {
            double uncore_freq = (double) m_uncore_freq_status.at(domain_idx).value;

            /////////////////////////////////////////////
            // L3 Total External Bandwidth Measurement //
            /////////////////////////////////////////////
            // Get max mem. bandwidth for uncore_freq. There may be uncore
            // frequencies for which an exact match doesn't exist. To handle
            // this case, we grab the entry prior to upper_bound() (as long as
            // it's not the first entry), in other words, the last entry that
            // is <= uncore_freq.
            auto qm_max_itr = m_qm_max_rate.upper_bound(uncore_freq);
            if (qm_max_itr != m_qm_max_rate.begin())
                --qm_max_itr;

            double scalability_uncore = 1.0;

            // Handle divided by zero, either numerator or
            // denominator being NAN
            if (!std::isnan(m_qm_rate.at(domain_idx).value) &&
                !std::isnan(qm_max_itr->second) &&
                qm_max_itr->second != 0) {
                scalability_uncore = (double) m_qm_rate.at(domain_idx).value /
                                         qm_max_itr->second;
            }

            // L3 usage, Network Traffic, HBM, and PCIE (GPUs) all use the uncore.
            // Eventually all these components should be considered when scaling
            // the uncore frequency in the efficient - performant range.
            // A more robust/future proof solution may be to directly query uncore
            // counters that indicate utilization (when/if available).
            // For now only L3 bandwidth metric is used.
            double uncore_req = m_resolved_f_uncore_efficient + f_uncore_range * scalability_uncore;

            // Clamp uncore request within policy limits
            if (uncore_req > m_resolved_f_uncore_max || uncore_req < m_resolved_f_uncore_efficient) {
                ++m_uncore_frequency_clamped;
            }
            uncore_req = std::max(m_resolved_f_uncore_efficient, uncore_req);
            uncore_req = std::min(m_resolved_f_uncore_max, uncore_req);
            uncore_freq_request.push_back(uncore_req);
        }

        // Per core freq
        std::vector<double> core_freq_request;

        for (int domain_idx = 0; domain_idx < m_num_freq_ctl_domain; ++domain_idx) {
            //////////////////////////////////
            // Core Scalability Measurement //
            //////////////////////////////////
            double scalability = (double) m_core_scal.at(domain_idx).value;
            if (std::isnan(scalability)) {
                scalability = 1.0;
            }

            double core_req = m_resolved_f_core_efficient + f_core_range * scalability;

            core_freq_request.push_back(core_req);
        }

        m_freq_governor->adjust_platform(core_freq_request);
        // Track number of core requests
        if (m_freq_governor->do_write_batch()) {
            ++m_core_batch_writes;
        }

        // Set per package controls
        for (int domain_idx = 0; domain_idx < M_NUM_PACKAGE; ++domain_idx) {
            if (std::isnan(uncore_freq_request.at(domain_idx))) {
                uncore_freq_request.at(domain_idx) = m_freq_uncore_max;
            }

            if (uncore_freq_request.at(domain_idx) !=
                m_uncore_freq_max_control.at(domain_idx).last_setting) {
                // Adjust
                m_platform_io.adjust(m_uncore_freq_max_control.at(domain_idx).batch_idx,
                                    uncore_freq_request.at(domain_idx));

                // Save the value for future comparison
                m_uncore_freq_max_control.at(domain_idx).last_setting = uncore_freq_request.at(domain_idx);
                ++m_uncore_frequency_requests;

                do_write_batch = true;
            }
        }
        return do_write_batch;
    }

    bool NodeActivityAgent::gpu_adjust_platform(const std::vector<double>& in_policy) {
        GEOPM_DEBUG_ASSERT(in_policy.size() == M_NUM_POLICY,
                           "NodeActivityAgent::adjust_platform(): policy vector incorrectly sized");

        m_do_send_policy = false;
        bool do_write_batch = false;

        // Per GPU freq
        std::vector<double> gpu_freq_request;
        std::vector<double> gpu_scoped_core_activity;

        double f_gpu_range = m_freq_gpu_max - m_freq_gpu_efficient;
        double phi = in_policy[M_POLICY_PHI];

        // Default phi = 0.5 case is full fe to fmax range
        // Core
        m_resolved_f_gpu_max = m_freq_gpu_max;
        m_resolved_f_gpu_efficient = m_freq_gpu_efficient;

        // If phi is not 0.5 we move into the energy or performance biased behavior
        if (phi > 0.5) {
            // Energy Biased.  Scale F_max down to F_efficient based upon phi value
            // Active region phi usage
            m_resolved_f_gpu_max = std::max(m_freq_gpu_efficient, m_freq_gpu_max -
                                                                  f_gpu_range *
                                                                  (phi-0.5) / 0.5);
        }
        else if (phi < 0.5) {
            // Perf Biased.  Scale F_efficient up to F_max based upon phi value
            // Active region phi usage
            m_resolved_f_gpu_efficient = std::min(m_freq_gpu_max, m_freq_gpu_efficient +
                                                                  f_gpu_range *
                                                                  (0.5-phi) / 0.5);
        }

        // Values after phi has been applied
        m_f_range = m_resolved_f_gpu_max - m_resolved_f_gpu_efficient;

        // Per GPU Frequency Selection
        for (int domain_idx = 0; domain_idx < m_agent_domain_count; ++domain_idx) {
            // gpu Compute Activity - Primary signal used for frequency recommendation
            double gpu_core_activity = m_gpu_core_activity.at(domain_idx).value;
            // gpu Utilization - Used to scale activity for short GPU phases
            double gpu_utilization = m_gpu_utilization.at(domain_idx).value;

            // Default to F_max
            double f_request = m_resolved_f_gpu_max;

            if (!std::isnan(gpu_core_activity)) {
                // Boundary Checking
                gpu_core_activity = std::min(gpu_core_activity, 1.0);

                // Frequency selection is based upon the gpu compute activity.
                // For active regions this means that we scale with the amount of work
                // being done (such as SM_ACTIVE for NVIDIA GPUs).
                //
                // The compute activity is scaled by the GPU Utilization, to help
                // address the issues that come from workloads that have short phases that are
                // frequency sensitive.  If a workload has a compute activity of 0.5, and
                // is resident on the GPU for 50% of cycles (0.5) it is treated as having
                // a 1.0 compute activity value
                //
                // For inactive regions the frequency selection is simply the efficient
                // frequency from system characterization.
                //
                // This approach assumes the efficient frequency is suitable as both a
                // baseline for active regions and inactive regions. This is generally
                // true when the efficient frequency consumes low power at idle due to clock
                // gating or other hardware PM techniques.
                //
                // If f_efficient does not meet these criteria this behavior can still be
                // achieved through tracking the GPU Utilization signal and setting frequency
                // to a separate idle value (f_idle) during regions where GPU Utilization is
                // zero (or below some bar).
                if (!std::isnan(gpu_utilization) &&
                    gpu_utilization > 0) {
                    gpu_utilization = std::min(gpu_utilization, 1.0);
                    f_request = m_resolved_f_gpu_efficient + m_f_range * (gpu_core_activity / gpu_utilization);
                }
                else {
                    f_request = m_resolved_f_gpu_efficient + m_f_range * gpu_core_activity;
                }

                // We're using the activity of the first
                // GPU_CHIP per GPU as a rough estimate of total GPU activity
                // for tracking the workload region of interest later on.
                // This is non-ideal, but is intended to be a temporary
                // solution to the lack of GPU region support and may be
                // removed when that support is added to GEOPM.
                if (domain_idx % (M_NUM_CHIP_PER_GPU) == 0) {
                    gpu_scoped_core_activity.push_back(gpu_core_activity);
                }
            }

            // Frequency clamping
            if (f_request > m_resolved_f_gpu_max || f_request < m_resolved_f_gpu_efficient) {
                ++m_gpu_frequency_clipped;
            }
            f_request = std::min(f_request, m_resolved_f_gpu_max);
            f_request = std::max(f_request, m_resolved_f_gpu_efficient);

            if (phi >= 0.5) {
                if (!std::isnan(gpu_utilization) &&
                    gpu_utilization == 0) {
                    if (m_gpu_idle_timer.at(domain_idx) > 0) {
                        m_gpu_idle_timer.at(domain_idx) = m_gpu_idle_timer.at(domain_idx) - 1;
                    }
                }
                else {
                    m_gpu_idle_timer.at(domain_idx) = 10;
                }

                if (m_gpu_idle_timer.at(domain_idx) <= 0) {
                    f_request = m_freq_gpu_min;
                    m_gpu_idle_samples.at(domain_idx) = m_gpu_idle_samples.at(domain_idx) + 1;
                }
            }

            // Store frequency request
            gpu_freq_request.push_back(f_request);
        }

        // Tracking logic.  This is not needed for any performance reason,
        // but does provide useful metrics for tracking agent behavior.  This
        // may be removed when GPU regions are added to GEOPM.
        if (!gpu_scoped_core_activity.empty()) {
            for (int domain_idx = 0; domain_idx < M_NUM_GPU; ++domain_idx) {
                if (gpu_scoped_core_activity.at(domain_idx) >= M_GPU_ACTIVITY_CUTOFF) {
                    // ROI proxy tracking
                    m_gpu_active_region_stop.at(domain_idx) = 0;
                    if (m_gpu_active_region_start.at(domain_idx) == 0) {
                        m_gpu_active_region_start.at(domain_idx) = m_time.value;
                        m_gpu_active_energy_start.at(domain_idx) = m_gpu_energy.at(domain_idx).value;
                        m_cpu_active_energy_start = m_cpu_energy.value;
                    }

                    m_gpu_on_time.at(domain_idx) += m_time.value - m_prev_time;
                    m_gpu_on_energy.at(domain_idx) += m_gpu_energy.at(domain_idx).value - m_gpu_prev_energy.at(domain_idx);
                    if (domain_idx == M_NUM_GPU-1) { //pick a GPU, I've picked the last
                        m_cpu_on_energy += m_cpu_energy.value - m_cpu_prev_energy;
                    }
                }
                else {
                    // ROI proxy tracking
                    if (m_gpu_active_region_stop.at(domain_idx) == 0) {
                        m_gpu_active_region_stop.at(domain_idx) = m_time.value;
                        m_gpu_active_energy_stop.at(domain_idx) = m_gpu_energy.at(domain_idx).value;
                        m_cpu_active_energy_stop = m_cpu_energy.value;
                    }
                }
            }
        }

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
                do_write_batch = true;
            }
        }
        return do_write_batch;
    }

    // If controls have a valid updated value write them.
    bool NodeActivityAgent::do_write_batch(void) const
    {
        return m_do_write_batch;
    }

    // Read signals from the platform and calculate samples to be sent up
    void NodeActivityAgent::sample_platform(std::vector<double> &out_sample)
    {
        GEOPM_DEBUG_ASSERT(out_sample.size() == M_NUM_SAMPLE,
                           "NodeActivityAgent::sample_platform(): sample output vector incorrectly sized");

        // Collect latest GPU values
        for (int domain_idx = 0; domain_idx < m_agent_domain_count; ++domain_idx) {
            m_gpu_core_activity.at(domain_idx).value = m_platform_io.sample(m_gpu_core_activity.at(
                                                                               domain_idx).batch_idx);
            m_gpu_utilization.at(domain_idx).value = m_platform_io.sample(m_gpu_utilization.at(
                                                                          domain_idx).batch_idx);
        }

        for (int domain_idx = 0; domain_idx < M_NUM_GPU; ++domain_idx) {
            m_gpu_prev_energy.at(domain_idx) = m_gpu_energy.at(domain_idx).value;
            m_gpu_energy.at(domain_idx).value = m_platform_io.sample(m_gpu_energy.at(
                                                                     domain_idx).batch_idx);
        }

        // Collect latest CPU values
        for (int domain_idx = 0; domain_idx < M_NUM_PACKAGE; ++domain_idx) {
            // Frequency signals
            m_uncore_freq_status.at(domain_idx).value = m_platform_io.sample(m_uncore_freq_status.at(domain_idx).batch_idx);

            // Uncore steering signals
            m_qm_rate.at(domain_idx).value = m_platform_io.sample(m_qm_rate.at(domain_idx).batch_idx);
        }

        for (int domain_idx = 0; domain_idx < m_num_freq_ctl_domain; ++domain_idx) {
            // Core steering signals
            m_core_scal.at(domain_idx).value = m_platform_io.sample(m_core_scal.at(domain_idx).batch_idx);
        }

        m_cpu_prev_energy = m_cpu_energy.value;
        m_cpu_energy.value = m_platform_io.sample(m_cpu_energy.batch_idx);

        // Collect time value
        m_prev_time = m_time.value;
        m_time.value = m_platform_io.sample(m_time.batch_idx);
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

        result.push_back({"Agent Domain", m_platform_topo.domain_type_to_name(m_agent_domain)});
        result.push_back({"GPU Frequency Requests", std::to_string(m_gpu_frequency_requests)});
        result.push_back({"GPU Clipped Frequency Requests", std::to_string(m_gpu_frequency_clipped)});
        result.push_back({"Resolved Max Frequency", std::to_string(m_resolved_f_gpu_max)});
        result.push_back({"Resolved Efficient Frequency", std::to_string(m_resolved_f_gpu_efficient)});
        result.push_back({"Resolved Frequency Range", std::to_string(m_f_range)});

        double total_gpu_roi_energy = 0;
        double total_gpu_on_energy = 0;

        for (int domain_idx = 0; domain_idx < M_NUM_GPU; ++domain_idx) {
            double energy_stop = m_gpu_active_energy_stop.at(domain_idx);
            double energy_start = m_gpu_active_energy_start.at(domain_idx);
            double region_stop = m_gpu_active_region_stop.at(domain_idx);
            double region_start =  m_gpu_active_region_start.at(domain_idx);
            result.push_back({"GPU " + std::to_string(domain_idx) +
                              " Active Region Energy", std::to_string(energy_stop - energy_start)});
            result.push_back({"GPU " + std::to_string(domain_idx) +
                              " Active Region Time", std::to_string(region_stop - region_start)});
            result.push_back({"GPU " + std::to_string(domain_idx) +
                              " On Energy", std::to_string(m_gpu_on_energy.at(domain_idx))});
            result.push_back({"GPU " + std::to_string(domain_idx) +
                              " On Time", std::to_string(m_gpu_on_time.at(domain_idx))});
            total_gpu_roi_energy += energy_stop - energy_start;
            total_gpu_on_energy += m_gpu_on_energy.at(domain_idx);
        }

        for (int domain_idx = 0; domain_idx < m_agent_domain_count; ++domain_idx) {
            result.push_back({"GPU Chip " + std::to_string(domain_idx) +
                              " Idle Agent Actions", std::to_string(m_gpu_idle_samples.at(domain_idx))});
        }

        result.push_back({"Total GPU Active Region Energy",
                          std::to_string(total_gpu_roi_energy)});
        result.push_back({"Total GPU On Energy",
                          std::to_string(total_gpu_on_energy)});

        result.push_back({"Core Batch Writes",
                          std::to_string(m_core_batch_writes)});
        result.push_back({"Core Frequency Requests Clamped",
                          std::to_string(m_freq_governor->get_clamp_count())});
        result.push_back({"Uncore Frequency Requests",
                          std::to_string(m_uncore_frequency_requests)});
        result.push_back({"Uncore Frequency Requests Clamped",
                          std::to_string(m_uncore_frequency_clamped)});
        result.push_back({"Resolved Maximum Core Frequency",
                          std::to_string(m_resolved_f_core_max)});
        result.push_back({"Resolved Efficient Core Frequency",
                          std::to_string(m_resolved_f_core_efficient)});
        result.push_back({"Resolved Core Frequency Range",
                          std::to_string(m_resolved_f_core_max - m_resolved_f_core_efficient)});
        result.push_back({"Resolved Maximum Uncore Frequency",
                          std::to_string(m_resolved_f_uncore_max)});
        result.push_back({"Resolved Efficient Uncore Frequency",
                          std::to_string(m_resolved_f_uncore_efficient)});
        result.push_back({"Resolved Uncore Frequency Range",
                          std::to_string(m_resolved_f_uncore_max - m_resolved_f_uncore_efficient)});

        result.push_back({"CPU Energy During GPU Active Region",
                          std::to_string(m_cpu_active_energy_stop - m_cpu_active_energy_start)});
        result.push_back({"CPU Energy During GPU On Time",
                          std::to_string(m_cpu_on_energy)});

        double node_roi_energy = total_gpu_roi_energy;
        double node_on_energy = total_gpu_on_energy;
        node_roi_energy += m_cpu_active_energy_stop - m_cpu_active_energy_start;
        node_on_energy += m_cpu_on_energy;
        result.push_back({"Node Energy During GPU Active Region",
                          std::to_string(node_roi_energy)});
        result.push_back({"Node Energy During GPU On Time",
                          std::to_string(node_on_energy)});
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
        return {"PHI"};
    }

    // Describes samples to be provided to the resource manager or user
    std::vector<std::string> NodeActivityAgent::sample_names(void)
    {
        return {};
    }
}
