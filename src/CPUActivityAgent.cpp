/*
 * Copyright (c) 2015 - 2022, Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "config.h"

#include "CPUActivityAgent.hpp"

#include <cmath>
#include <cassert>
#include <algorithm>
#include <iostream>
#include <string>

#include "geopm/Agg.hpp"
#include "geopm/Exception.hpp"
#include "geopm/Helper.hpp"
#include "geopm/PlatformIO.hpp"
#include "geopm/PlatformTopo.hpp"
#include "geopm/PluginFactory.hpp"
#include "geopm_debug.hpp"
#include "ActivityPerformanceModel.hpp"

#include "PlatformIOProf.hpp"

namespace geopm
{
    CPUActivityAgent::CPUActivityAgent()
        : CPUActivityAgent(platform_io(), platform_topo(),
                           cpu_activity_perf_model(),
                           uncore_activity_perf_model())
    {
    }

    CPUActivityAgent::CPUActivityAgent(PlatformIO &plat_io, const PlatformTopo &topo,
                                       ActivityPerformanceModel &cpu_perf_model,
                                       ActivityPerformanceModel &uncore_perf_model)
        : m_platform_io(plat_io)
        , m_platform_topo(topo)
        , m_cpu_perf_model(cpu_perf_model)
        , m_uncore_perf_model(uncore_perf_model)
        , m_last_wait{{0, 0}}
        , M_WAIT_SEC(0.010) // 10ms wait default
        , M_POLICY_PHI_DEFAULT(0.5)
        , M_NUM_PACKAGE(m_platform_topo.num_domain(GEOPM_DOMAIN_PACKAGE))
        , M_NUM_CORE(m_platform_topo.num_domain(GEOPM_DOMAIN_CORE))
        , m_do_write_batch(false)
        , m_do_send_policy(true)
        , m_core_frequency_requests(0)
        , m_uncore_frequency_requests(0)
        , m_resolved_f_uncore_efficient(0)
        , m_resolved_f_uncore_max(0)
        , m_resolved_f_core_efficient(0)
        , m_resolved_f_core_max(0)
    {
        geopm_time(&m_last_wait);
    }

    // Push signals and controls for future batch read/write
    void CPUActivityAgent::init(int level, const std::vector<int> &fan_in, bool is_level_root)
    {
        if (level == 0) {
            init_platform_io();
            m_cpu_perf_model.init();
            m_uncore_perf_model.init();
            if (!m_cpu_perf_model.algorithm_valid() &&
                !m_uncore_perf_model.algorithm_valid()) {
                throw Exception("CPUActivityAgent::" + std::string(__func__) +
                                "(): Neither performance model is valid",
                                GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }
        }
    }

    void CPUActivityAgent::init_platform_io(void)
    {
        //TODO: query perf model for controls and domains
        m_core_ctl_domain_map = m_cpu_perf_model.controls_recommended();
        m_uncore_ctl_domain_map = m_uncore_perf_model.controls_recommended();

        //TODO: track core and uncore domain
        for (auto sv : m_core_ctl_domain_map) {
            if (sv.first == "CPU_FREQUENCY_MAX_CONTROL" ) {
                for (int domain_idx = 0; domain_idx < m_platform_topo.num_domain(sv.second);
                     ++domain_idx) {
                    m_core_freq_max_control.push_back({m_platform_io.push_control(sv.first,
                                                                              sv.second,
                                                                              domain_idx), NAN});
                }
            }
        }

        for (auto sv : m_uncore_ctl_domain_map) {
            if (sv.first == "CPU_UNCORE_FREQUENCY_MIN_CONTROL" ) {
                for (int domain_idx = 0; domain_idx < m_platform_topo.num_domain(sv.second);
                     ++domain_idx) {
                    m_uncore_freq_min_control.push_back({m_platform_io.push_control(sv.first,
                                                                                    sv.second,
                                                                                    domain_idx), NAN});
                }
            }
            else if (sv.first == "CPU_UNCORE_FREQUENCY_MAX_CONTROL" ) {
                for (int domain_idx = 0; domain_idx < m_platform_topo.num_domain(sv.second);
                     ++domain_idx) {
                    m_uncore_freq_max_control.push_back({m_platform_io.push_control(sv.first,
                                                                                    sv.second,
                                                                                    domain_idx), NAN});
                }
            }
        }

        if (m_core_freq_max_control.size() == 0 ||
            m_uncore_freq_min_control.size() == 0 ||
            m_uncore_freq_max_control.size() == 0 ) {
                throw Exception("CPUActivityAgent::" + std::string(__func__) +
                                "(): Performance model did not provide Core or Uncore recommendations.",
                                GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
    }

    // Validate incoming policy and configure default policy requests.
    void CPUActivityAgent::validate_policy(std::vector<double> &in_policy) const
    {
        GEOPM_DEBUG_ASSERT(in_policy.size() == M_NUM_POLICY,
                           "CPUActivityAgent::" + std::string(__func__) +
                           "(): policy vector not correctly sized.  Expected  " +
                           std::to_string(M_NUM_POLICY) + ", actual: " +
                           std::to_string(in_policy.size()));

        std::vector<double> core_policy = {in_policy[M_POLICY_CPU_PHI],
                                           in_policy[M_POLICY_CPU_FREQ_MAX],
                                           in_policy[M_POLICY_CPU_FREQ_EFFICIENT]};
        m_cpu_perf_model.validate_policy(core_policy);

        //Is this needed?
        in_policy[M_POLICY_CPU_FREQ_MAX] = core_policy[M_POLICY_CPU_FREQ_MAX];
        in_policy[M_POLICY_CPU_FREQ_EFFICIENT] = core_policy[M_POLICY_CPU_FREQ_EFFICIENT];
        in_policy[M_POLICY_CPU_PHI] = core_policy[M_POLICY_CPU_PHI];

        std::vector<double> uncore_policy = {in_policy[M_POLICY_CPU_PHI],
                                             in_policy[M_POLICY_UNCORE_FREQ_MAX],
                                             in_policy[M_POLICY_UNCORE_FREQ_EFFICIENT],
                                             in_policy[M_POLICY_MAX_MEM_BW]
                                            };
        m_uncore_perf_model.validate_policy(uncore_policy);

        //Is this needed?
        in_policy[M_POLICY_CPU_FREQ_MAX] = uncore_policy[M_POLICY_UNCORE_FREQ_MAX];
        in_policy[M_POLICY_CPU_FREQ_EFFICIENT] = uncore_policy[M_POLICY_UNCORE_FREQ_EFFICIENT];
        in_policy[M_POLICY_MAX_MEM_BW] = uncore_policy[M_POLICY_MAX_MEM_BW];
        //TODO: one agent phi --> multiple model phis...need to resolve
        in_policy[M_POLICY_CPU_PHI] = uncore_policy[M_POLICY_CPU_PHI];
    }

    // Distribute incoming policy to children
    void CPUActivityAgent::split_policy(const std::vector<double>& in_policy,
                                        std::vector<std::vector<double> >& out_policy)
    {
        for (auto &child_pol : out_policy) {
            child_pol = in_policy;
        }
    }

    // Indicate whether to send the policy down to children
    bool CPUActivityAgent::do_send_policy(void) const
    {
        return m_do_send_policy;
    }

    void CPUActivityAgent::aggregate_sample(const std::vector<std::vector<double> > &in_sample,
                                            std::vector<double>& out_sample)
    {

    }

    // Indicate whether to send samples up to the parent
    bool CPUActivityAgent::do_send_sample(void) const
    {
        return false;
    }

    void CPUActivityAgent::adjust_platform(const std::vector<double>& in_policy)
    {
        m_do_send_policy = false;
        m_do_write_batch = false;

        // Per core freq
        std::vector<double> core_policy = {in_policy[M_POLICY_CPU_PHI],
                                           in_policy[M_POLICY_CPU_FREQ_MAX],
                                           in_policy[M_POLICY_CPU_FREQ_EFFICIENT]};

        m_resolved_f_core_max = in_policy[M_POLICY_CPU_FREQ_MAX];
        m_resolved_f_core_efficient = in_policy[M_POLICY_CPU_FREQ_EFFICIENT];

        m_cpu_perf_model.set_policy(core_policy);
        m_cpu_perf_model.update_recommendation();

        std::vector<double> core_freq_request = m_cpu_perf_model.sample_recommendation("CPU_FREQUENCY_STATUS_MAX_CONTROL");

        // Set per core controls
        //TODO: use the core and uncore domain we tracked earlier
        for (int domain_idx = 0; domain_idx < M_NUM_CORE; ++domain_idx) {
            if (std::isnan(core_freq_request.at(domain_idx))) {
                core_freq_request.at(domain_idx) = in_policy[M_POLICY_CPU_FREQ_MAX];
            }
            if (core_freq_request.at(domain_idx) !=
                m_core_freq_max_control.at(domain_idx).last_setting) {
                // Adjust
                m_platform_io.adjust(m_core_freq_max_control.at(domain_idx).batch_idx,
                                     core_freq_request.at(domain_idx));

                // Save the value for future comparison
                m_core_freq_max_control.at(domain_idx).last_setting = core_freq_request.at(domain_idx);
                ++m_core_frequency_requests;
                m_do_write_batch = true;
            }
        }

        std::vector<double> uncore_policy = {in_policy[M_POLICY_CPU_PHI],
                                             in_policy[M_POLICY_UNCORE_FREQ_MAX],
                                             in_policy[M_POLICY_UNCORE_FREQ_EFFICIENT],
                                             in_policy[M_POLICY_MAX_MEM_BW]};

        m_resolved_f_uncore_max = in_policy[M_POLICY_UNCORE_FREQ_MAX];
        m_resolved_f_uncore_efficient = in_policy[M_POLICY_UNCORE_FREQ_EFFICIENT];

        m_uncore_perf_model.set_policy(uncore_policy);
        m_uncore_perf_model.update_recommendation();

        std::vector<double> uncore_freq_min_request = m_uncore_perf_model.sample_recommendation("CPU_UNCORE_FREQUENCY_MIN_CONTROL");
        std::vector<double> uncore_freq_max_request = m_uncore_perf_model.sample_recommendation("CPU_UNCORE_FREQUENCY_MAX_CONTROL");

        //TODO: use the core and uncore domain we tracked earlier
                // Set per package controls
        for (int domain_idx = 0; domain_idx < M_NUM_PACKAGE; ++domain_idx) {
            if (std::isnan(uncore_freq_min_request.at(domain_idx))) {
                uncore_freq_min_request.at(domain_idx) = in_policy[M_POLICY_UNCORE_FREQ_EFFICIENT];
            }
            if (std::isnan(uncore_freq_max_request.at(domain_idx))) {
                uncore_freq_max_request.at(domain_idx) = in_policy[M_POLICY_UNCORE_FREQ_MAX];
            }

            if (uncore_freq_min_request.at(domain_idx) !=
                m_uncore_freq_min_control.at(domain_idx).last_setting ||
                uncore_freq_max_request.at(domain_idx) !=
                m_uncore_freq_max_control.at(domain_idx).last_setting) {
                // Adjust
                m_platform_io.adjust(m_uncore_freq_min_control.at(domain_idx).batch_idx,
                                    uncore_freq_min_request.at(domain_idx));

                m_platform_io.adjust(m_uncore_freq_max_control.at(domain_idx).batch_idx,
                                    uncore_freq_max_request.at(domain_idx));

                // Save the value for future comparison
                m_uncore_freq_min_control.at(domain_idx).last_setting = uncore_freq_min_request.at(domain_idx);
                m_uncore_freq_max_control.at(domain_idx).last_setting = uncore_freq_max_request.at(domain_idx);
                ++m_uncore_frequency_requests;

                m_do_write_batch = true;
            }
        }
    }

    // If controls have a valid updated value write them.
    bool CPUActivityAgent::do_write_batch(void) const
    {
        return m_do_write_batch;
    }

    // Read signals from the platform and calculate samples to be sent up
    void CPUActivityAgent::sample_platform(std::vector<double> &out_sample)
    {
        GEOPM_DEBUG_ASSERT(out_sample.size() == M_NUM_SAMPLE,
                           "CPUActivityAgent::" + std::string(__func__) +
                           "(): sample vector not correctly sized.  Expected  " +
                           std::to_string(M_NUM_SAMPLE) + ", actual: " +
                           std::to_string(out_sample.size()));
    }

    // Wait for the remaining cycle time to keep Controller loop cadence
    void CPUActivityAgent::wait(void)
    {
        geopm_time_s current_time;
        do {
            geopm_time(&current_time);
        }
        while(geopm_time_diff(&m_last_wait, &current_time) < M_WAIT_SEC);
        geopm_time(&m_last_wait);
    }

    // Adds the wait time to the top of the report
    std::vector<std::pair<std::string, std::string> > CPUActivityAgent::report_header(void) const
    {
        return {{"Wait time (sec)", std::to_string(M_WAIT_SEC)}};
    }

    // Adds number of frquency requests to the per-node section of the report
    std::vector<std::pair<std::string, std::string> > CPUActivityAgent::report_host(void) const
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
        return result;
    }

    // This Agent does not add any per-region details
    std::map<uint64_t, std::vector<std::pair<std::string, std::string> > > CPUActivityAgent::report_region(void) const
    {
        return {};
    }

    // Adds trace columns signals of interest
    std::vector<std::string> CPUActivityAgent::trace_names(void) const
    {
        return {};
    }

    // Updates the trace with values for signals from this Agent
    void CPUActivityAgent::trace_values(std::vector<double> &values)
    {
    }

    void CPUActivityAgent::enforce_policy(const std::vector<double> &policy) const
    {

    }

    std::vector<std::function<std::string(double)> > CPUActivityAgent::trace_formats(void) const
    {
        return {};
    }

    // Name used for registration with the Agent factory
    std::string CPUActivityAgent::plugin_name(void)
    {
        return "cpu_activity";
    }

    // Used by the factory to create objects of this type
    std::unique_ptr<Agent> CPUActivityAgent::make_plugin(void)
    {
        return geopm::make_unique<CPUActivityAgent>();
    }

    // Describes expected policies to be provided by the resource manager or user
    std::vector<std::string> CPUActivityAgent::policy_names(void)
    {
        std::vector<std::string> names{"CPU_FREQ_MAX", "CPU_FREQ_EFFICIENT", "CPU_UNCORE_FREQ_MAX",
                                       "CPU_UNCORE_FREQ_EFFICIENT", "CPU_PHI", "MAX_MEMORY_BANDWIDTH"};
        return names;
    }

    // Describes samples to be provided to the resource manager or user
    std::vector<std::string> CPUActivityAgent::sample_names(void)
    {
        return {};
    }
}
