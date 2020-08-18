/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "NVMLUtilizationAgent.hpp"

#include <cmath>
#include <cassert>
#include <algorithm>

#include "PluginFactory.hpp"
#include "PlatformIO.hpp"
#include "PlatformTopo.hpp"
#include "Helper.hpp"
#include "Agg.hpp"

#include <string>

namespace geopm
{
    NVMLUtilizationAgent::NVMLUtilizationAgent()
        : m_platform_io(platform_io())
        , m_platform_topo(platform_topo())
        , m_last_wait{{0, 0}}
        , M_WAIT_SEC(0.05) // 50mS wait
        , m_do_write_batch(false)
        // This agent approach is meant to allow for quick prototyping through simplifying
        // signal & control addition and usage.  Most changes to signals and controls
        // should be accomplishable with changes to the declaration below (instead of updating
        // init_platform_io, sample_platform, etc).  Signal & control usage is still
        // handled in adjust_platform per usual.
        , m_signal_available({{"NVML::FREQUENCY", {               // Name of signal to be queried
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR, // Domain for the signal
                                  true,                           // Should the signal appear in the trace
                                  {}                              // Empty Vector to contain the signal info
                                  }},
                              {"NVML::UTILIZATION_ACCELERATOR", {
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  true,
                                  {}
                                  }},
                              {"NVML::POWER", {
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  true,
                                  {}
                                  }},
                              {"NVML::TOTAL_ENERGY_CONSUMPTION", {
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  true,
                                  {}
                                  }},
                              {"NVML::CPU_ACCELERATOR_ACTIVE_AFFINITIZATION", {
                                  GEOPM_DOMAIN_CPU,
                                  true,
                                  {}
                                  }},
                              {"FREQUENCY", {
                                  GEOPM_DOMAIN_CPU,
                                  true,
                                  {}
                                  }}
                             })
        , m_control_available({{"NVML::FREQUENCY_CONTROL", {                // Name of contol to be queried
                                    GEOPM_DOMAIN_BOARD_ACCELERATOR, // Domain for the control
                                    false,                          // Should the controls appear in the trace
                                    {}                              // Empty Vector to contain the control info
                                    }},
                               {"FREQUENCY", {
                                    GEOPM_DOMAIN_CORE,
                                    false,
                                    {}
                                    }}
                              })
    {
        geopm_time(&m_last_wait);
    }

    // Push signals and controls for future batch read/write
    void NVMLUtilizationAgent::init(int level, const std::vector<int> &fan_in, bool is_level_root)
    {
        if (level == 0) {
            init_platform_io();
        }
    }

    void NVMLUtilizationAgent::init_platform_io(void)
    {
        // populate signals for each domain with batch idx info, default values, etc
        for (auto &sv : m_signal_available) {
            for (int domain_idx = 0; domain_idx < m_platform_topo.num_domain(sv.second.domain); ++domain_idx) {
                signal sgnl = signal{m_platform_io.push_signal(sv.first,
                                                               sv.second.domain,
                                                               domain_idx), NAN, NAN};
                sv.second.signals.push_back(sgnl);
            }
        }

        // populate controls for each domain
        for (auto &sv : m_control_available) {
            for (int domain_idx = 0; domain_idx < m_platform_topo.num_domain(sv.second.domain); ++domain_idx) {
                control ctrl = control{m_platform_io.push_control(sv.first,
                                                                  sv.second.domain,
                                                                  domain_idx), NAN};
                sv.second.controls.push_back(ctrl);
            }
        }
    }

    // Validate incoming policy and configure default policy requests.
    void NVMLUtilizationAgent::validate_policy(std::vector<double> &in_policy) const
    {
        assert(in_policy.size() == M_NUM_POLICY);

        //TODO: taken from example agent, moved to here...can we ever actually hit these beng NAN?
        // Check for NAN to set default values for policy
        if (std::isnan(in_policy[M_POLICY_ACCELERATOR_FREQ_LOW])) {
            in_policy[M_POLICY_ACCELERATOR_FREQ_LOW] =
                m_platform_io.read_signal("NVML::FREQUENCY_MIN", GEOPM_DOMAIN_BOARD, 0);
        }
        if (std::isnan(in_policy[M_POLICY_ACCELERATOR_FREQ_HIGH])) {
            in_policy[M_POLICY_ACCELERATOR_FREQ_HIGH] =
                m_platform_io.read_signal("NVML::FREQUENCY_MAX", GEOPM_DOMAIN_BOARD, 0);
        }
        if (std::isnan(in_policy[M_POLICY_XEON_FREQ_HIGH])) {
            in_policy[M_POLICY_XEON_FREQ_HIGH] =
                m_platform_io.read_signal("CPU_FREQUENCY_MAX", GEOPM_DOMAIN_BOARD, 0);
        }
        if (std::isnan(in_policy[M_POLICY_XEON_FREQ_LOW])) {
            in_policy[M_POLICY_XEON_FREQ_LOW] =
                m_platform_io.read_signal("CPU_FREQUENCY_STICKER", GEOPM_DOMAIN_BOARD, 0);
        }
    }

    // Distribute incoming policy to children
    void NVMLUtilizationAgent::split_policy(const std::vector<double>& in_policy,
                                    std::vector<std::vector<double> >& out_policy)
    {
        assert(in_policy.size() == M_NUM_POLICY);
        for (auto &child_pol : out_policy) {
            child_pol = in_policy;
        }
    }

    // Indicate whether to send the policy down to children
    bool NVMLUtilizationAgent::do_send_policy(void) const
    {
        return true;
    }

    void NVMLUtilizationAgent::aggregate_sample(const std::vector<std::vector<double> > &in_sample,
                                        std::vector<double>& out_sample)
    {

    }

    // Indicate whether to send samples up to the parent
    bool NVMLUtilizationAgent::do_send_sample(void) const
    {
        return false;
    }

    void NVMLUtilizationAgent::adjust_platform(const std::vector<double>& in_policy)
    {
        assert(in_policy.size() == M_NUM_POLICY);

        // Policy based reccomended frequency for each accelerator
        std::vector<double> reccomended_accel_freq;
        // Policy based reccomended frequency for cpus associated with
        // each accelerator
        std::vector<double> recommended_xeon_freq;
        // Policy based reccomended frequency per cpu based upon
        // CPU to Accelerator affinitization
        std::vector<double> cpu_freq_request;

        m_do_write_batch = false;
        bool core_req_change = false;
        bool accel_req_change = false;

        // Build frequency recommendation based on accelerator utilization
        auto sig_itr = m_signal_available.find("NVML::UTILIZATION_ACCELERATOR");
        for (int domain_idx = 0; domain_idx < sig_itr->second.signals.size(); ++domain_idx) {
            double utilization_accelerator = sig_itr->second.signals.at(domain_idx).m_last_signal;

            if (!std::isnan(utilization_accelerator)) {
                if (utilization_accelerator == 0) {
                    // All accelerators are idle
                    reccomended_accel_freq.push_back(in_policy[M_POLICY_ACCELERATOR_FREQ_LOW]);
                    recommended_xeon_freq.push_back(in_policy[M_POLICY_XEON_FREQ_HIGH]);
                } else if (utilization_accelerator == 1.0) {
                    // All accelerators are active
                    reccomended_accel_freq.push_back(in_policy[M_POLICY_ACCELERATOR_FREQ_HIGH]);
                    recommended_xeon_freq.push_back(in_policy[M_POLICY_XEON_FREQ_LOW]);
                } else {
                    // Any accelerator is active
                    // The performance degradation averse approach is to act as if
                    // the accelerators are critical to perf (high util accelerator freq)
                    // and the xeons are critical to perf (low util xeon freq)
                    reccomended_accel_freq.push_back(in_policy[M_POLICY_ACCELERATOR_FREQ_HIGH]);
                    recommended_xeon_freq.push_back(in_policy[M_POLICY_XEON_FREQ_HIGH]);
                }
            }
        }

        sig_itr = m_signal_available.find("NVML::CPU_ACCELERATOR_ACTIVE_AFFINITIZATION");
        for (int domain_idx = 0; domain_idx < sig_itr->second.signals.size(); ++domain_idx) {
            double gpu_affinity = sig_itr->second.signals.at(domain_idx).m_last_signal;

            if (std::isnan(gpu_affinity) || gpu_affinity == -1) {
                // This means we're not associated with any GPU or are associated with multiple GPUs.
                // The safe bet with respect to perforamnce in this case is is to assume the CPU is
                // doing work and set it to a high frequency
                cpu_freq_request.push_back(in_policy[M_POLICY_XEON_FREQ_HIGH]);
            }
            else {
                // The CPU frequency request should be based on which accelerator is is
                // associated with.
                cpu_freq_request.push_back(recommended_xeon_freq.at(gpu_affinity));
            }

        }

        // Check to make sure accel freq and recommended xeon freq are not empty
        // since write batch is set here, and all signals must be adjusted
        if (!reccomended_accel_freq.empty() && !recommended_xeon_freq.empty()) {
            // set Xeon frequency control per core
            auto freq_ctl_itr = m_control_available.find("FREQUENCY");
            for (int domain_idx = 0; domain_idx < freq_ctl_itr->second.controls.size(); ++domain_idx) {
                if (cpu_freq_request.at(domain_idx) != freq_ctl_itr->second.controls.at(domain_idx).m_last_setting) {
                    m_platform_io.adjust(freq_ctl_itr->second.controls.at(domain_idx).m_batch_idx, cpu_freq_request.at(domain_idx));
                    freq_ctl_itr->second.controls.at(domain_idx).m_last_setting = cpu_freq_request.at(domain_idx);
                    core_req_change = true;
                }
            }

            freq_ctl_itr = m_control_available.find("NVML::FREQUENCY");
            for (int domain_idx = 0; domain_idx < freq_ctl_itr->second.controls.size(); ++domain_idx) {
                // set accelerator frequency.
                if (reccomended_accel_freq.at(domain_idx) != freq_ctl_itr->second.controls.at(domain_idx).m_last_setting) {
                    m_platform_io.adjust(freq_ctl_itr->second.controls.at(domain_idx).m_batch_idx, reccomended_accel_freq.at(domain_idx));
                    freq_ctl_itr->second.controls.at(domain_idx).m_last_setting = reccomended_accel_freq.at(domain_idx);
                    ++m_accelerator_frequency_requests;
                    accel_req_change = true;
                }
            }

            if (core_req_change || accel_req_change) {
                m_do_write_batch = true;
            }
        }
    }

    // If controls have a valid updated value write them.
    bool NVMLUtilizationAgent::do_write_batch(void) const
    {
        return m_do_write_batch;
    }

    // Read signals from the platform and calculate samples to be sent up
    void NVMLUtilizationAgent::sample_platform(std::vector<double> &out_sample)
    {
        assert(out_sample.size() == M_NUM_SAMPLE);

        // Collect latest signal values
        for (auto &sv : m_signal_available) {
            for (int domain_idx = 0; domain_idx < sv.second.signals.size(); ++domain_idx) {
                sv.second.signals.at(domain_idx).m_last_signal = m_platform_io.sample(sv.second.signals.at(domain_idx).m_batch_idx);
                sv.second.signals.at(domain_idx).m_last_sample = sv.second.signals.at(domain_idx).m_last_signal;
            }
        }
    }

    // Wait for the remaining cycle time to keep Controller loop cadence
    void NVMLUtilizationAgent::wait(void)
    {
        geopm_time_s current_time;
        do {
            geopm_time(&current_time);
        }
        while(geopm_time_diff(&m_last_wait, &current_time) < M_WAIT_SEC);
        geopm_time(&m_last_wait);
    }

    // Adds the wait time to the top of the report
    std::vector<std::pair<std::string, std::string> > NVMLUtilizationAgent::report_header(void) const
    {
        return {{"Wait time (sec)", std::to_string(M_WAIT_SEC)}};
    }

    // Adds number of accelerator frquency requests to the per-node section of the report
    std::vector<std::pair<std::string, std::string> > NVMLUtilizationAgent::report_host(void) const
    {
        return {
            {"Accelerator Frequency Requests", std::to_string(m_accelerator_frequency_requests)}
        };
    }

    // This Agent does not add any per-region details
    std::map<uint64_t, std::vector<std::pair<std::string, std::string> > > NVMLUtilizationAgent::report_region(void) const
    {
        return {};
    }

    // Adds trace columns samples and signals of interest
    std::vector<std::string> NVMLUtilizationAgent::trace_names(void) const
    {
        std::vector<std::string> names;

        // Signals
        // Automatically build name in the format: "NVML::FREQUENCY-board_accelerator-0"
        for (auto &sv : m_signal_available) {
            if (sv.second.trace_signal) {
                for (int domain_idx = 0; domain_idx < sv.second.signals.size(); ++domain_idx) {
                    names.push_back(sv.first + "-" + m_platform_topo.domain_type_to_name(sv.second.domain) + "-" + std::to_string(domain_idx));
                }
            }
        }
        // Controls
        // Automatically build name in the format: "CONTROL::NVML::FREQUENCY_CONTROL-board_accelerator-0"
        for (auto &sv : m_control_available) {
            if (sv.second.trace_control) {
                for (int domain_idx = 0; domain_idx < sv.second.controls.size(); ++domain_idx) {
                    names.push_back("CONTROL:" + sv.first + "-" + m_platform_topo.domain_type_to_name(sv.second.domain) + "-" + std::to_string(domain_idx));
                }
            }
        }

        return names;

    }

    // Updates the trace with values for samples and signals from this Agent
    void NVMLUtilizationAgent::trace_values(std::vector<double> &values)
    {
        int values_idx = 0;

        //default assumption is that every signal added should be in the trace
        for (auto &sv : m_signal_available) {
            if (sv.second.trace_signal) {
                for (int domain_idx = 0; domain_idx < sv.second.signals.size(); ++domain_idx) {
                    values[values_idx] = sv.second.signals.at(domain_idx).m_last_signal;
                    ++values_idx;
                }
            }
        }

        for (auto &sv : m_control_available) {
            if (sv.second.trace_control) {
                for (int domain_idx = 0; domain_idx < sv.second.controls.size(); ++domain_idx) {
                    values[values_idx] = sv.second.controls.at(domain_idx).m_last_setting;
                    ++values_idx;
                }
            }
        }
    }

    std::vector<std::function<std::string(double)> > NVMLUtilizationAgent::trace_formats(void) const
    {
        std::vector<std::function<std::string(double)>> trace_formats;
        for (auto &sv : m_signal_available) {
            if (sv.second.trace_signal) {
                for (int domain_idx = 0; domain_idx < sv.second.signals.size(); ++domain_idx) {
                    trace_formats.push_back(m_platform_io.format_function(sv.first));
                }
            }
        }

        for (auto &sv : m_control_available) {
            if (sv.second.trace_control) {
                for (int domain_idx = 0; domain_idx < sv.second.controls.size(); ++domain_idx) {
                    trace_formats.push_back(m_platform_io.format_function(sv.first));
                }
            }
        }

        return trace_formats;
    }

    // Name used for registration with the Agent factory
    std::string NVMLUtilizationAgent::plugin_name(void)
    {
        return "nvml_utilization";
    }

    // Used by the factory to create objects of this type
    std::unique_ptr<Agent> NVMLUtilizationAgent::make_plugin(void)
    {
        return geopm::make_unique<NVMLUtilizationAgent>();
    }

    // Describes expected policies to be provided by the resource manager or user
    std::vector<std::string> NVMLUtilizationAgent::policy_names(void)
    {
        return {"ACCELERATOR_FREQUENCY_HIGH", "ACCELERATOR_FREQUENCY_LOW",
                "XEON_FREQUENCY_HIGH", "XEON_FREQUENCY_LOW"};
    }

    // Describes samples to be provided to the resource manager or user
    std::vector<std::string> NVMLUtilizationAgent::sample_names(void)
    {
        return {};
    }
}
