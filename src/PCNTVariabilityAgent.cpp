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

#include "PCNTVariabilityAgent.hpp"

#include <cmath>
#include <cassert>
#include <algorithm>

#include "PluginFactory.hpp"
#include "PlatformIO.hpp"
#include "PlatformTopo.hpp"
#include "Helper.hpp"
#include "Agg.hpp"

#include <string>

#include <iostream>

namespace geopm
{
    PCNTVariabilityAgent::PCNTVariabilityAgent()
        : m_platform_io(platform_io())
        , m_platform_topo(platform_topo())
        , m_last_wait{{0, 0}}
        , M_WAIT_SEC(5.000)
        , m_do_write_batch(false)
        // This agent approach is meant to allow for quick prototyping through simplifying
        // signal & control addition and usage.  Most changes to signals and controls
        // should be accomplishable with changes to the declaration below (instead of updating
        // init_platform_io, sample_platform, etc).  Signal & control usage is still
        // handled in adjust_platform per usual.
        , m_signal_available({{"MSR::APERF:ACNT", {  // Name of signal to be queried
                                  GEOPM_DOMAIN_CORE, // Domain for the signal
                                  true,              // Should the signal appear in the trace
                                  {}                 // Empty Vector to contain the signal info
                                  }},
                              {"MSR::PPERF:PCNT", {
                                  GEOPM_DOMAIN_CORE,
                                  true,
                                  {}
                                  }},
                              {"POWER_PACKAGE", {
                                  GEOPM_DOMAIN_PACKAGE,
                                  true,
                                  {}
                                  }}
                             })
        , m_control_available({{"POWER_PACKAGE_LIMIT", {  // Name of contol to be queried
                                    GEOPM_DOMAIN_PACKAGE, // Domain for the control
                                    true,                 // Should the controls appear in the trace
                                    {}                    // Empty Vector to contain the control info
                                    }}
                              })
    {
        geopm_time(&m_last_wait);
    }

    // Push signals and controls for future batch read/write
    void PCNTVariabilityAgent::init(int level, const std::vector<int> &fan_in, bool is_level_root)
    {
        if (level == 0) {
            init_platform_io();
        }
    }

    void PCNTVariabilityAgent::init_platform_io(void)
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
    void PCNTVariabilityAgent::validate_policy(std::vector<double> &in_policy) const
    {
        assert(in_policy.size() == M_NUM_POLICY);
        double power_limit_max = m_platform_io.read_signal("POWER_PACKAGE_TDP", GEOPM_DOMAIN_PACKAGE, 0);

        //TODO: taken from example agent, moved to here...can we ever actually hit these beng NAN?
        // Check for NAN to set default values for policy
        if (std::isnan(in_policy[M_POLICY_TARGET_POWER])) {
            in_policy[M_POLICY_TARGET_POWER] = power_limit_max;
        }
    }

    // Distribute incoming policy to children
    void PCNTVariabilityAgent::split_policy(const std::vector<double>& in_policy,
                                    std::vector<std::vector<double> >& out_policy)
    {
        assert(in_policy.size() == M_NUM_POLICY);
        for (auto &child_pol : out_policy) {
            child_pol = in_policy;
        }
    }

    // Indicate whether to send the policy down to children
    bool PCNTVariabilityAgent::do_send_policy(void) const
    {
        return true;
    }

    void PCNTVariabilityAgent::aggregate_sample(const std::vector<std::vector<double> > &in_sample,
                                        std::vector<double>& out_sample)
    {

    }

    // Indicate whether to send samples up to the parent
    bool PCNTVariabilityAgent::do_send_sample(void) const
    {
        return false;
    }

    void PCNTVariabilityAgent::adjust_platform(const std::vector<double>& in_policy)
    {
        assert(in_policy.size() == M_NUM_POLICY);

        // Policy based recommended power per package
        std::vector<double> scalability_vec;
        std::vector<double> power_request;

        m_do_write_batch = false;

        // Build frequency recommendation based on accelerator utilization
        auto pcnt_itr = m_signal_available.find("MSR::PPERF:PCNT");
        auto acnt_itr = m_signal_available.find("MSR::APERF:ACNT");

        for (int domain_idx = 0; domain_idx < pcnt_itr->second.signals.size(); ++domain_idx) {
            double scalability = pcnt_itr->second.signals.at(domain_idx).m_last_sample / acnt_itr->second.signals.at(domain_idx).m_last_sample;
            if (!std::isnan(scalability)) {
                //TODO: this is hardcoded for some specific workloads and systems to ignore the OS and GEOPM cores
                //      It should be passed in via policy, geopm launch, or some other mechanism
                if ((domain_idx > 1) && (domain_idx < 22)) {
                    scalability_vec.at(0) += scalability;
                }
                else if ((domain_idx > 23) && (domain_idx < 44)) {
                    scalability_vec.at(1) += scalability;
                }
            }
        }

        if (!scalability_vec.empty()) {

            double delta_s;
            double power_limit = in_policy[M_POLICY_TARGET_POWER];
            scalability_vec.at(0) = scalability_vec.at(0)/20;
            scalability_vec.at(1) = scalability_vec.at(1)/20;

            if (scalability_vec.at(0) > scalability_vec.at(1)) {
                // Package 0 is at a higher average scalability than Package 1.
                // Decrease Package 0 power_limit
                delta_s = (scalability_vec.at(0) - scalability_vec.at(1))/scalability_vec.at(0);
                power_limit = power_limit - power_limit*delta_s;

                //Package 0
                power_request.push_back(power_limit);
                //Package 1
                power_request.push_back(in_policy[M_POLICY_TARGET_POWER]);
            }
            else if (scalability_vec.at(0) < scalability_vec.at(1)) {
                // Package 1 is at a higher average scalability than Package 0.
                // Decrease Package 1 power_limit
                delta_s = (scalability_vec.at(1) - scalability_vec.at(0))/scalability_vec.at(1);
                power_limit = power_limit - power_limit*delta_s;

                //Package 0
                power_request.push_back(in_policy[M_POLICY_TARGET_POWER]);
                //Package 1
                power_request.push_back(power_limit - power_limit*delta_s);
            }
        } else {
            power_request.push_back(in_policy[M_POLICY_TARGET_POWER]);
            power_request.push_back(in_policy[M_POLICY_TARGET_POWER]);
        }

        if (!power_request.empty()) {
            // set Xeon frequency control per core
            auto power_ctl_itr = m_control_available.find("POWER_PACKAGE_LIMIT");
            for (int domain_idx = 0; domain_idx < power_ctl_itr->second.controls.size(); ++domain_idx) {
                if (power_request.at(domain_idx) != power_ctl_itr->second.controls.at(domain_idx).m_last_setting) {
                    m_platform_io.adjust(power_ctl_itr->second.controls.at(domain_idx).m_batch_idx, power_request.at(domain_idx));
                    power_ctl_itr->second.controls.at(domain_idx).m_last_setting = power_request.at(domain_idx);
                    ++m_frequency_requests;
                }
            }
            m_do_write_batch = true;
        }
    }

    // If controls have a valid updated value write them.
    bool PCNTVariabilityAgent::do_write_batch(void) const
    {
        return m_do_write_batch;
    }

    // Read signals from the platform and calculate samples to be sent up
    void PCNTVariabilityAgent::sample_platform(std::vector<double> &out_sample)
    {
        assert(out_sample.size() == M_NUM_SAMPLE);

        // Collect latest signal values
        for (auto &sv : m_signal_available) {
            for (int domain_idx = 0; domain_idx < sv.second.signals.size(); ++domain_idx) {
                double curr_value = m_platform_io.sample(sv.second.signals.at(domain_idx).m_batch_idx);

                if (sv.first == "MSR::PPERF:PCNT" ||
                    sv.first == "MSR::APERF:ACNT" ) {
                    sv.second.signals.at(domain_idx).m_last_sample = curr_value - sv.second.signals.at(domain_idx).m_last_signal;
                }
                else {
                    sv.second.signals.at(domain_idx).m_last_sample = sv.second.signals.at(domain_idx).m_last_signal;
                }

                sv.second.signals.at(domain_idx).m_last_signal = curr_value;
            }
        }
    }

    // Wait for the remaining cycle time to keep Controller loop cadence
    void PCNTVariabilityAgent::wait(void)
    {
        geopm_time_s current_time;
        do {
            geopm_time(&current_time);
        }
        while(geopm_time_diff(&m_last_wait, &current_time) < M_WAIT_SEC);
        geopm_time(&m_last_wait);
    }

    // Adds the wait time to the top of the report
    std::vector<std::pair<std::string, std::string> > PCNTVariabilityAgent::report_header(void) const
    {
        return {{"Wait time (sec)", std::to_string(M_WAIT_SEC)}};
    }

    // Adds number of frquency requests to the per-node section of the report
    std::vector<std::pair<std::string, std::string> > PCNTVariabilityAgent::report_host(void) const
    {
        return {
            {"PCNT Based Frequency Requests", std::to_string(m_frequency_requests)}
        };
    }

    // This Agent does not add any per-region details
    std::map<uint64_t, std::vector<std::pair<std::string, std::string> > > PCNTVariabilityAgent::report_region(void) const
    {
        return {};
    }

    // Adds trace columns samples and signals of interest
    std::vector<std::string> PCNTVariabilityAgent::trace_names(void) const
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
    void PCNTVariabilityAgent::trace_values(std::vector<double> &values)
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

    std::vector<std::function<std::string(double)> > PCNTVariabilityAgent::trace_formats(void) const
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
    std::string PCNTVariabilityAgent::plugin_name(void)
    {
        return "pcnt_variability";
    }

    // Used by the factory to create objects of this type
    std::unique_ptr<Agent> PCNTVariabilityAgent::make_plugin(void)
    {
        return geopm::make_unique<PCNTVariabilityAgent>();
    }

    // Describes expected policies to be provided by the resource manager or user
    std::vector<std::string> PCNTVariabilityAgent::policy_names(void)
    {
        return {"TARGET_POWER"};
    }

    // Describes samples to be provided to the resource manager or user
    std::vector<std::string> PCNTVariabilityAgent::sample_names(void)
    {
        return {};
    }
}
