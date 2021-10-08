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

#include "GPUUtilizationActivityAgent.hpp"

#include <cmath>
#include <cassert>
#include <algorithm>

#include "geopm/PluginFactory.hpp"
#include "geopm/PlatformIO.hpp"
#include "geopm/PlatformTopo.hpp"
#include "geopm/Helper.hpp"
#include "geopm/Agg.hpp"

#include <string>

#include <iostream>

//#define SAMPLE_PERIOD_SECONDS 0.025 // 25mS wait
//#define DECISION_WINDOW_SECONDS 0.100
//#define SAMPLE_PERIOD_SECONDS 0.015 // 15mS wait
#define SAMPLE_PERIOD_SECONDS 0.020 // 20mS wait
#define DECISION_WINDOW_SECONDS 0.100
#define DECISION_WINDOW_SAMPLES (DECISION_WINDOW_SECONDS / SAMPLE_PERIOD_SECONDS)
#define M_POLICY_ENERGY_PERF_BIAS_DEFAULT 50;

namespace geopm
{
    GPUUtilizationActivityAgent::GPUUtilizationActivityAgent()
        : m_platform_io(platform_io())
        , m_platform_topo(platform_topo())
        , m_last_wait{{0, 0}}
        , M_WAIT_SEC(SAMPLE_PERIOD_SECONDS)
        , m_do_write_batch(false)
        // This agent approach is meant to allow for quick prototyping through simplifying
        // signal & control addition and usage.  Most changes to signals and controls
        // should be accomplishable with changes to the declaration below (instead of updating
        // init_platform_io, sample_platform, etc).  Signal & control usage is still
        // handled in adjust_platform per usual.
        , m_signal_available({
                              //{"FREQUENCY", {        // Name of signal to be queried
                              //    GEOPM_DOMAIN_CORE, // Domain for the signal
                              //    true,              // Should the signal appear in the trace
                              //    {}                 // Empty Vector to contain the signal info
                              //    }},
                              {"NVML::FREQUENCY", {
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  true,
                                  {}
                                  }},
                              {"NVML::UTILIZATION_ACCELERATOR", {
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  true,
                                  {}
                                  }},
                              {"NVML::TOTAL_ENERGY_CONSUMPTION", {
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  true,
                                  {}
                                  }},
                              {"DCGM::SM_ACTIVE", {
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  true,
                                  {}
                                  }},
                             })
        , m_control_available({
                               //{"FREQUENCY", {         // Name of contol to be queried
                               //     GEOPM_DOMAIN_CORE, // Domain for the control
                               //     false,             // Should the controls appear in the trace
                               //     {}                 // Empty Vector to contain the control info
                               //     }},
                               {"NVML::FREQUENCY_CONTROL", {
                                    GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                    false,
                                    {}
                                    }},
                              })
    {
        geopm_time(&m_last_wait);
    }

    // Push signals and controls for future batch read/write
    void GPUUtilizationActivityAgent::init(int level, const std::vector<int> &fan_in, bool is_level_root)
    {
        m_accelerator_frequency_requests = 0;
        m_accelerator_low_util_samples = 0;
        m_accelerator_high_util_samples = 0;
        m_accelerator_sm_active_low_util_samples = 0;

        if (level == 0) {
            init_platform_io();
        }
    }

    void GPUUtilizationActivityAgent::init_platform_io(void)
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

        for (size_t gpu_idx = 0;
                gpu_idx < static_cast<size_t>(m_platform_topo.num_domain(GEOPM_DOMAIN_BOARD_ACCELERATOR)); ++gpu_idx) {
            m_gpu_utilization.push_back(geopm::make_unique<CircularBuffer<double> >(DECISION_WINDOW_SAMPLES));
        }
    }

    // Validate incoming policy and configure default policy requests.
    void GPUUtilizationActivityAgent::validate_policy(std::vector<double> &in_policy) const
    {
        assert(in_policy.size() == M_NUM_POLICY);
        double accel_min_freq = m_platform_io.read_signal("NVML::FREQUENCY_MIN", GEOPM_DOMAIN_BOARD, 0);
        double accel_max_freq = m_platform_io.read_signal("NVML::FREQUENCY_MAX", GEOPM_DOMAIN_BOARD, 0);

        // Check for NAN to set default values for policy
        if (std::isnan(in_policy[M_POLICY_ACCELERATOR_FREQ_MAX])) {
            in_policy[M_POLICY_ACCELERATOR_FREQ_MAX] = accel_max_freq;
        }
        if (std::isnan(in_policy[M_POLICY_ACCELERATOR_FREQ_MIN])) {
            in_policy[M_POLICY_ACCELERATOR_FREQ_MIN] = accel_min_freq;
        }
        if (std::isnan(in_policy[M_POLICY_ACCELERATOR_FREQ_EFFICIENT])) {
            in_policy[M_POLICY_ACCELERATOR_FREQ_EFFICIENT] = (in_policy[M_POLICY_ACCELERATOR_FREQ_MAX]
                                                             +in_policy[M_POLICY_ACCELERATOR_FREQ_MIN])/2;
        }
        if (std::isnan(in_policy[M_POLICY_ACCELERATOR_ENERGY_PERF_BIAS])) {
            in_policy[M_POLICY_ACCELERATOR_ENERGY_PERF_BIAS] = M_POLICY_ENERGY_PERF_BIAS_DEFAULT;
        }
    }

    // Distribute incoming policy to children
    void GPUUtilizationActivityAgent::split_policy(const std::vector<double>& in_policy,
                                    std::vector<std::vector<double> >& out_policy)
    {
        assert(in_policy.size() == M_NUM_POLICY);
        for (auto &child_pol : out_policy) {
            child_pol = in_policy;
        }
    }

    // Indicate whether to send the policy down to children
    bool GPUUtilizationActivityAgent::do_send_policy(void) const
    {
        return true;
    }

    void GPUUtilizationActivityAgent::aggregate_sample(const std::vector<std::vector<double> > &in_sample,
                                        std::vector<double>& out_sample)
    {

    }

    // Indicate whether to send samples up to the parent
    bool GPUUtilizationActivityAgent::do_send_sample(void) const
    {
        return false;
    }

    void GPUUtilizationActivityAgent::adjust_platform(const std::vector<double>& in_policy)
    {
        assert(in_policy.size() == M_NUM_POLICY);

        m_do_write_batch = false;

        // Build frequency recommendation based on accelerator utilization
        auto util_itr = m_signal_available.find("NVML::UTILIZATION_ACCELERATOR");
        auto sm_active_itr = m_signal_available.find("DCGM::SM_ACTIVE");

        //Per GPU freq
        std::vector<double> board_gpu_freq_request;

        double f_max = in_policy[M_POLICY_ACCELERATOR_FREQ_MAX];
        double f_efficient = in_policy[M_POLICY_ACCELERATOR_FREQ_EFFICIENT];
        double f_min = in_policy[M_POLICY_ACCELERATOR_FREQ_MIN];
        double energy_perf_bias = in_policy[M_POLICY_ACCELERATOR_ENERGY_PERF_BIAS];

        double f_range = f_max - f_efficient;
        //std::cout << "F_eff: " << std::to_string(f_efficient) << std::endl;
        //std::cout << "F_max: " << std::to_string(f_max) << std::endl;
        //std::cout << "F_min: " << std::to_string(in_policy[M_POLICY_ACCELERATOR_FREQ_MIN]) << std::endl;
        //std::cout << "F_range: " << std::to_string(f_range) << std::endl;
        if (energy_perf_bias > 50) {
            //Energy Biased.  Scale F_max down to F_efficient based upon EPB value

            //Inactive region EPB usage
            //f_min = std::max(f_min, f_efficient-(f_efficient-f_min)*(energy_perf_bias-50)/50);

            //Active region EPB usage
            f_max = std::max(f_efficient, f_max-f_range*(energy_perf_bias-50)/50);
        }
        else if (energy_perf_bias < 50) {
            //Perf Biased.  Scale F_efficient up to F_max based upon EPB value

            //Inactive region EPB usage
            //f_min = std::max(f_min, f_min+(f_efficient-f_min)*(50-energy_perf_bias)/50);

            //Active region EPB usage
            f_efficient = std::min(f_max, f_efficient+f_range*(50-energy_perf_bias)/50);
        }
        f_range = f_max - f_efficient;

        //std::cout << "\tF_eff_res: " << std::to_string(f_efficient) << std::endl;
        //std::cout << "\tF_max_res: " << std::to_string(f_max) << std::endl;

        // GPU
        for (int domain_idx = 0; domain_idx < util_itr->second.signals.size(); ++domain_idx) {
            double utilization_accelerator = util_itr->second.signals.at(domain_idx).m_last_signal;
            double sm_active_accelerator = sm_active_itr->second.signals.at(domain_idx).m_last_signal;

            double f_request = f_max;
            if (!std::isnan(utilization_accelerator)) {
                m_gpu_utilization[domain_idx]->insert(utilization_accelerator);
                auto gpu_samples = m_gpu_utilization[domain_idx]->make_vector();
                auto gpu_sample_max = Agg::max(gpu_samples);

                if (gpu_sample_max > 0.0) {
                    //Scaled freq with SM Active
                    if (!std::isnan(sm_active_accelerator)) {
                        //last sample only
                        if(utilization_accelerator != 0) {
                            f_request = (f_efficient + (f_range)*std::min(1.0,(sm_active_accelerator/utilization_accelerator)));
                        }
                        else {
                            f_request = (f_efficient + (f_range)*(std::min(1.0,sm_active_accelerator)));
                        }
                    }
                    ++m_accelerator_high_util_samples;
                }
                else if(!std::isnan(sm_active_accelerator) && sm_active_accelerator != 0) {
                    // In some instances NVML::UTILIZATION_ACCELERATOR can be 0 when DCGM::SM_ACTIVE
                    // is non-zero.
                    f_request = (f_efficient + (f_range)*(std::min(1.0,sm_active_accelerator)));
                    ++m_accelerator_sm_active_low_util_samples;
                    ++m_accelerator_low_util_samples;
                }
                else {
                    ++m_accelerator_low_util_samples;
                    f_request = f_min;
                }
            } else {
                utilization_accelerator = 0;
            }

            //std::cout << "F_request: " << std::to_string(f_request) << std::endl;
            f_request = std::min(f_request, f_max);
            f_request = std::max(f_request, f_min);

            board_gpu_freq_request.push_back(f_request);
            //std::cout << "\tF_request_res: " << std::to_string(f_request) << std::endl;
        }

        if (!board_gpu_freq_request.empty()) {
            // set NVML frequency control per accelerator
            auto freq_ctl_itr = m_control_available.find("NVML::FREQUENCY_CONTROL");
            for (int domain_idx = 0; domain_idx < freq_ctl_itr->second.controls.size(); ++domain_idx) {
                if (board_gpu_freq_request.at(domain_idx) != freq_ctl_itr->second.controls.at(domain_idx).m_last_setting) {
                    m_platform_io.adjust(freq_ctl_itr->second.controls.at(domain_idx).m_batch_idx, board_gpu_freq_request.at(domain_idx));
                    freq_ctl_itr->second.controls.at(domain_idx).m_last_setting = board_gpu_freq_request.at(domain_idx);
                    ++m_accelerator_frequency_requests;
                }
            }
            m_do_write_batch = true;
        }
    }

    // If controls have a valid updated value write them.
    bool GPUUtilizationActivityAgent::do_write_batch(void) const
    {
        return m_do_write_batch;
    }

    // Read signals from the platform and calculate samples to be sent up
    void GPUUtilizationActivityAgent::sample_platform(std::vector<double> &out_sample)
    {
        assert(out_sample.size() == M_NUM_SAMPLE);

        // Collect latest signal values
        for (auto &sv : m_signal_available) {
            for (int domain_idx = 0; domain_idx < sv.second.signals.size(); ++domain_idx) {
                double curr_value = m_platform_io.sample(sv.second.signals.at(domain_idx).m_batch_idx);
                sv.second.signals.at(domain_idx).m_last_sample = sv.second.signals.at(domain_idx).m_last_signal;
                sv.second.signals.at(domain_idx).m_last_signal = curr_value;
            }
        }
    }

    // Wait for the remaining cycle time to keep Controller loop cadence
    void GPUUtilizationActivityAgent::wait(void)
    {
        geopm_time_s current_time;
        do {
            geopm_time(&current_time);
        }
        while(geopm_time_diff(&m_last_wait, &current_time) < M_WAIT_SEC);
        geopm_time(&m_last_wait);
    }

    // Adds the wait time to the top of the report
    std::vector<std::pair<std::string, std::string> > GPUUtilizationActivityAgent::report_header(void) const
    {
        return {{"Wait time (sec)", std::to_string(M_WAIT_SEC)}};
    }

    // Adds number of frquency requests to the per-node section of the report
    std::vector<std::pair<std::string, std::string> > GPUUtilizationActivityAgent::report_host(void) const
    {
        std::vector<std::pair<std::string, std::string> > result;

        result.push_back({"Accelerator Frequency Requests", std::to_string(m_accelerator_frequency_requests)});
        result.push_back({"Accelerator Low Utilization Samples", std::to_string(m_accelerator_low_util_samples)});
        result.push_back({"Accelerator High Utilization Samples", std::to_string(m_accelerator_high_util_samples)});
        result.push_back({"Accelerator Low Utilization w/SM Active Samples", std::to_string(m_accelerator_sm_active_low_util_samples)});

        return result;
    }

    // This Agent does not add any per-region details
    std::map<uint64_t, std::vector<std::pair<std::string, std::string> > > GPUUtilizationActivityAgent::report_region(void) const
    {
        return {};
    }

    // Adds trace columns samples and signals of interest
    std::vector<std::string> GPUUtilizationActivityAgent::trace_names(void) const
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
    void GPUUtilizationActivityAgent::trace_values(std::vector<double> &values)
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

    std::vector<std::function<std::string(double)> > GPUUtilizationActivityAgent::trace_formats(void) const
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
    std::string GPUUtilizationActivityAgent::plugin_name(void)
    {
        return "gpu_utilization_activity";
    }

    // Used by the factory to create objects of this type
    std::unique_ptr<Agent> GPUUtilizationActivityAgent::make_plugin(void)
    {
        return geopm::make_unique<GPUUtilizationActivityAgent>();
    }

    // Describes expected policies to be provided by the resource manager or user
    std::vector<std::string> GPUUtilizationActivityAgent::policy_names(void)
    {
        return {"ACCELERATOR_FREQ_MAX", "ACCELERATOR_FREQ_EFFICIENT", "ACCELERATOR_FREQ_MIN", "ACCELERATOR_ENERGY_PERF_BIAS"};
    }

    // Describes samples to be provided to the resource manager or user
    std::vector<std::string> GPUUtilizationActivityAgent::sample_names(void)
    {
        return {};
    }
}
