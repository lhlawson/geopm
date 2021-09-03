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

#include "BottleneckTrackerAgent.hpp"

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

#define SAMPLE_PERIOD_SECONDS 0.025 // 25mS wait
#define DECISION_WINDOW_SECONDS 0.100
#define DECISION_WINDOW_SAMPLES (DECISION_WINDOW_SECONDS / SAMPLE_PERIOD_SECONDS)

namespace geopm
{
    BottleneckTrackerAgent::BottleneckTrackerAgent()
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
                              {"FREQUENCY", {
                                  GEOPM_DOMAIN_CORE,
                                  true,
                                  {}
                                  }},
                              {"MSR::IA32_PMC0:PERFCTR", {
                                  GEOPM_DOMAIN_CORE,
                                  true,
                                  {}
                                  }},
                              {"MSR::IA32_PMC1:PERFCTR", {
                                  GEOPM_DOMAIN_CORE,
                                  true,
                                  {}
                                  }},
                              {"MSR::IA32_PMC2:PERFCTR", {
                                  GEOPM_DOMAIN_CORE,
                                  true,
                                  {}
                                  }},
                              {"MSR::IA32_PMC3:PERFCTR", {
                                  GEOPM_DOMAIN_CORE,
                                  true,
                                  {}
                                  }},
                              {"MSR::C6_RESIDENCY:RESIDENCY", {
                                  GEOPM_DOMAIN_CORE,
                                  true,
                                  {}
                                  }},
                              {"QM_CTR_SCALED_RATE", {
                                  GEOPM_DOMAIN_PACKAGE,
                                  true,
                                  {}
                                  }},
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
                              {"NVML::UTILIZATION_MEMORY", {
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
                              {"INSTRUCTIONS_RETIRED", {
                                  GEOPM_DOMAIN_CORE,
                                  true,
                                  {}
                                  }},
                              {"CYCLES_THREAD", {
                                  GEOPM_DOMAIN_CORE,
                                  true,
                                  {}
                                  }},
                              {"NVML::CPU_ACCELERATOR_ACTIVE_AFFINITIZATION", {
                                  GEOPM_DOMAIN_CPU,
                                  true,
                                  {}
                                  }},
                              //{"CYCLES_REFERENCE", {
                              //    GEOPM_DOMAIN_CORE,
                              //    true,
                              //    {}
                              //    }}
                             })
        , m_control_available({
                               {"FREQUENCY", {         // Name of contol to be queried
                                    GEOPM_DOMAIN_CORE, // Domain for the control
                                    false,             // Should the controls appear in the trace
                                    {}                 // Empty Vector to contain the control info
                                    }},
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
    void BottleneckTrackerAgent::init(int level, const std::vector<int> &fan_in, bool is_level_root)
    {
        m_frequency_requests = 0;
        //m_license_0_cycles = 0;
        //m_license_1_cycles = 0;
        //m_license_2_cycles = 0;
        m_pmon_avx = false; //Gather and make decisions based upon AVX
        m_do_per_core = true;
        m_do_c6_res = true;
        m_license_0_samples = 0;
        m_license_1_samples = 0;
        m_license_2_samples = 0;
        m_accelerator_frequency_requests = 0;
        m_accelerator_low_util_samples = 0;
        m_accelerator_high_util_samples = 0;

        if (level == 0) {
            init_platform_io();
        }
    }

    void BottleneckTrackerAgent::init_platform_io(void)
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
        for (size_t cpu_idx = 0;
                cpu_idx < static_cast<size_t>(m_platform_topo.num_domain(GEOPM_DOMAIN_CORE)); ++cpu_idx) {
            m_ipc.push_back(geopm::make_unique<CircularBuffer<double> >(DECISION_WINDOW_SAMPLES));
        }

        //Directly initialize perfevtsel1-3 fields and perf_global_ctrl en_pmc0-3

        if (m_pmon_avx) {
            //AVX
            //CORE_POWER.LVL0_TURBO_LICENSE
            m_platform_io.write_control("MSR::IA32_PERFEVTSEL0:EVENT_SELECT", GEOPM_DOMAIN_BOARD, 0, 0x28);
            m_platform_io.write_control("MSR::IA32_PERFEVTSEL0:UMASK", GEOPM_DOMAIN_BOARD, 0, 0x07);
            m_platform_io.write_control("MSR::IA32_PERFEVTSEL0:USR", GEOPM_DOMAIN_BOARD, 0, 1);
            m_platform_io.write_control("MSR::IA32_PERFEVTSEL0:OS", GEOPM_DOMAIN_BOARD, 0, 1);
            m_platform_io.write_control("MSR::IA32_PERFEVTSEL0:EN", GEOPM_DOMAIN_BOARD, 0, 1);
            m_platform_io.write_control("MSR::PERF_GLOBAL_CTRL:EN_PMC0", GEOPM_DOMAIN_BOARD, 0, 1);

            //CORE_POWER.LVL1_TURBO_LICENSE
            m_platform_io.write_control("MSR::IA32_PERFEVTSEL1:EVENT_SELECT", GEOPM_DOMAIN_BOARD, 0, 0x28);
            m_platform_io.write_control("MSR::IA32_PERFEVTSEL1:UMASK", GEOPM_DOMAIN_BOARD, 0, 0x18);
            m_platform_io.write_control("MSR::IA32_PERFEVTSEL1:USR", GEOPM_DOMAIN_BOARD, 0, 1);
            m_platform_io.write_control("MSR::IA32_PERFEVTSEL1:OS", GEOPM_DOMAIN_BOARD, 0, 1);
            m_platform_io.write_control("MSR::IA32_PERFEVTSEL1:EN", GEOPM_DOMAIN_BOARD, 0, 1);
            m_platform_io.write_control("MSR::PERF_GLOBAL_CTRL:EN_PMC1", GEOPM_DOMAIN_BOARD, 0, 1);

            //CORE_POWER.LVL2_TURBO_LICENSE
            m_platform_io.write_control("MSR::IA32_PERFEVTSEL2:EVENT_SELECT", GEOPM_DOMAIN_BOARD, 0, 0x28);
            m_platform_io.write_control("MSR::IA32_PERFEVTSEL2:UMASK", GEOPM_DOMAIN_BOARD, 0, 0x20);
            m_platform_io.write_control("MSR::IA32_PERFEVTSEL2:USR", GEOPM_DOMAIN_BOARD, 0, 1);
            m_platform_io.write_control("MSR::IA32_PERFEVTSEL2:OS", GEOPM_DOMAIN_BOARD, 0, 1);
            m_platform_io.write_control("MSR::IA32_PERFEVTSEL2:EN", GEOPM_DOMAIN_BOARD, 0, 1);
            m_platform_io.write_control("MSR::PERF_GLOBAL_CTRL:EN_PMC2", GEOPM_DOMAIN_BOARD, 0, 1);
        }

        //setup MBM monitoring
        m_platform_io.write_control("MSR::PQR_ASSOC:RMID", GEOPM_DOMAIN_BOARD, 0 ,0);
        m_platform_io.write_control("MSR::QM_EVTSEL:RMID", GEOPM_DOMAIN_BOARD, 0, 0);
        m_platform_io.write_control("MSR::QM_EVTSEL:EVENT_ID", GEOPM_DOMAIN_BOARD, 0, 2);
    }

    // Validate incoming policy and configure default policy requests.
    void BottleneckTrackerAgent::validate_policy(std::vector<double> &in_policy) const
    {
        assert(in_policy.size() == M_NUM_POLICY);
        //double min_freq = m_platform_io.read_signal("CPU_FREQUENCY_MIN", GEOPM_DOMAIN_BOARD, 0);
        //double max_freq = m_platform_io.read_signal("CPU_FREQUENCY_MAX", GEOPM_DOMAIN_BOARD, 0);
        //double sticker_freq = m_platform_io.read_signal("FREQUENCY_STICKER", GEOPM_DOMAIN_BOARD, 0);

        ////TODO: taken from example agent, moved to here...can we ever actually hit these beng NAN?
        //// Check for NAN to set default values for policy
        //if (std::isnan(in_policy[M_POLICY_THRESH_0])) {
        //    in_policy[M_POLICY_THRESH_0] = 0.5;
        //}
        //if (std::isnan(in_policy[M_POLICY_THRESH_1])) {
        //    in_policy[M_POLICY_THRESH_1] = 0.7;
        //}
        //if (std::isnan(in_policy[M_POLICY_FREQ_SUB_THRESH_0])) {
        //    in_policy[M_POLICY_FREQ_SUB_THRESH_0] = min_freq;
        //}
        //if (std::isnan(in_policy[M_POLICY_FREQ_SUB_THRESH_1])) {
        //    in_policy[M_POLICY_FREQ_SUB_THRESH_1] = min_freq;
        //}
        //if (std::isnan(in_policy[M_POLICY_FREQ_ABOVE_THRESH_1])) {
        //    in_policy[M_POLICY_FREQ_ABOVE_THRESH_1] = max_freq;
        //}
    }

    // Distribute incoming policy to children
    void BottleneckTrackerAgent::split_policy(const std::vector<double>& in_policy,
                                    std::vector<std::vector<double> >& out_policy)
    {
        assert(in_policy.size() == M_NUM_POLICY);
        for (auto &child_pol : out_policy) {
            child_pol = in_policy;
        }
    }

    // Indicate whether to send the policy down to children
    bool BottleneckTrackerAgent::do_send_policy(void) const
    {
        return true;
    }

    void BottleneckTrackerAgent::aggregate_sample(const std::vector<std::vector<double> > &in_sample,
                                        std::vector<double>& out_sample)
    {

    }

    // Indicate whether to send samples up to the parent
    bool BottleneckTrackerAgent::do_send_sample(void) const
    {
        return false;
    }

    // Not exact, but close enough for our needs
    //static double quantile(std::vector<double> &samples, double q)
    //{
    //    const size_t idx = q * samples.size();
    //    std::nth_element(samples.begin(),
    //    samples.begin() + idx, samples.end());
    //    return samples[idx];
    //}
    //
    //

    void BottleneckTrackerAgent::adjust_platform(const std::vector<double>& in_policy)
    {
        assert(in_policy.size() == M_NUM_POLICY);

        m_do_write_batch = false;

        // Build frequency recommendation based on accelerator utilization
        //auto pcnt_itr = m_signal_available.find("MSR::PPERF:PCNT");
        //auto acnt_itr = m_signal_available.find("MSR::APERF:ACNT");
        //auto freq_itr = m_signal_available.find("FREQUENCY");
        auto pmc0_itr = m_signal_available.find("MSR::IA32_PMC0:PERFCTR");
        auto pmc1_itr = m_signal_available.find("MSR::IA32_PMC1:PERFCTR");
        auto pmc2_itr = m_signal_available.find("MSR::IA32_PMC2:PERFCTR");
        auto c6_res_itr =  m_signal_available.find("MSR::C6_RESIDENCY:RESIDENCY");
        auto inst_retired_itr = m_signal_available.find("INSTRUCTIONS_RETIRED");
        auto cycle_thread_itr = m_signal_available.find("CYCLES_THREAD");
        auto qm_itr = m_signal_available.find("QM_CTR_SCALED_RATE");
        //auto cycle_ref_itr = m_signal_available.find("CYCLES_REFERENCE");

        auto util_itr = m_signal_available.find("NVML::UTILIZATION_ACCELERATOR");
        auto util_mem_itr = m_signal_available.find("NVML::UTILIZATION_MEMORY");

        auto freq_ctl_itr = m_control_available.find("FREQUENCY");

        auto cpu_gpu_affin = m_signal_available.find("NVML::CPU_ACCELERATOR_ACTIVE_AFFINITIZATION");

        //Per package freq - TODO: make this get the num packages honestly.
        std::vector<double> package_freq_request;
        std::vector<double> package_P0a;

        //Per GPU freq
        std::vector<double> board_gpu_freq_request;
        std::vector<double> gpu_util;

        //Package indexed
        std::vector<unsigned int> active_cores_package;
        std::vector<unsigned int> avx_package_max;

        //core indexed
        std::vector<unsigned int> avx_core;
        std::vector<double> cpu_freq_request;

        unsigned int active_cores = 0;
        int package = 0;
        int cores_per_package = m_platform_topo.num_domain(GEOPM_DOMAIN_CORE)/m_platform_topo.num_domain(GEOPM_DOMAIN_PACKAGE);


        // GPU
        for (int domain_idx = 0; domain_idx < util_itr->second.signals.size(); ++domain_idx) {
            double utilization_accelerator = util_itr->second.signals.at(domain_idx).m_last_signal;
            double utilization_accelerator_mem = util_itr->second.signals.at(domain_idx).m_last_signal;

            if (!std::isnan(utilization_accelerator)) {
                //std::cout << "utilization_accel is: " << std::to_string(utilization_accelerator) << std::endl;
                //m_scalable_freq[domain_idx]->insert(scalability);
                m_gpu_utilization[domain_idx]->insert(utilization_accelerator);
                auto gpu_samples = m_gpu_utilization[domain_idx]->make_vector();
                //auto qtile = quantile(core_samples, QUANTILE);
                auto m = Agg::max(gpu_samples);
                if (m > 0.0) {
                    board_gpu_freq_request.push_back(m_gpu_P0_freq);
                }
                else {
                    board_gpu_freq_request.push_back(m_gpu_PN_freq);
                }
                gpu_util.push_back(utilization_accelerator);
            } else {
                gpu_util.push_back(0);
            }
        }

        // XEON
        max_cores = m_freq_p0x.at(0).size();
        if(m_do_c6_res) {
            for (int domain_idx = 0; domain_idx < pmc0_itr->second.signals.size(); ++domain_idx) {
                package = domain_idx/cores_per_package; //TODO: PlatformTopo-ify this?

                double c6_res_diff = c6_res_itr->second.signals.at(domain_idx).m_last_sample;
                double c6_res_sig = c6_res_itr->second.signals.at(domain_idx).m_last_signal;

                if (std::isnan(c6_res_diff) //TODO: review.  if a core reports NAN for C6 we're assuming it's active.
                    || c6_res_diff == 0) { //TODO: make <= some number
                    ++active_cores;
                }

                if((domain_idx+1) % cores_per_package == 0) {
                    if (active_cores > 0) {
                        active_cores_package.push_back(active_cores);
                    }
                    else if (active_cores >= max_cores) { //should be max_cores
                        active_cores_package.push_back(max_cores);
                    } else {
                        active_cores_package.push_back(1);
                        std::cerr << "ERROR: Where'd everyone go?" << std::endl;
                    }
                    //std::cout << "Active cores: " << std::to_string(active_cores) << std::endl;
                    active_cores = 0;
                }
            }
        }
        else {
            active_cores_package.push_back(max_cores);
            active_cores_package.push_back(max_cores);
        }

        if (m_pmon_avx) {
            for (int domain_idx = 0; domain_idx < pmc0_itr->second.signals.size(); ++domain_idx) {
                package = domain_idx/cores_per_package; //TODO: PlatformTopo-ify this?
                double pmc0_diff = pmc0_itr->second.signals.at(domain_idx).m_last_sample;
                double pmc1_diff = pmc1_itr->second.signals.at(domain_idx).m_last_sample;
                double pmc2_diff = pmc2_itr->second.signals.at(domain_idx).m_last_sample;
                int max_avx_seen = 0;

                if (!std::isnan(pmc0_diff)
                    && !std::isnan(pmc1_diff)
                    && !std::isnan(pmc2_diff) ) {
                    //m_license_0_cycles += pmc0_diff;
                    //m_license_1_cycles += pmc1_diff;
                    //m_license_2_cycles += pmc2_diff;
                    //std::cout << "debug1: package: " << std::to_string(package) << std::endl;
                    //std::cout << "debug1: core: " << std::to_string(domain_idx) << std::endl;

                    if (pmc0_diff >= pmc1_diff && pmc0_diff >= pmc2_diff) {
                        //Minimal time in AVX2 or AVX512
                        //std::cout << "SSE" << std::endl;
                        ++m_license_0_samples;
                        //cpu_freq_request.push_back(m_freq_p0x.at(0).at(active_cores));
                        avx_core.push_back(0);
                   }
                    else if (pmc1_diff >= pmc0_diff && pmc1_diff >= pmc2_diff) {
                        //more time in AVX2 than AVX512
                        //std::cout << "AVX2" << std::endl;
                        ++m_license_1_samples;
                        //cpu_freq_request.push_back(m_freq_p0x.at(1).at(active_cores));
                        avx_core.push_back(1);
                        if(max_avx_seen < 1) {
                            max_avx_seen = 1;
                        }
                    } else {
                        //more time in AVX512 than AVX2
                        //std::cout << "AVX512" << std::endl;
                        ++m_license_2_samples;
                        //cpu_freq_request.push_back(m_freq_p0x.at(2).at(active_cores));
                        avx_core.push_back(2);
                        if(max_avx_seen < 2) {
                            max_avx_seen = 2;
                        }
                    }

                    //std::cout << "\tSSE    cycles: " << m_license_0_cycles << std::endl;
                    //std::cout << "\tAVX2   cycles: " << m_license_1_cycles << std::endl;
                    //std::cout << "\tAVX512 cycles: " << m_license_2_cycles << std::endl;
                    //std::cout << "\tSSE    samples: " << m_license_0_samples << std::endl;
                    //std::cout << "\tAVX2   samples: " << m_license_1_samples << std::endl;
                    //std::cout << "\tAVX512 samples: " << m_license_2_samples << std::endl;
                }
                else {
                    avx_core.push_back(0);
                }
                if((domain_idx+1) % cores_per_package == 0) { //TODO: PlatformTopo-ify this?
                    avx_package_max.push_back(max_avx_seen);
                    //reset for next package
                    max_avx_seen = 0;
                }
            }
        }
        else {
            avx_package_max.push_back(0);
            avx_package_max.push_back(0);
            for (int domain_idx = 0; domain_idx < pmc0_itr->second.signals.size(); ++domain_idx) {
                avx_core.push_back(0);
            }
        }

        //TODO: consider moving this to a package level loop.  Assumption is that in a bulk synchronous workload
        //      all cores should be set to the same maximum frequency for AVX regions.  This will save energy
        //      when cores finish early and needlessly 'pop up' to SSE frequencies, if we set all cores to the
        //      minimum of the maximum frequencies (i.e. most restrictive)..  HACC might be a good test?
        for (int domain_idx = 0; domain_idx < pmc0_itr->second.signals.size(); ++domain_idx) {
            package = domain_idx/cores_per_package; //TODO: PlatformTopo-ify this?
            //double package_P0a = package_freq_request.at(package); //The P0a max, where a=active cores
            int avx_level = avx_core.at(domain_idx); //TODO: could use package level
            active_cores = active_cores_package.at(package);

            double package_P0a = m_freq_p0x.at(avx_level).at(active_cores-1); //The P0a max, where a=active cores
            //std::cout << "Core: " << std::to_string(domain_idx);
            //std::cout << ", avx level: " << std::to_string(avx_level);
            //std::cout << ", Active cores: " << std::to_string(active_cores);
            //std::cout << ", freq max (P0a_avx): " << std::to_string(package_P0a) << std::endl;

            double inst_retired = inst_retired_itr->second.signals.at(domain_idx).m_last_sample;
            double cycle_thread = cycle_thread_itr->second.signals.at(domain_idx).m_last_sample;
            double ipc = inst_retired/cycle_thread;

            //double cycle_ref = cycle_ref_itr->second.signals.at(domain_idx).m_last_sample;

            // TODO: check that the GPU associated with this CPU is active (GPU_Util != 0).
            //       See notes on better NVML CPU to GPU affinitization testing
            if (!std::isnan(ipc)) {
                m_ipc[domain_idx]->insert(ipc);
                auto ipc_samples = m_ipc[domain_idx]->make_vector();
                auto ipc_max = Agg::max(ipc_samples);
                auto ipc_min = Agg::min(ipc_samples);
                auto ipc_avg = Agg::average(ipc_samples);
                auto ipc_std = Agg::stddev(ipc_samples);
                double ipc_std_perc = ipc_std/ipc_avg;

                double associated_gpu = cpu_gpu_affin->second.signals.at(domain_idx).m_last_signal;
                //std::cout << "debug: associated_gpu: " << std::to_string(associated_gpu) << std::endl;
                if(associated_gpu != -1                         // -1 means associated to no GPUs
                   && !std::isnan(associated_gpu)               // NAN means associated to multiple GPUs
                   && gpu_util.at((int)associated_gpu) >= 0.95
                   && ipc_std_perc < 0.05
                   && ipc_avg < 3) {                            // TODO: And IPC < CUTOFF?  IPC of 3?
                    double last_request = freq_ctl_itr->second.controls.at(domain_idx).m_last_setting;
                    double request = last_request - 1e8; //step down 100MHz
                    //if(request >= m_freq_sticker.at(avx_level)) { //TODO: consider re-enabling
                    if(request >= m_freq_sticker.at(0)) { //Using SSE limits to be more perf conscious
                        cpu_freq_request.push_back(request);
                        //TODO: dump history?
                    } else {
                        cpu_freq_request.push_back(last_request);
                    }
                } else {
                    cpu_freq_request.push_back(package_P0a);
                }
            }
            else {
                //cpu_freq_request.push_back(3.7*1e9);
                cpu_freq_request.push_back(package_P0a);
            }
        }

        if (m_do_per_core) {
            if (!cpu_freq_request.empty() && !board_gpu_freq_request.empty()) {
                // set Xeon frequency control per core
                //auto freq_ctl_itr = m_control_available.find("FREQUENCY");
                freq_ctl_itr = m_control_available.find("FREQUENCY");
                for (int domain_idx = 0; domain_idx < freq_ctl_itr->second.controls.size(); ++domain_idx) {
                    if (cpu_freq_request.at(domain_idx) != freq_ctl_itr->second.controls.at(domain_idx).m_last_setting) {
                        m_platform_io.adjust(freq_ctl_itr->second.controls.at(domain_idx).m_batch_idx, cpu_freq_request.at(domain_idx));
                        freq_ctl_itr->second.controls.at(domain_idx).m_last_setting = cpu_freq_request.at(domain_idx);
                        ++m_frequency_requests;
                    }
                }

                // set NVML frequency control per accelerator
                freq_ctl_itr = m_control_available.find("NVML::FREQUENCY_CONTROL");
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
        else {
            if (!package_freq_request.empty() && !board_gpu_freq_request.empty()) {

                // set NVML frequency control per accelerator
                freq_ctl_itr = m_control_available.find("NVML::FREQUENCY_CONTROL");
                for (int domain_idx = 0; domain_idx < freq_ctl_itr->second.controls.size(); ++domain_idx) {
                    if (board_gpu_freq_request.at(domain_idx) != freq_ctl_itr->second.controls.at(domain_idx).m_last_setting) {
                        m_platform_io.adjust(freq_ctl_itr->second.controls.at(domain_idx).m_batch_idx, board_gpu_freq_request.at(domain_idx));
                        freq_ctl_itr->second.controls.at(domain_idx).m_last_setting = board_gpu_freq_request.at(domain_idx);
                        ++m_accelerator_frequency_requests;
                    }
                }

                // set Xeon frequency control per core
                freq_ctl_itr = m_control_available.find("FREQUENCY");
                for (int domain_idx = 0; domain_idx < freq_ctl_itr->second.controls.size(); ++domain_idx) {
                    package = domain_idx/cores_per_package; //TODO: PlatformTopo-ify this?
                    //std::cout << "debug2: package: " << std::to_string(package) << std::endl;
                    //std::cout << "debug2: core: " << std::to_string(domain_idx) << std::endl;
                    //std::cout << "debug2: cores_per_package " << std::to_string(cores_per_package) << std::endl;
                    if (package_freq_request.at(package) != freq_ctl_itr->second.controls.at(domain_idx).m_last_setting) {
                        m_platform_io.adjust(freq_ctl_itr->second.controls.at(domain_idx).m_batch_idx, package_freq_request.at(package));
                        freq_ctl_itr->second.controls.at(domain_idx).m_last_setting = package_freq_request.at(package);
                        ++m_frequency_requests;
                    }
                }
                m_do_write_batch = true;
            }
        }
    }

    // If controls have a valid updated value write them.
    bool BottleneckTrackerAgent::do_write_batch(void) const
    {
        return m_do_write_batch;
    }

    // Read signals from the platform and calculate samples to be sent up
    void BottleneckTrackerAgent::sample_platform(std::vector<double> &out_sample)
    {
        assert(out_sample.size() == M_NUM_SAMPLE);

        // Collect latest signal values
        for (auto &sv : m_signal_available) {
            for (int domain_idx = 0; domain_idx < sv.second.signals.size(); ++domain_idx) {
                double curr_value = m_platform_io.sample(sv.second.signals.at(domain_idx).m_batch_idx);

                if (sv.first == "MSR::PPERF:PCNT" ||
                    sv.first == "MSR::APERF:ACNT" ||
                    sv.first == "MSR::IA32_PMC0:PERFCTR" ||
                    sv.first == "MSR::IA32_PMC1:PERFCTR" ||
                    sv.first == "MSR::IA32_PMC2:PERFCTR" ||
                    sv.first == "MSR::IA32_PMC3:PERFCTR" ||
                    sv.first == "INSTRUCTIONS_RETIRED" ||
                    sv.first == "CYCLES_THREAD" ||
                    sv.first == "MSR::C6_RESIDENCY:RESIDENCY") { //||
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
    void BottleneckTrackerAgent::wait(void)
    {
        geopm_time_s current_time;
        do {
            geopm_time(&current_time);
        }
        while(geopm_time_diff(&m_last_wait, &current_time) < M_WAIT_SEC);
        geopm_time(&m_last_wait);
    }

    // Adds the wait time to the top of the report
    std::vector<std::pair<std::string, std::string> > BottleneckTrackerAgent::report_header(void) const
    {
        return {{"Wait time (sec)", std::to_string(M_WAIT_SEC)}};
    }

    // Adds number of frquency requests to the per-node section of the report
    std::vector<std::pair<std::string, std::string> > BottleneckTrackerAgent::report_host(void) const
    {
        std::vector<std::pair<std::string, std::string> > result;
        if(m_do_per_core) {
            result.push_back({"Per core Frequency Requests", std::to_string(m_frequency_requests)});
        } else {
            result.push_back({"Package Frequency Requests", std::to_string(m_frequency_requests/m_platform_topo.num_domain(GEOPM_DOMAIN_CORE))});
        }
        if (m_pmon_avx) {
            result.push_back({"Package License 0 samples", std::to_string(m_license_0_samples/m_platform_topo.num_domain(GEOPM_DOMAIN_CORE))});
            result.push_back({"Package License 1 samples", std::to_string(m_license_1_samples/m_platform_topo.num_domain(GEOPM_DOMAIN_CORE))});
            result.push_back({"Package License 2 samples", std::to_string(m_license_2_samples/m_platform_topo.num_domain(GEOPM_DOMAIN_CORE))});
            result.push_back({"Frequency Requests", std::to_string(m_frequency_requests)});
            //, {"License 0 cycles", std::to_string(m_license_0_cycles)}
            //, {"License 1 cycles", std::to_string(m_license_1_cycles)}
            //, {"License 2 cycles", std::to_string(m_license_2_cycles)}
        }
        result.push_back({"Accelerator Frequency Requests", std::to_string(m_accelerator_frequency_requests)});
        result.push_back({"Accelerator Low Utilization Samples", std::to_string(m_accelerator_low_util_samples)});
        result.push_back({"Accelerator High Utilization Samples", std::to_string(m_accelerator_high_util_samples)});

        return result;
    }

    // This Agent does not add any per-region details
    std::map<uint64_t, std::vector<std::pair<std::string, std::string> > > BottleneckTrackerAgent::report_region(void) const
    {
        return {};
    }

    // Adds trace columns samples and signals of interest
    std::vector<std::string> BottleneckTrackerAgent::trace_names(void) const
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
    void BottleneckTrackerAgent::trace_values(std::vector<double> &values)
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

    std::vector<std::function<std::string(double)> > BottleneckTrackerAgent::trace_formats(void) const
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
    std::string BottleneckTrackerAgent::plugin_name(void)
    {
        return "bottleneck_tracker";
    }

    // Used by the factory to create objects of this type
    std::unique_ptr<Agent> BottleneckTrackerAgent::make_plugin(void)
    {
        return geopm::make_unique<BottleneckTrackerAgent>();
    }

    // Describes expected policies to be provided by the resource manager or user
    std::vector<std::string> BottleneckTrackerAgent::policy_names(void)
    {
        //return {"THRESH_0", "THRESH_1", "FREQ_SUB_THRESH_0", "FREQ_SUB_THRESH_1", "FREQ_ABOVE_THRESH_1"};
        return {};
    }

    // Describes samples to be provided to the resource manager or user
    std::vector<std::string> BottleneckTrackerAgent::sample_names(void)
    {
        return {};
    }
}
