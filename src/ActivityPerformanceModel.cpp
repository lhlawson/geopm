/*
 * Copyright (c) 2015 - 2022, Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ActivityPerformanceModelImp.hpp"

#include <cmath>
#include <unistd.h>

#include "PlatformIOProf.hpp"
#include "geopm/Exception.hpp"
#include "geopm/Helper.hpp"
#include "geopm/PlatformIO.hpp"
#include "geopm/PlatformTopo.hpp"
#include "config.h"

namespace geopm
{
    //TODO: require domain as part of constructor
    ActivityPerformanceModel &activity_perf_model()
    {
        static ActivityPerformanceModelImp instance;
        return instance;
    }

    ActivityPerformanceModelImp::ActivityPerformanceModelImp()
        : ActivityPerformanceModelImp(PlatformIOProf::platform_io(), platform_topo())
    {

    }

    ActivityPerformanceModelImp::ActivityPerformanceModelImp(PlatformIO &platform_io, const PlatformTopo &platform_topo)
        : m_platform_io(platform_io)
        , m_platform_topo(platform_topo)
        , M_NUM_PACKAGE(m_platform_topo.num_domain(GEOPM_DOMAIN_PACKAGE))
        , M_NUM_CORE(m_platform_topo.num_domain(GEOPM_DOMAIN_CORE))
        , M_NUM_GPU(m_platform_topo.num_domain(GEOPM_DOMAIN_GPU))
    {
        init();
    }

    ActivityPerformanceModelImp::~ActivityPerformanceModelImp()
    {

    }

    void ActivityPerformanceModelImp::init(void)
    {
        init_platform_io();
    }

    void ActivityPerformanceModelImp::init_platform_io(void) {
        // push back signals of interest
        auto all_names = m_platform_io.signal_names();

        // Setup Core Algorithm Signals
        if (all_names.count("CPU_FREQUENCY_MIN_AVAIL") != 0 &&
            all_names.count("CPU_FREQUENCY_MAX_AVAIL") != 0 &&
            all_names.count("CPU_FREQUENCY_STICKER") != 0 &&
            all_names.count("CPU_FREQUENCY_STEP") != 0 ) {

            m_freq_core_min = m_platform_io.read_signal("CPU_FREQUENCY_MIN_AVAIL", GEOPM_DOMAIN_BOARD, 0);
            m_freq_core_max = m_platform_io.read_signal("CPU_FREQUENCY_MAX_AVAIL", GEOPM_DOMAIN_BOARD, 0);
            m_freq_core_sticker = m_platform_io.read_signal("CPU_FREQUENCY_STICKER", GEOPM_DOMAIN_BOARD, 0);
            m_freq_core_step = m_platform_io.read_signal("CPU_FREQUENCY_STEP", GEOPM_DOMAIN_BOARD, 0);

            // Core Scalability
            std::string core_scalability_signal = "MSR::CPU_SCALABILITY_RATIO";
            if (all_names.count(core_scalability_signal) != 0) {
                for (int domain_idx = 0; domain_idx < M_NUM_CORE; ++domain_idx) {
                    m_core_scal.push_back({m_platform_io.push_signal(core_scalability_signal,
                                                                     GEOPM_DOMAIN_CORE,
                                                                     domain_idx), NAN});
                }

                // If all the above checks have been met we support these controls
                m_supported_controls["CPU_FREQUENCY_MAX_CONTROL"] = GEOPM_DOMAIN_CORE;
            }

        }

        // Setup Uncore Algorithm Signals
        if (all_names.count("CPU_UNCORE_FREQUENCY_MIN_CONTROL") != 0 &&
            all_names.count("CPU_UNCORE_FREQUENCY_MAX_CONTROL") != 0) {
            m_freq_uncore_min = m_platform_io.read_signal("CPU_UNCORE_FREQUENCY_MIN_CONTROL", GEOPM_DOMAIN_BOARD, 0);
            m_freq_uncore_max = m_platform_io.read_signal("CPU_UNCORE_FREQUENCY_MAX_CONTROL", GEOPM_DOMAIN_BOARD, 0);

            // Uncore Scalability
            std::string uncore_scalability_signal = "MSR::QM_CTR_SCALED_RATE";
            std::string uncore_frequency_signal = "CPU_UNCORE_FREQUENCY_STATUS";
            if ((all_names.count(uncore_scalability_signal) != 0) &&
               (all_names.count(uncore_frequency_signal) != 0)) {
                for (int domain_idx = 0; domain_idx < M_NUM_PACKAGE; ++domain_idx) {
                    m_qm_rate.push_back({m_platform_io.push_signal(uncore_scalability_signal,
                                                                   GEOPM_DOMAIN_PACKAGE,
                                                                   domain_idx), NAN});

                    m_uncore_freq_status.push_back({m_platform_io.push_signal(uncore_frequency_signal,
                                                                              GEOPM_DOMAIN_PACKAGE,
                                                                              domain_idx), NAN});
                }

                // If all the above checks have been met we support these controls
                m_supported_controls["CPU_UNCORE_FREQUENCY_MIN_CONTROL"] = GEOPM_DOMAIN_PACKAGE;
                m_supported_controls["CPU_UNCORE_FREQUENCY_MAX_CONTROL"] = GEOPM_DOMAIN_PACKAGE;
            }
        }

        if (all_names.count("GPU_FREQUENCY_MIN_AVAIL") != 0 &&
            all_names.count("GPU_FREQUENCY_MAX_AVAIL") != 0) {
            m_freq_gpu_min = m_platform_io.read_signal("GPU_FREQUENCY_MIN_AVAIL", GEOPM_DOMAIN_BOARD, 0);
            m_freq_gpu_max = m_platform_io.read_signal("GPU_FREQUENCY_MAX_AVAIL", GEOPM_DOMAIN_BOARD, 0);

            // GPU Scalability
            std::string gpu_scalability_signal = "GPU_CORE_ACTIVITY";
            if (all_names.count(gpu_scalability_signal) != 0) {
                for (int domain_idx = 0; domain_idx < M_NUM_GPU; ++domain_idx) {
                    m_gpu_scal.push_back({m_platform_io.push_signal(gpu_scalability_signal,
                                                                    GEOPM_DOMAIN_GPU,
                                                                    domain_idx), NAN});
                }

                // If all the above checks have been met we support these controls
                m_supported_controls["GPU_FREQUENCY_MIN_CONTROL"] = GEOPM_DOMAIN_GPU;
                m_supported_controls["GPU_FREQUENCY_MAX_CONTROL"] = GEOPM_DOMAIN_GPU;
            }

        }

        std::string constconfig_fe_gpu = "CONSTCONFIG::GPU_CORE_FREQUENCY_EFFICIENT";
        std::string constconfig_fe_cpu = "CONSTCONFIG::CPU_CORE_FREQUENCY_EFFICIENT";
        std::string constconfig_fe_uncore = "CONSTCONFIG::CPU_UNCORE_FREQUENCY_EFFICIENT";
        //TODO: Use ConstConfigIO to set defaults for Fe, Fmax, QM Max Rate, etc
    }

    bool ActivityPerformanceModelImp::algorithm_valid(void) {
        return m_supported_controls.size() > 0;
    }

    std::map<std::string, int> ActivityPerformanceModelImp::controls_recommended(void) {
        return m_supported_controls;
    }

    //void ActivityPerformanceModelImp::update_uncore_bandwidth_map(std::map<double, double> uncore_max_mem_bw) {
    //    m_max_mem_bw = uncore_max_mem_bw;
    //}

    //double ActivityPerformanceModelImp::get_uncore_activity(double uncore_freq,
    //                                                        double uncore_bandwidth)
    //                                                       const {
    //    double uncore_activity = NAN;
    //    if (m_max_mem_bw.size() != 0) {
    //        auto bw_max_itr = m_max_mem_bw.lower_bound(uncore_freq);
    //        if(bw_max_itr != m_max_mem_bw.begin()) {
    //            bw_max_itr = std::prev(bw_max_itr, 1);
    //        }

    //        // Handle divided by zero, either numerator or
    //        // denominator being NAN, and the un-characterized case
    //        if (!std::isnan(uncore_bandwidth) &&
    //            !std::isnan(bw_max_itr->second)) {
    //            uncore_activity  = (double) uncore_bandwidth /
    //                                        bw_max_itr->second;
    //        }
    //    }

    //    return uncore_activity;
    //}

    std::vector<double> ActivityPerformanceModelImp::sample_recommendation(std::string control_name) const {
        std::vector<double> result = {};

        if (m_recommendation.count(control_name) != 0 &&
            m_supported_controls.count(control_name) != 0) {
            result = m_recommendation.at(control_name);
        }
        return result;
    }

    void ActivityPerformanceModelImp::update_recommendation(double phi) {
        double f_e = NAN;
        double f_max = NAN;

        // CORE
        m_recommendation["CPU_FREQUENCY_MAX_CONTROL"] = {};
        if (m_supported_controls.count("CPU_FREQUENCY_MAX_CONTROL") != 0) {
            // Gather Fe value
            //use constconfig efficient freq or sticker

            if (std::isnan(f_e)) {
                if (! std::isnan(m_freq_core_sticker) &&
                    ! std::isnan(m_freq_core_step)) {
                    // Sticker - 2 steps is generally energy efficient
                    f_e = m_freq_core_sticker - m_freq_core_step * 2;
                }
                else {
                    // Use min
                    f_e = m_freq_core_min;
                }
            }

            if (std::isnan(f_max)) {
                //use system maximum
                f_max = m_freq_core_max;
            }

            // Generate per core frequency recommendation
            for (int domain_idx = 0; domain_idx < M_NUM_CORE; ++domain_idx) {
                m_core_scal.at(domain_idx).value = m_platform_io.sample(m_core_scal.at(domain_idx).batch_idx);
                double freq_rec = frequency_fit(f_e, f_max, m_core_scal.at(domain_idx).value, 0.5);
                m_recommendation["CPU_FREQUENCY_MAX_CONTROL"].push_back(freq_rec);
            }
        }
        else {
            for (int domain_idx = 0; domain_idx < M_NUM_CORE; ++domain_idx) {
                m_recommendation["CPU_FREQUENCY_MAX_CONTROL"].push_back(NAN);
            }
        }

        // UNCORE
        m_recommendation["CPU_UNCORE_FREQUENCY_MIN_CONTROL"] = {};
        m_recommendation["CPU_UNCORE_FREQUENCY_MAX_CONTROL"] = {};
        if (m_supported_controls.count("CPU_UNCORE_FREQUENCY_MIN_CONTROL") != 0 &&
            m_supported_controls.count("CPU_UNCORE_FREQUENCY_MAX_CONTROL") != 0) {
            for (int domain_idx = 0; domain_idx < M_NUM_PACKAGE; ++domain_idx) {
                m_recommendation["CPU_UNCORE_FREQUENCY_MIN_CONTROL"].push_back(NAN);
                m_recommendation["CPU_UNCORE_FREQUENCY_MAX_CONTROL"].push_back(NAN);
            }
        }
        else {
            for (int domain_idx = 0; domain_idx < M_NUM_PACKAGE; ++domain_idx) {
                m_recommendation["CPU_UNCORE_FREQUENCY_MIN_CONTROL"].push_back(NAN);
                m_recommendation["CPU_UNCORE_FREQUENCY_MAX_CONTROL"].push_back(NAN);
            }
        }

        //GPU
        m_recommendation["GPU_FREQUENCY_MIN_CONTROL"] = {};
        m_recommendation["GPU_FREQUENCY_MAX_CONTROL"] = {};
        if (m_supported_controls.count("GPU_CORE_FREQUENCY_MIN_CONTROL") != 0 &&
            m_supported_controls.count("GPU_CORE_FREQUENCY_MAX_CONTROL") != 0) {
                // Gather Fe value

                // TODO: use constconfig efficient freq or sticker

                if (std::isnan(f_e)) {
                    // TODO: check for F_e signal

                    // If not available as a signal estimate it
                    f_e = (m_freq_gpu_min + m_freq_gpu_max) / 2;
                }

                if (std::isnan(f_max)) {
                    //use system maximum
                    f_max = m_freq_gpu_max;
                }

                // Generate per GPU frequency recommendation
                for (int domain_idx = 0; domain_idx < M_NUM_GPU; ++domain_idx) {
                    m_gpu_scal.at(domain_idx).value = m_platform_io.sample(m_gpu_scal.at(domain_idx).batch_idx);
                    double freq_rec = frequency_fit(f_e, f_max, m_gpu_scal.at(domain_idx).value, 0.5);
                    m_recommendation["GPU_FREQUENCY_MIN_CONTROL"].push_back(freq_rec);
                    m_recommendation["GPU_FREQUENCY_MAX_CONTROL"].push_back(freq_rec);
                }
        }
        else {
            for (int domain_idx = 0; domain_idx < M_NUM_GPU; ++domain_idx) {
                m_recommendation["GPU_FREQUENCY_MIN_CONTROL"].push_back(NAN);
                m_recommendation["GPU_FREQUENCY_MAX_CONTROL"].push_back(NAN);
            }
        }
    }

    double ActivityPerformanceModelImp::frequency_fit(double f_e, double f_max, double scalability, double phi) {
        double freq_rec = f_max;
        // Core steering signals
        if (std::isnan(scalability)) {
            scalability = 1;
        }

        freq_rec = f_e +  (f_max - f_e) * scalability;

        // Request should never be above the per domain f_max
        freq_rec = std::min(f_max, freq_rec);
        // Request should never be below the per domain f_e
        freq_rec = std::max(f_e, freq_rec);

        return freq_rec;
    }
}
