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
    }

    ActivityPerformanceModelImp::~ActivityPerformanceModelImp()
    {
    }

    void ActivityPerformanceModelImp::init(void)
    {
        m_supported_controls = {};
        m_recommendation = {};

        init_platform_core_io();
        init_platform_uncore_io();
        init_platform_gpu_io();
    }

    void ActivityPerformanceModelImp::init_platform_core_io(void) {
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

            // Gather Fe value
            //use constconfig efficient freq or sticker
            m_freq_core_efficient = NAN;
            std::string constconfig_fe_core = "CONSTCONFIG::CPU_CORE_FREQUENCY_EFFICIENT_HIGH_INTENSITY";
            if (all_names.count(constconfig_fe_core) != 0) {
                m_freq_core_efficient = m_platform_io.read_signal(constconfig_fe_core,
                                                                  GEOPM_DOMAIN_BOARD, 0);
            }

            if (std::isnan(m_freq_core_efficient)) {
                if (! std::isnan(m_freq_core_sticker) &&
                    ! std::isnan(m_freq_core_step)) {
                    // Sticker - 2 steps is generally energy efficient
                    m_freq_core_efficient = m_freq_core_sticker - m_freq_core_step * 2;
                }
                else {
                    // Use min
                    m_freq_core_efficient = m_freq_core_min;
                }
            }

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
    }

    void ActivityPerformanceModelImp::init_platform_uncore_io(void) {
        auto all_names = m_platform_io.signal_names();

        // Setup Uncore Algorithm Signals
        if (all_names.count("CPU_UNCORE_FREQUENCY_MIN_CONTROL") != 0 &&
            all_names.count("CPU_UNCORE_FREQUENCY_MAX_CONTROL") != 0) {
            m_freq_uncore_min = m_platform_io.read_signal("CPU_UNCORE_FREQUENCY_MIN_CONTROL", GEOPM_DOMAIN_BOARD, 0);
            m_freq_uncore_max = m_platform_io.read_signal("CPU_UNCORE_FREQUENCY_MAX_CONTROL", GEOPM_DOMAIN_BOARD, 0);

            //TODO: Use ConstConfigIO to set defaults for Fe, Fmax, QM Max Rate, etc
            std::string constconfig_fe_uncore = "CONSTCONFIG::CPU_UNCORE_FREQUENCY_EFFICIENT_HIGH_INTENSITY";
            m_freq_uncore_efficient = NAN;

            if (all_names.count(constconfig_fe_uncore) != 0) {
                m_freq_uncore_efficient = m_platform_io.read_signal(constconfig_fe_uncore,
                                                                    GEOPM_DOMAIN_BOARD, 0);
            }

            if (std::isnan(m_freq_uncore_efficient)) {
                // If not available as a signal estimate it
                m_freq_uncore_efficient = (m_freq_uncore_min + m_freq_uncore_max) / 2;
            }

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
    }

    void ActivityPerformanceModelImp::init_platform_gpu_io(void) {
        auto all_names = m_platform_io.signal_names();

        if (all_names.count("GPU_FREQUENCY_MIN_AVAIL") != 0 &&
            all_names.count("GPU_FREQUENCY_MAX_AVAIL") != 0) {
            m_freq_gpu_min = m_platform_io.read_signal("GPU_FREQUENCY_MIN_AVAIL", GEOPM_DOMAIN_BOARD, 0);
            m_freq_gpu_max = m_platform_io.read_signal("GPU_FREQUENCY_MAX_AVAIL", GEOPM_DOMAIN_BOARD, 0);

            // Gather Fe value
            // TODO: use constconfig efficient freq or sticker
            m_freq_gpu_efficient = NAN;
            std::string constconfig_fe_gpu = "CONSTCONFIG::GPU_CORE_FREQUENCY_EFFICIENT_HIGH_INTENSITY";

            if (std::isnan(m_freq_gpu_efficient)) {
                // TODO: check for F_e signal from level zero
                if (all_names.count("LEVELZERO::GPU_FREQUENCY_EFFICIENT") != 0) {
                    m_freq_gpu_efficient = m_platform_io.read_signal("LEVELZERO::GPU_FREQUENCY_EFFICIENT", GEOPM_DOMAIN_BOARD, 0);
                }
                else {
                    // If not available as a signal estimate it
                    m_freq_gpu_efficient = (m_freq_gpu_min + m_freq_gpu_max) / 2;
                }
            }

            update_uncore_bandwidth_map();

            // GPU Scalability
            std::string gpu_scalability_signal = "GPU_CORE_ACTIVITY";
            if (all_names.count(gpu_scalability_signal) != 0) {
                for (int domain_idx = 0; domain_idx < M_NUM_GPU; ++domain_idx) {
                    m_gpu_scal.push_back({m_platform_io.push_signal(gpu_scalability_signal,
                                                                    GEOPM_DOMAIN_GPU,
                                                                    domain_idx), NAN});
                }

                // If all the above checks have been met we support these controls
                m_supported_controls["GPU_CORE_FREQUENCY_MIN_CONTROL"] = GEOPM_DOMAIN_GPU;
                m_supported_controls["GPU_CORE_FREQUENCY_MAX_CONTROL"] = GEOPM_DOMAIN_GPU;
            }

        }
    }

    bool ActivityPerformanceModelImp::algorithm_valid(void) {
        return m_supported_controls.size() > 0;
    }

    std::map<std::string, int> ActivityPerformanceModelImp::controls_recommended(void) {
        return m_supported_controls;
    }

    void ActivityPerformanceModelImp::update_uncore_bandwidth_map(void) {
        auto all_names = m_platform_io.signal_names();
        // We do not guarantee an ordering or limit to the MBM characterization entries,
        // so we check all characterization entries to see if they are MBM characterization
        for (int entry_idx = 0; entry_idx < (int)all_names.size(); ++entry_idx) {
            std::string key_name = "CONSTCONFIG::CPU_UNCORE_FREQUENCY_" +
                                   std::to_string(entry_idx);
            std::string val_name = "CONSTCONFIG::CPU_UNCORE_MAXIMUM_MEMORY_BANDWIDTH_" +
                                   std::to_string(entry_idx);
            if (all_names.find(key_name) != all_names.end() &&
                all_names.find(val_name) != all_names.end()) {
                double uncore_freq = m_platform_io.read_signal(key_name, GEOPM_DOMAIN_BOARD, 0);
                double max_mem_bw = m_platform_io.read_signal(val_name, GEOPM_DOMAIN_BOARD, 0);
                if (!std::isnan(uncore_freq) && uncore_freq != 0 &&
                    max_mem_bw != 0) {
                    m_max_mem_bw[uncore_freq] = max_mem_bw;
                }
            }
        }
    }

    double ActivityPerformanceModelImp::get_uncore_activity(double uncore_freq,
                                                            double uncore_bandwidth)
                                                           const {
        double uncore_activity = NAN;
        if (m_max_mem_bw.size() != 0) {
            auto bw_max_itr = m_max_mem_bw.lower_bound(uncore_freq);
            if(bw_max_itr != m_max_mem_bw.begin()) {
                bw_max_itr = std::prev(bw_max_itr, 1);
            }

            // Handle divided by zero, either numerator or
            // denominator being NAN, and the un-characterized case
            if (!std::isnan(uncore_bandwidth) &&
                !std::isnan(bw_max_itr->second)) {
                uncore_activity  = (double) uncore_bandwidth /
                                            bw_max_itr->second;
            }
        }

        return uncore_activity;
    }

    std::vector<double> ActivityPerformanceModelImp::sample_recommendation(std::string control_name) const {
        std::vector<double> result = {};

        if (m_recommendation.count(control_name) != 0 &&
            m_supported_controls.count(control_name) != 0) {
            result = m_recommendation.at(control_name);
        }
        return result;
    }

    void ActivityPerformanceModelImp::update_recommendation(double phi) {
        // CORE
        m_recommendation["CPU_FREQUENCY_MAX_CONTROL"] = {};
        if (m_supported_controls.count("CPU_FREQUENCY_MAX_CONTROL") != 0) {
            // Generate per core frequency recommendation
            for (int domain_idx = 0; domain_idx < M_NUM_CORE; ++domain_idx) {
                m_core_scal.at(domain_idx).value = m_platform_io.sample(m_core_scal.at(domain_idx).batch_idx);
                double freq_rec = frequency_fit(m_freq_core_efficient,
                                                m_freq_core_max,
                                                m_core_scal.at(domain_idx).value,
                                                phi);

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
                m_qm_rate.at(domain_idx).value = m_platform_io.sample(m_qm_rate.at(domain_idx).batch_idx);
                m_uncore_freq_status.at(domain_idx).value = m_platform_io.sample(m_uncore_freq_status.at(domain_idx).batch_idx);

                double uncore_scalability = get_uncore_activity(m_qm_rate.at(domain_idx).value,
                                                                m_uncore_freq_status.at(domain_idx).value);

                double freq_rec = frequency_fit(m_freq_uncore_efficient,
                                                m_freq_uncore_max,
                                                uncore_scalability,
                                                phi);

                m_recommendation["CPU_UNCORE_FREQUENCY_MIN_CONTROL"].push_back(freq_rec);
                m_recommendation["CPU_UNCORE_FREQUENCY_MAX_CONTROL"].push_back(freq_rec);
            }
        }
        else {
            for (int domain_idx = 0; domain_idx < M_NUM_PACKAGE; ++domain_idx) {
                m_recommendation["CPU_UNCORE_FREQUENCY_MIN_CONTROL"].push_back(NAN);
                m_recommendation["CPU_UNCORE_FREQUENCY_MAX_CONTROL"].push_back(NAN);
            }
        }

        //GPU
        m_recommendation["GPU_CORE_FREQUENCY_MIN_CONTROL"] = {};
        m_recommendation["GPU_CORE_FREQUENCY_MAX_CONTROL"] = {};
        if (m_supported_controls.count("GPU_CORE_FREQUENCY_MIN_CONTROL") != 0 &&
            m_supported_controls.count("GPU_CORE_FREQUENCY_MAX_CONTROL") != 0) {
                // Generate per GPU frequency recommendation
                for (int domain_idx = 0; domain_idx < M_NUM_GPU; ++domain_idx) {
                    m_gpu_scal.at(domain_idx).value = m_platform_io.sample(m_gpu_scal.at(domain_idx).batch_idx);
                    double freq_rec = frequency_fit(m_freq_gpu_efficient,
                                                    m_freq_gpu_max,
                                                    m_gpu_scal.at(domain_idx).value,
                                                    phi);
                    m_recommendation["GPU_CORE_FREQUENCY_MIN_CONTROL"].push_back(freq_rec);
                    m_recommendation["GPU_CORE_FREQUENCY_MAX_CONTROL"].push_back(freq_rec);
                }
        }
        else {
            for (int domain_idx = 0; domain_idx < M_NUM_GPU; ++domain_idx) {
                m_recommendation["GPU_CORE_FREQUENCY_MIN_CONTROL"].push_back(NAN);
                m_recommendation["GPU_CORE_FREQUENCY_MAX_CONTROL"].push_back(NAN);
            }
        }
    }

    double ActivityPerformanceModelImp::frequency_fit(double f_e, double f_max, double scalability, double phi)
    {
        // If phi is not 0.5 we move into the energy or performance biased behavior
        if (phi > 0.5) {
            // Energy Biased.  Scale F_max down to F_efficient based upon phi value
            // Active region phi usage
            f_max = std::max(f_e, f_max - (f_max - f_e) *
                                  (phi-0.5) / 0.5);
        }
        else if (phi < 0.5) {
            // Perf Biased.  Scale F_efficient up to F_max based upon phi value
            // Active region phi usage
            f_e = std::min(f_max, f_e + (f_max - f_e) *
                                  (0.5-phi) / 0.5);
        }

        double freq_rec = f_max;
        // Core steering signals
        if (std::isnan(scalability)) {
            scalability = 1;
        }

        freq_rec = f_e + (f_max - f_e) * scalability;

        // Request should never be above the per domain f_max
        freq_rec = std::min(f_max, freq_rec);
        // Request should never be below the per domain f_e
        freq_rec = std::max(f_e, freq_rec);

        return freq_rec;
    }
}
