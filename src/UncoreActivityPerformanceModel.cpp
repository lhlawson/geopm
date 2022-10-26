/*
 * Copyright (c) 2015 - 2022, Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "UncoreActivityPerformanceModelImp.hpp"

#include <cmath>
#include <unistd.h>

#include "PlatformIOProf.hpp"
#include "geopm/Exception.hpp"
#include "geopm/Helper.hpp"
#include "geopm_debug.hpp"
#include "geopm/PlatformIO.hpp"
#include "geopm/PlatformTopo.hpp"
#include "config.h"

namespace geopm
{
    //TODO: require domain as part of constructor
    ActivityPerformanceModel &uncore_activity_perf_model()
    {
        static UncoreActivityPerformanceModelImp instance;
        return instance;
    }

    UncoreActivityPerformanceModelImp::UncoreActivityPerformanceModelImp()
        : UncoreActivityPerformanceModelImp(PlatformIOProf::platform_io(), platform_topo())
    {

    }

    UncoreActivityPerformanceModelImp::UncoreActivityPerformanceModelImp(PlatformIO &platform_io, const PlatformTopo &platform_topo)
        : m_platform_io(platform_io)
        , m_platform_topo(platform_topo)
        , M_POLICY_PHI_DEFAULT(0.5)
        , M_NUM_PACKAGE(m_platform_topo.num_domain(GEOPM_DOMAIN_PACKAGE))
    {
    }

    UncoreActivityPerformanceModelImp::~UncoreActivityPerformanceModelImp()
    {
    }

    void UncoreActivityPerformanceModelImp::init(void)
    {
        m_supported_controls = {};
        m_recommendation = {};

        init_platform_io();
    }

    void UncoreActivityPerformanceModelImp::init_platform_io(void) {
        auto all_names = m_platform_io.signal_names();

        // Setup Uncore Algorithm Signals
        if (all_names.count("CPU_UNCORE_FREQUENCY_MIN_CONTROL") != 0 &&
            all_names.count("CPU_UNCORE_FREQUENCY_MAX_CONTROL") != 0) {
            m_freq_min = m_platform_io.read_signal("CPU_UNCORE_FREQUENCY_MIN_CONTROL", GEOPM_DOMAIN_BOARD, 0);
            m_freq_max = m_platform_io.read_signal("CPU_UNCORE_FREQUENCY_MAX_CONTROL", GEOPM_DOMAIN_BOARD, 0);

            //TODO: Use ConstConfigIO to set defaults for Fe, Fmax, QM Max Rate, etc
            std::string constconfig_fe_uncore = "CONSTCONFIG::CPU_UNCORE_FREQUENCY_EFFICIENT_HIGH_INTENSITY";
            m_freq_efficient = NAN;

            if (all_names.count(constconfig_fe_uncore) != 0) {
                m_freq_efficient = m_platform_io.read_signal(constconfig_fe_uncore,
                                                                    GEOPM_DOMAIN_BOARD, 0);
            }

            if (std::isnan(m_freq_efficient)) {
                // If not available as a signal estimate it
                m_freq_efficient = (m_freq_min + m_freq_max) / 2;
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

    bool UncoreActivityPerformanceModelImp::algorithm_valid(void) {
        return m_supported_controls.size() > 0;
    }

    std::map<std::string, int> UncoreActivityPerformanceModelImp::controls_recommended(void) {
        return m_supported_controls;
    }

    std::vector<double> UncoreActivityPerformanceModelImp::sample_recommendation(std::string control_name) const {
        std::vector<double> result = {};

        if (m_recommendation.count(control_name) != 0 &&
            m_supported_controls.count(control_name) != 0) {
            result = m_recommendation.at(control_name);
        }
        return result;
    }

    void UncoreActivityPerformanceModelImp::update_recommendation() {
        // UNCORE
        m_recommendation["CPU_UNCORE_FREQUENCY_MIN_CONTROL"] = {};
        m_recommendation["CPU_UNCORE_FREQUENCY_MAX_CONTROL"] = {};
        if (m_supported_controls.count("CPU_UNCORE_FREQUENCY_MIN_CONTROL") != 0 &&
            m_supported_controls.count("CPU_UNCORE_FREQUENCY_MAX_CONTROL") != 0) {
            for (int domain_idx = 0; domain_idx < M_NUM_PACKAGE; ++domain_idx) {
                m_qm_rate.at(domain_idx).value = m_platform_io.sample(m_qm_rate.at(domain_idx).batch_idx);
                m_uncore_freq_status.at(domain_idx).value = m_platform_io.sample(m_uncore_freq_status.at(domain_idx).batch_idx);

                double uncore_scalability = m_qm_rate.at(domain_idx).value / m_max_mem_bw;

                double freq_rec = frequency_fit(m_freq_efficient,
                                                m_freq_max,
                                                uncore_scalability);

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
    }

    // Describes expected policies to be provided by the resource manager or user
//    std::vector<std::string> UncoreActivityPerformanceModelImp::policy_names(void) const
//    {
//        std::vector<std::string> names{"PHI", "FREQ_MAX", "FREQ_EFFICIENT", "MAX_MEM_BW"};
//        return names;
//    }

    void UncoreActivityPerformanceModelImp::validate_policy(std::vector<double> &in_policy) const
    {
        GEOPM_DEBUG_ASSERT(in_policy.size() == M_NUM_POLICY,
                           "UncoreActivityPerfModel::" + std::string(__func__) +
                           "(): policy vector not correctly sized.  Expected  " +
                           std::to_string(M_NUM_POLICY) + ", actual: " +
                           std::to_string(in_policy.size()));

        // If no phi value is provided assume the default behavior.
        if (std::isnan(in_policy[M_POLICY_PHI])) {
            in_policy[M_POLICY_PHI] = M_POLICY_PHI_DEFAULT;
        }

        if (std::isnan(in_policy[M_POLICY_MAX_MEM_BW]) ||
            in_policy[M_POLICY_MAX_MEM_BW] == 0) {
            throw Exception("UncoreActivityPerformanceModel::" + std::string(__func__) +
                            "(): Provided Maximum memory bandwidth is invalid (NAN or 0).",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        // Check for NAN to set default values for policy
        if (std::isnan(in_policy[M_POLICY_FREQ_MAX])) {
            in_policy[M_POLICY_FREQ_MAX] = m_freq_max;
        }

        if (in_policy[M_POLICY_FREQ_MAX] > m_freq_max ||
            in_policy[M_POLICY_FREQ_MAX] < m_freq_min ) {
            throw Exception("UncoreActivityPerformanceModel::" + std::string(__func__) +
                            "():FREQ_MAX out of range: " +
                            std::to_string(in_policy[M_POLICY_FREQ_MAX]) +
                            ".", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        // Check for NAN to set default values for policy
        if (!std::isnan(in_policy[M_POLICY_FREQ_EFFICIENT])) {
            in_policy[M_POLICY_FREQ_EFFICIENT] = m_freq_min;
        }

        if (in_policy[M_POLICY_FREQ_EFFICIENT] > m_freq_max ||
            in_policy[M_POLICY_FREQ_EFFICIENT] < m_freq_min ) {
            throw Exception("UncoreActivityPerformanceModel::" + std::string(__func__) +
                            "():FREQ_EFFICIENT out of range: " +
                            std::to_string(in_policy[M_POLICY_FREQ_EFFICIENT]) +
                            ".", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        if (in_policy[M_POLICY_FREQ_EFFICIENT] > in_policy[M_POLICY_FREQ_MAX]) {
            throw Exception("UncoreActivityPerformanceModel::" + std::string(__func__) +
                            "():FREQ_EFFICIENT (" +
                            std::to_string(in_policy[M_POLICY_FREQ_EFFICIENT]) +
                            ") value exceeds FREQ_MAX (" +
                            std::to_string(in_policy[M_POLICY_FREQ_MAX]) +
                            ").", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        double f_max = in_policy[M_POLICY_FREQ_MAX];
        double f_efficient = in_policy[M_POLICY_FREQ_EFFICIENT];
        double f_range = f_max - f_efficient;

        double phi = in_policy[M_POLICY_PHI];

        // If phi is not 0.5 we move into the energy or performance biased behavior
        if (phi > 0.5) {
            // Energy Biased.  Scale F_max down to F_efficient based upon phi value
            // Active region phi usage
            f_max = std::max(f_efficient, f_max - f_range * (phi-0.5) / 0.5);
        }
        else if (phi < 0.5) {
            // Perf Biased.  Scale F_efficient up to F_max based upon phi value
            // Active region phi usage
            f_efficient = std::min(f_max, f_efficient + f_range * (0.5-phi) / 0.5);
        }

        //Update Policy
        in_policy[M_POLICY_FREQ_MAX] = f_max;
        in_policy[M_POLICY_FREQ_EFFICIENT] = f_efficient;
    }

    void UncoreActivityPerformanceModelImp::set_policy(std::vector<double> &in_policy)
    {
        GEOPM_DEBUG_ASSERT(in_policy.size() == M_NUM_POLICY,
                           "CPUActivityPerfModel::" + std::string(__func__) +
                           "(): policy vector not correctly sized.  Expected  " +
                           std::to_string(M_NUM_POLICY) + ", actual: " +
                           std::to_string(in_policy.size()));

        m_freq_max = in_policy[M_POLICY_FREQ_MAX];
        m_freq_efficient = in_policy[M_POLICY_FREQ_EFFICIENT];
        m_max_mem_bw = in_policy[M_POLICY_MAX_MEM_BW];
    }

    double UncoreActivityPerformanceModelImp::frequency_fit(double f_e, double f_max, double scalability)
    {
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
