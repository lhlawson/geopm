/*
 * Copyright (c) 2015 - 2022, Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "CPUActivityPerformanceModelImp.hpp"

#include <cmath>
#include <unistd.h>
#include <iostream>

#include "PlatformIOProf.hpp"
#include "geopm/Exception.hpp"
#include "geopm/Helper.hpp"
#include "geopm_debug.hpp"
#include "geopm/PlatformIO.hpp"
#include "geopm/PlatformTopo.hpp"
#include "config.h"
#include "geopm/Helper.hpp"

namespace geopm
{
    std::unique_ptr<ActivityPerformanceModel> CPUActivityPerformanceModelImp::make_unique(void)
    {
        return geopm::make_unique<CPUActivityPerformanceModelImp>();
    }

    std::shared_ptr<ActivityPerformanceModel> CPUActivityPerformanceModelImp::make_shared(void)
    {
        return std::make_shared<CPUActivityPerformanceModelImp>();
    }

    CPUActivityPerformanceModelImp::CPUActivityPerformanceModelImp()
        : CPUActivityPerformanceModelImp(PlatformIOProf::platform_io(), platform_topo())
    {
    }

    CPUActivityPerformanceModelImp::CPUActivityPerformanceModelImp(PlatformIO &platform_io, const PlatformTopo &platform_topo)
        : m_platform_io(platform_io)
        , m_platform_topo(platform_topo)
        , M_POLICY_PHI_DEFAULT(0.5)
        , M_NUM_CORE(m_platform_topo.num_domain(GEOPM_DOMAIN_CORE))
    {
    }

    CPUActivityPerformanceModelImp::~CPUActivityPerformanceModelImp()
    {
    }

    void CPUActivityPerformanceModelImp::init(void)
    {
        m_supported_controls = {};
        m_recommendation = {};

        init_platform_core_io();
    }

    void CPUActivityPerformanceModelImp::init_platform_core_io(void) {
        // push back signals of interest
        auto all_names = m_platform_io.signal_names();

        // Setup Core Algorithm Signals
        if (all_names.count("CPU_FREQUENCY_MIN_AVAIL") != 0 &&
            all_names.count("CPU_FREQUENCY_MAX_AVAIL") != 0 &&
            all_names.count("CPU_FREQUENCY_STICKER") != 0 &&
            all_names.count("CPU_FREQUENCY_STEP") != 0 ) {

            m_freq_min = m_platform_io.read_signal("CPU_FREQUENCY_MIN_AVAIL", GEOPM_DOMAIN_BOARD, 0);
            m_freq_max = m_platform_io.read_signal("CPU_FREQUENCY_MAX_AVAIL", GEOPM_DOMAIN_BOARD, 0);
            m_freq_sticker = m_platform_io.read_signal("CPU_FREQUENCY_STICKER", GEOPM_DOMAIN_BOARD, 0);
            m_freq_step = m_platform_io.read_signal("CPU_FREQUENCY_STEP", GEOPM_DOMAIN_BOARD, 0);

            // Gather Fe value
            //use constconfig efficient freq or sticker
            m_freq_efficient = NAN;
            std::string constconfig_fe_core = "CONSTCONFIG::CPU_CORE_FREQUENCY_EFFICIENT_HIGH_INTENSITY";
            if (all_names.count(constconfig_fe_core) != 0) {
                m_freq_efficient = m_platform_io.read_signal(constconfig_fe_core,
                                                                  GEOPM_DOMAIN_BOARD, 0);
            }

            if (std::isnan(m_freq_efficient)) {
                m_freq_efficient = m_freq_min;

                if (! std::isnan(m_freq_sticker) &&
                    ! std::isnan(m_freq_step)) {
                    // Sticker - 2 steps is generally energy efficient
                    m_freq_efficient = m_freq_sticker - m_freq_step * 2;
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

    bool CPUActivityPerformanceModelImp::algorithm_valid(void) {
        return m_supported_controls.size() > 0;
    }

    std::map<std::string, int> CPUActivityPerformanceModelImp::controls_recommended(void) {
        return m_supported_controls;
    }

    std::vector<double> CPUActivityPerformanceModelImp::sample_recommendation(std::string control_name) const {
        std::vector<double> result = {};

        if (m_recommendation.count(control_name) != 0 &&
            m_supported_controls.count(control_name) != 0) {
            result = m_recommendation.at(control_name);
        }
        return result;
    }

    void CPUActivityPerformanceModelImp::update_recommendation() {
        m_recommendation["CPU_FREQUENCY_MAX_CONTROL"] = {};
        if (m_supported_controls.count("CPU_FREQUENCY_MAX_CONTROL") != 0) {
            // Generate per core frequency recommendation
            for (int domain_idx = 0; domain_idx < M_NUM_CORE; ++domain_idx) {
                m_core_scal.at(domain_idx).value = m_platform_io.sample(m_core_scal.at(domain_idx).batch_idx);
                double freq_rec = frequency_fit(m_freq_efficient,
                                                m_freq_max,
                                                m_core_scal.at(domain_idx).value);

                m_recommendation["CPU_FREQUENCY_MAX_CONTROL"].push_back(freq_rec);
            }
        }
        else {
            for (int domain_idx = 0; domain_idx < M_NUM_CORE; ++domain_idx) {
                m_recommendation["CPU_FREQUENCY_MAX_CONTROL"].push_back(NAN);
            }
        }
    }

    // Describes expected policies to be provided by the resource manager or user
//    std::vector<std::string> CPUActivityPerformanceModelImp::policy_names(void) const
//    {
//        std::vector<std::string> names{"PHI", "FREQ_MAX", "FREQ_EFFICIENT"};
//        return names;
//    }

    void CPUActivityPerformanceModelImp::validate_policy(std::vector<double> &in_policy) const
    {
        GEOPM_DEBUG_ASSERT(in_policy.size() == M_NUM_POLICY,
                           "CPUActivityPerfModel::" + std::string(__func__) +
                           "(): policy vector not correctly sized.  Expected  " +
                           std::to_string(M_NUM_POLICY) + ", actual: " +
                           std::to_string(in_policy.size()));

        // If no phi value is provided assume the default behavior.
        if (std::isnan(in_policy[M_POLICY_PHI])) {
            in_policy[M_POLICY_PHI] = M_POLICY_PHI_DEFAULT;
        }

        if (in_policy[M_POLICY_PHI] < 0.0 ||
            in_policy[M_POLICY_PHI] > 1.0) {
            throw Exception("CPUActivityPerformanceModel::" + std::string(__func__) +
                            "(): POLICY_PHI value out of range: " +
                            std::to_string(in_policy[M_POLICY_PHI]) + ".",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        // Check for NAN to set default values for policy
        if (std::isnan(in_policy[M_POLICY_FREQ_MAX])) {
            in_policy[M_POLICY_FREQ_MAX] = m_freq_max;
        }

        if (in_policy[M_POLICY_FREQ_MAX] > m_freq_max ||
            in_policy[M_POLICY_FREQ_MAX] < m_freq_min ) {
            throw Exception("CPUActivityPerformanceModel::" + std::string(__func__) +
                            "():FREQ_MAX out of range: " +
                            std::to_string(in_policy[M_POLICY_FREQ_MAX]) +
                            ".", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        // Check for NAN to set default values for policy
        if (!std::isnan(in_policy[M_POLICY_FREQ_EFFICIENT])) {
            in_policy[M_POLICY_FREQ_EFFICIENT] = m_freq_efficient;
        }

        if (in_policy[M_POLICY_FREQ_EFFICIENT] > m_freq_max ||
            in_policy[M_POLICY_FREQ_EFFICIENT] < m_freq_min ) {
            throw Exception("CPUActivityPerformanceModel::" + std::string(__func__) +
                            "():FREQ_EFFICIENT out of range: " +
                            std::to_string(in_policy[M_POLICY_FREQ_EFFICIENT]) +
                            ".", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        if (in_policy[M_POLICY_FREQ_EFFICIENT] > in_policy[M_POLICY_FREQ_MAX]) {
            throw Exception("CPUActivityPerformanceModel::" + std::string(__func__) +
                            "():FREQ_EFFICIENT (" +
                            std::to_string(in_policy[M_POLICY_FREQ_EFFICIENT]) +
                            ") value exceeds FREQ_MAX (" +
                            std::to_string(in_policy[M_POLICY_FREQ_MAX]) +
                            ").", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        double f_core_max = in_policy[M_POLICY_FREQ_MAX];
        double f_core_efficient = in_policy[M_POLICY_FREQ_EFFICIENT];
        double f_core_range = f_core_max - f_core_efficient;

        double phi = in_policy[M_POLICY_PHI];

        // If phi is not 0.5 we move into the energy or performance biased behavior
        if (phi > 0.5) {
            // Energy Biased.  Scale F_max down to F_efficient based upon phi value
            // Active region phi usage
            f_core_max = std::max(f_core_efficient, f_core_max -
                                                    f_core_range * (phi-0.5) / 0.5);
        }
        else if (phi < 0.5) {
            // Perf Biased.  Scale F_efficient up to F_max based upon phi value
            // Active region phi usage
            f_core_efficient = std::min(f_core_max, f_core_efficient +
                                                    f_core_range * (0.5-phi) / 0.5);
        }

        //Update Policy
        in_policy[M_POLICY_FREQ_MAX] = f_core_max;
        in_policy[M_POLICY_FREQ_EFFICIENT] = f_core_efficient;
    }

    void CPUActivityPerformanceModelImp::apply_policy(std::vector<double> &in_policy)
    {
        GEOPM_DEBUG_ASSERT(in_policy.size() == M_NUM_POLICY,
                           "CPUActivityPerfModel::" + std::string(__func__) +
                           "(): policy vector not correctly sized.  Expected  " +
                           std::to_string(M_NUM_POLICY) + ", actual: " +
                           std::to_string(in_policy.size()));

        m_freq_max = in_policy[M_POLICY_FREQ_MAX];
        m_freq_efficient = in_policy[M_POLICY_FREQ_EFFICIENT];
    }

    double CPUActivityPerformanceModelImp::frequency_fit(double f_e, double f_max, double scalability)
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
