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
    {
        init_platform_io();
    }

    ActivityPerformanceModelImp::~ActivityPerformanceModelImp()
    {

    }

    void ActivityPerformanceModelImp::init_platform_io(void)
    {
        //TODO: setup min & max freq defaults

        //TODO: Use platcharIO to set defaults
    }

    void ActivityPerformanceModelImp::set_frequency_bounds(int freq_domain,
                                                           double min_freq,
                                                           double max_freq) {
        if (freq_domain < M_DOMAIN_SIZE) {
            m_min_frequency[freq_domain] = min_freq;
            m_max_frequency[freq_domain] = max_freq;
        }
    }

    void ActivityPerformanceModelImp::update_uncore_bandwidth_map(std::map<double, double> uncore_max_mem_bw) {
        m_max_mem_bw = uncore_max_mem_bw;
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

    double ActivityPerformanceModelImp::get_frequency_recommendation(int freq_domain, double activity) const {
        double freq_rec = NAN;

        freq_rec = m_min_frequency.at(freq_domain) +
                   (m_max_frequency.at(freq_domain) - m_min_frequency.at(freq_domain)) *
                   activity;

        // Request should never be above the per domain max
        freq_rec = std::min(m_max_frequency.at(freq_domain), freq_rec);
        // Request should never be below the per domain min
        freq_rec = std::max(m_min_frequency.at(freq_domain), freq_rec);

        return freq_rec;
    }
}
