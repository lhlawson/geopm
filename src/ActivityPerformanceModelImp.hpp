/*
 * Copyright (c) 2015 - 2022, Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef ACTIVITYPERFORMANCEMODELIMP_HPP_INCLUDE
#define ACTIVITYPERFORMANCEMODELIMP_HPP_INCLUDE

#include <vector>
#include <map>

#include "ActivityPerformanceModel.hpp"

namespace geopm
{
    class PlatformTopo;
    class PlatformIO;

    class ActivityPerformanceModelImp : public ActivityPerformanceModel
    {
        public:
            ActivityPerformanceModelImp();
            ActivityPerformanceModelImp(PlatformIO &platform_io, const PlatformTopo &platform_topo);
            virtual ~ActivityPerformanceModelImp();
            void init_platform_io(void) override;
            void set_frequency_bounds(int freq_domain, double min_freq,
                                      double max_freq) override;

            void update_uncore_bandwidth_map(std::map<double, double> uncore_max_mem_bw) override;

            double get_uncore_activity(double uncore_freq,
                                       double uncore_bandwidth) const override;

            double get_frequency_recommendation(int freq_domain,
                                                double activity) const override;

        private:
            PlatformIO &m_platform_io;
            const PlatformTopo &m_platform_topo;

            std::map<int, double> m_min_frequency;
            std::map<int, double> m_max_frequency;
            std::map<double, double> m_max_mem_bw;
    };
}

#endif
