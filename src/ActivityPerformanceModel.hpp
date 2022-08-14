/*
 * Copyright (c) 2015 - 2022, Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef ACTIVITYPERFORMANCEMODEL_HPP_INCLUDE
#define ACTIVITYPERFORMANCEMODEL_HPP_INCLUDE

#include <memory>

namespace geopm
{
    class PlatformTopo;
    class PlatformIO;

    class ActivityPerformanceModel
    {
        public:
            enum geopm_perf_domain_e {
                M_DOMAIN_CPU_CORE = 0,
                M_DOMAIN_CPU_UNCORE = 1,
                M_DOMAIN_GPU_CORE = 3,
                M_DOMAIN_SIZE = 4
            };

            ActivityPerformanceModel() = default;
            virtual ~ActivityPerformanceModel() = default;
            /// @brief Registers signals and controls with PlatformIO
            virtual void init_platform_io(void) = 0;

            virtual void set_frequency_bounds(int freq_domain, double min_freq,
                                              double max_freq) = 0;

            virtual void update_uncore_bandwidth_map(std::map<double, double> uncore_max_mem_bw) = 0;

            virtual double get_uncore_activity(double uncore_freq,
                                               double uncore_bandwidth) const = 0;

            virtual double get_frequency_recommendation(int freq_domain,
                                                        double activity) const = 0;
    };

    ActivityPerformanceModel &activity_perf_model();
}

#endif
