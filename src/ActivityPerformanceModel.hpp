/*
 * Copyright (c) 2015 - 2022, Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef ACTIVITYPERFORMANCEMODEL_HPP_INCLUDE
#define ACTIVITYPERFORMANCEMODEL_HPP_INCLUDE

namespace geopm
{
    class PlatformTopo;
    class PlatformIO;

    class ActivityPerformanceModel
    {
        public:
            ActivityPerformanceModel() = default;
            virtual ~ActivityPerformanceModel() = default;
            /// @brief Query the system for signals & controls at the domains
            /// required for the algorithm
            virtual void init(void) = 0;

            /// @brief Informs user if algorithm is valid/functioning (i.e. if all signals needed are present)
            /// @return Boolean indicating if the algorithm is functioning as expected
            virtual bool algorithm_valid() = 0;

            /// @brief Provides calling classes a map of controls algorithm will
            ///        provide settings for and the GEOPM control domain
            virtual std::map<std::string, int> controls_recommended() = 0;

            /// @brief Queries all signals of interest and updates the recommendations for controls
            virtual void update_recommendation() = 0;

            /// @brief Provides recommendations for control provided
            /// @returns recommended value for specified control
            virtual std::vector<double> sample_recommendation(std::string control_name) const = 0;

            /// @brief Validates the policy provided to the performance model.
            ///        Policy sender can request default value with 'NaN'.
            virtual void validate_policy(std::vector<double> &in_policy) const = 0;

            /// @brief
            virtual void apply_policy(std::vector<double> &in_policy) = 0;
    };
}

#endif
