/*
 * Copyright (c) 2015 - 2022, Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef UNCOREACTIVITYPERFORMANCEMODELIMP_HPP_INCLUDE
#define UNCOREACTIVITYPERFORMANCEMODELIMP_HPP_INCLUDE

#include <vector>
#include <map>
#include <memory>

#include "ActivityPerformanceModel.hpp"

namespace geopm
{
    class PlatformTopo;
    class PlatformIO;

    class UncoreActivityPerformanceModelImp : public ActivityPerformanceModel
    {
        public:
            UncoreActivityPerformanceModelImp();
            UncoreActivityPerformanceModelImp(PlatformIO &platform_io, const PlatformTopo &platform_topo);
            virtual ~UncoreActivityPerformanceModelImp();
            void init(void) override;
            bool algorithm_valid(void) override;
            std::map<std::string, int> controls_recommended() override;
            void update_recommendation() override;
            std::vector<double> sample_recommendation(std::string control_name) const override;
//            std::vector<std::string> policy_names(void) const;
            void validate_policy(std::vector<double> &in_policy) const override;
            void apply_policy(std::vector<double> &in_policy) override;
            static std::unique_ptr<ActivityPerformanceModel> make_unique(void);
            static std::shared_ptr<ActivityPerformanceModel> make_shared(void);

        private:
            PlatformIO &m_platform_io;
            const PlatformTopo &m_platform_topo;

            const double M_POLICY_PHI_DEFAULT;
            const int M_NUM_PACKAGE;
            double m_freq_min;
            double m_freq_max;
            double m_freq_efficient;
            double m_max_mem_bw;

            // Policy indices; must match policy_names()
            enum m_policy_e {
                M_POLICY_PHI,
                M_POLICY_FREQ_MAX,
                M_POLICY_FREQ_EFFICIENT,
                M_POLICY_MAX_MEM_BW,
                M_NUM_POLICY
            };

            struct signal
            {
                int batch_idx;
                double value;
            };

            std::vector<signal> m_qm_rate;
            std::vector<signal> m_uncore_freq_status;

            std::map< std::string, std::vector<double> > m_recommendation;
            std::map<std::string, int> m_supported_controls;

            void init_platform_io(void);
            double frequency_fit(double f_e, double f_max, double scalability);
    };
}

#endif
