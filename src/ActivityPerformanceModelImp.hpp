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
            void init(void) override;
            virtual bool algorithm_valid(void) override;
            virtual std::map<std::string, int> controls_recommended() override;
            virtual void update_recommendation(double phi) override;
            virtual std::vector<double> sample_recommendation(std::string control_name) const override;

        private:
            PlatformIO &m_platform_io;
            const PlatformTopo &m_platform_topo;

            const int M_NUM_PACKAGE;
            const int M_NUM_CORE;
            const int M_NUM_GPU;
            double m_freq_uncore_min;
            double m_freq_uncore_max;
            double m_freq_core_min;
            double m_freq_core_max;
            double m_freq_core_sticker;
            double m_freq_core_step;
            double m_freq_gpu_min;
            double m_freq_gpu_max;

            struct signal
            {
                int batch_idx;
                double value;
            };

            std::vector<signal> m_core_scal;
            std::vector<signal> m_gpu_scal;

            std::vector<signal> m_qm_rate;
            std::vector<signal> m_uncore_freq_status;

            std::map< std::string, std::vector<double> > m_recommendation;
            std::map<std::string, int> m_supported_controls;

            void init_platform_io(void);
            double frequency_fit(double f_e, double f_max, double scalability, double phi);
            //std::map<double, double> m_max_mem_bw;
    };
}

#endif
