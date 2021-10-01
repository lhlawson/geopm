/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef BOTTLENECKTRACKERAGENT_HPP_INCLUDE
#define BOTTLENECKTRACKERAGENT_HPP_INCLUDE

#include <vector>

#include "Agent.hpp"
#include "geopm_time.h"

#include "CircularBuffer.hpp"
namespace geopm
{
    class PlatformTopo;
    class PlatformIO;

    /// @brief Agent
    class BottleneckTrackerAgent : public Agent
    {
        public:
            BottleneckTrackerAgent();
            virtual ~BottleneckTrackerAgent() = default;
            void init(int level, const std::vector<int> &fan_in, bool is_level_root) override;
            void validate_policy(std::vector<double> &in_policy) const override;
            void split_policy(const std::vector<double> &in_policy,
                              std::vector<std::vector<double> > &out_policy) override;
            bool do_send_policy(void) const override;
            void aggregate_sample(const std::vector<std::vector<double> > &in_sample,
                                  std::vector<double> &out_sample) override;
            bool do_send_sample(void) const override;
            void adjust_platform(const std::vector<double> &in_policy) override;
            bool do_write_batch(void) const override;
            void sample_platform(std::vector<double> &out_sample) override;
            void wait(void) override;
            std::vector<std::pair<std::string, std::string> > report_header(void) const override;
            std::vector<std::pair<std::string, std::string> > report_host(void) const override;
            std::map<uint64_t, std::vector<std::pair<std::string, std::string> > > report_region(void) const override;
            std::vector<std::string> trace_names(void) const override;
            void trace_values(std::vector<double> &values) override;
            std::vector<std::function<std::string(double)> > trace_formats(void) const override;

            static std::string plugin_name(void);
            static std::unique_ptr<Agent> make_plugin(void);
            static std::vector<std::string> policy_names(void);
            static std::vector<std::string> sample_names(void);
        private:
            PlatformIO &m_platform_io;
            const PlatformTopo &m_platform_topo;
            geopm_time_s m_last_wait;
            const double M_WAIT_SEC;
            bool m_do_write_batch;

            struct signal
            {
                int m_batch_idx;
                double m_last_signal;
                double m_last_sample;
            };

            struct signal_info {
                int domain;
                bool trace_signal;
                std::vector<signal> signals;
            };
            std::map<std::string, signal_info> m_signal_available;

            struct control
            {
                int m_batch_idx;
                double m_last_setting;
            };

            struct control_info {
                int domain;
                bool trace_control;
                std::vector<control> controls;
            };
            std::map<std::string, control_info> m_control_available;

            // Policy indices; must match policy_names()
            enum m_policy_e {
                //M_POLICY_THRESH_0,
                //M_POLICY_THRESH_1,
                //M_POLICY_FREQ_SUB_THRESH_0,
                //M_POLICY_FREQ_SUB_THRESH_1,
                //M_POLICY_FREQ_ABOVE_THRESH_1,
                M_NUM_POLICY
            };

            // Sample indices; must match sample_names()
            enum m_sample_e {
                M_NUM_SAMPLE
            };

            std::map<std::string, double> m_policy_available;

            double m_frequency_requests;
            //double m_license_0_cycles;
            //double m_license_1_cycles;
            //double m_license_2_cycles;
            bool m_pmon_avx;
            bool m_do_per_core;
            bool m_do_c6_res;
            double m_license_0_samples;
            double m_license_1_samples;
            double m_license_2_samples;
            double m_accelerator_frequency_requests;
            double m_accelerator_low_util_samples;
            double m_accelerator_high_util_samples;

            std::vector<std::unique_ptr<CircularBuffer<double> > > m_gpu_utilization;
            std::vector<std::unique_ptr<CircularBuffer<double> > > m_gpu_mem_utilization;
            std::vector<std::unique_ptr<CircularBuffer<double> > > m_gpu_sm_active;
            std::vector<std::unique_ptr<CircularBuffer<double> > > m_ipc;

            double m_perf_margin;

            //System specific info:
            double m_gpu_PN_freq=0.420*1e9;
            double m_gpu_P0_freq=1.530*1e9;
            double m_gpu_mem_freq=0.877*1e9;

            double m_gpu_Fmin_energy=[967, 982, 945, 990];

            double m_gpu_power_fit_a=18455;
            double m_gpu_power_fit_b=-1.001;


            std::map<double,double> m_gpu_freq_deg_map ={{0,1530},{0.00687295, 1522},{0.008117384, 1515},{0.015095745, 1507},{0.020709328, 1500},
                                        {0.030520363, 1492},{0.028942624, 1485},{0.038276053, 1477},{0.03372123, 1470},{0.045311084, 1462},
                                        {0.0435139, 1455},{0.056389961, 1447},{0.053545624, 1440},{0.064988403, 1432},{0.064500297, 1425},
                                        {0.079652262, 1417},{0.07852481, 1410},{0.088125082, 1402},{0.087743119, 1395},{0.099846759, 1387},
                                        {0.098216536, 1380},{0.11155644, 1372},{0.115001947, 1365},{0.121930844, 1357},{0.123704228, 1350},
                                        {0.136824138, 1342},{0.141411752, 1335},{0.149838193, 1327},{0.154983399, 1320},{0.158245636, 1312},
                                        {0.161026303, 1305},{0.17233212, 1297},{0.172936303, 1290},{0.183275896, 1282},{0.187206168, 1275},
                                        {0.202512599, 1267},{0.199926423, 1260},{0.207084073, 1252},{0.214275899, 1245},{0.229599713, 1237},
                                        {0.227851133, 1230},{0.243803328, 1222},{0.243830923, 1215},{0.260499903, 1207},{0.258976592, 1200},
                                        {0.279254784, 1192},{0.274400813, 1185},{0.29261099, 1177},{0.290986406, 1170},{0.309801529, 1162},
                                        {0.305362585, 1155},{0.324926922, 1147},{0.32034274, 1140},{0.340244372, 1132},{0.344264545, 1125},
                                        {0.354911978, 1117},{0.361621995, 1110},{0.373500604, 1102},{0.372846755, 1095},{0.389309487, 1087},
                                        {0.392740312, 1080},{0.414663574, 1072},{0.413069586, 1065},{0.43378076, 1057},{0.432490052, 1050},
                                        {0.454536023, 1042},{0.450694702, 1035},{0.477003919, 1027},{0.472477491, 1020},{0.492623187, 1012},
                                        {0.496018287, 1005},{0.516340596, 997},{0.51768899, 990},{0.538336875, 982},{0.543622441, 975},
                                        {0.562218854, 967},{0.571106525, 960},{0.587608998, 952},{0.596655372, 945},{0.612441343, 937},
                                        {0.626295924, 930},{0.635948104, 922},{0.653683839, 915},{0.662506576, 907},{0.67862561, 900},
                                        {0.688958624, 892},{0.707315633, 885},{0.714120129, 877},
                                       };

            //TODO: reverse.  Currently using sort in the initializer
            //std::vector<double> m_gpu_supported_freqs = {1.530*1e9, 1.522*1e9, 1.515*1e9, 1.507*1e9, 1.500*1e9, 1.492*1e9, 1.485*1e9, 1.477*1e9,
            //                                             1.470*1e9, 1.462*1e9, 1.455*1e9, 1.447*1e9, 1.440*1e9, 1.432*1e9, 1.425*1e9, 1.417*1e9,
            //                                             1.410*1e9, 1.402*1e9, 1.395*1e9, 1.387*1e9, 1.380*1e9, 1.372*1e9, 1.365*1e9, 1.357*1e9,
            //                                             1.350*1e9, 1.342*1e9, 1.335*1e9, 1.327*1e9, 1.320*1e9, 1.312*1e9, };
            std::vector<double> m_gpu_supported_freqs = {1.530*1e9, 1.522*1e9, 1.515*1e9, 1.507*1e9, 1.500*1e9, 1.492*1e9, 1.485*1e9, 1.477*1e9,
                                                         1.470*1e9, 1.462*1e9, 1.455*1e9, 1.447*1e9, 1.440*1e9, 1.432*1e9, 1.425*1e9, 1.417*1e9,
                                                         1.410*1e9, 1.402*1e9, 1.395*1e9, 1.387*1e9, 1.380*1e9, 1.372*1e9, 1.365*1e9, 1.357*1e9,
                                                         1.350*1e9, 1.342*1e9, 1.335*1e9, 1.327*1e9, 1.320*1e9, 1.312*1e9, 1.305*1e9, 1.297*1e9,
                                                         1.290*1e9, 1.282*1e9, 1.275*1e9, 1.267*1e9, 1.260*1e9, 1.252*1e9, 1.245*1e9, 1.237*1e9,
                                                         1.230*1e9, 1.222*1e9, 1.215*1e9, 1.207*1e9, 1.200*1e9, 1.192*1e9, 1.185*1e9, 1.177*1e9,
                                                         1.170*1e9, 1.162*1e9, 1.155*1e9, 1.147*1e9, 1.140*1e9, 1.132*1e9, 1.125*1e9, 1.117*1e9,
                                                         1.110*1e9, 1.102*1e9, 1.095*1e9, 1.087*1e9, 1.080*1e9, 1.072*1e9, 1.065*1e9, 1.057*1e9,
                                                         1.050*1e9, 1.042*1e9, 1.035*1e9, 1.027*1e9, 1.020*1e9, 1.012*1e9, 1.005*1e9, 0.997*1e9,
                                                         0.990*1e9, 0.982*1e9, 0.975*1e9, 0.967*1e9, 0.960*1e9, 0.952*1e9, 0.945*1e9, 0.937*1e9,
                                                         0.930*1e9, 0.922*1e9, 0.915*1e9, 0.907*1e9, 0.900*1e9, 0.892*1e9, 0.885*1e9, 0.877*1e9,};
                                                         //0.870*1e9, 0.862*1e9,
                                                         //0.855*1e9, 0.847*1e9, 0.840*1e9, 0.832*1e9, 0.825*1e9, 0.817*1e9, 0.810*1e9, 0.802*1e9, 0.795*1e9,
                                                         //0.787*1e9, 0.780*1e9, 0.772*1e9, 0.765*1e9, 0.757*1e9, 0.750*1e9, 0.742*1e9, 0.735*1e9, 0.727*1e9,
                                                         //0.720*1e9, 0.712*1e9, 0.705*1e9, 0.697*1e9, 0.690*1e9, 0.682*1e9, 0.675*1e9, 0.667*1e9, 0.660*1e9,
                                                         //0.652*1e9, 0.645*1e9, 0.637*1e9, 0.630*1e9, 0.622*1e9, 0.615*1e9, 0.607*1e9, 0.600*1e9, 0.592*1e9,
                                                         //0.585*1e9, 0.577*1e9, 0.570*1e9, 0.562*1e9, 0.555*1e9, 0.547*1e9, 0.540*1e9, 0.532*1e9, 0.525*1e9,
                                                         //0.517*1e9, 0.510*1e9, 0.502*1e9, 0.495*1e9, 0.487*1e9, 0.480*1e9, 0.472*1e9, 0.465*1e9, 0.457*1e9,
                                                         //0.450*1e9, 0.442*1e9, 0.435*1e9, 0.427*1e9, 0.420*1e9};
            // I cut out the bottom range based on P vs F curve
            // 412, 405, 397, 390, 382, 375, 367, 360, 352, 345, 337, 330, 322, 315, 307, 300, 292, 285, 277, 270, 262, 255, 247, 240, 232, 225, 217, 210, 202, 195, 187, 180, 172, 165, 157, 150, 142, 135

            //MAX IPC
            double max_ipc = 5; //5?  https://stackoverflow.com/questions/37041009/what-is-the-maximum-possible-ipc-can-be-achieved-by-intel-nehalem-microarchitect
            double max_cores = 20;

            //Max mem bw
            //m_max_mem_bw = 64*1e9; 4GB/s

            //AVX Level Indexed frequencies
            std::vector<double> m_freq_sticker = {2.4*1e9,
                                                  1.9*1e9,
                                                  1.6*1e9};
            std::vector<std::vector<double> > m_freq_p0x = {{3.7*1e9, 3.7*1e9, 3.5*1e9, 3.5*1e9, 3.4*1e9, 3.4*1e9, 3.4*1e9,
                                                             3.4*1e9, 3.4*1e9, 3.4*1e9, 3.4*1e9, 3.4*1e9, 3.3*1e9, 3.3*1e9,
                                                             3.3*1e9, 3.3*1e9, 3.1*1e9, 3.1*1e9, 3.1*1e9, 3.1*1e9}
                                                           ,{3.6*1e9, 3.6*1e9, 3.4*1e9, 3.4*1e9, 3.3*1e9, 3.3*1e9, 3.3*1e9,
                                                             3.3*1e9, 3.1*1e9, 3.1*1e9, 3.1*1e9, 3.1*1e9, 2.8*1e9, 2.8*1e9,
                                                             2.8*1e9, 2.8*1e9, 2.6*1e9, 2.6*1e9, 2.6*1e9, 2.6*1e9}
                                                           ,{3.5*1e9, 3.5*1e9, 3.3*1e9, 3.3*1e9, 3.1*1e9, 3.1*1e9, 3.1*1e9,
                                                             3.1*1e9, 2.6*1e9, 2.6*1e9, 2.6*1e9, 2.6*1e9, 2.3*1e9, 2.3*1e9,
                                                             2.3*1e9, 2.3*1e9, 2.2*1e9, 2.2*1e9, 2.2*1e9, 2.2*1e9}};

            void init_platform_io(void);
    };
}

#endif
