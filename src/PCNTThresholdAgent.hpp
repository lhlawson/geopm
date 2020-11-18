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

#ifndef PCNTTHRESHOLDAGENT_HPP_INCLUDE
#define PCNTTHRESHOLDAGENT_HPP_INCLUDE

#include <vector>

#include "Agent.hpp"
#include "geopm_time.h"

namespace geopm
{
    class PlatformTopo;
    class PlatformIO;

    /// @brief Agent
    class PCNTThresholdAgent : public Agent
    {
        public:
            PCNTThresholdAgent();
            virtual ~PCNTThresholdAgent() = default;
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
            void init_platform_io(void);

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
                M_POLICY_THRESH_0,
                M_POLICY_THRESH_1,
                M_POLICY_FREQ_SUB_THRESH_0,
                M_POLICY_FREQ_SUB_THRESH_1,
                M_POLICY_FREQ_ABOVE_THRESH_1,
                M_NUM_POLICY
            };

            // Sample indices; must match sample_names()
            enum m_sample_e {
                M_NUM_SAMPLE
            };

            std::map<std::string, double> m_policy_available;

            PlatformIO &m_platform_io;
            const PlatformTopo &m_platform_topo;

            geopm_time_s m_last_wait;
            const double M_WAIT_SEC;

            bool m_do_write_batch;

            double m_frequency_requests;
    };
}

#endif
