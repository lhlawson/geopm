/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, 2020, Intel Corporation
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

#ifndef SCALABILITYREGIONSIGNAL_HPP_INCLUDE
#define SCALABILITYREGIONSIGNAL_HPP_INCLUDE

#include <memory>

#include "Signal.hpp"

namespace geopm
{
    /// A composite signal used by an IOGroup to produce a signal as
    /// the Scalability of two signals.
    class ScalabilityRegionSignal : public Signal
    {
        public:
            ScalabilityRegionSignal(std::shared_ptr<Signal> scalability_sig, std::shared_ptr<Signal> time_sig,
                              double range_upper, double range_lower, double sleep_time);
            ScalabilityRegionSignal(const ScalabilityRegionSignal &other) = delete;
            virtual ~ScalabilityRegionSignal() = default;
            void setup_batch(void) override;
            double sample(void) override;
            double read(void) const override;
        private:
            static double compute_region_time(double scalability,
                                              double curr_time,
                                              double prev_time,
                                              double upper,
                                              double lower);

            std::shared_ptr<Signal> m_scalability;
            std::shared_ptr<Signal> m_time;
            bool m_is_batch_ready;
            double m_range_upper;
            double m_range_lower;
            double m_region_time = 0;
            double m_prev_time = 0;
            double m_sleep_time;
    };
}

#endif
