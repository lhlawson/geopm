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

#include "config.h"

#include "DivisionSignal.hpp"

#include "Exception.hpp"
#include "Helper.hpp"
#include "geopm_debug.hpp"

namespace geopm
{
    DivisionSignal::DivisionSignal(std::shared_ptr<Signal> numerator,
                                       std::shared_ptr<Signal> denominator)
        : m_numerator(numerator)
        , m_denominator(denominator)
        , m_is_batch_ready(false)
    {
        GEOPM_DEBUG_ASSERT(m_numerator && m_denominator,
                           "Signal pointers for numerator and denominator cannot be null.");
    }

    void DivisionSignal::setup_batch(void)
    {
        if (!m_is_batch_ready) {
            m_numerator->setup_batch();
            m_denominator->setup_batch();
            m_is_batch_ready = true;
        }
    }

    double DivisionSignal::sample(void)
    {
        if (!m_is_batch_ready) {
            throw Exception("setup_batch() must be called before sample().",
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }

        double result = 0;
        double numer = m_numerator->sample();
        double denom = m_denominator->sample();

        if (denom != 0) {
            result = numer / denom;
        }
        return result;
    }

    double DivisionSignal::read(void) const
    {
        double result = 0;
        double numer = m_numerator->read();
        double denom = m_denominator->read();

        if (denom != 0) {
            result = numer / denom;
        }
        return result;
    }
}
