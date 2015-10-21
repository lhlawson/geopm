/*
 * Copyright (c) 2015, Intel Corporation
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

#include "Phase.hpp"

namespace geopm
{
    Phase::Phase(const std::string &name, long identifier, int hint)
        : m_name(name),
          m_identifier(identifier),
          m_hint(hint) {}
    Phase::~Phase() {}

    void Phase::observation_insert(int index, double value)
    {
        m_obs.insert(index, value);
    }

    long Phase::identifier(void) const
    {
        return m_identifier;
    }

    void Phase::name(std::string &name) const
    {
        name = m_name;
    }

    int Phase::hint(void) const
    {
        return m_hint;
    }

    void Phase::policy(Policy* policy)
    {
        m_last_policy = m_policy;
        m_policy = *policy;
    }

    Policy* Phase::policy(void)
    {
        return &m_policy;
    }

    Policy* Phase::last_policy(void)
    {
        return &m_last_policy;
    }

    double Phase::observation_mean(int buffer_index) const
    {
        return m_obs.mean(buffer_index);
    }

    double Phase::observation_median(int buffer_index) const
    {
        return m_obs.median(buffer_index);
    }

    double Phase::observation_stddev(int buffer_index) const
    {
        return m_obs.stddev(buffer_index);
    }

    double Phase::observation_max(int buffer_index) const
    {
        return m_obs.max(buffer_index);
    }

    double Phase::observation_min(int buffer_index) const
    {
        return m_obs.min(buffer_index);
    }

    double Phase::observation_integrate_time(int buffer_index) const
    {
        return m_obs.integrate_time(buffer_index);
    }
}
