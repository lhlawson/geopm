/*
 * Copyright (c) 2015 - 2021, Intel Corporation
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

#ifndef MOCKNVMLDEVICEPOOL_HPP_INCLUDE
#define MOCKNVMLDEVICEPOOL_HPP_INCLUDE

#include "gmock/gmock.h"

#include "LevelZeroDevicePool.hpp"

class MockLevelZeroDevicePool : public geopm::LevelZeroDevicePool
{
    public:
        MOCK_CONST_METHOD0(num_accelerator,
                           int(void));
        MOCK_CONST_METHOD1(frequency_gpu_status,
                           double(unsigned int));
        MOCK_CONST_METHOD1(frequency_mem_status,
                           double(unsigned int));
        MOCK_CONST_METHOD1(core_clock_rate,
                           double(unsigned int));
        MOCK_CONST_METHOD1(frequency_gpu_min,
                           double(unsigned int));
        MOCK_CONST_METHOD1(frequency_gpu_max,
                           double(unsigned int));
        MOCK_CONST_METHOD1(frequency_mem_min,
                           double(unsigned int));
        MOCK_CONST_METHOD1(frequency_mem_max,
                           double(unsigned int));
        MOCK_CONST_METHOD1(frequency_gpu_range_min,
                           double(unsigned int));
        MOCK_CONST_METHOD1(frequency_gpu_range_max,
                           double(unsigned int));
        MOCK_CONST_METHOD1(utilization,
                           double(unsigned int));
        MOCK_CONST_METHOD3(active_time,
                           int(unsigned int,
                           std::vector<uint64_t>&,
                           std::vector<uint64_t>&));
        MOCK_CONST_METHOD3(energy,
                           int(unsigned int,
                           std::vector<uint64_t>&,
                           std::vector<uint64_t>&));
        MOCK_CONST_METHOD1(temperature,
                           double(unsigned int));
        MOCK_CONST_METHOD1(temperature_gpu,
                           double(unsigned int));
        MOCK_CONST_METHOD1(temperature_memory,
                           double(unsigned int));
        MOCK_CONST_METHOD1(utilization_compute,
                           double(unsigned int));
        MOCK_CONST_METHOD1(utilization_copy,
                           double(unsigned int));
        MOCK_CONST_METHOD1(utilization_media_decode,
                           double(unsigned int));
        MOCK_CONST_METHOD1(power,
                           double(unsigned int));
        MOCK_CONST_METHOD1(power_tdp,
                           int32_t(unsigned int));
        MOCK_CONST_METHOD1(power_limit_min,
                           int32_t(unsigned int));
        MOCK_CONST_METHOD1(power_limit_max,
                           int32_t(unsigned int));
        MOCK_CONST_METHOD1(power_limit_sustained_enabled,
                           bool(unsigned int));
        MOCK_CONST_METHOD1(power_limit_sustained_power,
                           int32_t(unsigned int));
        MOCK_CONST_METHOD1(power_limit_sustained_interval,
                           int32_t(unsigned int));
        MOCK_CONST_METHOD1(power_limit_burst_enabled,
                           bool(unsigned int));
        MOCK_CONST_METHOD1(power_limit_burst_power,
                           int32_t(unsigned int));
        MOCK_CONST_METHOD1(power_limit_peak_ac,
                           int32_t(unsigned int));
        MOCK_CONST_METHOD1(energy,
                           uint64_t(unsigned int));
        MOCK_CONST_METHOD1(performance_factor,
                           double(unsigned int));
        MOCK_CONST_METHOD1(active_process_list,
                           std::vector<uint32_t>(unsigned int));
        MOCK_CONST_METHOD1(standby_mode,
                           double(unsigned int));
        MOCK_CONST_METHOD1(memory_allocated,
                           double(unsigned int));
        MOCK_CONST_METHOD2(energy_threshold_control,
                           void(unsigned int, double));
        MOCK_CONST_METHOD3(frequency_gpu_control,
                           void(unsigned int, double, double));
        MOCK_CONST_METHOD2(standby_mode_control,
                           void(unsigned int, double));
};

#endif
