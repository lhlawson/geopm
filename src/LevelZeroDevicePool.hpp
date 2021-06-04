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

#ifndef LEVELZERODEVICEPOOL_HPP_INCLUDE
#define LEVELZERODEVICEPOOL_HPP_INCLUDE

#include <vector>
#include <string>
#include <cstdint>

#include "geopm_sched.h"
#include <level_zero/ze_api.h>
#include <level_zero/zes_api.h>

namespace geopm
{

    class LevelZeroDevicePool
    {
        public:
            LevelZeroDevicePool() = default;
            virtual ~LevelZeroDevicePool() = default;
            /// @brief Number of accelerators on the platform.
            /// @return Number of LevelZero accelerators.
            virtual int num_accelerator(void) const = 0;
            /// @brief Get the LevelZero device core clock rate
            ///        in MHz.
            /// @param [in] accel_idx The index indicating a particular
            ///        accelerator.
            /// @return Accelerator device core clock rate in MHz.
            virtual uint64_t frequency_gpu_status(unsigned int accel_idx) const = 0;
            virtual uint64_t frequency_mem_status(unsigned int accel_idx) const = 0;
            virtual double core_clock_rate(unsigned int accel_idx) const = 0;
            virtual uint64_t frequency_gpu_min(unsigned int accel_idx) const = 0;
            virtual uint64_t frequency_gpu_max(unsigned int accel_idx) const = 0;
            virtual uint64_t frequency_mem_min(unsigned int accel_idx) const = 0;
            virtual uint64_t frequency_mem_max(unsigned int accel_idx) const = 0;

            /// @brief Get the LevelZero device temperature.
            /// @param [in] accel_idx The index indicating a particular
            ///        accelerator.
            /// @return temperature in degrees Celsius
            virtual double temperature(unsigned int accel_idx) const = 0;
            virtual double temperature_gpu(unsigned int accel_idx) const = 0;
            virtual double temperature_memory(unsigned int accel_idx) const = 0;

            /// @brief Get the LevelZero device utilization metric.
            /// @param [in] accel_idx The index indicating a particular
            ///        accelerator.
            /// @return Accelerator streaming multiprocessor utilization
            ///          percentage
            virtual double utilization(unsigned int accel_idx) const = 0;
            virtual double utilization_compute(unsigned int accel_idx) const = 0;
            virtual double utilization_copy(unsigned int accel_idx) const = 0;
            virtual double utilization_media_decode(unsigned int accel_idx) const = 0;
            /// @brief Get the LevelZero device power in ???.
            /// @param [in] accel_idx The index indicating a particular
            ///        accelerator.
            /// @return Accelerator power consumption in milliwatts.
            virtual double power(unsigned int accel_idx) const = 0;
            virtual uint64_t power_tdp(unsigned int accel_idx) const = 0;
            virtual uint64_t power_limit_min(unsigned int accel_idx) const = 0;
            virtual uint64_t power_limit_max(unsigned int accel_idx) const = 0;
            virtual uint64_t power_limit_sustained_enabled(unsigned int accel_idx) const = 0;
            virtual uint64_t power_limit_sustained_power(unsigned int accel_idx) const = 0;
            virtual uint64_t power_limit_sustained_interval(unsigned int accel_idx) const = 0;
            virtual uint64_t power_limit_burst_enabled(unsigned int accel_idx) const = 0;
            virtual uint64_t power_limit_burst_power(unsigned int accel_idx) const = 0;
            virtual uint64_t power_limit_peak_ac(unsigned int accel_idx) const = 0;

            /// @brief Get the LevelZero device energy in ???.
            /// @param [in] accel_idx The index indicating a particular
            ///        accelerator.
            /// @return Accelerator power consumption in milliwatts.
            virtual uint64_t energy (unsigned int accel_idx) const = 0;
            virtual uint64_t performance_factor(unsigned int accel_idx) const = 0;
            virtual uint64_t performance_factor_gpu(unsigned int accel_idx) const = 0;
            virtual uint64_t performance_factor_mem(unsigned int accel_idx) const = 0;
            virtual std::vector<uint32_t> active_process_list(unsigned int accel_idx) const = 0;
            virtual uint64_t standby_mode(unsigned int accel_idx) const = 0;
            virtual double memory_allocated(unsigned int accel_idx) const = 0;

            /// @brief Set min and max frequency for LevelZero device.
            /// @param [in] accel_idx The index indicating a particular
            ///        accelerator.
            /// @param [in] min_freq Target min frequency in MHz.
            /// @param [in] max_freq Target max frequency in MHz.
            virtual void energy_threshold_control(unsigned int accel_idx, double setting) const = 0;
            virtual void frequency_gpu_control(unsigned int accel_idx, double min_freq, double max_freq) const = 0;
            virtual void standby_mode_control(unsigned int accel_idx, double setting) const = 0;

        private:
    };

    const LevelZeroDevicePool &levelzero_device_pool(int num_cpu);
}
#endif
