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
            //         in MHz.
            /// @param [in] accel_idx The index indicating a particular
            ///        accelerator.
            /// @return Accelerator device core clock rate in MHz.
            virtual uint64_t frequency_status_gpu(int accel_idx) const = 0;
            virtual uint64_t frequency_status_mem(int accel_idx) const = 0;
            virtual double core_clock_rate(int accel_idx) const = 0;
            virtual uint64_t frequency_min_gpu(int accel_idx) const = 0;
            virtual uint64_t frequency_max_gpu(int accel_idx) const = 0;
            virtual uint64_t frequency_min_mem(int accel_idx) const = 0;
            virtual uint64_t frequency_max_mem(int accel_idx) const = 0;
            virtual double frequency_throttle_gpu(int accel_idx) const = 0;
            virtual double frequency_throttle_mem(int accel_idx) const = 0;
            /// @brief Get the LevelZero device utilization metric.
            /// @param [in] accel_idx The index indicating a particular
            ///        accelerator.
            /// @return Accelerator streaming multiprocessor utilization
            //          percentage as a whole number from 0 to 100.
            virtual double utilization(int accel_idx) const = 0;
            virtual double utilization_compute(int accel_idx) const = 0;
            virtual double utilization_copy(int accel_idx) const = 0;
            /// @brief Get the LevelZero device power in ???.
            /// @param [in] accel_idx The index indicating a particular
            ///        accelerator.
            /// @return Accelerator power consumption in milliwatts.
            virtual double power(int accel_idx) const = 0;
            virtual uint64_t power_tdp(int accel_idx) const = 0;
            virtual uint64_t power_limit_min(int accel_idx) const = 0;
            virtual uint64_t power_limit_max(int accel_idx) const = 0;
            virtual std::tuple<zes_power_sustained_limit_t,
                               zes_power_burst_limit_t,
                               zes_power_peak_limit_t> power_limit(int accel_idx) const = 0;
            /// @brief Get the LevelZero device energy in ???.
            /// @param [in] accel_idx The index indicating a particular
            ///        accelerator.
            /// @return Accelerator power consumption in milliwatts.
            virtual uint64_t energy (int accel_idx) const = 0;
            virtual zes_energy_threshold_t energy_threshold(int accel_idx) const = 0;
            virtual double performance_factor(int accel_idx) const = 0;
            virtual double performance_factor_gpu(int accel_idx) const = 0;
            virtual double performance_factor_mem(int accel_idx) const = 0;
            virtual std::vector<zes_process_state_t> active_process_list(int accel_idx) const = 0;
            virtual uint64_t standby_mode(int accel_idx) const = 0;
            virtual std::pair<double, double> memory_bandwidth(int accel_idx) const = 0;
            virtual double memory_allocated(int accel_idx) const = 0;
            virtual uint64_t frequency_domains(int accel_idx) const = 0;
            virtual uint64_t power_domains(int accel_idx) const = 0;
            virtual uint64_t engine_domains(int accel_idx) const = 0;
            virtual uint64_t performance_domains(int accel_idx) const = 0;
            virtual uint64_t standby_domains(int accel_idx) const = 0;
            virtual uint64_t memory_domains(int accel_idx) const = 0;
            virtual uint64_t fabric_domains(int accel_idx) const = 0;
            virtual uint64_t temperature_domains(int accel_idx) const = 0;
            virtual uint64_t fan_domains(int accel_idx) const = 0;
            virtual uint64_t engine_compute_domains(int accel_idx) const = 0;
            virtual uint64_t engine_copy_domains(int accel_idx) const = 0;
            virtual uint64_t engine_media_decode_domains(int accel_idx) const = 0;
            virtual uint64_t engine_media_encode_domains(int accel_idx) const = 0;

            /// @brief Set min and max frequency for LevelZero device.
            /// @param [in] accel_idx The index indicating a particular
            ///        accelerator.
            /// @param [in] min_freq Target min frequency in MHz.
            /// @param [in] max_freq Target max frequency in MHz.
            virtual void frequency_control_gpu(int accel_idx, double min_freq, double max_freq) const = 0;
            virtual void frequency_control_mem(int accel_idx, double min_freq, double max_freq) const = 0;
            virtual void power_control_sustained(int accel_idx, double enable, double limit, double interval) const = 0;
            virtual void power_control_burst(int accel_idx, double enable, double limit) const = 0;
            virtual void power_control_peak(int accel_idx, double limit) const = 0;
            virtual void power_control(int accel_idx, zes_power_sustained_limit_t sustained,
                                                      zes_power_burst_limit_t burst,
                                                      zes_power_peak_limit_t peak) const = 0;
            virtual void energy_threshold_control(int accel_idx, double setting) const = 0;
            virtual void performance_factor_control(int accel_idx, double setting) const = 0;
            virtual void standby_mode_control(int accel_idx, double setting) const = 0;

        private:
    };

    const LevelZeroDevicePool &levelzero_device_pool(int num_cpu);
}
#endif
