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

#ifndef LEVELZERODEVICEPOOLIMP_HPP_INCLUDE
#define LEVELZERODEVICEPOOLIMP_HPP_INCLUDE

#include <string>

#include "LevelZeroDevicePool.hpp"
#include <level_zero/ze_api.h>
#include <level_zero/zes_api.h>

#include "geopm_time.h"

namespace geopm
{

    class LevelZeroDevicePoolImp : public LevelZeroDevicePool
    {
        public:
            LevelZeroDevicePoolImp(const int num_cpu);
            virtual ~LevelZeroDevicePoolImp();
            virtual int num_accelerator(void) const override;
            virtual double frequency_status_gpu(unsigned int accel_idx) const;
            virtual double frequency_status_mem(unsigned int accel_idx) const;
            virtual double core_clock_rate(unsigned int accel_idx) const;
            virtual uint64_t frequency_min_gpu(unsigned int accel_idx) const override;
            virtual uint64_t frequency_max_gpu(unsigned int accel_idx) const override;
            virtual uint64_t frequency_min_mem(unsigned int accel_idx) const override;
            virtual uint64_t frequency_max_mem(unsigned int accel_idx) const override;
            virtual double frequency_throttle_gpu(unsigned int accel_idx) const override;
            virtual double frequency_throttle_mem(unsigned int accel_idx) const override;
            virtual double utilization(unsigned int accel_idx) const override;
            virtual double utilization_compute(unsigned int accel_idx) const override;
            virtual double utilization_copy(unsigned int accel_idx) const override;
            virtual double power(unsigned int accel_idx) const override;
            virtual uint64_t power_tdp(unsigned int accel_idx) const override;
            virtual uint64_t power_limit_min(unsigned int accel_idx) const override;
            virtual uint64_t power_limit_max(unsigned int accel_idx) const override;
            virtual std::tuple<zes_power_sustained_limit_t,
                               zes_power_burst_limit_t,
                               zes_power_peak_limit_t> power_limit(unsigned int accel_idx) const override;
            virtual uint64_t energy(unsigned int accel_idx) const override;
            virtual zes_energy_threshold_t energy_threshold(unsigned int accel_idx) const override;
            //TODO: performance_factor_compute instead
            virtual double performance_factor(unsigned int accel_idx) const override;
            virtual double performance_factor_gpu(unsigned int accel_idx) const override;
            virtual double performance_factor_mem(unsigned int accel_idx) const override;
            virtual std::vector<zes_process_state_t> active_process_list(unsigned int accel_idx) const override;
            virtual uint64_t standby_mode(unsigned int accel_idx) const override;
            virtual std::pair<double, double> memory_bandwidth(unsigned int accel_idx) const override;
            virtual double memory_allocated(unsigned int accel_idx) const override;
            virtual uint64_t frequency_domains(unsigned int accel_idx) const override;
            virtual uint64_t power_domains(unsigned int accel_idx) const override;
            virtual uint64_t engine_domains(unsigned int accel_idx) const override;
            virtual uint64_t performance_domains(unsigned int accel_idx) const override;
            virtual uint64_t standby_domains(unsigned int accel_idx) const override;
            virtual uint64_t memory_domains(unsigned int accel_idx) const override;
            virtual uint64_t fabric_domains(unsigned int accel_idx) const override;
            virtual uint64_t temperature_domains(unsigned int accel_idx) const override;
            virtual uint64_t fan_domains(unsigned int accel_idx) const override;
            virtual uint64_t engine_compute_domains(unsigned int accel_idx) const override;
            virtual uint64_t engine_copy_domains(unsigned int accel_idx) const override;
            virtual uint64_t engine_media_decode_domains(unsigned int accel_idx) const override;
            virtual uint64_t engine_media_encode_domains(unsigned int accel_idx) const override;

            virtual void energy_threshold_control(unsigned int accel_idx, double setting) const override;
            virtual void performance_factor_control(unsigned int accel_idx, double setting) const override;

        private:
            const unsigned int M_NUM_CPU;
            virtual void domain_cache(unsigned int accel_idx);
            virtual void check_accel_range(unsigned int accel_idx) const;
            virtual void check_domain_range(int size, const char *func, int line) const;
            virtual void check_ze_result(ze_result_t ze_result, int error, std::string message, int line) const;
            virtual int num_accelerator(ze_device_type_t type) const;

            virtual std::tuple<uint64_t, uint64_t, uint64_t> power_limit_default(unsigned int accel_idx) const;
            virtual double frequency_status(int accel_idx, zes_freq_domain_t) const;
            virtual std::pair<uint64_t,uint64_t> frequency_min_max(int accel_idx, zes_freq_domain_t type) const;
            virtual double frequency_throttle(int accel_idx, zes_freq_domain_t type) const;
            virtual double utilization(int accel_idx, zes_engine_group_t engine_type) const;
            virtual uint64_t engine_domain_types(int accel_idx, zes_engine_group_t engine_type) const;

            std::string ze_error_string(ze_result_t result);

            uint32_t m_num_driver;
            uint32_t m_num_accelerator;
            uint32_t m_num_package_gpu;
            uint32_t m_num_board_gpu;
            uint32_t m_num_cpu;
            uint32_t m_num_fpga;
            uint32_t m_num_mca;

            std::vector<ze_driver_handle_t> m_levelzero_driver;
            std::vector<ze_device_handle_t> m_levelzero_device;
            std::vector<zes_device_handle_t> m_sysman_device;

            std::vector<std::vector<zes_freq_handle_t> > m_freq_domain;
            std::vector<std::vector<zes_pwr_handle_t> > m_power_domain;
            std::vector<std::vector<zes_engine_handle_t> > m_engine_domain;
            std::vector<std::vector<zes_perf_handle_t> > m_perf_domain;
            std::vector<std::vector<zes_standby_handle_t> > m_standby_domain;
            std::vector<std::vector<zes_mem_handle_t> > m_mem_domain;
            std::vector<std::vector<zes_fabric_port_handle_t> > m_fabric_domain;
            std::vector<std::vector<zes_temp_handle_t> > m_temperature_domain;
            std::vector<std::vector<zes_fan_handle_t> > m_fan_domain;

    };
}
#endif
