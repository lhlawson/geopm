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

#include <cmath>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <thread>
#include <chrono>
#include <time.h>

#include "Exception.hpp"
#include "Agg.hpp"
#include "Helper.hpp"
#include "geopm_sched.h"

#include <level_zero/ze_api.h>
#include <level_zero/zes_api.h>

#include "LevelZeroDevicePoolImp.hpp"

namespace geopm
{

    //////////////////////
    // Timing Functions //
    //////////////////////
    const LevelZeroDevicePool &levelzero_device_pool(const int num_cpu)
    {
        static LevelZeroDevicePoolImp instance(num_cpu);
        return instance;
    }

    LevelZeroDevicePoolImp::LevelZeroDevicePoolImp(const int num_cpu)
        : M_NUM_CPU(num_cpu)
    {
        //TODO: change to a check and error if not enabled.  All ENV handling goes through environment class
        char *zes_enable_sysman = getenv("ZES_ENABLE_SYSMAN");
        if (zes_enable_sysman == NULL || strcmp(zes_enable_sysman, "1") != 0) {
            std::cout << "GEOPM Debug: ZES_ENABLE_SYSMAN not set to 1.  Forcing to 1" << std::endl;
            setenv("ZES_ENABLE_SYSMAN", "1", 1);
        }

        ze_result_t ze_result;
        //Initialize
        ze_result = zeInit(0);
        check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                        ": LevelZero Driver failed to initialize.", __LINE__);
        // Discover drivers
        ze_result = zeDriverGet(&m_num_driver, nullptr);
        check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                        ": LevelZero Driver enumeration failed.", __LINE__);
        m_levelzero_driver.resize(m_num_driver);
        ze_result = zeDriverGet(&m_num_driver, m_levelzero_driver.data());
        check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                        ": LevelZero Driver acquisition failed.", __LINE__);

        for (unsigned int driver = 0; driver < m_num_driver; driver++) {
            // Discover devices in a driver
            uint32_t num_device = 0;

            ze_result = zeDeviceGet(m_levelzero_driver.at(driver), &num_device, nullptr);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": LevelZero Device enumeration failed.", __LINE__);
            std::vector<zes_device_handle_t> device_handle(num_device);
            ze_result = zeDeviceGet(m_levelzero_driver.at(driver), &num_device, device_handle.data());
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": LevelZero Device acquisition failed.", __LINE__);

            //TODO: wrap in a property.type = ZE_DEVICE_TYPE_GPU
            for(unsigned int dev_idx = 0; dev_idx < num_device; ++dev_idx) {
                ze_device_properties_t property;
                ze_result = zeDeviceGetProperties(device_handle.at(dev_idx), &property);

                if (property.type == ZE_DEVICE_TYPE_GPU) {
                    if ((property.flags & ZE_DEVICE_PROPERTY_FLAG_INTEGRATED) != 0) {
                        m_sysman_integrated_gpu.push_back(device_handle.at(dev_idx));
                        ++m_num_integrated_gpu;
                        std::cerr << "Warning: <geopm> LevelZeroDevicePool: Integrated GPU access is not "
                                     "currently supported by GEOPM.\n";
                    }
                    else {
                        m_sysman_board_gpu.push_back(device_handle.at(dev_idx));
                        ++m_num_board_gpu;
                    }
                }
                else if (property.type == ZE_DEVICE_TYPE_CPU) {
                    // All CPU functionality is handled by GEOPM & MSR Safe currently
                    //m_sysman_cpu.push_back(device_handle.at(dev_idx));
                    //++m_num_cpu;
                    std::cerr << "Warning: <geopm> LevelZeroDevicePool: CPU access via LevelZero is not "
                                 "currently supported by GEOPM.\n";
                }
                else if (property.type == ZE_DEVICE_TYPE_FPGA) {
                    // FPGA functionality is not currently supported by GEOPM, but should not cause
                    // an error if the devices are present
                    m_sysman_fpga.push_back(device_handle.at(dev_idx));
                    ++m_num_fpga;
                    std::cerr << "Warning: <geopm> LevelZeroDevicePool: Field Programmable Gate Arrays are not "
                                 "currently supported by GEOPM.\n";
                }
                else if (property.type == ZE_DEVICE_TYPE_MCA) {
                    // MCA functionality is not currently supported by GEOPM, but should not cause
                    // an error if the devices are present
                    m_sysman_mca.push_back(device_handle.at(dev_idx));
                    ++m_num_mca;
                    std::cerr << "Warning: <geopm> LevelZeroDevicePool: Memory Copy Accelerators are not "
                                 "currently supported by GEOPM.\n";
                }

                //std::string type_string;
                //switch(property.type) {
                //    case ZE_DEVICE_TYPE_CPU: type_string = "Central Processing Unit";
                //        break;
                //    case ZE_DEVICE_TYPE_FPGA: type_string = "Field Programmable Gate Array";
                //        break;
                //    case ZE_DEVICE_TYPE_MCA: type_string = "Memory Copy Accelerator";
                //        break;
                //    default: type_string = "Unknown";
                //        break;
                //}
                //throw Exception("LevelZeroDevicePool::" + std::string(__func__) + ": board_gpu_idx " +
                //                std::to_string(board_gpu_idx) + "  is an unsupported device type: " +
                //                type_string, GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }

            // This approach is far simpler, but does not allow for functioning in systems with multiple
            // accelerator types but unsupported accel types OR split accel indexes (i.e. BOARD_ACCELERATOR
            // vs PACKAGE_ACCELERATOR)
            //m_sysman_device.insert(m_sysman_device.end(), device_handle.begin(), device_handle.end());
            //m_num_device = m_num_device + num_device;
        }
        m_sysman_device = m_sysman_board_gpu;

        //TODO: cleanup resize of evertyhing to the same size
        //resize domain caching vectors based on number of board gpu accelerators
        m_fan_domain.resize(m_num_board_gpu);
        m_temperature_domain.resize(m_num_board_gpu);
        m_fabric_domain.resize(m_num_board_gpu);
        m_mem_domain.resize(m_num_board_gpu);
        m_standby_domain.resize(m_num_board_gpu);
        m_freq_domain.resize(m_num_board_gpu);
        m_power_domain.resize(m_num_board_gpu);
        m_engine_domain.resize(m_num_board_gpu);
        m_perf_domain.resize(m_num_board_gpu);


        for (unsigned int board_gpu_idx = 0; board_gpu_idx < m_num_board_gpu; board_gpu_idx++) {
            ze_device_properties_t property;
            ze_result = zeDeviceGetProperties(m_sysman_device.at(board_gpu_idx), &property);

            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": failed to get device properties.", __LINE__);
            domain_cache(board_gpu_idx);
       }
    }

    void LevelZeroDevicePoolImp::domain_cache(unsigned int accel_idx) {
        ze_result_t ze_result;
        uint32_t num_domain = 0;

        //Cache frequency domains
        ze_result = zesDeviceEnumFrequencyDomains(m_sysman_device.at(accel_idx), &num_domain, nullptr);
        if (ze_result == ZE_RESULT_ERROR_UNSUPPORTED_FEATURE) {
            std::cerr << "Warning: <geopm> LevelZeroDevicePool: Frequency domain detection is "
                         "not supported.\n";
        }
        else {
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains.", __LINE__);
            m_freq_domain.at(accel_idx).resize(num_domain);

            ze_result = zesDeviceEnumFrequencyDomains(m_sysman_device.at(accel_idx), &num_domain, m_freq_domain.at(accel_idx).data());
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                        ": Sysman failed to get domain handles.", __LINE__);
        }

        //Cache power domains
        num_domain = 0;
        ze_result = zesDeviceEnumPowerDomains(m_sysman_device.at(accel_idx), &num_domain, nullptr);
        if (ze_result == ZE_RESULT_ERROR_UNSUPPORTED_FEATURE) {
            std::cerr << "Warning: <geopm> LevelZeroDevicePool: Power domain detection is "
                         "not supported.\n";
        }
        else {
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains", __LINE__);
            m_power_domain.at(accel_idx).resize(num_domain);
            ze_result = zesDeviceEnumPowerDomains(m_sysman_device.at(accel_idx), &num_domain, m_power_domain.at(accel_idx).data());
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get domain handle(s).", __LINE__);
        }

        //Cache engine domains
        num_domain = 0;
        ze_result = zesDeviceEnumEngineGroups(m_sysman_device.at(accel_idx), &num_domain, nullptr);
        if (ze_result == ZE_RESULT_ERROR_UNSUPPORTED_FEATURE) {
            std::cerr << "Warning: <geopm> LevelZeroDevicePool: Engine domain detection is "
                         "not supported.\n";
        }
        else {
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains", __LINE__);
            m_engine_domain.at(accel_idx).resize(num_domain);
            ze_result = zesDeviceEnumEngineGroups(m_sysman_device.at(accel_idx), &num_domain, m_engine_domain.at(accel_idx).data());
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains", __LINE__);
        }

        //Cache performance domains
        num_domain = 0;
        ze_result = zesDeviceEnumPerformanceFactorDomains(m_sysman_device.at(accel_idx), &num_domain, nullptr);
        if (ze_result == ZE_RESULT_ERROR_UNSUPPORTED_FEATURE) {
            std::cerr << "Warning: <geopm> LevelZeroDevicePool: Performance Factor domain detection is "
                         "not supported.\n";
        }
        else {
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains", __LINE__);
            m_perf_domain.at(accel_idx).resize(num_domain);
            ze_result = zesDeviceEnumPerformanceFactorDomains(m_sysman_device.at(accel_idx), &num_domain, m_perf_domain.at(accel_idx).data());
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains", __LINE__);
        }

        //Standby domain signals
        num_domain = 0;
        ze_result = zesDeviceEnumStandbyDomains(m_sysman_device.at(accel_idx), &num_domain, nullptr);
        if (ze_result == ZE_RESULT_ERROR_UNSUPPORTED_FEATURE) {
            std::cerr << "Warning: <geopm> LevelZeroDevicePool: Standby domain detection is "
                         "not supported.\n";
        }
        else {
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains", __LINE__);
            m_standby_domain.at(accel_idx).resize(num_domain);
            ze_result = zesDeviceEnumStandbyDomains(m_sysman_device.at(accel_idx), &num_domain, m_standby_domain.at(accel_idx).data());
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains", __LINE__);
        }

        //Memory domain signals
        num_domain = 0;
        ze_result = zesDeviceEnumMemoryModules(m_sysman_device.at(accel_idx), &num_domain, nullptr);
        if (ze_result == ZE_RESULT_ERROR_UNSUPPORTED_FEATURE) {
            std::cerr << "Warning: <geopm> LevelZeroDevicePool: Memory module detection is "
                         "not supported.\n";
        }
        else {
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains", __LINE__);
            m_mem_domain.at(accel_idx).resize(num_domain);
            ze_result = zesDeviceEnumMemoryModules(m_sysman_device.at(accel_idx), &num_domain, m_mem_domain.at(accel_idx).data());
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains", __LINE__);
        }

        //Fabric domain signals
        num_domain = 0;
        ze_result = zesDeviceEnumFabricPorts(m_sysman_device.at(accel_idx), &num_domain, nullptr);
        if (ze_result == ZE_RESULT_ERROR_UNSUPPORTED_FEATURE) {
            std::cerr << "Warning: <geopm> LevelZeroDevicePool: Fabric port detection is not supported.\n";
        }
        else {
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains", __LINE__);
            m_fabric_domain.at(accel_idx).resize(num_domain);
            ze_result = zesDeviceEnumFabricPorts(m_sysman_device.at(accel_idx), &num_domain, m_fabric_domain.at(accel_idx).data());
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains", __LINE__);
        }

        //Temperature domain signals
        num_domain = 0;
        ze_result = zesDeviceEnumTemperatureSensors(m_sysman_device.at(accel_idx), &num_domain, nullptr);
        if (ze_result == ZE_RESULT_ERROR_UNSUPPORTED_FEATURE) {
            std::cerr << "Warning: <geopm> LevelZeroDevicePool: Temperature sensor domain detection is "
                         "not supported.\n";
        }
        else {
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains", __LINE__);
            m_temperature_domain.at(accel_idx).resize(num_domain);
            ze_result = zesDeviceEnumTemperatureSensors(m_sysman_device.at(accel_idx), &num_domain, m_temperature_domain.at(accel_idx).data());
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains", __LINE__);
        }

        //Fan domain signals
        num_domain = 0;
        ze_result = zesDeviceEnumFans(m_sysman_device.at(accel_idx), &num_domain, nullptr);
        if (ze_result == ZE_RESULT_ERROR_UNSUPPORTED_FEATURE) {
            std::cerr << "Warning: <geopm> LevelZeroDevicePool: Fan detection is not supported.\n";
        }
        else {
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains", __LINE__);
            m_fan_domain.at(accel_idx).resize(num_domain);
            ze_result = zesDeviceEnumFans(m_sysman_device.at(accel_idx), &num_domain, m_fan_domain.at(accel_idx).data());
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get number of domains", __LINE__);
        }

        //TODO: Consider caching all accelerator properties
    }

    LevelZeroDevicePoolImp::~LevelZeroDevicePoolImp()
    {
        //Shutdown
        //TODO: Actually shutdown anything needed.
    }

    int LevelZeroDevicePoolImp::num_accelerator() const
    {
        return num_accelerator(ZE_DEVICE_TYPE_GPU);
    }

    int LevelZeroDevicePoolImp::num_accelerator(ze_device_type_t type) const
    {
        if (type == ZE_DEVICE_TYPE_GPU) {
            // TODO: add Integrated vs Board nuance
            return m_num_board_gpu;
        }
        else if (type == ZE_DEVICE_TYPE_CPU) {
            return m_num_cpu;
        }
        else if (type == ZE_DEVICE_TYPE_FPGA) {
            return m_num_fpga;
        }
        else if (type == ZE_DEVICE_TYPE_MCA) {
            return m_num_mca;
        }
        else {
            throw Exception("LevelZeroDevicePool::" + std::string(__func__) + ": accelerator type " +
                            std::to_string(type) + "  is unsupported", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
    }

    void LevelZeroDevicePoolImp::check_accel_range(unsigned int accel_idx) const
    {
        if (accel_idx >= (unsigned int) num_accelerator()) {
            throw Exception("LevelZeroDevicePool::" + std::string(__func__) + ": accel_idx " +
                            std::to_string(accel_idx) + "  is out of range",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
    }

    uint64_t LevelZeroDevicePoolImp::frequency_domains(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        return m_freq_domain.at(accel_idx).size();
    }

    uint64_t LevelZeroDevicePoolImp::power_domains(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        return m_power_domain.at(accel_idx).size();
    }

    uint64_t LevelZeroDevicePoolImp::engine_domains(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        return m_engine_domain.at(accel_idx).size();
    }


    uint64_t LevelZeroDevicePoolImp::engine_compute_domains(unsigned int accel_idx) const
    {
        return engine_domain_types(accel_idx, ZES_ENGINE_GROUP_COMPUTE_SINGLE);
    }

    uint64_t LevelZeroDevicePoolImp::engine_copy_domains(unsigned int accel_idx) const
    {
        return engine_domain_types(accel_idx, ZES_ENGINE_GROUP_COPY_SINGLE);
    }

    uint64_t LevelZeroDevicePoolImp::engine_media_decode_domains(unsigned int accel_idx) const
    {
        return engine_domain_types(accel_idx, ZES_ENGINE_GROUP_MEDIA_DECODE_SINGLE);
    }

    uint64_t LevelZeroDevicePoolImp::engine_media_encode_domains(unsigned int accel_idx) const
    {
        return engine_domain_types(accel_idx, ZES_ENGINE_GROUP_MEDIA_ENCODE_SINGLE);
    }

    //TODO: these should not change throughout runtime and may be cached.
    uint64_t LevelZeroDevicePoolImp::engine_domain_types(int accel_idx, zes_engine_group_t engine_type) const
    {
        check_accel_range(accel_idx);
        check_domain_range(m_engine_domain.at(accel_idx).size(), __func__, __LINE__);

        ze_result_t ze_result;
        uint64_t result = 0;
        zes_engine_properties_t property;

        for (auto handle : m_engine_domain.at(accel_idx)) {
            ze_result = zesEngineGetProperties(handle, &property);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get engine properties.", __LINE__);
            if (engine_type == property.type) {
                result++;
            }
        }

        return result;
    }

    uint64_t LevelZeroDevicePoolImp::performance_domains(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        return m_perf_domain.at(accel_idx).size();
    }

    uint64_t LevelZeroDevicePoolImp::standby_domains(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        return m_standby_domain.at(accel_idx).size();
    }

    uint64_t LevelZeroDevicePoolImp::memory_domains(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        return m_mem_domain.at(accel_idx).size();
    }

    uint64_t LevelZeroDevicePoolImp::fabric_domains(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        return m_fabric_domain.at(accel_idx).size();
    }

    uint64_t LevelZeroDevicePoolImp::temperature_domains(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        return m_temperature_domain.at(accel_idx).size();
    }

    uint64_t LevelZeroDevicePoolImp::fan_domains(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        return m_fan_domain.at(accel_idx).size();
    }

    uint64_t LevelZeroDevicePoolImp::frequency_status_gpu(unsigned int accel_idx) const
    {
        return (uint64_t)frequency_status(accel_idx, ZES_FREQ_DOMAIN_GPU);
    }

    uint64_t LevelZeroDevicePoolImp::frequency_status_mem(unsigned int accel_idx) const
    {
        return (uint64_t)frequency_status(accel_idx, ZES_FREQ_DOMAIN_MEMORY);
    }

    //TODO: provide frequency: efficient (analogous to sticker?), tdp, and requested.
    //      see: https://spec.oneapi.com/level-zero/latest/sysman/api.html#_CPPv416zes_freq_state_t
    double LevelZeroDevicePoolImp::frequency_status(int accel_idx, zes_freq_domain_t type) const
    {
        check_accel_range(accel_idx);
        check_domain_range(m_freq_domain.at(accel_idx).size(), __func__, __LINE__);
        ze_result_t ze_result;
        double result = 0;
        double result_cnt = 0;

        for (auto handle : m_freq_domain.at(accel_idx)) {
            zes_freq_properties_t property;
            ze_result = zesFrequencyGetProperties(handle, &property);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get domain properties.", __LINE__);

            if (type == property.type) {
                zes_freq_state_t state; // = {ZES_STRUCTURE_TYPE_FREQ_STATE, nullptr}; TODO: this does not appear necessary?
                ze_result = zesFrequencyGetState(handle, &state);
                check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                                ": Sysman failed to get frequency state", __LINE__);
                result += state.actual;
                ++result_cnt; //multi-tile support
            }
        }

        return result/result_cnt;
    }

    //TODO: Determine if coreClockRate is the 'actual' frequency, or if the sysman result should be used
    double LevelZeroDevicePoolImp::core_clock_rate(unsigned int accel_idx) const
    {
        ze_device_properties_t result;
        zeDeviceGetProperties(m_sysman_device.at(accel_idx), &result);
        return result.coreClockRate;
    }

    uint64_t LevelZeroDevicePoolImp::frequency_min_gpu(unsigned int accel_idx) const
    {
        return frequency_min_max(accel_idx, ZES_FREQ_DOMAIN_GPU).first;
    }

    uint64_t LevelZeroDevicePoolImp::frequency_max_gpu(unsigned int accel_idx) const
    {
        return frequency_min_max(accel_idx, ZES_FREQ_DOMAIN_GPU).second;
    }

    uint64_t LevelZeroDevicePoolImp::frequency_min_mem(unsigned int accel_idx) const
    {
        return frequency_min_max(accel_idx, ZES_FREQ_DOMAIN_MEMORY).first;
    }

    uint64_t LevelZeroDevicePoolImp::frequency_max_mem(unsigned int accel_idx) const
    {
        return frequency_min_max(accel_idx, ZES_FREQ_DOMAIN_MEMORY).second;
    }

    std::pair<uint64_t, uint64_t> LevelZeroDevicePoolImp::frequency_min_max(int accel_idx, zes_freq_domain_t type) const
    {
        check_accel_range(accel_idx);
        check_domain_range(m_freq_domain.at(accel_idx).size(), __func__, __LINE__);
        ze_result_t ze_result;
        uint64_t result_min = 0;
        uint64_t result_max = 0;
        double result_cnt = 0;

        for (auto handle : m_freq_domain.at(accel_idx)) {
            zes_freq_properties_t property;
            //TODO: it may be necessary to switch this to zesFrequencyGetRange instead of using properties min and max
            ze_result = zesFrequencyGetProperties(handle, &property);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get domain properties.", __LINE__);
            if (type == property.type) { //For initial GEOPM support we're not handling sub-devices
                result_min += property.min;
                result_max += property.max;
                ++result_cnt;
            }
        }

        //TODO: this will probably cause problems, uint/double casted to uint
        return {result_min/result_cnt, result_max/result_cnt};
    }

    //TODO: add zesFrequencyGetAvailableClocks for getting all available frequencies?

    double LevelZeroDevicePoolImp::utilization(unsigned int accel_idx) const
    {
        return utilization(accel_idx, ZES_ENGINE_GROUP_ALL);
    }

    double LevelZeroDevicePoolImp::utilization_compute(unsigned int accel_idx) const
    {
        //TODO: identify if compute_all exists, if not use compute_single and aggregate
        //TODO: transition to return utilization(accel_idx, ZES_ENGINE_GROUP_COMPUTE_ALL)
        //      or return utilization(accel_idx, ZES_ENGINE_GROUP_3D_RENDER_COMPUTE_ALL)?
        return utilization(accel_idx, ZES_ENGINE_GROUP_COMPUTE_SINGLE);
    }


    double LevelZeroDevicePoolImp::utilization_media_decode(unsigned int accel_idx) const
    {
        //TODO: identify if copy_all exists, if not use copy_single and aggregate
        //return utilization(accel_idx, ZES_ENGINE_GROUP_COPY_ALL);
        return utilization(accel_idx, ZES_ENGINE_GROUP_MEDIA_DECODE_SINGLE);
    }


    double LevelZeroDevicePoolImp::utilization_copy(unsigned int accel_idx) const
    {
        //TODO: identify if copy_all exists, if not use copy_single and aggregate
        //return utilization(accel_idx, ZES_ENGINE_GROUP_COPY_ALL);
        return utilization(accel_idx, ZES_ENGINE_GROUP_COPY_SINGLE);
    }

    double LevelZeroDevicePoolImp::utilization(int accel_idx, zes_engine_group_t engine_type) const
    {
        check_accel_range(accel_idx);
        check_domain_range(m_engine_domain.at(accel_idx).size(), __func__, __LINE__);
        ze_result_t ze_result;
        double result = 0;
        double result_cnt = 0;
        bool domain_match = false;

        zes_engine_properties_t property;
        zes_engine_stats_t stats_prev;
        zes_engine_stats_t stats_curr;

        //for each engine group
        for (auto handle : m_engine_domain.at(accel_idx)) {
            ze_result = zesEngineGetProperties(handle, &property);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get engine properties.", __LINE__);

            if (engine_type == property.type) {
                domain_match = true;

                ze_result = zesEngineGetActivity(handle, &stats_prev);
                check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                                ": Sysman failed to get engine group activity.", __LINE__);

                //TODO: wait approach?  May use geopm spin wait.
                std::this_thread::sleep_for(std::chrono::milliseconds(5));

                //TODO: track if any engine group met the criteria, throw if none?
                ze_result = zesEngineGetActivity(handle, &stats_curr);
                check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                                ": Sysman failed to get engine group activity.", __LINE__);

                if (stats_prev.timestamp >= stats_curr.timestamp) {
                    result = -1;
                } else {
                    //TODO: this is currently aggregating all instances reported, but never dividing by number of instances.
                    //      this is to get around issues with the same hardware being reported multiple times and not having access
                    //      to a _ALL variant (COMPUTE_ALL etc).
                    // see: https://spec.oneapi.com/level-zero/latest/sysman/api.html#zes-engine-group-t
                    result += double(stats_curr.activeTime - stats_prev.activeTime) / double(stats_curr.timestamp - stats_prev.timestamp);
                    ++result_cnt;
                }
            }
        }

        if (!domain_match) {
            result = NAN;
        }

        return result/result_cnt;
    }

    double LevelZeroDevicePoolImp::power(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        check_domain_range(m_power_domain.at(accel_idx).size(), __func__, __LINE__);
        ze_result_t ze_result;
        double result = NAN;

        zes_power_energy_counter_t energy_prev;
        zes_power_energy_counter_t energy_curr;

        for (auto handle : m_power_domain.at(accel_idx)) {
            zes_power_properties_t property; // = {ZES_STRUCTURE_TYPE_POWER_PROPERTIES, nullptr};
            ze_result = zesPowerGetProperties(handle, &property);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get domain power properties", __LINE__);

            //For initial GEOPM support we're only providing device level power
            //finding non-subdevice domain.
            if (property.onSubdevice == 0) {
                ze_result = zesPowerGetEnergyCounter(handle, &energy_prev);
                check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                                ": Sysman failed to get energy_counter value for power", __LINE__);
                //TODO: wait approach?  May use geopm spin wait.
                std::this_thread::sleep_for(std::chrono::milliseconds(5));

                ze_result = zesPowerGetEnergyCounter(handle, &energy_curr);
                check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                                ": Sysman failed to get energy_counter value for power", __LINE__);
                if (energy_prev.timestamp >= energy_curr.timestamp) {
                    result = -1;
                } else {
                    result = double(energy_curr.energy - energy_prev.energy) / double(energy_curr.timestamp - energy_prev.timestamp);
                }
            }
        }

        return result;
    }

    uint64_t LevelZeroDevicePoolImp::power_limit_min(unsigned int accel_idx) const
    {
        return std::get<0>(power_limit_default(accel_idx));
    }

    uint64_t LevelZeroDevicePoolImp::power_limit_max(unsigned int accel_idx) const
    {
        return std::get<1>(power_limit_default(accel_idx));
    }

    uint64_t LevelZeroDevicePoolImp::power_tdp(unsigned int accel_idx) const
    {
        return std::get<2>(power_limit_default(accel_idx));
    }

    std::tuple<uint64_t, uint64_t, uint64_t> LevelZeroDevicePoolImp::power_limit_default(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        check_domain_range(m_power_domain.at(accel_idx).size(), __func__, __LINE__);
        ze_result_t ze_result;

        zes_power_properties_t property;
        uint64_t tdp = 0;
        uint64_t min_power_limit = 0;
        uint64_t max_power_limit = 0;

        //TODO: replace with finding non-subdevice domain.
        for (auto handle : m_power_domain.at(accel_idx)) {
            ze_result = zesPowerGetProperties(handle, &property);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get domain power properties", __LINE__);

            //For initial GEOPM support we're only providing device level power
            //finding non-subdevice domain.
            if (property.onSubdevice == 0) {
                // Generally -1 indicates a lack of support for querying this value
                if (property.defaultLimit != -1) {
                    tdp = property.defaultLimit;
                }
                if (property.minLimit != -1) {
                    min_power_limit = property.minLimit;
                }
                if (property.maxLimit != -1) {
                    max_power_limit = property.maxLimit;
                }
            }
        }

        return std::make_tuple(min_power_limit, max_power_limit, tdp);
    }

    //TODO: move power limits to cache.  Add refresh signal in IO Group
    uint64_t LevelZeroDevicePoolImp::power_limit_peak_ac(unsigned int accel_idx) const
    {
        uint64_t result = 0;
        zes_power_peak_limit_t peak = {};
        peak = std::get<2>(power_limit(accel_idx));
        // Generally -1 indicates a lack of support for querying this value
        if (peak.powerDC != -1) {
            result = (uint64_t)peak.powerDC;
        }
        return result;
    }

    //TODO: move power limits to cache.  Add refresh signal in IO Group
    uint64_t LevelZeroDevicePoolImp::power_limit_burst_enabled(unsigned int accel_idx) const
    {
        zes_power_burst_limit_t burst = {};
        burst = std::get<1>(power_limit(accel_idx));
        return (uint64_t)burst.enabled;
    }

    //TODO: move power limits to cache.  Add refresh signal in IO Group
    uint64_t LevelZeroDevicePoolImp::power_limit_burst_power(unsigned int accel_idx) const
    {
        uint64_t result = 0;
        zes_power_burst_limit_t burst = {};
        burst = std::get<1>(power_limit(accel_idx));

        // Generally -1 indicates a lack of support for querying this value
        if (burst.power != -1) {
            result = (uint64_t)burst.power;
        }
        return result;
    }

    //TODO: move power limits to cache.  Add refresh signal in IO Group
    uint64_t LevelZeroDevicePoolImp::power_limit_sustained_enabled(unsigned int accel_idx) const
    {
        zes_power_sustained_limit_t sustained = {};
        sustained = std::get<0>(power_limit(accel_idx));
        return (uint64_t)sustained.enabled;
    }

    //TODO: move power limits to cache.  Add refresh signal in IO Group
    uint64_t LevelZeroDevicePoolImp::power_limit_sustained_power(unsigned int accel_idx) const
    {
        uint64_t result = 0;
        zes_power_sustained_limit_t sustained = {};
        sustained = std::get<0>(power_limit(accel_idx));

        // Generally -1 indicates a lack of support for querying this value
        if (sustained.power != -1) {
            result = (uint64_t)sustained.power;
        }
        return result;
    }

    //TODO: move power limits to cache.  Add refresh signal in IO Group
    uint64_t LevelZeroDevicePoolImp::power_limit_sustained_interval(unsigned int accel_idx) const
    {
        uint64_t result = 0;
        zes_power_sustained_limit_t sustained = {};
        sustained = std::get<0>(power_limit(accel_idx));

        // Generally -1 indicates a lack of support for querying this value
        if (sustained.interval != -1) {
            result = (uint64_t)sustained.interval;
        }
        return result;
    }

    //TODO: move power limits to cache.  Add refresh signal in IO Group
    std::tuple<zes_power_sustained_limit_t, zes_power_burst_limit_t,
               zes_power_peak_limit_t> LevelZeroDevicePoolImp::power_limit(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        check_domain_range(m_power_domain.at(accel_idx).size(), __func__, __LINE__);
        ze_result_t ze_result;

        zes_power_sustained_limit_t sustained = {};
        zes_power_burst_limit_t burst = {};
        zes_power_peak_limit_t peak = {};
        //TODO: replace with finding non-subdevice domain.
        for (auto handle : m_power_domain.at(accel_idx)) {
            zes_power_properties_t property;
            ze_result = zesPowerGetProperties(handle, &property);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get domain power properties", __LINE__);

            //For initial GEOPM support we're only providing device level power
            //finding non-subdevice domain.
            if (property.onSubdevice == 0) {
                ze_result = zesPowerGetLimits(handle, &sustained, &burst, &peak);
                check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get power limits", __LINE__);
            }
        }

//#ifdef GEOPM_DEBUG
        std::cout << "Debug: levelZero sustained_limit_t.sustained: \n" <<
                     "\t enabled: " << std::to_string(sustained.enabled) << "\n"
                     "\t power: " << std::to_string(sustained.power) << " mW\n"
                     "\t interval: " << std::to_string(sustained.interval) << " mS\n" << std::endl;

        std::cout << "Debug: levelZero burst_limit_t.burst: \n" <<
                     "\t enable: " << std::to_string(burst.enabled) << "\n"
                     "\t power: " << std::to_string(burst.power) << " mW\n" << std::endl;

        std::cout << "Debug: levelZero peak_limit_t.peak: \n" <<
                     "\t powerAC: " << std::to_string(peak.powerAC) << " mW\n" << std::endl;
//#endif

        return std::make_tuple(sustained, burst, peak);
    }

    uint64_t LevelZeroDevicePoolImp::energy(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        check_domain_range(m_power_domain.at(accel_idx).size(), __func__, __LINE__);
        ze_result_t ze_result;
        uint64_t result = 0;

        //TODO: replace with finding non-subdevice domain.
        for (auto handle : m_power_domain.at(accel_idx)) {
            zes_power_energy_counter_t energy_counter;
            zes_power_properties_t property;
            ze_result = zesPowerGetProperties(handle, &property);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get domain power properties", __LINE__);

            //For initial GEOPM support we're only providing device level power
            //finding non-subdevice domain.
            if (property.onSubdevice == 0) {
                ze_result = zesPowerGetEnergyCounter(handle, &energy_counter);
                check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                                ": Sysman failed to get energy_counter values", __LINE__);
                result += energy_counter.energy;
            }
        }

        return (uint64_t)result;
    }

    void LevelZeroDevicePoolImp::check_domain_range(int size, const char *func, int line) const
    {
        if (size == 0) {
            throw Exception("LevelZeroDevicePool::" + std::string(func) + ": Not supported on this hardware",
                             GEOPM_ERROR_INVALID, __FILE__, line);
        }
    }

    uint64_t LevelZeroDevicePoolImp::performance_factor(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        check_domain_range(m_perf_domain.at(accel_idx).size(), __func__, __LINE__);

        ze_result_t ze_result;
        double performance_factor = NAN;
        double result_cnt = 0;

        for (auto handle : m_perf_domain.at(accel_idx)) {
            zes_perf_properties_t property;
            ze_result = zesPerformanceFactorGetProperties(handle, &property);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get domain performance factor properties",
                                                             __LINE__);

            ze_result = zesPerformanceFactorGetConfig(handle, &performance_factor);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                          ": Sysman failed to get performance factor", __LINE__);
            ++result_cnt;
        }
        //TODO: this will probably cause problems, uint/double casted to uint
        return (uint64_t)performance_factor/result_cnt;
    }

    uint64_t LevelZeroDevicePoolImp::performance_factor_gpu(unsigned int accel_idx) const
    {
        //TODO: add passing argument for GPU
        check_domain_range(0, __func__, __LINE__); //forcing error for now
        return performance_factor(accel_idx);
    }

    uint64_t LevelZeroDevicePoolImp::performance_factor_mem(unsigned int accel_idx) const
    {
        //TODO: add passing argument for MEM
        check_domain_range(0, __func__, __LINE__); //forcing error for now
        return performance_factor(accel_idx);
    }

    std::vector<uint32_t> LevelZeroDevicePoolImp::active_process_list(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        ze_result_t ze_result;

        uint32_t num_process = 0;
        std::vector<zes_process_state_t> processes = {};
        std::vector<unsigned int> result;

        ze_result = zesDeviceProcessesGetState(m_sysman_device.at(accel_idx), &num_process, nullptr);
        check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                        ": Sysman failed to get running process count",
                                                        __LINE__);

        processes.resize(num_process);
        ze_result = zesDeviceProcessesGetState(m_sysman_device.at(accel_idx), &num_process, processes.data());
        check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                        ": Sysman failed to get running processes",
                                                        __LINE__);

        result.resize(num_process);
        for (uint32_t i = 0; i < num_process; i++) {
            result.push_back(processes.at(i).processId);
        }

        return result;
    }

    uint64_t LevelZeroDevicePoolImp::standby_mode(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        check_domain_range(m_standby_domain.at(accel_idx).size(), __func__, __LINE__);
        zes_standby_promo_mode_t mode = {};
        double result = 0;
        double result_cnt = 0;

        ze_result_t ze_result;
        for (auto handle : m_standby_domain.at(accel_idx)) {
            zes_standby_properties_t property;
            ze_result = zesStandbyGetProperties(handle, &property);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get domain standby properties",
                                                             __LINE__);

            ze_result = zesStandbyGetMode(handle, &mode);
            result += mode;
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                          ": Sysman failed to get standby mode", __LINE__);
            ++result_cnt;
        }
        //TODO: this will probably cause problems, uint/double casted to uint
        return (uint64_t)result/result_cnt;
    }

    double LevelZeroDevicePoolImp::memory_allocated(unsigned int accel_idx) const
    {
        check_accel_range(accel_idx);
        check_domain_range(m_mem_domain.at(accel_idx).size(), __func__, __LINE__);
        double allocated_ratio = NAN;
        double result_cnt = 0;

        for (auto handle : m_mem_domain.at(accel_idx)) {
            ze_result_t ze_result;
            zes_mem_properties_t property;
            zes_mem_state_t state = {};

            ze_result = zesMemoryGetProperties(handle, &property);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get domain memory properties",
                                                             __LINE__);
            //TODO: consider memory location (on device, in system)
            ze_result = zesMemoryGetState(handle, &state);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                          ": Sysman failed to get memory allocated", __LINE__);

            //TODO: Fix the assumption that there's only a single domain. For now we're assuming 1 or
            //      taking the last domain basically...could be HBM, DDR3/4/5, LPDDR, SRAM, GRF, ...
            allocated_ratio += (double)(state.size - state.free) / (double)state.size;
            ++result_cnt;
        }
        return allocated_ratio/result_cnt;
    }

    void LevelZeroDevicePoolImp::energy_threshold_control(unsigned int accel_idx, double setting) const
    {
        check_accel_range(accel_idx);
        check_domain_range(m_power_domain.at(accel_idx).size(), __func__, __LINE__);
        ze_result_t ze_result;

        for (auto handle : m_power_domain.at(accel_idx)) {
            zes_power_properties_t property;
            ze_result = zesPowerGetProperties(handle, &property);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get domain power properties", __LINE__);
            ze_result = zesPowerSetEnergyThreshold(handle, setting);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                        ": Sysman failed to set domain energy threshold", __LINE__);
        }
    }


    void LevelZeroDevicePoolImp::frequency_gpu_control(unsigned int accel_idx, double min_freq, double max_freq) const
    {
        frequency_control(accel_idx, min_freq, max_freq, ZES_FREQ_DOMAIN_GPU);
    }

    void LevelZeroDevicePoolImp::frequency_control(unsigned int accel_idx, double min_freq, double max_freq, zes_freq_domain_t type) const
    {
        check_accel_range(accel_idx);
        check_domain_range(m_freq_domain.at(accel_idx).size(), __func__, __LINE__);
        ze_result_t ze_result;

        zes_freq_properties_t property;
        zes_freq_range_t range;
        range.min = min_freq;
        range.max = max_freq;
        //zes_freq_range_t range_check;

        for (auto handle : m_freq_domain.at(accel_idx)) {
            //zes_freq_properties_t properts = {ZES_STRUCTURE_TYPE_FREQ_PROPERTIES,
            ze_result = zesFrequencyGetProperties(handle, &property);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get domain properties.", __LINE__);

            if (property.type == type) {
                if (property.canControl == 0) {
                    throw Exception("LevelZeroDevicePool::" + std::string(__func__) + ": Attempted to set frequency " +
                                    "for non controllable domain",
                                    GEOPM_ERROR_INVALID, __FILE__, __LINE__);
                }
//#ifdef GEOPM_DEBUG
                std::cout << "Writing freq range.min: "  << std::to_string(min_freq) << ", range.max; " << std::to_string(max_freq) << std::endl;
//#endif
                ze_result = zesFrequencySetRange(handle, &range);
                check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                                ": Sysman failed to set frequency.", __LINE__);
//#ifdef GEOPM_DEBUG
                std::cout << "\tWrite complete" << std::endl;
//#endif
            }
        }
    }

    void LevelZeroDevicePoolImp::standby_mode_control(unsigned int accel_idx, double setting) const
    {
        check_accel_range(accel_idx);
        check_domain_range(m_standby_domain.at(accel_idx).size(), __func__, __LINE__);

        ze_result_t ze_result;
        for (auto handle : m_standby_domain.at(accel_idx)) {
            zes_standby_properties_t property;
            ze_result = zesStandbyGetProperties(handle, &property);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                            ": Sysman failed to get domain standby properties",
                                                             __LINE__);

            ze_result = zesStandbySetMode(handle, (zes_standby_promo_mode_t)setting);
            check_ze_result(ze_result, GEOPM_ERROR_RUNTIME, "LevelZeroDevicePool::" + std::string(__func__) +
                                                          ": Sysman failed to set standby mode", __LINE__);
        }
    }

    void LevelZeroDevicePoolImp::check_ze_result(ze_result_t ze_result, int error, std::string message, int line) const
    {
        if(ze_result != ZE_RESULT_SUCCESS) {
            std::string error_string = "Unknown ze_result_t value";

            if (ze_result == ZE_RESULT_SUCCESS) {
                error_string = "ZE_RESULT_SUCCESS";
            }
            else if (ze_result == ZE_RESULT_NOT_READY) {
                error_string = "ZE_RESULT_NOT_READY";
            }
            else if (ze_result == ZE_RESULT_ERROR_UNINITIALIZED) {
                error_string = "ZE_RESULT_ERROR_UNINITIALIZED";
            }
            else if (ze_result == ZE_RESULT_ERROR_DEVICE_LOST) {
                error_string = "ZE_RESULT_ERROR_DEVICE_LOST";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_ARGUMENT) {
                error_string = "ZE_RESULT_ERROR_INVALID_ARGUMENT";
            }
            else if (ze_result == ZE_RESULT_ERROR_OUT_OF_HOST_MEMORY) {
                error_string = "ZE_RESULT_ERROR_OUT_OF_HOST_MEMORY";
            }
            else if (ze_result == ZE_RESULT_ERROR_OUT_OF_DEVICE_MEMORY) {
                error_string = "ZE_RESULT_ERROR_OUT_OF_DEVICE_MEMORY";
            }
            else if (ze_result == ZE_RESULT_ERROR_MODULE_BUILD_FAILURE) {
                error_string = "ZE_RESULT_ERROR_MODULE_BUILD_FAILURE";
            }
            else if (ze_result == ZE_RESULT_ERROR_INSUFFICIENT_PERMISSIONS) {
                error_string = "ZE_RESULT_ERROR_INSUFFICIENT_PERMISSIONS";
            }
            else if (ze_result == ZE_RESULT_ERROR_NOT_AVAILABLE) {
                error_string = "ZE_RESULT_ERROR_NOT_AVAILABLE";
            }
            else if (ze_result == ZE_RESULT_ERROR_UNSUPPORTED_VERSION) {
                error_string = "ZE_RESULT_ERROR_UNSUPPORTED_VERSION";
            }
            else if (ze_result == ZE_RESULT_ERROR_UNSUPPORTED_FEATURE) {
                error_string = "ZE_RESULT_ERROR_UNSUPPORTED_FEATURE";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_NULL_HANDLE) {
                error_string = "ZE_RESULT_ERROR_INVALID_NULL_HANDLE";
            }
            else if (ze_result == ZE_RESULT_ERROR_HANDLE_OBJECT_IN_USE) {
                error_string = "ZE_RESULT_ERROR_HANDLE_OBJECT_IN_USE";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_NULL_POINTER) {
                error_string = "ZE_RESULT_ERROR_INVALID_NULL_POINTER";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_SIZE) {
                error_string = "ZE_RESULT_ERROR_INVALID_SIZE";
            }
            else if (ze_result == ZE_RESULT_ERROR_UNSUPPORTED_SIZE) {
                error_string = "ZE_RESULT_ERROR_UNSUPPORTED_SIZE";
            }
            else if (ze_result == ZE_RESULT_ERROR_UNSUPPORTED_ALIGNMENT) {
                error_string = "ZE_RESULT_ERROR_UNSUPPORTED_ALIGNMENT";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_SYNCHRONIZATION_OBJECT) {
                error_string = "ZE_RESULT_ERROR_INVALID_SYNCHRONIZATION_OBJECT";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_ENUMERATION) {
                error_string = "ZE_RESULT_ERROR_INVALID_ENUMERATION";
            }
            else if (ze_result == ZE_RESULT_ERROR_UNSUPPORTED_ENUMERATION) {
                error_string = "ZE_RESULT_ERROR_UNSUPPORTED_ENUMERATION";
            }
            else if (ze_result == ZE_RESULT_ERROR_UNSUPPORTED_IMAGE_FORMAT) {
                error_string = "ZE_RESULT_ERROR_UNSUPPORTED_IMAGE_FORMAT";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_NATIVE_BINARY) {
                error_string = "ZE_RESULT_ERROR_INVALID_NATIVE_BINARY";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_GLOBAL_NAME) {
                error_string = "ZE_RESULT_ERROR_INVALID_GLOBAL_NAME";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_KERNEL_NAME) {
                error_string = "ZE_RESULT_ERROR_INVALID_KERNEL_NAME";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_FUNCTION_NAME) {
                error_string = "ZE_RESULT_ERROR_INVALID_FUNCTION_NAME";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_GROUP_SIZE_DIMENSION) {
                error_string = "ZE_RESULT_ERROR_INVALID_GROUP_SIZE_DIMENSION";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_GLOBAL_WIDTH_DIMENSION) {
                error_string = "ZE_RESULT_ERROR_INVALID_GLOBAL_WIDTH_DIMENSION";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_KERNEL_ARGUMENT_INDEX) {
                error_string = "ZE_RESULT_ERROR_INVALID_KERNEL_ARGUMENT_INDEX";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_KERNEL_ARGUMENT_SIZE) {
                error_string = "ZE_RESULT_ERROR_INVALID_KERNEL_ARGUMENT_SIZE";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_KERNEL_ATTRIBUTE_VALUE) {
                error_string = "ZE_RESULT_ERROR_INVALID_KERNEL_ATTRIBUTE_VALUE";
            }
            else if (ze_result == ZE_RESULT_ERROR_INVALID_COMMAND_LIST_TYPE) {
                error_string = "ZE_RESULT_ERROR_INVALID_COMMAND_LIST_TYPE";
            }
            else if (ze_result == ZE_RESULT_ERROR_OVERLAPPING_REGIONS) {
                error_string = "ZE_RESULT_ERROR_OVERLAPPING_REGIONS";
            }
            else if (ze_result == ZE_RESULT_ERROR_UNKNOWN) {
                error_string = "ZE_RESULT_ERROR_UNKNOWN";
            }

            throw Exception(message + "  Error: " + error_string, error, __FILE__, line);
        }
    }



}
