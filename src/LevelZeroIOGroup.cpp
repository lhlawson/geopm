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

#include "LevelZeroIOGroup.hpp"

#include <cmath>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <sched.h>
#include <errno.h>

#include "IOGroup.hpp"
#include "PlatformTopo.hpp"
#include "LevelZeroDevicePool.hpp"
#include "Exception.hpp"
#include "Agg.hpp"
#include "Helper.hpp"

namespace geopm
{

    LevelZeroIOGroup::LevelZeroIOGroup()
        : LevelZeroIOGroup(platform_topo(), levelzero_device_pool(platform_topo().num_domain(GEOPM_DOMAIN_CPU)))
    {
    }

    // Set up mapping between signal and control names and corresponding indices
    LevelZeroIOGroup::LevelZeroIOGroup(const PlatformTopo &platform_topo, const LevelZeroDevicePool &device_pool)
        : m_platform_topo(platform_topo)
        , m_levelzero_device_pool(device_pool)
        , m_is_batch_read(false)
        , m_signal_available({{"LEVELZERO::FREQUENCY_GPU", {
                                  "Accelerator compute/GPU domain frequency in hertz",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::FREQUENCY_GPU_RANGE_MIN_CONTROL", {
                                  "Accelerator compute/GPU domain user specified min frequency in hertz",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::FREQUENCY_GPU_RANGE_MAX_CONTROL", {
                                  "Accelerator compute/GPU domain user specified max frequency in hertz",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::FREQUENCY_MEMORY", {
                                  "Accelerator memory domain frequency in hertz",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::CORE_CLOCK_RATE", {
                                  "????????????????",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::UTILIZATION", {
                                  "All engine utilization.  NAN implies not supported",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::UTILIZATION_COMPUTE", {
                                  "Compute engine utilization"
                                  "\n  Level Zero logical engines may may to the same hardware"
                                  "\n  resulting in a reduced signal range (i.e. not 0 to 1)",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::UTILIZATION_COPY", {
                                  "Copy engine utilization"
                                  "\n  Level Zero logical engines may may to the same hardware"
                                  "\n  resulting in a reduced signal range (i.e. not 0 to 1)",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::UTILIZATION_MEDIA_DECODE", {
                                  "Media decode engine utilization"
                                  "\n  Level Zero logical engines may may to the same hardware"
                                  "\n  resulting in a reduced signal range (i.e. not 0 to 1)",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::MEMORY_ALLOCATED", {
                                  "Memory usage as a ratio of total memory",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::FREQUENCY_GPU_MIN", {
                                  "Accelerator compute/GPU domain minimum frequency in hertz",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::FREQUENCY_GPU_MAX", {
                                  "Accelerator compute/GPU domain maximum frequency in hertz",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::TEMPERATURE", {
                                  "Device Temperature in Celsius",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::TEMPERATURE_GPU", {
                                  "Device GPU Temperature in Celsius",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::TEMPERATURE_MEMORY", {
                                  "Device Memory Temperature in Celsius",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::ENERGY", {
                                  "Accelerator energy in Joules",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::STANDBY_MODE", {
                                  "Accelerator Standby Mode."
                                  "\n  0 indicates the device may go into standby"
                                  "\n  1 indicates the device will never go into standby",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::CPU_ACCELERATOR_ACTIVE_AFFINITIZATION", {
                                  "Returns the associated accelerator for a given CPU as determined by running processes."
                                  "\n  If no accelerators map to the CPU then -1 is returned"
                                  "\n  If multiple accelerators map to the CPU NAN is returned",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::POWER_LIMIT_SUSTAINED_ENABLED", {
                                  "Accelerator sustained power limit enable value",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::POWER_LIMIT_SUSTAINED", {
                                  "Accelerator sustained power limit in Watts",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::POWER_LIMIT_SUSTAINED_INTERVAL", {
                                  "Accelerator sustained power limit interval in seconds",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::POWER_LIMIT_BURST", {
                                  "Accelerator burst power limit in Watts",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::POWER_LIMIT_BURST_ENABLED", {
                                  "Accelerator burst power limit enable value",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::POWER_LIMIT_PEAK_AC", {
                                  "Accelerator peak AC power limit in Watts",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::POWER_LIMIT_DEFAULT", {
                                  "Default power limit in Watts",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::POWER_LIMIT_MIN", {
                                  "Minimum power limit in Watts",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::POWER_LIMIT_MAX", {
                                  "Maximum power limit in Watts",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"LEVELZERO::POWER", {
                                  "Accelerator power usage in watts",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::sum,
                                  string_format_double
                                  }}
                             })
        , m_control_available({{"LEVELZERO::FREQUENCY_GPU_CONTROL", {
                                    "Sets accelerator frequency (in hertz)",
                                    {},
                                    GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                    Agg::average,
                                    string_format_double
                                    }},
                                {"LEVELZERO::STANDBY_MODE_CONTROL", {
                                    "Accelerator Standby Mode."
                                    "\n  0 indicates the device may go into standby"
                                    "\n  1 indicates the device will never go into standby",
                                    {},
                                    GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                    Agg::average,
                                    string_format_double
                                    }}
                              })
    {
        // populate signals for each domain
        for (auto &sv : m_signal_available) {
            std::vector<std::shared_ptr<signal_s> > result;
            for (int domain_idx = 0; domain_idx < m_platform_topo.num_domain(signal_domain_type(sv.first)); ++domain_idx) {
                std::shared_ptr<signal_s> sgnl = std::make_shared<signal_s>(signal_s{0, false});
                result.push_back(sgnl);
            }
            sv.second.signals = result;
        }
        register_signal_alias("POWER_ACCELERATOR", "LEVELZERO::POWER");
        register_signal_alias("FREQUENCY_ACCELERATOR", "LEVELZERO::FREQUENCY_GPU");

        // populate controls for each domain
        for (auto &sv : m_control_available) {
            std::vector<std::shared_ptr<control_s> > result;
            for (int domain_idx = 0; domain_idx < m_platform_topo.num_domain(control_domain_type(sv.first)); ++domain_idx) {
                std::shared_ptr<control_s> ctrl = std::make_shared<control_s>(control_s{0, false});
                result.push_back(ctrl);
            }
            sv.second.controls = result;
        }
    }

    // Extract the set of all signal names from the index map
    std::set<std::string> LevelZeroIOGroup::signal_names(void) const
    {
        std::set<std::string> result;
        for (const auto &sv : m_signal_available) {
            result.insert(sv.first);
        }
        return result;
    }

    // Extract the set of all control names from the index map
    std::set<std::string> LevelZeroIOGroup::control_names(void) const
    {
        std::set<std::string> result;
        for (const auto &sv : m_control_available) {
            result.insert(sv.first);
        }
        return result;
    }

    // Check signal name using index map
    bool LevelZeroIOGroup::is_valid_signal(const std::string &signal_name) const
    {
        return m_signal_available.find(signal_name) != m_signal_available.end();
    }

    // Check control name using index map
    bool LevelZeroIOGroup::is_valid_control(const std::string &control_name) const
    {
        return m_control_available.find(control_name) != m_control_available.end();
    }

    // Return domain for all valid signals
    int LevelZeroIOGroup::signal_domain_type(const std::string &signal_name) const
    {
        int result = GEOPM_DOMAIN_INVALID;
        auto it = m_signal_available.find(signal_name);
        if (it != m_signal_available.end()) {
            result = it->second.domain;
        }
        return result;
    }

    // Return domain for all valid controls
    int LevelZeroIOGroup::control_domain_type(const std::string &control_name) const
    {
        int result = GEOPM_DOMAIN_INVALID;
        auto it = m_control_available.find(control_name);
        if (it != m_control_available.end()) {
            result = it->second.domain;
        }
        return result;
    }

    // Mark the given signal to be read by read_batch()
    int LevelZeroIOGroup::push_signal(const std::string &signal_name, int domain_type, int domain_idx)
    {
        if (!is_valid_signal(signal_name)) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": signal_name " + signal_name +
                            " not valid for LevelZeroIOGroup.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_type != signal_domain_type(signal_name)) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": " + signal_name + ": domain_type must be " +
                            std::to_string(signal_domain_type(signal_name)),
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_idx < 0 || domain_idx >= m_platform_topo.num_domain(signal_domain_type(signal_name))) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": domain_idx out of range.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (m_is_batch_read) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": cannot push signal after call to read_batch().",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        int result = -1;
        bool is_found = false;
        std::shared_ptr<signal_s> signal = m_signal_available.at(signal_name).signals.at(domain_idx);

        // Check if signal was already pushed
        for (size_t ii = 0; !is_found && ii < m_signal_pushed.size(); ++ii) {
            // same location means this signal or its alias was already pushed
            if (m_signal_pushed[ii].get() == signal.get()) {
                result = ii;
                is_found = true;
            }
        }
        if (!is_found) {
            // If not pushed, add to pushed signals and configure for batch reads
            result = m_signal_pushed.size();
            signal->m_do_read = true;
            m_signal_pushed.push_back(signal);
        }

        return result;
    }

    // Mark the given control to be written by write_batch()
    int LevelZeroIOGroup::push_control(const std::string &control_name, int domain_type, int domain_idx)
    {
        if (!is_valid_control(control_name)) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": control_name " + control_name +
                            " not valid for LevelZeroIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_type != control_domain_type(control_name)) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": " + control_name + ": domain_type must be " +
                            std::to_string(control_domain_type(control_name)),
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_idx < 0 || domain_idx >= m_platform_topo.num_domain(domain_type)) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": domain_idx out of range.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        int result = -1;
        bool is_found = false;
        std::shared_ptr<control_s> control = m_control_available.at(control_name).controls.at(domain_idx);

        // Check if control was already pushed
        for (size_t ii = 0; !is_found && ii < m_control_pushed.size(); ++ii) {
            // same location means this control or its alias was already pushed
            if (m_control_pushed[ii] == control) {
                result = ii;
                is_found = true;
            }
        }
        if (!is_found) {
            // If not pushed, add to pushed control
            result = m_control_pushed.size();
            m_control_pushed.push_back(control);
        }

        return result;
    }

    // Cache the processes in a PID <-> Accelerator map before using them elsewhere
    std::map<pid_t, double> LevelZeroIOGroup::accelerator_process_map(void) const
    {
        std::map<pid_t,double> accelerator_pid_map;

        for (int accel_idx = 0; accel_idx < m_platform_topo.num_domain(GEOPM_DOMAIN_BOARD_ACCELERATOR); ++accel_idx) {
            std::cout << "Processes associated with GPU " << std::to_string(accel_idx) << ":" << std::endl;

            std::vector<uint32_t> active_process_list = m_levelzero_device_pool.active_process_list(accel_idx);
            for (auto proc_itr : active_process_list) {
                std::cout << std::to_string(proc_itr) << ", ";
                // If a process is associated with multiple accelerators we have no good means of
                // signaling the user beyond providing an error value (NAN).
                if (!accelerator_pid_map.count((pid_t)proc_itr)) {
                    accelerator_pid_map[(pid_t)proc_itr] = accel_idx;
                }
                else {
                    accelerator_pid_map[(pid_t)proc_itr] = NAN;
                }
            }
            std::cout << std::endl;
        }
        return accelerator_pid_map;
    }

    // Parse PID to CPU affinitzation and use process list --> accelerator map to get CPU --> accelerator
    double LevelZeroIOGroup::cpu_accelerator_affinity(int cpu_idx, std::map<pid_t, double> process_map) const
    {
        double result = -1;
        size_t num_cpu = m_platform_topo.num_domain(GEOPM_DOMAIN_CPU);
        size_t alloc_size = CPU_ALLOC_SIZE(num_cpu);
        cpu_set_t *proc_cpuset = CPU_ALLOC(num_cpu);
        if (proc_cpuset == NULL) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) +
                            ": failed to allocate process CPU mask",
                            ENOMEM, __FILE__, __LINE__);
        }
        for (auto &proc : process_map) {
            int err = sched_getaffinity(proc.first, alloc_size, proc_cpuset);
            if (err == EINVAL || err == EFAULT) {
                throw Exception("LevelIOGroup::" + std::string(__func__) +
                                ": failed to get affinity mask for process: " +
                                std::to_string(proc.first), err, __FILE__, __LINE__);
            }
            if (!err && CPU_ISSET(cpu_idx, proc_cpuset)) {
                result = proc.second;
                // Return first match, w/o break will return last match
                break;
            }
        }
        return result;
    }

    // Parse and update saved values for signals
    void LevelZeroIOGroup::read_batch(void)
    {
        m_is_batch_read = true;
        for (auto &sv : m_signal_available) {
            for (unsigned int domain_idx = 0; domain_idx < sv.second.signals.size(); ++domain_idx) {
                if (sv.second.signals.at(domain_idx)->m_do_read) {
                    // TODO: numerous optimizations are possible, including:
                    //          grouped power limit reads
                    //          grouped min/max reads
                    //          grouped frequency domain reads (with device pool modification)
                    //
                    //  The majority of these require caching of generally static values in the devicepool,
                    //  more in-depth level zero data types being understood by the IOGroup, OR new data structures
                    //  being defined in the devicepool that replicate the underlying level zero data structs.
                    sv.second.signals.at(domain_idx)->m_value = read_signal(sv.first, sv.second.domain, domain_idx);
                }
            }
        }
    }

    // Write all controls that have been pushed and adjusted
    void LevelZeroIOGroup::write_batch(void)
    {
        for (auto &sv : m_control_available) {
            for (unsigned int domain_idx = 0; domain_idx < sv.second.controls.size(); ++domain_idx) {
                if (sv.second.controls.at(domain_idx)->m_is_adjusted) {
                    // TODO: numerous optimizations are possible, including:
                    //          grouped power limit writes
                    //          grouped freqeuncy writes (with device pool modification)
                    //
                    //  The majority of these require more in-depth level zero data types being understood by the
                    //  IOGroup, OR new data structures
                    write_control(sv.first, sv.second.domain, domain_idx, sv.second.controls.at(domain_idx)->m_setting);
                }
            }
        }
    }

    // Return the latest value read by read_batch()
    double LevelZeroIOGroup::sample(int batch_idx)
    {
        // Do conversion of signal values stored in read batch
        if (batch_idx < 0 || batch_idx >= (int)m_signal_pushed.size()) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": batch_idx " +std::to_string(batch_idx)+ " out of range",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (!m_is_batch_read) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": signal has not been read.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        return m_signal_pushed[batch_idx]->m_value;
    }

    // Save a setting to be written by a future write_batch()
    void LevelZeroIOGroup::adjust(int batch_idx, double setting)
    {
        if (batch_idx < 0 || (unsigned)batch_idx >= m_control_pushed.size()) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + "(): batch_idx " +std::to_string(batch_idx)+ " out of range",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        m_control_pushed.at(batch_idx)->m_setting = setting;
        m_control_pushed.at(batch_idx)->m_is_adjusted = true;
    }

    // Read the value of a signal immediately, bypassing read_batch().  Should not modify m_signal_value
    double LevelZeroIOGroup::read_signal(const std::string &signal_name, int domain_type, int domain_idx)
    {
        if (!is_valid_signal(signal_name)) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": " + signal_name +
                            " not valid for LevelZeroIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_type != signal_domain_type(signal_name)) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": " + signal_name + ": domain_type must be " +
                            std::to_string(signal_domain_type(signal_name)),
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_idx < 0 || domain_idx >= m_platform_topo.num_domain(signal_domain_type(signal_name))) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": domain_idx out of range.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        double result = NAN;
        if (signal_name == "LEVELZERO::FREQUENCY_GPU" || signal_name == "FREQUENCY_ACCELERATOR") {
            result = m_levelzero_device_pool.frequency_gpu_status(domain_idx)*1e6;
        }
        else if (signal_name == "LEVELZERO::FREQUENCY_MEMORY") {
            result = m_levelzero_device_pool.frequency_mem_status(domain_idx)*1e6;
        }
        else if (signal_name == "LEVELZERO::FREQUENCY_GPU_RANGE_MIN_CONTROL") {
            result = m_levelzero_device_pool.frequency_gpu_range_min(domain_idx)*1e6;
        }
        else if (signal_name == "LEVELZERO::FREQUENCY_GPU_RANGE_MAX_CONTROL") {
            result = m_levelzero_device_pool.frequency_gpu_range_max(domain_idx)*1e6;
        }
        else if (signal_name == "LEVELZERO::CORE_CLOCK_RATE") {
            result = m_levelzero_device_pool.core_clock_rate(domain_idx)*1e6;
        }
        else if (signal_name == "LEVELZERO::FREQUENCY_GPU_MIN") {
            result = m_levelzero_device_pool.frequency_gpu_min(domain_idx)*1e6;
        }
        else if (signal_name == "LEVELZERO::FREQUENCY_GPU_MAX") {
            result = m_levelzero_device_pool.frequency_gpu_max(domain_idx)*1e6;
        }
        else if (signal_name == "LEVELZERO::TEMPERATURE") {
            result = m_levelzero_device_pool.temperature(domain_idx);
        }
        else if (signal_name == "LEVELZERO::TEMPERATURE_GPU") {
            result = m_levelzero_device_pool.temperature_gpu(domain_idx);
        }
        else if (signal_name == "LEVELZERO::TEMPERATURE_MEMORY") {
            result = m_levelzero_device_pool.temperature_memory(domain_idx);
        }
        else if (signal_name == "LEVELZERO::UTILIZATION") {
            result = m_levelzero_device_pool.utilization(domain_idx);
        }
        else if (signal_name == "LEVELZERO::UTILIZATION_COMPUTE") {
            result = m_levelzero_device_pool.utilization_compute(domain_idx);
        }
        else if (signal_name == "LEVELZERO::UTILIZATION_COPY") {
            result = m_levelzero_device_pool.utilization_copy(domain_idx);
        }
        else if (signal_name == "LEVELZERO::UTILIZATION_MEDIA_DECODE") {
            result = m_levelzero_device_pool.utilization_media_decode(domain_idx);
        }
        else if (signal_name == "LEVELZERO::POWER" || signal_name == "POWER_ACCELERATOR") {
            result = m_levelzero_device_pool.power(domain_idx);
        }
        else if (signal_name == "LEVELZERO::ENERGY") {
            result = m_levelzero_device_pool.energy(domain_idx)/1e3;
        }
        else if (signal_name == "LEVELZERO::POWER_LIMIT_SUSTAINED") {
            result = m_levelzero_device_pool.power_limit_sustained_power(domain_idx)/1e3;
        }
        else if (signal_name == "LEVELZERO::POWER_LIMIT_SUSTAINED_INTERVAL") {
            result = m_levelzero_device_pool.power_limit_sustained_interval(domain_idx)/1e3;
        }
        else if (signal_name == "LEVELZERO::POWER_LIMIT_SUSTAINED_ENABLED") {
            result = m_levelzero_device_pool.power_limit_sustained_enabled(domain_idx);
        }
        else if (signal_name == "LEVELZERO::POWER_LIMIT_BURST") {
            result = m_levelzero_device_pool.power_limit_burst_power(domain_idx)/1e3;
        }
        else if (signal_name == "LEVELZERO::POWER_LIMIT_BURST_ENABLED") {
            result = m_levelzero_device_pool.power_limit_burst_enabled(domain_idx);
        }
        else if (signal_name == "LEVELZERO::POWER_LIMIT_PEAK_AC") {
            result = m_levelzero_device_pool.power_limit_peak_ac(domain_idx)/1e3;
        }
        else if (signal_name == "LEVELZERO::POWER_LIMIT_MIN") {
            result = m_levelzero_device_pool.power_limit_min(domain_idx)/1e3;
        }
        else if (signal_name == "LEVELZERO::POWER_LIMIT_MAX") {
            result = m_levelzero_device_pool.power_limit_max(domain_idx)/1e3;
        }
        else if (signal_name == "LEVELZERO::POWER_LIMIT_DEFAULT") {
            result = m_levelzero_device_pool.power_tdp(domain_idx)/1e3;
        }
        else if (signal_name == "LEVELZERO::CPU_ACCELERATOR_ACTIVE_AFFINITIZATION") {
            std::map<pid_t, double> process_map = accelerator_process_map();
            result = cpu_accelerator_affinity(domain_idx, process_map);
        }
        else if (signal_name == "LEVELZERO::STANDBY_MODE") {
            result = m_levelzero_device_pool.standby_mode(domain_idx);
        }
        else if (signal_name == "LEVELZERO::MEMORY_ALLOCATED") {
            result = m_levelzero_device_pool.memory_allocated(domain_idx);
        }
        else {
    #ifdef GEOPM_DEBUG
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": Handling not defined for " +
                            signal_name, GEOPM_ERROR_LOGIC, __FILE__, __LINE__);

    #endif
        }
        return result;
    }

    // Write to the control immediately, bypassing write_batch()
    void LevelZeroIOGroup::write_control(const std::string &control_name, int domain_type, int domain_idx, double setting)
    {
        if (!is_valid_control(control_name)) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": " + control_name +
                            " not valid for LevelZeroIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_type != GEOPM_DOMAIN_BOARD_ACCELERATOR) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": " + control_name + ": domain_type must be " +
                            std::to_string(control_domain_type(control_name)),
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_idx < 0 || domain_idx >= m_platform_topo.num_domain(GEOPM_DOMAIN_BOARD_ACCELERATOR)) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": domain_idx out of range.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        if (control_name == "LEVELZERO::FREQUENCY_GPU_CONTROL") {
            m_levelzero_device_pool.frequency_gpu_control(domain_idx, setting/1e6, setting/1e6);
        }
        else if (control_name == "LEVELZERO::STANDBY_MODE_CONTROL") {
            m_levelzero_device_pool.standby_mode_control(domain_idx, setting);
        }
        else {
    #ifdef GEOPM_DEBUG
                throw Exception("LevelZeroIOGroup::" + std::string(__func__) + "Handling not defined for "
                                + control_name, GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
    #endif
        }
    }

    // Implemented to allow an IOGroup to save platform settings before starting
    // to adjust them
    void LevelZeroIOGroup::save_control(void)
    {
        // Read LEVELZERO Power Limit
        for (int domain_idx = 0; domain_idx < m_platform_topo.num_domain(GEOPM_DOMAIN_BOARD_ACCELERATOR); ++domain_idx) {
        }
    }

    // Implemented to allow an IOGroup to restore previously saved
    // platform settings
    void LevelZeroIOGroup::restore_control(void)
    {
        /// @todo: Usage of the LEVELZERO API for setting frequency, power, etc requires root privileges.
        ///        As such several unit tests will fail when calling restore_control.  Once a non-
        ///        privileged solution is available this code may be restored
    }

    // Hint to Agent about how to aggregate signals from this IOGroup
    std::function<double(const std::vector<double> &)> LevelZeroIOGroup::agg_function(const std::string &signal_name) const
    {
        auto it = m_signal_available.find(signal_name);
        if (it == m_signal_available.end()) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": " + signal_name +
                            "not valid for LevelZeroIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return it->second.m_agg_function;
    }

    // Specifies how to print signals from this IOGroup
    std::function<std::string(double)> LevelZeroIOGroup::format_function(const std::string &signal_name) const
    {
        auto it = m_signal_available.find(signal_name);
        if (it == m_signal_available.end()) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": " + signal_name +
                            "not valid for LevelZeroIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return it->second.m_format_function;
    }

    // A user-friendly description of each signal
    std::string LevelZeroIOGroup::signal_description(const std::string &signal_name) const
    {
        if (!is_valid_signal(signal_name)) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": signal_name " + signal_name +
                            " not valid for LevelZeroIOGroup.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return m_signal_available.at(signal_name).m_description;
    }

    // A user-friendly description of each control
    std::string LevelZeroIOGroup::control_description(const std::string &control_name) const
    {
        if (!is_valid_control(control_name)) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": " + control_name +
                            "not valid for LevelZeroIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        return m_control_available.at(control_name).m_description;
    }

    // Name used for registration with the IOGroup factory
    std::string LevelZeroIOGroup::plugin_name(void)
    {
        return "levelzero";
    }

    int LevelZeroIOGroup::signal_behavior(const std::string &signal_name) const
    {
        // TODO: fix me
        return IOGroup::M_SIGNAL_BEHAVIOR_VARIABLE;
    }

    // Function used by the factory to create objects of this type
    std::unique_ptr<IOGroup> LevelZeroIOGroup::make_plugin(void)
    {
        return geopm::make_unique<LevelZeroIOGroup>();
    }

    void LevelZeroIOGroup::register_signal_alias(const std::string &alias_name,
                                            const std::string &signal_name)
    {
        if (m_signal_available.find(alias_name) != m_signal_available.end()) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": signal_name " + alias_name +
                            " was previously registered.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        auto it = m_signal_available.find(signal_name);
        if (it == m_signal_available.end()) {
            // skip adding an alias if underlying signal is not found
            return;
        }
        // copy signal info but append to description
        m_signal_available[alias_name] = it->second;
        m_signal_available[alias_name].m_description =
            m_signal_available[signal_name].m_description + '\n' + "    alias_for: " + signal_name;
    }

    void LevelZeroIOGroup::register_control_alias(const std::string &alias_name,
                                           const std::string &control_name)
    {
        if (m_control_available.find(alias_name) != m_control_available.end()) {
            throw Exception("LevelZeroIOGroup::" + std::string(__func__) + ": contro1_name " + alias_name +
                            " was previously registered.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        auto it = m_control_available.find(control_name);
        if (it == m_control_available.end()) {
            // skip adding an alias if underlying control is not found
            return;
        }
        // copy control info but append to description
        m_control_available[alias_name] = it->second;
        m_control_available[alias_name].m_description =
        m_control_available[control_name].m_description + '\n' + "    alias_for: " + control_name;
    }
}
