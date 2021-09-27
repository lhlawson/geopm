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

#include "config.h"

#include "DCGMIOGroup.hpp"

#include <cmath>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <cstring>
#include <sched.h>
#include <errno.h>

#include "IOGroup.hpp"
#include "PlatformTopo.hpp"
#include "Exception.hpp"
#include "Agg.hpp"
#include "Helper.hpp"

namespace geopm
{
    DCGMIOGroup::DCGMIOGroup()
        : DCGMIOGroup(platform_topo())
    {
    }

    // Set up mapping between signal and control names and corresponding indices
    DCGMIOGroup::DCGMIOGroup(const PlatformTopo &platform_topo)
        : m_platform_topo(platform_topo)
        , m_is_batch_read(false)
        , m_update_freq(100) //TODO: Review
        , m_max_keep_age(3600.0) //TODO: Review
        , m_max_keep_sample(3600) //TODO: Review
        //, m_dcgm_handle()
        //, m_dcgm_group_id()
        //, m_field_group_id()
        , m_signal_available({{"DCGM::SM_ACTIVE", {
                                  "SM activity expressed as a ratio of cycles",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"DCGM::SM_OCCUPANCY", {
                                  "Warp residency expressed as a ratio of maximum warps per cycles",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::average,
                                  string_format_double
                                  }},
                              {"DCGM::FP64_ACTIVE", {
                                  "Floating point activity expressed as a ratio of cycles",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::sum,
                                  string_format_double
                                  }},
                              {"DCGM::FP32_ACTIVE", {
                                  "Floating point activity expressed as a ratio of cycles",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::sum,
                                  string_format_double
                                  }},
                              {"DCGM::FP16_ACTIVE", {
                                  "Floating point activity expressed as a ratio of cycles",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::sum,
                                  string_format_double
                                  }},
                              {"DCGM::DRAM_ACTIVE", {
                                  "DRAM Send & Receive expresed as a ratio of cycles",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::sum,
                                  string_format_double
                                  }},
                              {"DCGM::PCIE_RX_BYTES", {
                                  "Bytes read via PCIE",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::sum,
                                  string_format_double
                                  }},
                              {"DCGM::PCIE_TX_BYTES", {
                                  "Bytes sent via PCIE",
                                  {},
                                  GEOPM_DOMAIN_BOARD_ACCELERATOR,
                                  Agg::sum,
                                  string_format_double
                                  }},
                             })
        , m_control_available({{"DCGM::FIELD_UPDATE_RATE", {
                                    "Rate at which field data is polled in Seconds",
                                    {},
                                    GEOPM_DOMAIN_BOARD,
                                    Agg::expect_same,
                                    string_format_double
                                    }},
                               {"DCGM::MAX_STORAGE_TIME", {
                                    "Maximum time field data is stored in seconds",
                                    {},
                                    GEOPM_DOMAIN_BOARD,
                                    Agg::expect_same,
                                    string_format_double
                                    }},
                               {"DCGM::MAX_SAMPLES", {
                                    "Maximum number of samples.  0=no limit",
                                    {},
                                    GEOPM_DOMAIN_BOARD,
                                    Agg::expect_same,
                                    string_format_integer
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

        // populate controls for each domain
        for (auto &sv : m_control_available) {
            std::vector<std::shared_ptr<control_s> > result;
            for (int domain_idx = 0; domain_idx < m_platform_topo.num_domain(control_domain_type(sv.first)); ++domain_idx) {
                std::shared_ptr<control_s> ctrl = std::make_shared<control_s>(control_s{0, false});
                result.push_back(ctrl);
            }
            sv.second.controls = result;
        }

        dcgmReturn_t result;
        //Initialize DCGM
        result = dcgmInit();
        dcgm_error_check(result, "Error Initializing DCGM.");
        //std::cout << "DELETE ME - DCGM initialized" << std::endl;

        //Launch DCGM
        //result = dcgmStartEmbedded(DCGM_OPERATION_MODE_AUTO, &m_dcgm_handle);
        //dcgm_error_check(result, "Error starting DCGM");

        char host_ip_address[16] = {0};
        strncpy(host_ip_address, "127.0.0.1", 15);
        result = dcgmConnect(host_ip_address, &m_dcgm_handle);
        //std::cout << "DELETE ME - DCGM embedded started" << std::endl;

        //Check all devices are DCGM enabled
        unsigned int dcgm_dev_id_list[DCGM_MAX_NUM_DEVICES];
        int dcgm_dev_count;
        result = dcgmGetAllSupportedDevices(m_dcgm_handle, dcgm_dev_id_list, &dcgm_dev_count);
        dcgm_error_check(result, "Error fetching devices.");
        //std::cout << "DELETE ME - DCGM device fetch complete.  Device count: " << std::to_string(dcgm_dev_count) << std::endl;

        if (dcgm_dev_count != m_platform_topo.num_domain(GEOPM_DOMAIN_BOARD_ACCELERATOR)) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": "
                            "DCGM enabled device count does not match BOARD_ACCELERATOR count",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        //Setup DCGM Group
        result = dcgmGroupCreate(m_dcgm_handle, DCGM_GROUP_DEFAULT, (char *)"GEOPM", &m_dcgm_group_id);
        dcgm_error_check(result, "Error creating GEOPM DCGM group.");
        //std::cout << "DELETE ME - Created DCGM Group ID: " << std::to_string((unsigned long)m_dcgm_group_id) << std::endl;

        //Setup Field Group
        //result = dcgmFieldGroupCreate(m_dcgm_handle, std::end(dcgm_field_ids) - std::begin(dcgm_field_ids), &dcgm_field_ids[0],
        //                              (char *)"geopm_fields", &m_field_group_id);
        result = dcgmFieldGroupCreate(m_dcgm_handle, sizeof(dcgm_field_ids)/sizeof(dcgm_field_ids[0]), &dcgm_field_ids[0],
                                      (char *)"geopm_fields", &m_field_group_id);
        dcgm_error_check(result, "Error creating field group.");
        //std::cout << "DELETE ME - Field Group Created" << std::endl;

        //Start DCGM
        //std::cout << "DELETE ME - pre DCGM watch" << std::endl;
        result = dcgmWatchFields(m_dcgm_handle, m_dcgm_group_id, m_field_group_id, m_update_freq,
                                 m_max_keep_age, m_max_keep_sample);
        //std::cout << "DELETE ME - post DCGM watch" << std::endl;
        dcgm_error_check(result, "Error setting default watch field configuration.");


        //TODO: is this needed?
        //dcgmUpdateAllFields(m_dcgm_handle, 1);

        //If you want to get names
        //for(const int &field_id : dcgm_field_ids){
        //    DcgmFieldGetById(fieldId)->tag

        //dcgmFieldValue_v1 values[sizeof(dcgm_field_ids)];?
    }

    DCGMIOGroup::~DCGMIOGroup(void)
    {
        dcgmStatusDestroy(NULL);
        dcgmGroupDestroy(m_dcgm_handle, m_dcgm_group_id);
        dcgmStopEmbedded(m_dcgm_handle);
        dcgmStopEmbedded(m_dcgm_handle);
    }

    void DCGMIOGroup::dcgm_error_check(const dcgmReturn_t result, const std::string error)
    {
        if (result != DCGM_ST_OK){
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": "
                            + error + ": " + errorString(result),
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
    }

    // Extract the set of all signal names from the index map
    std::set<std::string> DCGMIOGroup::signal_names(void) const
    {
        std::set<std::string> result;
        for (const auto &sv : m_signal_available) {
            result.insert(sv.first);
        }
        return result;
    }

    // Extract the set of all control names from the index map
    std::set<std::string> DCGMIOGroup::control_names(void) const
    {
        std::set<std::string> result;
        for (const auto &sv : m_control_available) {
            result.insert(sv.first);
        }
        return result;
    }

    // Check signal name using index map
    bool DCGMIOGroup::is_valid_signal(const std::string &signal_name) const
    {
        return m_signal_available.find(signal_name) != m_signal_available.end();
    }

    // Check control name using index map
    bool DCGMIOGroup::is_valid_control(const std::string &control_name) const
    {
        return m_control_available.find(control_name) != m_control_available.end();
    }

    // Return domain for all valid signals
    int DCGMIOGroup::signal_domain_type(const std::string &signal_name) const
    {
        int result = GEOPM_DOMAIN_INVALID;
        auto it = m_signal_available.find(signal_name);
        if (it != m_signal_available.end()) {
            result = it->second.domain;
        }
        return result;
    }

    // Return domain for all valid controls
    int DCGMIOGroup::control_domain_type(const std::string &control_name) const
    {
        int result = GEOPM_DOMAIN_INVALID;
        auto it = m_control_available.find(control_name);
        if (it != m_control_available.end()) {
            result = it->second.domain;
        }
        return result;
    }

    // Mark the given signal to be read by read_batch()
    int DCGMIOGroup::push_signal(const std::string &signal_name, int domain_type, int domain_idx)
    {
        if (!is_valid_signal(signal_name)) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": signal_name " + signal_name +
                            " not valid for DCGMIOGroup.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_type != signal_domain_type(signal_name)) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": " + signal_name + ": domain_type must be " +
                            std::to_string(signal_domain_type(signal_name)),
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_idx < 0 || domain_idx >= m_platform_topo.num_domain(signal_domain_type(signal_name))) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": domain_idx out of range.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (m_is_batch_read) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": cannot push signal after call to read_batch().",
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
    int DCGMIOGroup::push_control(const std::string &control_name, int domain_type, int domain_idx)
    {
        if (!is_valid_control(control_name)) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": control_name " + control_name +
                            " not valid for DCGMIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_type != control_domain_type(control_name)) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": " + control_name + ": domain_type must be " +
                            std::to_string(control_domain_type(control_name)),
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_idx < 0 || domain_idx >= m_platform_topo.num_domain(domain_type)) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": domain_idx out of range.",
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

    // Parse and update saved values for signals
    void DCGMIOGroup::read_batch(void)
    {
        m_is_batch_read = true;
        for (auto &sv : m_signal_available) {
        //    if (sv.first == "") {
        //    }
        //    else {
                for (unsigned int domain_idx = 0; domain_idx < sv.second.signals.size(); ++domain_idx) {
                    if (sv.second.signals.at(domain_idx)->m_do_read) {
                        sv.second.signals.at(domain_idx)->m_value = read_signal(sv.first, sv.second.domain, domain_idx);
                    }
                }
        //    }
        }
    }

    // Write all controls that have been pushed and adjusted
    void DCGMIOGroup::write_batch(void)
    {
        for (auto &sv : m_control_available) {
            for (unsigned int domain_idx = 0; domain_idx < sv.second.controls.size(); ++domain_idx) {
                if (sv.second.controls.at(domain_idx)->m_is_adjusted) {
                    write_control(sv.first, sv.second.domain, domain_idx, sv.second.controls.at(domain_idx)->m_setting);
                }
            }
        }
    }

    // Return the latest value read by read_batch()
    double DCGMIOGroup::sample(int batch_idx)
    {
        // Do conversion of signal values stored in read batch
        if (batch_idx < 0 || batch_idx >= (int)m_signal_pushed.size()) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": batch_idx " +std::to_string(batch_idx)+ " out of range",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (!m_is_batch_read) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": signal has not been read.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        return m_signal_pushed[batch_idx]->m_value;
    }

    // Save a setting to be written by a future write_batch()
    void DCGMIOGroup::adjust(int batch_idx, double setting)
    {
        if (batch_idx < 0 || (unsigned)batch_idx >= m_control_pushed.size()) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + "(): batch_idx " +std::to_string(batch_idx)+ " out of range",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        m_control_pushed.at(batch_idx)->m_setting = setting;
        m_control_pushed.at(batch_idx)->m_is_adjusted = true;
    }

    // Read the value of a signal immediately, bypassing read_batch().  Should not modify m_signal_value
    double DCGMIOGroup::read_signal(const std::string &signal_name, int domain_type, int domain_idx)
    {
        if (!is_valid_signal(signal_name)) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": " + signal_name +
                            " not valid for DCGMIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_type != signal_domain_type(signal_name)) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": " + signal_name + ": domain_type must be " +
                            std::to_string(signal_domain_type(signal_name)),
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_idx < 0 || domain_idx >= m_platform_topo.num_domain(signal_domain_type(signal_name))) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": domain_idx out of range.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        dcgmReturn_t dcgm_result;
        double result = NAN;

        dcgmFieldValue_v1 dcgm_field_values[sizeof(dcgm_field_ids)];
        dcgm_result = dcgmGetLatestValuesForFields(m_dcgm_handle, domain_idx, dcgm_field_ids, sizeof(dcgm_field_ids)/sizeof(dcgm_field_ids[0]), dcgm_field_values);
        dcgm_error_check(dcgm_result, "Error getting latest values for fields");

        for (int i = 0; i < sizeof(dcgm_field_ids)/sizeof(dcgm_field_ids[0]); i++) {
            if(i == 0)
                switch (dcgm_field_values[i].fieldType) {
                    case DCGM_FT_BINARY:
                        break;
                    case DCGM_FT_DOUBLE:
                        result = dcgm_field_values[i].value.dbl;
                        break;
                    case DCGM_FT_INT64:
                        result = (double) dcgm_field_values[i].value.i64;
                        break;
                    case DCGM_FT_STRING:
                        break;
                    case DCGM_FT_TIMESTAMP:
                        break;
                    default:
                        //std::cout << "Error in field types. " << gpulist[gpu_id].values[i].fieldType
                        //          << "\n";
                        //result = "";
                        break;
                }
        }
        if (signal_name == "DCGM::SM_ACTIVE") {
        }
        else if (signal_name == "DCGM::SM_OCCUPANCY") {
        }
        else if (signal_name == "DCGM::FP64_ACTIVE") {
        }
        else if (signal_name == "DCGM::FP32_ACTIVE") {
        }
        else if (signal_name == "DCGM::FP16_ACTIVE") {
        }
        else if (signal_name == "DCGM::DRAM_ACTIVE") {
        }
        else if (signal_name == "DCGM::PCIE_RX_BYTES") {
        }
        else if (signal_name == "DCGM::PCIE_TX_BYTES") {
        }
        else {
    #ifdef GEOPM_DEBUG
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": Handling not defined for " +
                            signal_name, GEOPM_ERROR_LOGIC, __FILE__, __LINE__);

    #endif
        }
        return result;
    }

    // Write to the control immediately, bypassing write_batch()
    void DCGMIOGroup::write_control(const std::string &control_name, int domain_type, int domain_idx, double setting)
    {
        if (!is_valid_control(control_name)) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": " + control_name +
                            " not valid for DCGMIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_type != control_domain_type(control_name)) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": " + control_name + ": domain_type must be " +
                            std::to_string(control_domain_type(control_name)),
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_idx < 0 || domain_idx >= m_platform_topo.num_domain(GEOPM_DOMAIN_BOARD_ACCELERATOR)) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": domain_idx out of range.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        // These 3 controls all apply to dcgmWatchFields
        // https://docs.nvidia.com/datacenter/dcgm/latest/dcgm-api/group__DCGMAPI__FI.html#group__DCGMAPI__FI_1gd04fa06405af6165fd85310dee9eae2a
        dcgmReturn_t result;
        if (control_name == "DCGM::FIELD_UPDATE_RATE") {
            m_update_freq = setting*1e6; //second to usec conversion
            result = dcgmWatchFields(m_dcgm_handle, m_dcgm_group_id, m_field_group_id, m_update_freq,
                                     m_max_keep_age, m_max_keep_sample);
        }
        else if (control_name == "DCGM::MAX_STORAGE_TIME") {
            m_max_keep_age = setting; //second to second conversion
            result = dcgmWatchFields(m_dcgm_handle, m_dcgm_group_id, m_field_group_id, m_update_freq,
                                     m_max_keep_age, m_max_keep_sample);
        }
        else if (control_name == "DCGM::MAX_SAMPLES") {
            m_max_keep_age = setting;
            result = dcgmWatchFields(m_dcgm_handle, m_dcgm_group_id, m_field_group_id, m_update_freq,
                                     m_max_keep_age, m_max_keep_sample);
        }
        else {
    #ifdef GEOPM_DEBUG
                throw Exception("DCGMIOGroup::" + std::string(__func__) + "Handling not defined for "
                                + control_name, GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
    #endif
        }
        dcgm_error_check(result, "Error Updating " + control_name);
    }

    // Implemented to allow an IOGroup to save platform settings before starting
    // to adjust them
    void DCGMIOGroup::save_control(void)
    {
    }

    // Implemented to allow an IOGroup to restore previously saved
    // platform settings
    void DCGMIOGroup::restore_control(void)
    {
    }

    // Hint to Agent about how to aggregate signals from this IOGroup
    std::function<double(const std::vector<double> &)> DCGMIOGroup::agg_function(const std::string &signal_name) const
    {
        auto it = m_signal_available.find(signal_name);
        if (it == m_signal_available.end()) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": " + signal_name +
                            "not valid for DCGMIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return it->second.m_agg_function;
    }

    // Specifies how to print signals from this IOGroup
    std::function<std::string(double)> DCGMIOGroup::format_function(const std::string &signal_name) const
    {
        auto it = m_signal_available.find(signal_name);
        if (it == m_signal_available.end()) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": " + signal_name +
                            "not valid for DCGMIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return it->second.m_format_function;
    }

    // A user-friendly description of each signal
    std::string DCGMIOGroup::signal_description(const std::string &signal_name) const
    {
        if (!is_valid_signal(signal_name)) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": signal_name " + signal_name +
                            " not valid for DCGMIOGroup.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return m_signal_available.at(signal_name).m_description;
    }

    // A user-friendly description of each control
    std::string DCGMIOGroup::control_description(const std::string &control_name) const
    {
        if (!is_valid_control(control_name)) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": " + control_name +
                            "not valid for DCGMIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        return m_control_available.at(control_name).m_description;
    }

    int DCGMIOGroup::signal_behavior(const std::string &signal_name) const
    {
        return IOGroup::M_SIGNAL_BEHAVIOR_VARIABLE;
    }

    // Name used for registration with the IOGroup factory
    std::string DCGMIOGroup::plugin_name(void)
    {
        return "dcgm";
    }

    // Function used by the factory to create objects of this type
    std::unique_ptr<IOGroup> DCGMIOGroup::make_plugin(void)
    {
        return geopm::make_unique<DCGMIOGroup>();
    }

    void DCGMIOGroup::register_signal_alias(const std::string &alias_name,
                                            const std::string &signal_name)
    {
        if (m_signal_available.find(alias_name) != m_signal_available.end()) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": signal_name " + alias_name +
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

    void DCGMIOGroup::register_control_alias(const std::string &alias_name,
                                           const std::string &control_name)
    {
        if (m_control_available.find(alias_name) != m_control_available.end()) {
            throw Exception("DCGMIOGroup::" + std::string(__func__) + ": contro1_name " + alias_name +
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
