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


#include <unistd.h>
#include <limits.h>

#include <fstream>
#include <string>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "config.h"
#include "Helper.hpp"
#include "Exception.hpp"
#include "PlatformTopo.hpp"
#include "PluginFactory.hpp"
#include "LevelZeroIOGroup.hpp"
#include "geopm_test.hpp"
#include "MockLevelZeroDevicePool.hpp"
#include "MockPlatformTopo.hpp"

using geopm::LevelZeroIOGroup;
using geopm::PlatformTopo;
using geopm::Exception;
using testing::Return;

class LevelZeroIOGroupTest : public :: testing :: Test
{
    protected:
        void SetUp();
        void TearDown();
        void write_affinitization(const std::string &affinitization_str);

        std::shared_ptr<MockLevelZeroDevicePool> m_device_pool;
        std::unique_ptr<MockPlatformTopo> m_platform_topo;
};

void LevelZeroIOGroupTest::SetUp()
{
    const int num_board = 1;
    const int num_package = 2;
    const int num_board_accelerator = 4;
    const int num_core = 20;
    const int num_cpu = 40;

    m_device_pool = std::make_shared<MockLevelZeroDevicePool>();
    m_platform_topo = geopm::make_unique<MockPlatformTopo>();

    //Platform Topo prep
    ON_CALL(*m_platform_topo, num_domain(GEOPM_DOMAIN_BOARD))
        .WillByDefault(Return(num_board));
    ON_CALL(*m_platform_topo, num_domain(GEOPM_DOMAIN_PACKAGE))
        .WillByDefault(Return(num_package));
    ON_CALL(*m_platform_topo, num_domain(GEOPM_DOMAIN_BOARD_ACCELERATOR))
        .WillByDefault(Return(num_board_accelerator));
    ON_CALL(*m_platform_topo, num_domain(GEOPM_DOMAIN_CPU))
        .WillByDefault(Return(num_cpu));
    ON_CALL(*m_platform_topo, num_domain(GEOPM_DOMAIN_CORE))
        .WillByDefault(Return(num_core));

    for (int cpu_idx = 0; cpu_idx < num_cpu; ++cpu_idx) {
        if (cpu_idx < 10) {
            ON_CALL(*m_platform_topo, domain_idx(GEOPM_DOMAIN_BOARD_ACCELERATOR, cpu_idx))
                .WillByDefault(Return(0));
        }
        else if (cpu_idx < 20) {
            ON_CALL(*m_platform_topo, domain_idx(GEOPM_DOMAIN_BOARD_ACCELERATOR, cpu_idx))
                .WillByDefault(Return(1));
        }
        else if (cpu_idx < 30) {
            ON_CALL(*m_platform_topo, domain_idx(GEOPM_DOMAIN_BOARD_ACCELERATOR, cpu_idx))
                .WillByDefault(Return(2));
        }
        else {
            ON_CALL(*m_platform_topo, domain_idx(GEOPM_DOMAIN_BOARD_ACCELERATOR, cpu_idx))
                .WillByDefault(Return(3));
        }
    }

    EXPECT_CALL(*m_device_pool, num_accelerator()).WillRepeatedly(Return(num_board_accelerator));
}

void LevelZeroIOGroupTest::TearDown()
{
}

TEST_F(LevelZeroIOGroupTest, valid_signals)
{
    LevelZeroIOGroup levelzero_io(*m_platform_topo, *m_device_pool);
    for (const auto &sig : levelzero_io.signal_names()) {
        EXPECT_TRUE(levelzero_io.is_valid_signal(sig));
        EXPECT_NE(GEOPM_DOMAIN_INVALID, levelzero_io.signal_domain_type(sig));
        EXPECT_LT(-1, levelzero_io.signal_behavior(sig));
    }
}

TEST_F(LevelZeroIOGroupTest, push_control_adjust_write_batch)
{
//    const int num_accelerator = m_platform_topo->num_domain(GEOPM_DOMAIN_BOARD_ACCELERATOR);
//    std::map<int, double> batch_value;
//    LevelZeroIOGroup levelzero_io(*m_platform_topo, *m_device_pool);
//
//    std::vector<double> mock_freq = {1530, 1320, 420, 135};
//    std::vector<double> mock_power = {153600, 70000, 300000, 50000};
//    for (int accel_idx = 0; accel_idx < num_accelerator; ++accel_idx) {
//        batch_value[(levelzero_io.push_control("LEVELZERO::FREQUENCY_CONTROL",
//                                        GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx))] = mock_freq.at(accel_idx)*1e6;
//        batch_value[(levelzero_io.push_control("FREQUENCY_ACCELERATOR_CONTROL",
//                                        GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx))] = mock_freq.at(accel_idx)*1e6;
//        EXPECT_CALL(*m_device_pool,
//                    frequency_control_sm(accel_idx, mock_freq.at(accel_idx),
//                                         mock_freq.at(accel_idx))).Times(2);
//
//        batch_value[(levelzero_io.push_control("LEVELZERO::FREQUENCY_RESET_CONTROL",
//                                        GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx))] = mock_freq.at(accel_idx);
//        EXPECT_CALL(*m_device_pool, frequency_reset_control(accel_idx)).Times(1);
//
//        batch_value[(levelzero_io.push_control("LEVELZERO::POWER_LIMIT_CONTROL",
//                                        GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx))] = mock_power.at(accel_idx)/1e3;
//        batch_value[(levelzero_io.push_control("POWER_ACCELERATOR_LIMIT_CONTROL",
//                                        GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx))] = mock_power.at(accel_idx)/1e3;
//        EXPECT_CALL(*m_device_pool, power_control(accel_idx, mock_power.at(accel_idx))).Times(2);
//    }
//
//    for (auto& sv: batch_value) {
//
//        // Given that we are mocking LEVELZERODevicePool the actual setting here doesn't matter
//        EXPECT_NO_THROW(levelzero_io.adjust(sv.first, sv.second));
//    }
//    EXPECT_NO_THROW(levelzero_io.write_batch());
}

TEST_F(LevelZeroIOGroupTest, write_control)
{
//    const int num_accelerator = m_platform_topo->num_domain(GEOPM_DOMAIN_BOARD_ACCELERATOR);
//    LevelZeroIOGroup levelzero_io(*m_platform_topo, *m_device_pool);
//
//    std::vector<double> mock_freq = {1530, 1320, 420, 135};
//    std::vector<double> mock_power = {153600, 70000, 300000, 50000};
//    for (int accel_idx = 0; accel_idx < num_accelerator; ++accel_idx) {
//        EXPECT_CALL(*m_device_pool,
//                    frequency_control_sm(accel_idx, mock_freq.at(accel_idx),
//                                         mock_freq.at(accel_idx))).Times(2);
//        EXPECT_NO_THROW(levelzero_io.write_control("LEVELZERO::FREQUENCY_CONTROL",
//                                              GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx,
//                                              mock_freq.at(accel_idx)*1e6));
//        EXPECT_NO_THROW(levelzero_io.write_control("FREQUENCY_ACCELERATOR_CONTROL",
//                                              GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx,
//                                              mock_freq.at(accel_idx)*1e6));
//
//        EXPECT_CALL(*m_device_pool, frequency_reset_control(accel_idx)).Times(1);
//        EXPECT_NO_THROW(levelzero_io.write_control("LEVELZERO::FREQUENCY_RESET_CONTROL",
//                                              GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx, 12345));
//
//        EXPECT_CALL(*m_device_pool, power_control(accel_idx, mock_power.at(accel_idx))).Times(2);
//        EXPECT_NO_THROW(levelzero_io.write_control("LEVELZERO::POWER_LIMIT_CONTROL",
//                                              GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx,
//                                              mock_power.at(accel_idx)/1e3));
//        EXPECT_NO_THROW(levelzero_io.write_control("POWER_ACCELERATOR_LIMIT_CONTROL",
//                                              GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx,
//                                              mock_power.at(accel_idx)/1e3));
//    }
}

TEST_F(LevelZeroIOGroupTest, read_signal_and_batch)
{
    const int num_accelerator = m_platform_topo->num_domain(GEOPM_DOMAIN_BOARD_ACCELERATOR);

    std::vector<double> mock_freq = {1530, 1320, 420, 135};
    std::vector<int> batch_idx;

    LevelZeroIOGroup levelzero_io(*m_platform_topo, *m_device_pool);

    for (int accel_idx = 0; accel_idx < num_accelerator; ++accel_idx) {
        EXPECT_CALL(*m_device_pool, frequency_status_gpu(accel_idx)).WillRepeatedly(Return(mock_freq.at(accel_idx)));
    }
    for (int accel_idx = 0; accel_idx < num_accelerator; ++accel_idx) {
        batch_idx.push_back(levelzero_io.push_signal("LEVELZERO::FREQUENCY_GPU", GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx));
    }
    levelzero_io.read_batch();
    for (int accel_idx = 0; accel_idx < num_accelerator; ++accel_idx) {
        double frequency = levelzero_io.read_signal("LEVELZERO::FREQUENCY_GPU", GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx);
        double frequency_batch = levelzero_io.sample(batch_idx.at(accel_idx));

        EXPECT_DOUBLE_EQ(frequency, mock_freq.at(accel_idx)*1e6);
        EXPECT_DOUBLE_EQ(frequency, frequency_batch);
    }

    mock_freq = {1630, 1420, 520, 235};
    //second round of testing with a modified value
    for (int accel_idx = 0; accel_idx < num_accelerator; ++accel_idx) {
        EXPECT_CALL(*m_device_pool, frequency_status_gpu(accel_idx)).WillRepeatedly(Return(mock_freq.at(accel_idx)));
    }
    levelzero_io.read_batch();
    for (int accel_idx = 0; accel_idx < num_accelerator; ++accel_idx) {
        double frequency = levelzero_io.read_signal("LEVELZERO::FREQUENCY_GPU", GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx);
        double frequency_batch = levelzero_io.sample(batch_idx.at(accel_idx));

        EXPECT_DOUBLE_EQ(frequency, (mock_freq.at(accel_idx))*1e6);
        EXPECT_DOUBLE_EQ(frequency, frequency_batch);
    }
}

TEST_F(LevelZeroIOGroupTest, read_signal)
{
    //EXPECT_CALL(*m_platform_topo, num_domain(GEOPM_DOMAIN_BOARD_ACCELERATOR)).WillRepeatedly(Return(4));
    const int num_accelerator = m_platform_topo->num_domain(GEOPM_DOMAIN_BOARD_ACCELERATOR);

    std::vector<double> mock_freq_gpu = {1530, 1320, 420, 135};
    std::vector<uint64_t> mock_active_time_compute = {1, 90, 50, 0};
    std::vector<uint64_t> mock_active_time_timestamp_compute = {12, 90, 150, 3};
    std::vector<uint64_t> mock_energy = {630000000, 280000000, 470000000, 950000000};
    std::vector<uint64_t> mock_energy_timestamp = {153, 70, 300, 50};

    //std::vector<double> mock_freq_mem = {877, 877, 877, 877};
    //std::vector<double> mock_core_clock_rate = {800, 900, 1000, 2000};
    //std::vector<double> mock_freq_gpu_min = {100, 120, 55, 75};
    //std::vector<double> mock_freq_gpu_max = {1530, 1320, 420, 135};
    //std::vector<double> mock_freq_mem_min = {530, 120, 40, 35};
    //std::vector<double> mock_freq_mem_max = {1530, 1320, 420, 135};
    //std::vector<double> mock_freq_gpu_range_min = {980, 190, 35, 25};
    //std::vector<double> mock_freq_gpu_range_max = {1300, 1200, 40, 735};
    //std::vector<double> mock_utilization = {10, 9, 5, 0};
    //std::vector<double> mock_utilization_mem = {25, 50, 100, 75};
    //std::vector<double> mock_power_limit_sustained = {300000, 270000, 300000, 250000};
    //std::vector<double> mock_power_limit_enabled = {0, 1, 0, 1};
    //std::vector<double> mock_throttle_reasons = {0, 1, 3, 128};
    //std::vector<double> mock_temperature = {45, 60, 68, 92};
    //std::vector<int> active_process_list = {40961, 40962, 40963};



    for (int accel_idx = 0; accel_idx < num_accelerator; ++accel_idx) {
        EXPECT_CALL(*m_device_pool, frequency_status_gpu(accel_idx)).WillRepeatedly(Return(mock_freq_gpu.at(accel_idx)));
        //EXPECT_CALL(*m_device_pool, power(accel_idx)).WillRepeatedly(Return(mock_power.at(accel_idx)));;
        EXPECT_CALL(*m_device_pool, energy(accel_idx)).WillRepeatedly(Return(mock_energy.at(accel_idx)));
        EXPECT_CALL(*m_device_pool, energy_timestamp(accel_idx)).WillRepeatedly(Return(mock_energy_timestamp.at(accel_idx)));
        //EXPECT_CALL(*m_device_pool, utilization_compute(accel_idx)).WillRepeatedly(Return(mock_utilization_compute.at(accel_idx)));
        EXPECT_CALL(*m_device_pool, active_time_compute(accel_idx)).WillRepeatedly(Return(mock_active_time_compute.at(accel_idx)));
        EXPECT_CALL(*m_device_pool, active_time_timestamp_compute(accel_idx)).WillRepeatedly(Return(mock_active_time_timestamp_compute.at(accel_idx)));
        //TODO: Test ALL Signals
    }

    //const int num_cpu = m_platform_topo->num_domain(GEOPM_DOMAIN_CPU);
    //for (int cpu_idx = 0; cpu_idx < num_cpu; ++cpu_idx) {
    //    EXPECT_CALL(*m_device_pool, active_process_list(cpu_idx)).WillRepeatedly(Return(active_process_list));;
    //}

    LevelZeroIOGroup levelzero_io(*m_platform_topo, *m_device_pool);

    for (int accel_idx = 0; accel_idx < num_accelerator; ++accel_idx) {
        double frequency = levelzero_io.read_signal("LEVELZERO::FREQUENCY_GPU", GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx);
        double frequency_alias = levelzero_io.read_signal("FREQUENCY_ACCELERATOR", GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx);
        EXPECT_DOUBLE_EQ(frequency, frequency_alias);
        EXPECT_DOUBLE_EQ(frequency, mock_freq_gpu.at(accel_idx)*1e6);

        double active_time_compute = levelzero_io.read_signal("LEVELZERO::ACTIVE_TIME_COMPUTE", GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx);
        EXPECT_DOUBLE_EQ(active_time_compute, mock_active_time_compute.at(accel_idx));
        double active_time_timestamp_compute = levelzero_io.read_signal("LEVELZERO::ACTIVE_TIME_TIMESTAMP_COMPUTE", GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx);
        EXPECT_DOUBLE_EQ(active_time_timestamp_compute, mock_active_time_timestamp_compute.at(accel_idx));

        double energy = levelzero_io.read_signal("LEVELZERO::ENERGY", GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx);
        EXPECT_DOUBLE_EQ(energy, mock_energy.at(accel_idx)/1e6);

        double energy_timestamp = levelzero_io.read_signal("LEVELZERO::ENERGY_TIMESTAMP", GEOPM_DOMAIN_BOARD_ACCELERATOR, accel_idx);
        EXPECT_DOUBLE_EQ(energy_timestamp, mock_energy_timestamp.at(accel_idx));


    }

}

//Test case: Error path testing including:
//              - Attempt to push a signal at an invalid domain level
//              - Attempt to push an invalid signal
//              - Attempt to sample without a read_batch prior
//              - Attempt to read a signal at an invalid domain level
//              - Attempt to push a control at an invalid domain level
//              - Attempt to adjust a non-existent batch index
//              - Attempt to write a control at an invalid domain level
TEST_F(LevelZeroIOGroupTest, error_path)
{
    const int num_accelerator = m_platform_topo->num_domain(GEOPM_DOMAIN_BOARD_ACCELERATOR);

    std::vector<int> batch_idx;

    std::vector<double> mock_freq = {1530, 1320, 420, 135};
    for (int accel_idx = 0; accel_idx < num_accelerator; ++accel_idx) {
        EXPECT_CALL(*m_device_pool, frequency_status_gpu(accel_idx)).WillRepeatedly(Return(mock_freq.at(accel_idx)));
    }
    LevelZeroIOGroup levelzero_io(*m_platform_topo, *m_device_pool);

    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.push_signal("LEVELZERO::FREQUENCY_GPU", GEOPM_DOMAIN_BOARD, 0),
                               GEOPM_ERROR_INVALID, "domain_type must be");
    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.sample(0),
                               GEOPM_ERROR_INVALID, "batch_idx 0 out of range");
    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.read_signal("LEVELZERO::FREQUENCY_GPU", GEOPM_DOMAIN_BOARD, 0),
                               GEOPM_ERROR_INVALID, "domain_type must be");

    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.push_signal("LEVELZERO::INVALID", GEOPM_DOMAIN_BOARD_ACCELERATOR, 0),
                               GEOPM_ERROR_INVALID, "signal_name LEVELZERO::INVALID not valid for LevelZeroIOGroup");
    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.read_signal("LEVELZERO::INVALID", GEOPM_DOMAIN_BOARD_ACCELERATOR, 0),
                               GEOPM_ERROR_INVALID, "LEVELZERO::INVALID not valid for LevelZeroIOGroup");

    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.push_control("LEVELZERO::FREQUENCY_GPU_CONTROL", GEOPM_DOMAIN_BOARD, 0),
                               GEOPM_ERROR_INVALID, "domain_type must be");
    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.adjust(0, 12345.6),
                               GEOPM_ERROR_INVALID, "batch_idx 0 out of range");
    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.write_control("LEVELZERO::FREQUENCY_GPU_CONTROL", GEOPM_DOMAIN_BOARD, 0, 1530000000),
                               GEOPM_ERROR_INVALID, "domain_type must be");

    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.push_control("LEVELZERO::INVALID", GEOPM_DOMAIN_BOARD_ACCELERATOR, 0),
                               GEOPM_ERROR_INVALID, "control_name LEVELZERO::INVALID not valid for LevelZeroIOGroup");
    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.write_control("LEVELZERO::INVALID", GEOPM_DOMAIN_BOARD_ACCELERATOR, 0, 1530000000),
                               GEOPM_ERROR_INVALID, "LEVELZERO::INVALID not valid for LevelZeroIOGroup");

    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.push_signal("LEVELZERO::FREQUENCY_GPU", GEOPM_DOMAIN_BOARD_ACCELERATOR, num_accelerator),
                               GEOPM_ERROR_INVALID, "domain_idx out of range");
    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.push_signal("LEVELZERO::FREQUENCY_GPU", GEOPM_DOMAIN_BOARD_ACCELERATOR, -1),
                               GEOPM_ERROR_INVALID, "domain_idx out of range");
    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.read_signal("LEVELZERO::FREQUENCY_GPU", GEOPM_DOMAIN_BOARD_ACCELERATOR, num_accelerator),
                               GEOPM_ERROR_INVALID, "domain_idx out of range");
    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.read_signal("LEVELZERO::FREQUENCY_GPU", GEOPM_DOMAIN_BOARD_ACCELERATOR, -1),
                               GEOPM_ERROR_INVALID, "domain_idx out of range");

    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.push_control("LEVELZERO::FREQUENCY_GPU_CONTROL", GEOPM_DOMAIN_BOARD_ACCELERATOR, num_accelerator),
                               GEOPM_ERROR_INVALID, "domain_idx out of range");
    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.push_control("LEVELZERO::FREQUENCY_GPU_CONTROL", GEOPM_DOMAIN_BOARD_ACCELERATOR, -1),
                               GEOPM_ERROR_INVALID, "domain_idx out of range");
    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.write_control("LEVELZERO::FREQUENCY_GPU_CONTROL", GEOPM_DOMAIN_BOARD_ACCELERATOR, num_accelerator, 1530000000),
                               GEOPM_ERROR_INVALID, "domain_idx out of range");
    GEOPM_EXPECT_THROW_MESSAGE(levelzero_io.write_control("LEVELZERO::FREQUENCY_GPU_CONTROL", GEOPM_DOMAIN_BOARD_ACCELERATOR, -1, 1530000000),
                               GEOPM_ERROR_INVALID, "domain_idx out of range");
}
