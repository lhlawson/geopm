/*
 * Copyright (c) 2015 - 2022, Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "config.h"

#include <stdlib.h>
#include <map>
#include <memory>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "CPUActivityPerformanceModelImp.hpp"
#include "geopm/Exception.hpp"
#include "geopm/Helper.hpp"
#include "geopm/Agg.hpp"
#include "MockPlatformIO.hpp"
#include "MockPlatformTopo.hpp"
#include "geopm/PlatformTopo.hpp"
#include "geopm_prof.h"
#include "geopm_test.hpp"

using ::testing::_;
using ::testing::Invoke;
using ::testing::Sequence;
using ::testing::Return;
using ::testing::AtLeast;
using ::testing::DoubleNear;
using geopm::CPUActivityPerformanceModelImp;
using geopm::PlatformTopo;
using testing::StrictMock;

class CPUActivityPerformanceModelTest : public ::testing::Test
{
    protected:
        enum mock_pio_idx_e {
            QM_CTR_SCALED_RATE_IDX,
            CPU_SCALABILITY_IDX,
            CPU_UNCORE_FREQUENCY_IDX,
            CPU_FREQUENCY_CONTROL_IDX,
            CPU_UNCORE_MIN_CONTROL_IDX,
            CPU_UNCORE_MAX_CONTROL_IDX,
            GPU_CORE_MIN_CONTROL_IDX,
            GPU_CORE_MAX_CONTROL_IDX,
            GPU_ACTIVITY_IDX
        };

        enum policy_idx_e {
            PHI = 0,
            CPU_FREQ_MAX = 1,
            CPU_FREQ_EFFICIENT = 2,
        };

        void SetUp();
        void TearDown();
        static const int M_NUM_CPU;
        static const int M_NUM_CORE;
        static const int M_NUM_BOARD;
        static const int M_NUM_PACKAGE;
        static const int M_NUM_GPU;
        static const size_t M_NUM_UNCORE_MBM_READINGS;
        std::unique_ptr<CPUActivityPerformanceModelImp> m_perf;
        std::vector<double> m_default_policy;
        size_t m_num_policy;
        double m_cpu_freq_min;
        double m_cpu_freq_sticker;
        double m_cpu_freq_step;
        double m_cpu_freq_max;
        double m_cpu_uncore_freq_min;
        double m_cpu_uncore_freq_max;
        double m_gpu_freq_min;
        double m_gpu_freq_max;
        std::vector<double> m_cpu_uncore_freqs;
        std::vector<double> m_mbm_max;
        std::unique_ptr<MockPlatformIO> m_platform_io;
        std::unique_ptr<MockPlatformTopo> m_platform_topo;
};

const int CPUActivityPerformanceModelTest::M_NUM_CPU = 1;
const int CPUActivityPerformanceModelTest::M_NUM_CORE = 1;
const int CPUActivityPerformanceModelTest::M_NUM_BOARD = 1;
const int CPUActivityPerformanceModelTest::M_NUM_PACKAGE = 1;
const int CPUActivityPerformanceModelTest::M_NUM_GPU = 1;
const size_t CPUActivityPerformanceModelTest::M_NUM_UNCORE_MBM_READINGS = 13;

void CPUActivityPerformanceModelTest::SetUp()
{

    m_platform_io = geopm::make_unique<MockPlatformIO>();
    m_platform_topo = geopm::make_unique<MockPlatformTopo>();
    //m_platform_io = geopm::make_unique<StrictMock<MockPlatformIO> >();
    //m_platform_topo = geopm::make_unique<StrictMock<MockPlatformTopo> >();

    ON_CALL(*m_platform_topo, num_domain(GEOPM_DOMAIN_CORE))
        .WillByDefault(Return(M_NUM_CORE));

    m_cpu_freq_min = 1000000000.0;
    m_cpu_freq_sticker = 2100000000.0;
    m_cpu_freq_step = 100000000.0;
    m_cpu_freq_max = 3700000000.0;
    m_cpu_uncore_freq_min = 1200000000.0;
    m_cpu_uncore_freq_max = 2400000000.0;
    m_gpu_freq_min =  400000000.0;
    m_gpu_freq_max = 1600000000.0;

    ON_CALL(*m_platform_io, read_signal("CPU_FREQUENCY_MIN_AVAIL", GEOPM_DOMAIN_BOARD, 0))
            .WillByDefault(Return(m_cpu_freq_min));
    ON_CALL(*m_platform_io, read_signal("CPU_FREQUENCY_MAX_AVAIL", GEOPM_DOMAIN_BOARD, 0))
            .WillByDefault(Return(m_cpu_freq_max));

    ON_CALL(*m_platform_io, read_signal("CPU_FREQUENCY_STICKER", GEOPM_DOMAIN_BOARD, 0))
            .WillByDefault(Return(m_cpu_freq_sticker));
    ON_CALL(*m_platform_io, read_signal("CPU_FREQUENCY_STEP", GEOPM_DOMAIN_BOARD, 0))
            .WillByDefault(Return(m_cpu_freq_step));

    ON_CALL(*m_platform_io, read_signal("CPU_UNCORE_FREQUENCY_MIN_CONTROL", GEOPM_DOMAIN_BOARD, 0))
            .WillByDefault(Return(m_cpu_uncore_freq_min));
    ON_CALL(*m_platform_io, read_signal("CPU_UNCORE_FREQUENCY_MAX_CONTROL", GEOPM_DOMAIN_BOARD, 0))
            .WillByDefault(Return(m_cpu_uncore_freq_max));

    ON_CALL(*m_platform_io, read_signal("GPU_FREQUENCY_MIN_AVAIL", GEOPM_DOMAIN_BOARD, 0))
            .WillByDefault(Return(m_gpu_freq_min));
    ON_CALL(*m_platform_io, read_signal("GPU_FREQUENCY_MAX_AVAIL", GEOPM_DOMAIN_BOARD, 0))
            .WillByDefault(Return(m_gpu_freq_max));

    EXPECT_CALL(*m_platform_topo, num_domain(GEOPM_DOMAIN_CORE)).Times(1);

    // Signals
    ON_CALL(*m_platform_io, push_signal("MSR::QM_CTR_SCALED_RATE", _, _))
        .WillByDefault(Return(QM_CTR_SCALED_RATE_IDX));
    ON_CALL(*m_platform_io, push_signal("MSR::CPU_SCALABILITY_RATIO", _, _))
        .WillByDefault(Return(CPU_SCALABILITY_IDX));
    ON_CALL(*m_platform_io, push_signal("CPU_UNCORE_FREQUENCY_STATUS", _, _))
        .WillByDefault(Return(CPU_UNCORE_FREQUENCY_IDX));
    ON_CALL(*m_platform_io, push_signal("GPU_CORE_ACTIVITY", _, _))
        .WillByDefault(Return(GPU_ACTIVITY_IDX));
    ON_CALL(*m_platform_io, agg_function(_))
        .WillByDefault(Return(geopm::Agg::average));

    m_default_policy = {NAN, m_cpu_freq_max, m_cpu_freq_min};

    m_perf = geopm::make_unique<CPUActivityPerformanceModelImp>(*m_platform_io, *m_platform_topo);
}

void CPUActivityPerformanceModelTest::TearDown()
{

}

TEST_F(CPUActivityPerformanceModelTest, valid)
{
    std::set<std::string> signal_set = {"CPU_FREQUENCY_MIN_AVAIL", "CPU_FREQUENCY_MAX_AVAIL",
                                        "CPU_FREQUENCY_STICKER", "CPU_FREQUENCY_STEP",
                                        "MSR::CPU_SCALABILITY_RATIO"
                                       };
    EXPECT_CALL(*m_platform_io, signal_names()).WillRepeatedly(Return(signal_set));
    m_perf->init();
    EXPECT_EQ(true, m_perf->algorithm_valid());

    std::set<std::string> invalid_signal_set = {"INVALID"};
    EXPECT_CALL(*m_platform_io, signal_names()).WillRepeatedly(Return(invalid_signal_set));
    m_perf->init();
    EXPECT_EQ(false, m_perf->algorithm_valid());
}

TEST_F(CPUActivityPerformanceModelTest, control_recommendation)
{
    // All Controls
    std::set<std::string> signal_set = {"CPU_FREQUENCY_MIN_AVAIL", "CPU_FREQUENCY_MAX_AVAIL",
                                        "CPU_FREQUENCY_STICKER", "CPU_FREQUENCY_STEP",
                                        "MSR::CPU_SCALABILITY_RATIO",
                                       };
    EXPECT_CALL(*m_platform_io, signal_names()).WillRepeatedly(Return(signal_set));
    m_perf->init();

    std::map<std::string, int> expected = {{"CPU_FREQUENCY_MAX_CONTROL", GEOPM_DOMAIN_CORE},
                                          };

    std::map<std::string, int> actual = m_perf->controls_recommended();

    for (auto itr : actual) {
        EXPECT_EQ(expected.at(itr.first), itr.second);
    }

    // CPU + GPU only
    signal_set = {"CPU_FREQUENCY_MIN_AVAIL", "CPU_FREQUENCY_MAX_AVAIL",
                  "CPU_FREQUENCY_STICKER", "CPU_FREQUENCY_STEP",
                  "MSR::CPU_SCALABILITY_RATIO",
                  "CPU_UNCORE_FREQUENCY_STATUS", "MSR::QM_CTR_SCALED_RATE",
                  "GPU_FREQUENCY_MIN_AVAIL", "GPU_FREQUENCY_MAX_AVAIL",
                  "GPU_CORE_ACTIVITY"
                 };
    EXPECT_CALL(*m_platform_io, signal_names()).WillRepeatedly(Return(signal_set));
    m_perf->init();

    expected = {{"CPU_FREQUENCY_MAX_CONTROL", GEOPM_DOMAIN_CORE},
               };
    actual = m_perf->controls_recommended();

    for (auto itr : actual) {
        EXPECT_EQ(expected.count(itr.first), 1);
        EXPECT_EQ(expected.at(itr.first), itr.second);
    }
}

TEST_F(CPUActivityPerformanceModelTest, update_and_sample_recommendation)
{
    // All Controls
    std::set<std::string> signal_set = {"CPU_FREQUENCY_MIN_AVAIL", "CPU_FREQUENCY_MAX_AVAIL",
                                        "CPU_FREQUENCY_STICKER", "CPU_FREQUENCY_STEP",
                                        "MSR::CPU_SCALABILITY_RATIO",
                                       };
    EXPECT_CALL(*m_platform_io, signal_names()).WillRepeatedly(Return(signal_set));
    m_perf->init();

    // Size should be 0 by default
    std::vector<double> rec = m_perf->sample_recommendation("CPU_FREQUENCY_MAX_CONTROL");
    EXPECT_EQ(rec.size(), 0);

    std::vector<double> policy;
    policy = m_default_policy;
    m_perf->validate_policy(policy);
    EXPECT_EQ(0.5, policy[PHI]);
    m_perf->set_policy(policy);
    m_perf->update_recommendation();

    rec = m_perf->sample_recommendation("CPU_FREQUENCY_MAX_CONTROL");
    EXPECT_EQ(rec.size(), M_NUM_CORE);
}

TEST_F(CPUActivityPerformanceModelTest, update_sample_check_recommendation)
{
    // All Controls
    std::set<std::string> signal_set = {"CPU_FREQUENCY_MIN_AVAIL", "CPU_FREQUENCY_MAX_AVAIL",
                                        "CPU_FREQUENCY_STICKER", "CPU_FREQUENCY_STEP",
                                        "MSR::CPU_SCALABILITY_RATIO"
                                       };
    EXPECT_CALL(*m_platform_io, signal_names()).WillRepeatedly(Return(signal_set));

    m_perf->init();

    // Size should be 0 by default
    std::vector<double> rec = m_perf->sample_recommendation("CPU_FREQUENCY_MAX_CONTROL");
    EXPECT_EQ(rec.size(), 0);
    rec = m_perf->sample_recommendation("GPU_CORE_FREQUENCY_MAX_CONTROL");
    EXPECT_EQ(rec.size(), 0);

    double mock_active = 0.5;
    EXPECT_CALL(*m_platform_io, sample(CPU_SCALABILITY_IDX))
                .WillRepeatedly(Return(mock_active));

    double f_e = m_cpu_freq_sticker - m_cpu_freq_step * 2;
    double expected_core_freq = f_e + mock_active *
                                (m_cpu_freq_max - f_e);

    std::vector<double> policy;
    policy = m_default_policy;
    m_perf->validate_policy(policy);
    EXPECT_EQ(0.5, policy[PHI]);
    m_perf->set_policy(policy);
    m_perf->update_recommendation();

    rec = m_perf->sample_recommendation("CPU_FREQUENCY_MAX_CONTROL");
    EXPECT_EQ(rec.size(), M_NUM_CORE);

    for (auto r : rec) {
        EXPECT_EQ(r, expected_core_freq);
    }

    rec = m_perf->sample_recommendation("GPU_CORE_FREQUENCY_MIN_CONTROL");
    EXPECT_EQ(rec.size(), 0);

    // Lower intensity
    mock_active = 0.2;
    EXPECT_CALL(*m_platform_io, sample(CPU_SCALABILITY_IDX))
                .WillRepeatedly(Return(mock_active));

    expected_core_freq = f_e + mock_active *
                         (m_cpu_freq_max - f_e);

    policy = m_default_policy;
    m_perf->validate_policy(policy);
    EXPECT_EQ(0.5, policy[PHI]);
    m_perf->set_policy(policy);
    m_perf->update_recommendation();

    rec = m_perf->sample_recommendation("CPU_FREQUENCY_MAX_CONTROL");
    EXPECT_EQ(rec.size(), M_NUM_CORE);

    for (auto r : rec) {
        EXPECT_EQ(r, expected_core_freq);
    }

    // Higher intensity
    mock_active = 0.8;
    EXPECT_CALL(*m_platform_io, sample(CPU_SCALABILITY_IDX))
                .WillRepeatedly(Return(mock_active));

    expected_core_freq = f_e + mock_active *
                         (m_cpu_freq_max - f_e);

    policy = m_default_policy;
    m_perf->validate_policy(policy);
    EXPECT_EQ(0.5, policy[PHI]);
    m_perf->set_policy(policy);
    m_perf->update_recommendation();

    rec = m_perf->sample_recommendation("CPU_FREQUENCY_MAX_CONTROL");
    EXPECT_EQ(rec.size(), M_NUM_CORE);

    for (auto r : rec) {
        EXPECT_EQ(r, expected_core_freq);
    }
}


TEST_F(CPUActivityPerformanceModelTest, update_and_sample_phi_low)
{

}

TEST_F(CPUActivityPerformanceModelTest, update_and_sample_phi_high)
{

}
