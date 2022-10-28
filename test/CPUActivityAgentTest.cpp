/*
 * Copyright (c) 2015 - 2022, Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "config.h"

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <map>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "geopm_agent.h"
#include "geopm_hash.h"

#include "Agent.hpp"
#include "CPUActivityAgent.hpp"
#include "geopm/Exception.hpp"
#include "geopm/Helper.hpp"
#include "geopm/Agg.hpp"
#include "MockPlatformIO.hpp"
#include "MockPlatformTopo.hpp"
#include "MockActivityPerformanceModel.hpp"
#include "geopm/PlatformTopo.hpp"
#include "geopm_prof.h"
#include "geopm_test.hpp"

using ::testing::_;
using ::testing::Invoke;
using ::testing::Sequence;
using ::testing::Return;
using ::testing::AtLeast;
using ::testing::DoubleNear;
using geopm::CPUActivityAgent;
using geopm::PlatformTopo;
using testing::SetArgReferee;

class CPUActivityAgentTest : public ::testing::Test
{
    protected:
        enum mock_pio_idx_e {
            CPU_FREQUENCY_CONTROL_IDX,
            CPU_UNCORE_MIN_CONTROL_IDX,
            CPU_UNCORE_MAX_CONTROL_IDX
        };
        enum policy_idx_e {
            CPU_FREQ_MAX = 0,
            CPU_FREQ_EFFICIENT = 1,
            CPU_UNCORE_FREQ_MAX = 2,
            CPU_UNCORE_FREQ_EFFICIENT = 3,
            PHI = 4,
            UNCORE_MEM_BW = 5,
        };

        void SetUp();
        void TearDown();
        static const int M_NUM_CPU;
        static const int M_NUM_CORE;
        static const int M_NUM_PACKAGE;
        std::unique_ptr<CPUActivityAgent> m_agent;
        std::vector<double> m_default_policy;
        size_t m_num_policy;
        double m_cpu_freq_min;
        double m_cpu_freq_max;
        double m_cpu_uncore_freq_min;
        double m_cpu_uncore_freq_max;
        double m_mbm_max;
        std::unique_ptr<MockPlatformIO> m_platform_io;
        std::unique_ptr<MockPlatformTopo> m_platform_topo;
        std::unique_ptr<MockActivityPerformanceModel> m_cpu_model;
        std::unique_ptr<MockActivityPerformanceModel> m_uncore_model;
};

const int CPUActivityAgentTest::M_NUM_CPU = 1;
const int CPUActivityAgentTest::M_NUM_CORE = 1;
const int CPUActivityAgentTest::M_NUM_PACKAGE = 1;

void CPUActivityAgentTest::SetUp()
{
    m_platform_io = geopm::make_unique<MockPlatformIO>();
    m_platform_topo = geopm::make_unique<MockPlatformTopo>();
    m_cpu_model = geopm::make_unique<MockActivityPerformanceModel>();
    m_uncore_model = geopm::make_unique<MockActivityPerformanceModel>();

    ON_CALL(*m_platform_topo, num_domain(GEOPM_DOMAIN_CORE))
        .WillByDefault(Return(M_NUM_CORE));
    ON_CALL(*m_platform_topo, num_domain(GEOPM_DOMAIN_PACKAGE))
        .WillByDefault(Return(M_NUM_PACKAGE));

    // Controls
    ON_CALL(*m_platform_io, push_control("CPU_FREQUENCY_MAX_CONTROL", _, _))
        .WillByDefault(Return(CPU_FREQUENCY_CONTROL_IDX));
    ON_CALL(*m_platform_io, push_control("CPU_UNCORE_FREQUENCY_MIN_CONTROL", _, _))
        .WillByDefault(Return(CPU_UNCORE_MIN_CONTROL_IDX));
    ON_CALL(*m_platform_io, push_control("CPU_UNCORE_FREQUENCY_MAX_CONTROL", _, _))
        .WillByDefault(Return(CPU_UNCORE_MAX_CONTROL_IDX));
    ON_CALL(*m_platform_io, agg_function(_))
        .WillByDefault(Return(geopm::Agg::average));

    std::map<std::string, int> cpu_ctl_map = {{"CPU_FREQUENCY_MAX_CONTROL", GEOPM_DOMAIN_CORE}};
    ON_CALL(*m_cpu_model, controls_recommended())
        .WillByDefault(Return(cpu_ctl_map));
    ON_CALL(*m_cpu_model, algorithm_valid())
        .WillByDefault(Return(true));

    std::map<std::string, int> uncore_ctl_map = {{"CPU_UNCORE_FREQUENCY_MIN_CONTROL", GEOPM_DOMAIN_PACKAGE},
                                                 {"CPU_UNCORE_FREQUENCY_MAX_CONTROL", GEOPM_DOMAIN_PACKAGE}};
    ON_CALL(*m_uncore_model, controls_recommended())
        .WillByDefault(Return(uncore_ctl_map));
    ON_CALL(*m_uncore_model, algorithm_valid())
        .WillByDefault(Return(true));


    EXPECT_CALL(*m_platform_io, push_control("CPU_FREQUENCY_MAX_CONTROL", _, _)).Times(1);
    EXPECT_CALL(*m_platform_io, push_control("CPU_UNCORE_FREQUENCY_MIN_CONTROL", _, _)).Times(1);
    EXPECT_CALL(*m_platform_io, push_control("CPU_UNCORE_FREQUENCY_MAX_CONTROL", _, _)).Times(1);

    m_cpu_freq_min = 1000000000.0;
    m_cpu_freq_max = 3700000000.0;
    m_cpu_uncore_freq_min = 1200000000.0;
    m_cpu_uncore_freq_max = 2400000000.0;

    m_agent = geopm::make_unique<CPUActivityAgent>(*m_platform_io, *m_platform_topo,
                                                   *m_cpu_model, *m_uncore_model);
    m_num_policy = m_agent->policy_names().size();

    m_mbm_max = 104748888888.88889;
    m_default_policy = {m_cpu_freq_max, m_cpu_freq_min, m_cpu_uncore_freq_max,
                        m_cpu_uncore_freq_min, NAN, m_mbm_max};

    // leaf agent
    m_agent->init(0, {}, false);
}

void CPUActivityAgentTest::TearDown()
{

}

TEST_F(CPUActivityAgentTest, name)
{
    EXPECT_EQ("cpu_activity", m_agent->plugin_name());
    EXPECT_NE("bad_string", m_agent->plugin_name());
}

TEST_F(CPUActivityAgentTest, validate_policy)
{
    const std::vector<double> policy_nan(m_num_policy, NAN);
    std::vector<double> policy;

    std::vector<double> valid_cpu = {0.5, m_cpu_freq_max, m_cpu_freq_min};
    EXPECT_CALL(*m_cpu_model, validate_policy(_)).WillRepeatedly(testing::SetArgReferee<0>(valid_cpu));

    std::vector<double> valid_uncore = {0.5, m_cpu_uncore_freq_max, m_cpu_uncore_freq_min};
    EXPECT_CALL(*m_uncore_model, validate_policy(_)).WillRepeatedly(testing::SetArgReferee<0>(valid_uncore));

    // default policy with 1.2-2.4GHz MBM
    // max rates defined are accepted
    // load default policy
    policy = m_default_policy;

    m_agent->validate_policy(policy);
    // validate policy is unmodified except Phi
    ASSERT_EQ(m_default_policy.size(), policy.size());

    EXPECT_EQ(m_cpu_freq_max, policy[CPU_FREQ_MAX]);
    EXPECT_EQ(m_cpu_freq_min, policy[CPU_FREQ_EFFICIENT]);

    EXPECT_EQ(m_cpu_uncore_freq_max, policy[CPU_UNCORE_FREQ_MAX]);
    EXPECT_EQ(m_cpu_uncore_freq_min, policy[CPU_UNCORE_FREQ_EFFICIENT]);
    // Default value when NAN is passed is 0.5
    EXPECT_EQ(0.5, policy[PHI]);

    // all-NAN policy is accepted
    // setup & load NAN policy
    policy = policy_nan;
    m_agent->validate_policy(policy);
    // validate policy defaults are applied
    ASSERT_EQ(m_num_policy, policy.size());

    EXPECT_EQ(m_cpu_freq_max, policy[CPU_FREQ_MAX]);
    EXPECT_EQ(m_cpu_freq_min, policy[CPU_FREQ_EFFICIENT]);

    EXPECT_EQ(m_cpu_uncore_freq_max, policy[CPU_UNCORE_FREQ_MAX]);
    EXPECT_EQ(m_cpu_uncore_freq_min, policy[CPU_UNCORE_FREQ_EFFICIENT]);
    EXPECT_EQ(0.5, policy[PHI]);

    // non-default policy is accepted
    // setup & load policy
    policy[CPU_FREQ_MAX] = m_cpu_freq_max;
    policy[CPU_FREQ_EFFICIENT] = m_cpu_freq_max / 2;
    policy[CPU_UNCORE_FREQ_MAX] = m_cpu_uncore_freq_max;
    policy[CPU_UNCORE_FREQ_EFFICIENT] = m_cpu_uncore_freq_max / 2;
    policy[PHI] = 0.1;
    EXPECT_NO_THROW(m_agent->validate_policy(policy));
}

TEST_F(CPUActivityAgentTest, adjust_platform)
{
    std::vector<double> policy;
    policy = m_default_policy;

    //Sample
    std::vector<double> tmp;
    double mock_active = 1.0;
    m_agent->sample_platform(tmp);

    std::vector<double> cpu_freq_req(M_NUM_CORE, m_cpu_freq_max);
    EXPECT_CALL(*m_cpu_model, sample_recommendation("CPU_FREQUENCY_STATUS_MAX_CONTROL")).WillRepeatedly(Return(cpu_freq_req));

    std::vector<double> uncore_freq_req_min(M_NUM_PACKAGE, m_cpu_uncore_freq_min);
    EXPECT_CALL(*m_uncore_model, sample_recommendation("CPU_UNCORE_FREQUENCY_MIN_CONTROL")).WillRepeatedly(Return(uncore_freq_req_min));
    std::vector<double> uncore_freq_req_max(M_NUM_PACKAGE, m_cpu_uncore_freq_max);
    EXPECT_CALL(*m_uncore_model, sample_recommendation("CPU_UNCORE_FREQUENCY_MAX_CONTROL")).WillRepeatedly(Return(uncore_freq_req_max));

    //Check frequency
    EXPECT_CALL(*m_platform_io, adjust(CPU_FREQUENCY_CONTROL_IDX, m_cpu_freq_max)).Times(M_NUM_CORE);

    EXPECT_CALL(*m_platform_io, adjust(CPU_UNCORE_MIN_CONTROL_IDX, m_cpu_uncore_freq_min)).Times(M_NUM_PACKAGE);
    EXPECT_CALL(*m_platform_io, adjust(CPU_UNCORE_MAX_CONTROL_IDX, m_cpu_uncore_freq_max)).Times(M_NUM_PACKAGE);

    //Adjust
    m_agent->adjust_platform(policy);
    //Check a frequency decision resulted in write batch being true
    EXPECT_TRUE(m_agent->do_write_batch());
}
