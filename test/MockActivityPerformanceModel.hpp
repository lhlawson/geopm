/*
 * Copyright (c) 2015 - 2022, Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MOCKACTIVITYPERFORMANCEMODEL_HPP_INCLUDE
#define MOCKACTIVITYPERFORMANCEMODEL_HPP_INCLUDE

#include "gmock/gmock.h"
#include "ActivityPerformanceModel.hpp"

class MockActivityPerformanceModel : public geopm::ActivityPerformanceModel
{
    public:
        MOCK_METHOD(void, init, (), (override));
        MOCK_METHOD(bool, algorithm_valid, (), (override));
        MOCK_METHOD((std::map<std::string, int>), controls_recommended, (), (override));
        MOCK_METHOD(void, update_recommendation, (), (override));
        MOCK_METHOD((std::vector<double>), sample_recommendation, (std::string), (const override));
        MOCK_METHOD(void, validate_policy, (std::vector<double> &in_policy), (const override));
        MOCK_METHOD(void, set_policy, (std::vector<double> &in_policy), (override));
};

#endif
