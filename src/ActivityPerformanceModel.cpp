/*
 * Copyright (c) 2015 - 2022, Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <cmath>
#include <unistd.h>

#include "PlatformIOProf.hpp"
#include "geopm/Exception.hpp"
#include "geopm/Helper.hpp"
#include "geopm/PlatformIO.hpp"
#include "geopm/PlatformTopo.hpp"
#include "config.h"
#include "CPUActivityPerformanceModelImp.hpp"
#include "UncoreActivityPerformanceModelImp.hpp"

namespace geopm
{
    //TODO: require domain as part of constructor
    static std::unique_ptr<ActivityPerformanceModel> make_unique_perf_model(int domain)
    {
        std::unique_ptr<ActivityPerformanceModel> result;
        if (domain == 0) {
            result = geopm::make_unique<CPUActivityPerformanceModelImp>();
        }
        else if (domain == 1) {
            result = geopm::make_unique<UncoreActivityPerformanceModelImp>();
        }
        return result;
    }

    ActivityPerformanceModel &perf_model(int domain)
    {
        static std::unique_ptr<ActivityPerformanceModel> instance = make_unique_perf_model(domain);
        return *instance;
    }
}
