#!/usr/bin/env python
#
#  Copyright (c) 2015 - 2021, Intel Corporation
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in
#        the documentation and/or other materials provided with the
#        distribution.
#
#      * Neither the name of Intel Corporation nor the names of its
#        contributors may be used to endorse or promote products derived
#        from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

from __future__ import absolute_import

import os
import sys
import unittest
import subprocess
import io
import json

import geopm_context
import geopmpy.agent
import geopm_test_launcher
import util
import geopmpy.topo


class TestIntegrationLevelZeroSignals(unittest.TestCase):
    """Test the levelzero signals

    """
    @classmethod
    def setUpClass(cls):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        cls._app_exec_path = os.path.join(script_dir, '.libs', 'test_levelzero_signals')

    def setUp(self):
        #TODO: get num accelerators
        #TODO: verify they're level zero accels and not nvidia

        self._stdout = None
        self._stderr = None

    def tearDown(self):
        pass

    def test_power(self):
        power = geopm_test_launcher.geopmread("LEVELZERO::POWER board_accelerator 0")
        power_max = geopm_test_launcher.geopmread("LEVELZERO::POWER_LIMIT_MAX board_accelerator 0")

        print("Power: {}".format(power));
        print("Power max: {}".format(power_max));

        self.assertGreater(power, 0)

        if(power_max > 0): #Negative value indicates max was not supported
            self.assertLessEqual(power, power_max)

    def test_frequency(self):
        standby_mode = geopm_test_launcher.geopmread("LEVELZERO::STANDBY_MODE board_accelerator 0")
        frequency_gpu = geopm_test_launcher.geopmread("LEVELZERO::FREQUENCY_GPU board_accelerator 0")
        frequency_min_gpu = geopm_test_launcher.geopmread("LEVELZERO::FREQUENCY_MIN_GPU board_accelerator 0")
        frequency_max_gpu = geopm_test_launcher.geopmread("LEVELZERO::FREQUENCY_MAX_GPU board_accelerator 0")
        print("Standby Mode: {}".format(standby_mode));
        print("Frequency GPU: {}".format(frequency_gpu));
        print("Frequency GPU Min: {}".format(frequency_min_gpu));
        print("Frequency GPU Max: {}".format(frequency_max_gpu));

        if(standby_mode == 0): #We may enter idle and see 0 Hz
            self.assertGreaterEqual(frequency_gpu, 0)
        else:
            self.assertGreaterEqual(frequency_gpu, frequency_min_gpu)

        if(frequency_max_gpu > 0): #Negative value indicates max was not supported
            self.assertLessEqual(frequency_gpu, frequency_max_gpu)


if __name__ == '__main__':
    unittest.main()
