#!/usr/bin/env python
from udb_test_helper import UDBTestResources, ContextTest
from unittest import TestCase, skipUnless
from udb_serial_device import UDBSerialTestDevice, OopsError
from typing import Generator
import re

class ShellCommandTest(ContextTest):
    udb_serial: UDBSerialTestDevice

    def context(self) -> Generator:
        with UDBSerialTestDevice() as self.udb_serial:
            yield

    def test_help(self) -> None:
        output = self.udb_serial.command("help")
        self.assertRegex(output, "show available commands")

    def test_gpio(self) -> None:
        output = self.udb_serial.command("gpio read E 9")
        self.assertRegex(output, "GPIO port E pin 9 is 0")
        output = self.udb_serial.command("gpio pp_set E 9")
        self.assertRegex(output, "GPIO port E pin 9 is 1")
        output = self.udb_serial.command("gpio read E 9")
        self.assertRegex(output, "GPIO port E pin 9 is 1")
        output = self.udb_serial.command("gpio pp_clear E 9")
        self.assertRegex(output, "GPIO port E pin 9 is 0")
        output = self.udb_serial.command("gpio read E 9")
        self.assertRegex(output, "GPIO port E pin 9 is 0")

    def test_pwm(self) -> None:
        self.udb_serial.command("pwm start 50 60")
        self.udb_serial.command("pwm stop")

    def test_version(self) -> None:
        output = self.udb_serial.command("version")
        self.assertRegex(output, "Interface ver: udb_(.*)_hw:([0-9]*)\r\nBootloader " \
                                 "ver: (.*)\r\n\r\nDONE 0\r")

    def test_adapter_type(self) -> None:
        self.udb_serial.command("adapter_type")

    def test_i2c_probe(self) -> None:
        output = self.udb_serial.command("i2c probe 2")
        self.assertRegex(output, "probing...\r\n0x17\r\n0x50\r\n0x51\r\n0x52\r\n" \
                                            "0x53\r\n0x54\r\n0x55\r\n0x56\r\n0x57")

    def test_measure_power(self) -> None:
        output = self.udb_serial.command("measure_power")
        # TODO (b/220215522): Check if measurement are within expected limits when the
        # implementation is finalized
        bug_220215522_is_done = False
        if bug_220215522_is_done:
            result = re.search(output, "DEFINE YOUR PATTERN")
            if result != None:
                # needs this assert otherwise typing complains cause result is of type
                # Optional[Match]
                assert result is not None
                self.assertLess(int(result.group(1)), 5500, "Voltage is too large")
                self.assertGreater(int(result.group(1)), 4500, "Voltage is too small")
                self.assertLess(int(result.group(2)), 150000, "Current is to large")
                self.assertGreater(int(result.group(2)), 50000, "Current is too small")
            else:
                self.assertTrue(False, "Can't find expected output")

class ShellCommandWithResetTest(TestCase):
    def test_reset(self) -> None:
        with UDBSerialTestDevice() as udb_serial:
            try:
                output = udb_serial.command("reset")
                self.assertTrue(False, f"Expected UDB to reset, but it didn't. Serial " \
                                       f"output: {output}")
            except OSError:
                pass
        with UDBSerialTestDevice() as udb_serial:
            self.assertLess(udb_serial.get_time_to_open(),
                            UDBTestResources.get_expected_boot_timedelta(),
                            msg="Regression in boot time")

    @skipUnless(UDBTestResources.should_run_all_tests(),
                "this test runs only with the --run-all flag and you have to diconnect your " \
                "debugger from UDB otherwise the assert will halt UDB")
    def test_assert(self) -> None:
        with UDBSerialTestDevice() as udb_serial:
            try:
                output = udb_serial.command("fault test_assert")
                self.assertTrue(False, "Expected UDB to reset, but it didn't. Please make sure " \
                                       "there is no debugger connected to UDB, because then the " \
                                       "assert will cause UDB to halt! Serial output: " \
                                       f"{output}")
            except OopsError:
                pass
        with UDBSerialTestDevice() as udb_serial:
            self.assertLess(udb_serial.get_time_to_open(),
                            UDBTestResources.get_expected_boot_timedelta(),
                            msg="Regression in boot time")
