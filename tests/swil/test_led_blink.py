# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# SWIL smoke test for apps/led_blink. Boots the firmware in Renode against
# sim/renode/boards/nucleo_h743zi2.repl and watches usart3 (the ST-LINK VCP)
# for the "LED ON"/"LED OFF" lines apps/led_blink/main.cpp prints on every
# toggle, confirming the blink task is actually alternating rather than
# stuck in one state (or hung during boot).
# ------------------------------------------------------------------------------
from pyrenode3.wrappers import Emulation, TerminalTester

from conftest import firmware_elf, peripheral


def wait_for(tester, pattern: str):
    """TerminalTester.WaitFor has two overloads (str and str[] pattern) that
    differ only in the trailing bool flags -- pythonnet can't disambiguate a
    bare WaitFor(pattern) call, so the (timeout, treatAsRegex,
    includeUnfinishedLine, pauseEmulation, keep) defaults have to be spelled
    out to pin down the str overload.
    """
    return tester.WaitFor(pattern, None, False, False, False, False)


def test_led_blinks(renode_machine):
    renode_machine.load_elf(str(firmware_elf("led_blink")))

    tester = TerminalTester(peripheral(renode_machine, "sysbus.usart3"), timeout=5.0)
    Emulation().StartAll()

    assert wait_for(tester, "LED ON") is not None
    assert wait_for(tester, "LED OFF") is not None
    assert wait_for(tester, "LED ON") is not None
