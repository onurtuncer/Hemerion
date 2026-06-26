# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Renode machine fixture for tests/swil -- see tests/README.md's "SWIL test
# harness" section. Boots sim/renode/scripts/swil_lockstep.resc once per test;
# the test itself still has to load_elf() its own firmware before starting
# the emulation, since the .resc only brings up the bare platform.
# ------------------------------------------------------------------------------
import pathlib

import pytest
from pyrenode3.wrappers import Emulation, Monitor, Peripheral

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
RESC_PATH = REPO_ROOT / "sim" / "renode" / "scripts" / "swil_lockstep.resc"
REPL_PATH = REPO_ROOT / "sim" / "renode" / "boards" / "nucleo_h743zi2.repl"
MACHINE_NAME = "nucleo_h743zi2"


def peripheral(machine, path: str) -> Peripheral:
    """Look up a peripheral by its full path (e.g. "sysbus.usart3").

    Machine.sysbus.<name> attribute access only resolves *direct* sysbus
    children through Peripheral._elements()'s GetChildrenPeripherals() call,
    which doesn't reliably see the converted wrapper instance -- Machine's
    own GetByName()/indexer (machine["sysbus.usart3"]) is the path Renode's
    own tooling (robot framework, the monitor's `peripheral` command) uses
    and works for any depth.
    """
    return Peripheral(machine.internal[path])


def firmware_elf(app: str) -> pathlib.Path:
    """Path to a cross-compiled app's ELF under the renode-h743 build directory.

    Skips the test rather than failing outright when the firmware hasn't
    been built yet -- tests/swil is meant to run against whatever the
    renode-h743 preset most recently produced, not to drive that build itself.
    """
    elf = REPO_ROOT / "build" / "renode-h743" / "apps" / app / app
    if not elf.exists():
        pytest.skip(f"{elf} not built -- run: cmake --build --preset renode-h743 --target {app}")
    return elf


@pytest.fixture
def renode_machine():
    # execute_script() runs with cwd set to Renode's own install root (see
    # pyrenode3's RenodeLoader.in_root()), not the repo -- swil_lockstep.resc's
    # "@sim/renode/boards/..." relative path would resolve against the wrong
    # directory. Its "$repl?=..." only takes that default when $repl is still
    # unset, so setting it here first (absolute path) overrides it cleanly.
    Monitor().execute(f"$repl=@{REPL_PATH.as_posix()}")
    Monitor().execute_script(str(RESC_PATH))
    machine = Emulation().get_mach(MACHINE_NAME)

    yield machine

    Emulation().clear()
