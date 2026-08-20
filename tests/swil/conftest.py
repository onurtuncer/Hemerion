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
import os
import pathlib
import struct

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


def missing(what: str, remedy: str):
    """Skip -- or, under HEMERION_SWIL_STRICT, fail.

    tests/swil runs against whatever was last built rather than driving the
    builds itself, so locally a missing artefact is a convenience skip. In CI
    that politeness is a trap: ctest reports a pytest skip as a *pass*, so a
    job that happens to build none of the artefacts goes green having tested
    nothing. CI sets HEMERION_SWIL_STRICT=1 and gets a hard failure naming
    the build step it is missing.
    """
    message = f"{what} -- {remedy}"
    if os.environ.get("HEMERION_SWIL_STRICT", "") not in ("", "0", "false", "False"):
        pytest.fail(f"{message} [HEMERION_SWIL_STRICT]", pytrace=False)
    pytest.skip(message)


def _is_cross_elf(path: pathlib.Path) -> bool:
    """True for an ELF built for 32-bit ARM (EM_ARM), i.e. one Renode's
    Cortex-M platform can actually load. The build/ glob below would otherwise
    happily hand back a native_linux build of the same app name and fail much
    later, inside load_elf(), with nothing pointing at the real cause."""
    try:
        with open(path, "rb") as elf:
            header = elf.read(20)
    except OSError:
        return False
    return header[:4] == b"\x7fELF" and int.from_bytes(header[18:20], "little") == 0x28


def function_at(elf_path: pathlib.Path, address: int) -> str:
    """Name the function containing *address*, by reading the ELF's own
    symbol table.

    Done here rather than by shelling out to arm-none-eabi-addr2line because
    the harness runs where the *toolchain need not be* -- and because a stray
    local build of the same app resolves the same address to a different
    (wrong) symbol, which is worse than no answer. Parsing the ELF the test
    actually loaded cannot drift from it.
    """
    try:
        data = elf_path.read_bytes()
        section_offset, = struct.unpack_from("<I", data, 0x20)
        entry_size, count, _ = struct.unpack_from("<HHH", data, 0x2E)
        sections = [
            struct.unpack_from("<IIIIIIIIII", data, section_offset + i * entry_size) for i in range(count)
        ]
        symtab = next(sec for sec in sections if sec[1] == 2)  # SHT_SYMTAB
        strtab = sections[symtab[6]]

        best = None
        for i in range(symtab[5] // symtab[9]):
            name, value, size, info, _, _ = struct.unpack_from("<IIIBBH", data, symtab[4] + i * symtab[9])
            # STT_FUNC, and clear the Thumb bit the low address carries
            if (info & 0xF) != 2 or not size:
                continue
            start = value & ~1
            if start <= address < start + size:
                end = data.index(b"\0", strtab[4] + name)
                best = (data[strtab[4] + name : end].decode(), address - start, size)
                break
        if best is None:
            return "no function covers that address"
        return f"{best[0]} +{best[1]} of {best[2]}"
    except Exception as error:  # noqa: BLE001 -- diagnostics only
        return f"(unresolved: {error})"


def firmware_elf(app: str) -> pathlib.Path:
    """Path to a cross-compiled app's ELF.

    HEMERION_SWIL_FIRMWARE_DIR -- set by tests/swil/CMakeLists.txt to the
    apps/ directory of the build tree that registered these tests -- wins, so
    the harness follows whichever cross preset is driving it. The old
    hardcoded build/renode-h743 was invisible to the test-swil preset CI
    actually builds (binaryDir build/test-swil), which is how the whole SWIL
    suite came to silently skip itself there. The build/ glob remains for a
    bare `pytest` run outside ctest.
    """
    candidates = []
    firmware_dir = os.environ.get("HEMERION_SWIL_FIRMWARE_DIR")
    if firmware_dir:
        candidates.append(pathlib.Path(firmware_dir) / app / app)
    candidates.extend(sorted((REPO_ROOT / "build").glob(f"*/apps/{app}/{app}")))
    for candidate in candidates:
        if candidate.exists() and _is_cross_elf(candidate):
            return candidate
    missing(
        f"{app} firmware not built for the target",
        f"run: cmake --build --preset test-swil --target {app}",
    )


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
