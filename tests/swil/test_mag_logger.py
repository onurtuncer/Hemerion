# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# SWIL test for apps/mag_logger: the emulated STM32H743's own I2C controller
# drives the real MMC5983MA device model across every layer that exists
# between them -- Renode's STM32F7_I2C, the runtime-loaded HemerionI2cShmBridge
# C# peripheral, mmc5983ma_shm_peripheral's TCP hop via i2c_shm_tcp_bridge, the
# shared-memory bus, and the register-accurate Mmc5983maI2cSlave fed by the ISA
# measurement model. The firmware is the unmodified mag_logger ELF.
#
# **What this proves that test_baro_logger.py cannot.** The BMP390 is
# configured once and then read: its whole SWIL loop is write-some-registers,
# poll, burst. This part is *commanded*, and its bring-up is a stateful,
# blocking, multi-transaction handshake -- SET, trigger, poll until done,
# burst; RESET, trigger, poll until done, burst; average; SET again -- with
# the driver blocked on the emulated core the entire time. Every one of those
# steps crosses all five layers above. If any of them reorders, drops, or
# mis-addresses a transaction, the recovered bridge offset is wrong and the
# assertion below catches it.
#
# So the load-bearing assertion here is not the field lines: it is that the
# offset the firmware prints matches the one the peripheral independently
# reports having been born with. A firmware that skipped the pair entirely
# would still print plausible field values -- wrong by ~30 uT, which looks
# like a field -- and only the offset comparison distinguishes the two.
#
# Skips when the firmware or the two host tools have not been built -- the
# tools are Linux binaries here, built by a *native* preset's
# sim/i2c_shm/tools targets, which a cross build cannot produce (see
# conftest's philosophy on firmware_elf()). Under HEMERION_SWIL_STRICT, which
# CI sets, those skips become failures: ctest scores a skipped pytest as a
# pass, so a job missing the artefacts would report success having run none
# of this.
# ------------------------------------------------------------------------------
import contextlib
import os
import pathlib
import secrets
import re
import shutil
import socket
import subprocess
import tempfile
import time

from pyrenode3.wrappers import Emulation, Monitor, TerminalTester

from conftest import REPO_ROOT, firmware_elf, function_at, missing, peripheral

CS_PATH = REPO_ROOT / "sim" / "renode" / "i2c_bridge" / "HemerionI2cBridge.cs"

# The truth field the peripheral holds: roughly Istanbul's, ~48 uT total and
# steeply inclined. Passed in explicitly rather than relying on the tool's
# defaults, so this file states what it expects to read back.
TRUTH_X_UT = 22.0
TRUTH_Y_UT = -6.0
TRUTH_Z_UT = 41.0

# The MMC5983MA has no address strap -- the ordering guide defines one I2C
# address code -- so this is the address, not a choice. Stated once here and
# written twice into the .repl, because Renode never tells a peripheral its
# own registration address.
PART_ADDRESS = "0x30"

# "MAG offset x=3061 y=-5217 z=-4083 LSB"
OFFSET_PATTERN = r"MAG offset x=(-?\d+) y=(-?\d+) z=(-?\d+) LSB"
# "MAG t=163840 us x=22001 y=-6000 z=41002 nT"
FIELD_PATTERN = r"MAG t=\d+ us x=(-?\d+) y=(-?\d+) z=(-?\d+) nT"
# The peripheral's own banner: "bridge offset (seed 42): 3061/-5217/-4083 LSB = ..."
PERIPHERAL_OFFSET_PATTERN = r"bridge offset \(seed \d+\): (-?\d+)/(-?\d+)/(-?\d+) LSB"

# One count of the part's fixed 163.84 LSB/uT scale, in nT -- the floor on any
# comparison of a printed field value.
QUANTUM_NT = 1000.0 / 163.84


def host_tool(name: str) -> pathlib.Path:
    """Locate a host-side bridge tool built for *this* OS (the one pytest
    runs under -- Linux/WSL in the usual split), searching every build tree
    plus an explicit HEMERION_SWIL_TOOLS_DIR override.

    These cannot come from the cross build that registers this test: sim/ is
    host-only and CMAKE_CROSSCOMPILING hard-errors on HEMERION_BUILD_SIM, so
    a companion native build has to supply them (see .github/workflows/
    swil.yml). Missing, that is a skip locally and -- via missing() -- a
    failure under HEMERION_SWIL_STRICT, because a skipped pytest is a passed
    ctest and this whole loop would otherwise be green and vacuous.
    """
    candidates = []
    override = os.environ.get("HEMERION_SWIL_TOOLS_DIR")
    if override:
        candidates.append(pathlib.Path(override) / name)
    candidates.extend(sorted((REPO_ROOT / "build").glob(f"*/sim/i2c_shm/tools/{name}")))
    for candidate in candidates:
        if candidate.exists() and os.access(candidate, os.X_OK):
            return candidate
    missing(
        f"{name} not built for this OS",
        "run: cmake --build --preset test-native --target "
        "i2c_shm_tcp_bridge mmc5983ma_shm_peripheral "
        "(or point HEMERION_SWIL_TOOLS_DIR at them)",
    )


def stop(process: subprocess.Popen) -> None:
    """SIGTERM first, SIGKILL only if it will not go.

    mmc5983ma_shm_peripheral unlinks its shared-memory segment on main()'s
    return path and handles SIGTERM precisely so that path is reachable; a
    SIGKILL skips it and strands /dev/shm/<bus> for the life of the machine.
    """
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def tail(log_path: pathlib.Path, limit: int = 4000) -> str:
    """The last of a helper's captured stdout+stderr, for a failure message."""
    try:
        text = log_path.read_text(errors="replace").strip()
    except OSError as error:
        return f"(could not read {log_path}: {error})"
    return text[-limit:] if text else "(no output)"


def renode_tail(log_path: pathlib.Path, limit: int = 3000) -> str:
    """The Renode log with the RCC chatter dropped.

    stm32h743.repl models the clock tree as an unimplemented region, so boot
    alone emits hundreds of "rcc: Unhandled read/write" warnings -- enough to
    fill any tail budget and hide the lines that actually matter: the C#
    bridge's own logging, and any unhandled access to the I2C region.
    """
    kept = [line for line in tail(log_path, limit=400_000).split("\n") if "rcc:" not in line]
    text = "\n".join(kept).strip()
    return text[-limit:] if text else "(nothing but rcc chatter)"


def cpu_state(machine, elf: pathlib.Path, samples: int = 3, gap_s: float = 0.5) -> str:
    """Where the CPU is, sampled more than once.

    A single program counter says nothing about whether the core is wedged or
    merely busy. Three samples separated in time do: the same address every
    time is a tight spin, different addresses inside one function is a loop
    that is not escaping, and different functions means the CPU is still
    getting work done and the fault is elsewhere.

    Worth more here than in the barometer's test: this firmware spends its
    bring-up *blocked* inside Mmc5983maDriver::trigger_and_wait, so "spinning
    in one function" is the expected shape of a stalled handshake and names
    the culprit directly.
    """
    try:
        cpu = peripheral(machine, "sysbus.cpu").internal
        seen = []
        for index in range(samples):
            if index:
                time.sleep(gap_s)
            raw = cpu.PC
            seen.append(int(getattr(raw, "RawValue", raw)))

        described = [function_at(elf, pc) for pc in seen]
        # function_at() renders as "name +offset of size"; group on the name
        # alone, or three offsets within one function read as three functions.
        names = {text.split(" +")[0] for text in described}

        lines = [f"executed={cpu.ExecutedInstructions}"]
        lines += [f"  PC={pc:#x}  {text}" for pc, text in zip(seen, described)]
        if len(set(seen)) == 1:
            lines.append("  -> identical every sample: wedged at one instruction, not looping")
        elif len(names) == 1:
            lines.append("  -> moving inside one function: a loop it is not escaping")
        else:
            lines.append("  -> spread across functions: the CPU is still doing work")
        return "\n".join(lines)
    except Exception as error:  # noqa: BLE001 -- diagnostics only
        return f"(unavailable: {error})"


def check_alive(helper) -> None:
    """Raise with the helper's *own* diagnostics if it has exited.

    Both helpers fail in ways that would otherwise be invisible here -- a
    stale bus segment, a taken port, no peripheral answering within --wait-s.
    Unchecked, the test carries on, every Transact gets ECONNREFUSED, and the
    run fails much later on a missing MAG banner, blaming mag_logger for a
    harness problem.
    """
    process, name, log_path = helper
    if process.poll() is None:
        return
    raise AssertionError(
        "\n".join(
            [f"{name} exited with code {process.returncode} -- its output was:", tail(log_path)]
        )
    )


def bound_port(bridge, timeout_s: float = 40.0) -> int:
    """The port i2c_shm_tcp_bridge actually bound, read back from its banner.

    Replaces picking a free port here and hoping: the probe socket had to be
    closed before the port was handed over, and the bridge does not bind until
    it has finished attaching to the shm bus -- up to its whole --wait-s. Any
    other process taking the port in that window won the race and the bridge
    exited. Launching it with --port 0 and reading back what the kernel gave
    it closes the window entirely.
    """
    deadline = time.monotonic() + timeout_s
    _, name, log_path = bridge
    while time.monotonic() < deadline:
        check_alive(bridge)
        match = re.search(r"127\.0\.0\.1:(\d+)", tail(log_path))
        if match:
            return int(match.group(1))
        time.sleep(0.2)
    raise AssertionError(f"{name} never reported a bound port within {timeout_s:.0f} s")


def peripheral_bridge_offset(part, timeout_s: float = 40.0):
    """The bridge offset the peripheral drew for this run, from its banner.

    This is the truth the firmware's SET/RESET pair has to recover, and it is
    read back rather than hardcoded so the test does not silently depend on
    the tool's RNG staying byte-identical across libstdc++ versions.
    """
    deadline = time.monotonic() + timeout_s
    _, name, log_path = part
    while time.monotonic() < deadline:
        check_alive(part)
        match = re.search(PERIPHERAL_OFFSET_PATTERN, tail(log_path))
        if match:
            return tuple(int(group) for group in match.groups())
        time.sleep(0.2)
    raise AssertionError(f"{name} never reported its bridge offset within {timeout_s:.0f} s")


def wait_until_serving(port: int, helpers, timeout_s: float = 40.0) -> None:
    """Block until the bridge accepts a connection, failing fast if a helper
    dies first.

    Also removes the start-order race the firmware's 1 s probe retry would
    otherwise absorb: with the bridge known to be listening before the
    emulation starts, the first transactions are real ones rather than
    expected-to-fail noise. A probe connect is safe -- the bridge returns to
    accept() on a clean disconnect, by design.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for helper in helpers:
            check_alive(helper)
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=1.0):
                return
        except OSError:
            time.sleep(0.2)
    raise AssertionError(f"i2c_shm_tcp_bridge never listened on 127.0.0.1:{port} within {timeout_s:.0f} s")


def try_monitor(monitor, command: str) -> None:
    """Run a monitor command for diagnostics only, swallowing failures.

    A Renode build that spells one of these differently must not turn a
    diagnosable test failure into an unrelated harness error.
    """
    try:
        monitor.execute(command)
    except Exception:  # noqa: BLE001 -- diagnostics must never be the cause of a failure
        pass


def wait_for(tester, pattern: str, treat_as_regex: bool = False):
    """Pin the str overload of TerminalTester.WaitFor -- see test_led_blink.py."""
    return tester.WaitFor(pattern, None, treat_as_regex, False, False, False)


def test_mag_logger_cancels_the_simulated_parts_bridge_offset(renode_machine):
    elf = firmware_elf("mag_logger")
    peripheral_tool = host_tool("mmc5983ma_shm_peripheral")
    bridge_tool = host_tool("i2c_shm_tcp_bridge")

    # A bus name unique to this *run*, not merely to this PID: the segment
    # outlives a crashed run, and a PID-recycling host (short-lived
    # containers, a low kernel.pid_max) would then collide with it -- the
    # peripheral exits on its O_EXCL create and the real cause surfaces much
    # later as a missing firmware banner.
    bus_name = f"hemerion_swil_mmc5983ma_{os.getpid()}_{secrets.token_hex(4)}"

    # Every resource is handed to the stack as soon as it exists, so anything
    # raising part-way through setup still tears down what already started --
    # the peripheral's default mode never exits by itself.
    with contextlib.ExitStack() as stack:
        work_dir = pathlib.Path(tempfile.mkdtemp(prefix="hemerion_swil_mmc5983ma_"))
        stack.callback(shutil.rmtree, work_dir, ignore_errors=True)

        def launch(tool, name, *arguments):
            """Start a helper with its output captured, registering the
            shutdown and the log the failure path will quote."""
            log_path = work_dir / f"{name}.log"
            log = stack.enter_context(log_path.open("w"))
            process = subprocess.Popen([str(tool), *arguments], stdout=log, stderr=subprocess.STDOUT)
            stack.callback(stop, process)
            return (process, name, log_path)

        part = launch(
            peripheral_tool,
            "mmc5983ma_shm_peripheral",
            "--bus", bus_name,
            "--bx", str(TRUTH_X_UT),
            "--by", str(TRUTH_Y_UT),
            "--bz", str(TRUTH_Z_UT),
        )
        bridge = launch(bridge_tool, "i2c_shm_tcp_bridge", "--bus", bus_name, "--port", "0", "--wait-s", "30")
        helpers = (part, bridge)

        # --port 0 and read back what the kernel gave it: choosing a free port
        # here instead meant releasing it before the bridge -- which does not
        # bind until it has attached to the shm bus -- could claim it.
        port = bound_port(bridge)
        truth_offset = peripheral_bridge_offset(part)

        # Both helpers healthy and the bridge accepting before the emulation
        # starts, so a harness fault is reported as itself rather than as
        # firmware that failed to print its banner.
        wait_until_serving(port, helpers)

        # The C# class must exist before the platform description names it;
        # both are per-emulation, so this runs after the conftest fixture's
        # bare platform boot.
        repl_overlay = work_dir / "mmc5983ma.repl"
        # targetAddress repeats the registration address deliberately: Renode
        # matches the transfer on the registration but the bridge puts
        # targetAddress on the shm bus, and the class no longer defaults it.
        repl_overlay.write_text(
            f"mmc5983ma: I2C.HemerionI2cShmBridge @ i2c1 {PART_ADDRESS}\n"
            f"    targetAddress: {PART_ADDRESS}\n"
            f"    port: {port}\n"
        )
        monitor = Monitor()
        monitor.execute(f"include @{CS_PATH.as_posix()}")
        monitor.execute(f"machine LoadPlatformDescription @{repl_overlay.as_posix()}")

        # Capture both sides of the emulation before it starts. Without these a
        # failure downstream of the harness -- which is where the interesting
        # ones live, since the C# bridge logs its connect and NACK warnings to
        # the Renode log and the firmware narrates itself over usart3 -- reports
        # only that a pattern never appeared, with nothing to say why.
        renode_log = work_dir / "renode.log"
        uart_log = work_dir / "usart3.log"
        try_monitor(monitor, f"logFile @{renode_log.as_posix()}")
        try_monitor(monitor, f"sysbus.usart3 CreateFileBackend @{uart_log.as_posix()}")

        renode_machine.load_elf(str(elf))
        tester = TerminalTester(peripheral(renode_machine, "sysbus.usart3"), timeout=45.0)
        Emulation().StartAll()

        def expect(pattern: str, treat_as_regex: bool = False):
            match = wait_for(tester, pattern, treat_as_regex)
            if match is None:
                # A helper that died is both the likelier cause and the one
                # with the useful message; only with both still up is the
                # firmware itself genuinely the thing that failed.
                for helper in helpers:
                    check_alive(helper)
                raise AssertionError(
                    "\n".join(
                        [
                            f"firmware never printed {pattern!r}, and both helpers were still running --",
                            "so this is the emulated side of the chain, not the harness.",
                            f"--- usart3 ---\n{tail(uart_log)}",
                            f"--- cpu ---\n{cpu_state(renode_machine, elf)}",
                            f"--- renode (rcc chatter dropped) ---\n{renode_tail(renode_log)}",
                            f"--- {part[1]} ---\n{tail(part[2])}",
                            f"--- {bridge[1]} ---\n{tail(bridge[2])}",
                        ]
                    )
                )
            return match

        # Bring-up got all the way through the SET/RESET handshake: this line
        # is printed after calibrate_offset() returned, not before it started.
        expect("MAG up: MMC5983MA identified")

        # The assertion this test exists for. TerminalTester's match object
        # does not expose groups portably across Renode versions, so the line
        # is re-read from the UART capture.
        expect(OFFSET_PATTERN, treat_as_regex=True)
        offset_match = None
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and offset_match is None:
            offset_match = re.search(OFFSET_PATTERN, uart_log.read_text(errors="replace"))
            if offset_match is None:
                time.sleep(0.2)
        assert offset_match is not None, f"offset line never reached the UART capture:\n{tail(uart_log)}"

        recovered = tuple(int(group) for group in offset_match.groups())
        # One count of slack per axis: the pair averages two rounded 18-bit
        # words and halves the sum with integer division.
        for axis, (got, want) in enumerate(zip(recovered, truth_offset)):
            assert abs(got - want) <= 1, (
                f"axis {'xyz'[axis]}: firmware recovered a bridge offset of {got} LSB but the part was born "
                f"with {want} LSB. The SET/RESET pair crossed Renode, the C# bridge, TCP and the shm bus -- "
                f"one of those layers did not deliver it intact.\n"
                f"--- usart3 ---\n{tail(uart_log)}\n"
                f"--- {part[1]} ---\n{tail(part[2])}"
            )

        # And with that offset subtracted, the field the firmware reports is
        # the truth field -- which it would not be if the part had been left
        # RESET (every axis negated) or the offset left uncancelled.
        for _ in range(2):
            # Two lines, not one: the second proves the data-ready cycle
            # re-arms -- Meas_M_Done acknowledged by the read, set again by
            # the next measurement -- rather than the first read being a
            # lucky power-on state.
            field_match = expect(FIELD_PATTERN, treat_as_regex=True)
            assert field_match is not None

        field_match = None
        for line in reversed(uart_log.read_text(errors="replace").splitlines()):
            field_match = re.search(FIELD_PATTERN, line)
            if field_match:
                break
        assert field_match is not None, f"no field line in the UART capture:\n{tail(uart_log)}"

        got_nt = tuple(int(group) for group in field_match.groups())
        want_nt = (TRUTH_X_UT * 1000.0, TRUTH_Y_UT * 1000.0, TRUTH_Z_UT * 1000.0)
        # The peripheral's default hard-iron sigma is 1 uT per axis and its
        # noise 0.04 uT; neither is cancellable by SET/RESET, so bound at
        # 5 sigma of both plus the print quantum.
        bound_nt = 5.0 * (1000.0 + 40.0) + QUANTUM_NT
        for axis, (got, want) in enumerate(zip(got_nt, want_nt)):
            assert abs(got - want) <= bound_nt, (
                f"axis {'xyz'[axis]}: firmware reported {got} nT against a truth field of {want:.0f} nT "
                f"(bound {bound_nt:.0f} nT). A sign flip here means the part was left RESET; an error near "
                f"the bridge offset means the calibration was not applied.\n"
                f"--- usart3 ---\n{tail(uart_log)}"
            )
