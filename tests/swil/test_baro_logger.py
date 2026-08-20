# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# SWIL test for apps/baro_logger: the emulated STM32H743's own I2C controller
# drives the real BMP390 device model across every layer that exists between
# them -- Renode's STM32F7_I2C, the runtime-loaded HemerionI2cShmBridge C#
# peripheral, i2c_shm_tcp_bridge's TCP hop, the shared-memory bus, and the
# register-accurate Bmp390I2cSlave fed by the ISA measurement model
# (bmp390_shm_peripheral, the FMU minus FMI). The firmware is the unmodified
# baro_logger ELF; the test asserts its probe banner and that the compensated
# pressure it prints matches the ISA at the peripheral's truth altitude.
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

from conftest import REPO_ROOT, firmware_elf, missing, peripheral

CS_PATH = REPO_ROOT / "sim" / "renode" / "i2c_bridge" / "HemerionI2cBridge.cs"

# ISA pressure at the peripheral's 500 m truth altitude is 95461 Pa; the
# part's turn-on bias (sigma 30 Pa, drawn from seed 42) and noise keep every
# sample comfortably inside 95xxx.
TRUTH_ALTITUDE_M = "500"
# BMP390 with SDO tied low. Stated once here and written twice into the .repl,
# because Renode never tells a peripheral its own registration address.
PART_ADDRESS = "0x76"
PRESSURE_PATTERN = r"BARO t=\d+ us p=95\d{3} Pa"


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
        "i2c_shm_tcp_bridge bmp390_shm_peripheral "
        "(or point HEMERION_SWIL_TOOLS_DIR at them)",
    )


def stop(process: subprocess.Popen) -> None:
    """SIGTERM first, SIGKILL only if it will not go.

    bmp390_shm_peripheral unlinks its shared-memory segment on main()'s
    return path and handles SIGTERM precisely so that path is reachable; the
    SIGKILL this used to send skipped it and stranded /dev/shm/<bus> on every
    single run.
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


def cpu_state(machine) -> str:
    """Best-effort program counter and instruction count.

    Distinguishes a CPU spinning in a fault handler from one still making
    progress, which is the question when the firmware produces no output at
    all. Renode's API surface varies by version, so this must never be the
    reason a test errors.
    """
    try:
        cpu = peripheral(machine, "sysbus.cpu").internal
        return f"PC={cpu.PC} executed={cpu.ExecutedInstructions}"
    except Exception as error:  # noqa: BLE001 -- diagnostics only
        return f"(unavailable: {error})"


def check_alive(helper) -> None:
    """Raise with the helper's *own* diagnostics if it has exited.

    Both helpers fail in ways that used to be invisible here -- a stale bus
    segment, a taken port, no peripheral answering within --wait-s. Nothing
    checked, so the test carried on, every Transact got ECONNREFUSED, and the
    run failed 30 s later on the missing BARO banner, blaming baro_logger for
    a harness problem.
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


def wait_until_serving(port: int, helpers, timeout_s: float = 40.0) -> None:
    """Block until the bridge accepts a connection, failing fast if a helper
    dies first.

    Also removes the start-order race the firmware's 1 s probe retry used to
    absorb: with the bridge known to be listening before the emulation starts,
    the first transactions are real ones rather than expected-to-fail noise.
    A probe connect is safe -- the bridge returns to accept() on a clean
    disconnect, by design, so Renode can be restarted against a live bridge.
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


def test_baro_logger_reads_the_simulated_part(renode_machine):
    elf = firmware_elf("baro_logger")
    peripheral_tool = host_tool("bmp390_shm_peripheral")
    bridge_tool = host_tool("i2c_shm_tcp_bridge")

    # A bus name unique to this *run*, not merely to this PID: the segment
    # outlives a crashed run, and a PID-recycling host (short-lived
    # containers, a low kernel.pid_max) would then collide with it -- the
    # peripheral exits on its O_EXCL create and the real cause surfaces much
    # later as a missing firmware banner.
    bus_name = f"hemerion_swil_bmp390_{os.getpid()}_{secrets.token_hex(4)}"

    # Every resource is handed to the stack as soon as it exists. Creating them
    # ahead of a try: meant anything raising part-way through setup -- the
    # second Popen, a full /tmp -- skipped cleanup entirely and left a
    # peripheral running forever, since its default mode never exits by itself.
    with contextlib.ExitStack() as stack:
        work_dir = pathlib.Path(tempfile.mkdtemp(prefix="hemerion_swil_bmp390_"))
        stack.callback(shutil.rmtree, work_dir, ignore_errors=True)

        def launch(tool, name, *arguments):
            """Start a helper with its output captured, registering the
            shutdown and the log the failure path will quote."""
            log_path = work_dir / f"{name}.log"
            log = stack.enter_context(log_path.open("w"))
            process = subprocess.Popen([str(tool), *arguments], stdout=log, stderr=subprocess.STDOUT)
            stack.callback(stop, process)
            return (process, name, log_path)

        part = launch(peripheral_tool, "bmp390_shm_peripheral", "--bus", bus_name, "--alt", TRUTH_ALTITUDE_M)
        bridge = launch(bridge_tool, "i2c_shm_tcp_bridge", "--bus", bus_name, "--port", "0", "--wait-s", "30")
        helpers = (part, bridge)

        # --port 0 and read back what the kernel gave it: choosing a free port
        # here instead meant releasing it before the bridge -- which does not
        # bind until it has attached to the shm bus -- could claim it.
        port = bound_port(bridge)

        # Both helpers healthy and the bridge accepting before the emulation
        # starts, so a harness fault is reported as itself rather than as
        # firmware that failed to print its banner.
        wait_until_serving(port, helpers)

        # The C# class must exist before the platform description names it;
        # both are per-emulation, so this runs after the conftest fixture's
        # bare platform boot.
        repl_overlay = work_dir / "bmp390.repl"
        # targetAddress repeats the registration address deliberately: Renode
        # matches the transfer on the registration but the bridge puts
        # targetAddress on the shm bus, and the class no longer defaults it.
        repl_overlay.write_text(
            f"bmp390: I2C.HemerionI2cShmBridge @ i2c1 {PART_ADDRESS}\n"
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
        tester = TerminalTester(peripheral(renode_machine, "sysbus.usart3"), timeout=30.0)
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
                            f"--- cpu ---\n{cpu_state(renode_machine)}",
                            f"--- renode (rcc chatter dropped) ---\n{renode_tail(renode_log)}",
                            f"--- {part[1]} ---\n{tail(part[2])}",
                            f"--- {bridge[1]} ---\n{tail(bridge[2])}",
                        ]
                    )
                )
            return match

        expect("BARO up: BMP390 identified")
        # Two pressure lines, not one: the second proves the data-ready cycle
        # re-arms -- STATUS cleared by the burst, set again by the next
        # conversion -- rather than the first read being a lucky power-on state.
        expect(PRESSURE_PATTERN, treat_as_regex=True)
        expect(PRESSURE_PATTERN, treat_as_regex=True)
