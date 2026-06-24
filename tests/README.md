# tests/

Cross-cutting integration and SWIL tests. Unit tests that test a single module in isolation live inside that module's own `test/` subdirectory. This directory is for tests that span multiple modules, test the full RTOS task graph, or require Renode SWIL to exercise real firmware behaviour.

---

## Test categories

| Directory | What it tests | Requires |
|---|---|---|
| `unit/` | Individual module interfaces, no RTOS | Native host only |
| `integration/` | Multi-module message flows, queue wiring | Native host only |
| `swil/` | Full firmware running in Renode, observable via pyrenode3 | Renode + pyrenode3 |
| `fmu/` | FMU packaging, FMI variable naming, step correctness | `fmu-native` build |

---

## Running tests

```bash
# Native unit + integration tests (fast, no Renode)
cmake --preset test-native
cmake --build --preset test-native
ctest --preset test-native --output-on-failure

# SWIL tests (requires Renode on PATH)
cmake --preset test-swil
cmake --build --preset test-swil
ctest --preset test-swil -L swil --output-on-failure

# FMU tests
cmake --preset fmu-native
cmake --build --preset fmu-native
ctest --preset fmu-native -L fmu --output-on-failure
```

---

## SWIL test harness (`swil/`)

SWIL tests use `pyrenode3` to launch a Renode instance, load the firmware ELF, and observe behaviour through the machine API. Each test is a Python file consumed by pytest via a CTest `add_test(... COMMAND pytest ...)` entry.

```
tests/swil/
├── conftest.py             # Renode machine fixture (start / teardown)
├── test_sensor_pipeline.py # IMU task publishes to gnc queue within deadline
├── test_fault_injection.py # Force sensor timeout; assert fault task fires
└── test_can_gateway.py     # EtherCAT frame → CAN frame round-trip latency
```

The `conftest.py` fixture:
1. Creates a `pyrenode3.RenodeWrapper` instance.
2. Loads `sim/renode/scripts/swil_lockstep.resc`.
3. Loads the firmware ELF from the build directory.
4. Yields the machine handle to the test.
5. On teardown, stops Renode and captures the UART log.

Example test structure:

```python
def test_imu_task_deadline(renode_machine):
    machine = renode_machine
    machine.StartEmulation()
    # Advance 100 ticks deterministically
    machine.TimeSource.AdvanceBy(TimeInterval.FromMilliseconds(100))
    uart_log = machine.GetUartOutput("usart3")
    assert "IMU_OK" in uart_log
    assert "DEADLINE_MISS" not in uart_log
```

---

## FMU tests (`fmu/`)

Verify that each module FMU:
- Loads without error via fmi4c
- Exposes the expected variable names and causalities from `model_description.xml`
- Produces numerically correct output for a known input sequence
- Handles `fmi2Reset` and re-instantiation without memory leaks (checked with ASan on Linux)

---

## CI matrix

| Preset | Runner | Trigger |
|---|---|---|
| `test-native` | Ubuntu 24.04 | Every push |
| `fmu-native` | Ubuntu 24.04 | Every push |
| `renode-h743` (build only) | Ubuntu 24.04 | Every push |
| `test-swil` | Ubuntu 24.04 + Renode container | PR merge to `main` |
| `cross-stm32f446` (build only) | Ubuntu 24.04 | Every push |

SWIL tests are gated to PR merge because Renode startup adds ~30 s per test suite.
