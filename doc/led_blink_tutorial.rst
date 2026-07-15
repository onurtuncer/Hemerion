.. ------------------------------------------------------------------------------
.. Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _led_blink_tutorial:

LED Blink Tutorial (``apps/led_blink``)
========================================

``apps/led_blink`` is the project's first complete firmware application and its
primary SWIL (Software-in-the-Loop) example. A single FreeRTOS task toggles
**LD1** (green LED, PB0) on the NUCLEO-H743ZI2 baseboard every 500 ms and
prints ``LED ON`` / ``LED OFF`` over **USART3** (ST-LINK VCP). The companion
test ``tests/swil/test_led_blink.py`` boots this firmware inside Renode and
asserts on those UART strings, proving the scheduler, GPIO HAL, and UART HAL
all work end-to-end without real hardware.

.. contents:: On this page
   :local:
   :depth: 2

---

What the app does
-----------------

``main.cpp`` follows the minimal BSP bring-up sequence:

.. code-block:: cpp

    int main() {
        hal_board_init();                                            // clocks, MPU, caches
        hal_uart_init(3, 115200);                                    // USART3 → ST-LINK VCP
        hal_gpio_set_mode('B', 0, HAL_GPIO_MODE_OUTPUT_PUSH_PULL);  // LD1 = PB0

        xTaskCreate(led_blink_task, "led_blink",
                    configMINIMAL_STACK_SIZE, nullptr,
                    tskIDLE_PRIORITY + 1, nullptr);

        vTaskStartScheduler();   // does not return on success
        for (;;) {}
    }

The ``led_blink_task`` runs in an infinite loop:

.. code-block:: cpp

    void led_blink_task(void*) {
        bool led_on = false;
        for (;;) {
            led_on = !led_on;
            hal_gpio_write('B', 0, led_on);
            uart_println(led_on ? "LED ON" : "LED OFF");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

At 1 Hz (500 ms on, 500 ms off), the UART output is:

.. code-block:: text

    LED ON
    LED OFF
    LED ON
    ...

---

Build
-----

The ``renode-h743`` CMake preset cross-compiles for the STM32H743 using the
ARM GNU Toolchain. Run from the repository root:

.. code-block:: powershell

    cmake --preset renode-h743
    cmake --build --preset renode-h743 --target led_blink

Outputs land in ``build/renode-h743/apps/led_blink/``:

.. list-table::
   :header-rows: 1

   * - File
     - Description
   * - ``led_blink``
     - ELF (used by Renode and ``arm-none-eabi-*`` tools)
   * - ``led_blink.hex``
     - Intel HEX (STM32CubeProgrammer / OpenOCD)
   * - ``led_blink.bin``
     - Raw binary (``st-flash`` / DFU)

The target is guarded by ``if(TARGET hemerion_bsp_stm32h743_nucleo)`` in its
``CMakeLists.txt``, so presets that select a different or absent BSP (e.g.,
``test-native``) skip it silently rather than fail.

---

Flash to real hardware
----------------------

Connect the board over ST-LINK (the built-in USB debugger on the Nucleo-144
baseboard).

**STM32CubeProgrammer CLI:**

.. code-block:: bash

    STM32_Programmer_CLI -c port=SWD \
        -w build/renode-h743/apps/led_blink/led_blink.hex -v -rst

**OpenOCD:**

.. code-block:: bash

    openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
        -c "program build/renode-h743/apps/led_blink/led_blink.hex verify reset exit"

After flashing, LD1 (green, top-left of the board) blinks at 1 Hz.

---

Monitor serial output
---------------------

USART3 is routed through the ST-LINK USB to a virtual COM port (CDC-ACM).
Baud rate: **115200 8N1**.

.. code-block:: bash

    # Linux / WSL
    minicom -b 115200 -D /dev/ttyACM0

    # Windows — replace COM<N> with the Device Manager port
    putty.exe -serial COM<N> -sercfg 115200,8,n,1,N

---

Run under Renode (SWIL)
-----------------------

No hardware required. Renode simulates the STM32H743 die
(``sim/renode/boards/nucleo_h743zi2.repl``) and pyrenode3 observes the
simulated USART3 from Python.

.. note::

   **Windows users:** pyrenode3 requires Renode's CoreCLR/Linux build and must
   run from WSL2 — the Windows Renode installer ships a .NET-Framework build
   that pyrenode3 cannot load. See :ref:`swil_windows_setup` for the one-time
   setup before running the commands below.

Build the firmware on the Windows host first (the WSL side only runs tests):

.. code-block:: powershell

    cmake --build --preset renode-h743 --target led_blink

Then, from WSL2:

.. code-block:: bash

    cd /mnt/d/Dev/Hemerion/tests/swil

    # Required every shell (or add to ~/.bashrc):
    export PYRENODE_BUILD_DIR=/opt/renode
    export PYRENODE_BUILD_OUTPUT=bin

    ~/swilvenv/bin/python3 -m pytest test_led_blink.py -v

Expected output:

.. code-block:: text

    test_led_blink.py::test_led_blinks PASSED    [100%]
    1 passed in Xs

The test boots the firmware, waits (up to 5 s per assertion) for
``LED ON`` → ``LED OFF`` → ``LED ON`` in the simulated UART stream, and passes
when all three are received.

.. note::

   Renode runs the Cortex-M7 at virtual time, not wall-clock time. The first
   run can take several minutes on a host where Renode hasn't warmed up its
   JIT yet. Subsequent runs in the same pytest session are faster.

---

How the SWIL harness works
---------------------------

``tests/swil/conftest.py`` provides the ``renode_machine`` fixture:

1. Sets ``$repl`` to the absolute path of ``nucleo_h743zi2.repl`` via a
   ``Monitor`` command (``execute_script`` resolves relative paths against
   Renode's own install root, not the repo).
2. Runs ``sim/renode/scripts/swil_lockstep.resc``, which creates the Renode
   machine and loads the platform description.
3. Yields the ``Machine`` handle to the test function.
4. Calls ``Emulation().clear()`` on teardown.

``test_led_blinks`` then:

1. Loads the ELF via ``machine.load_elf()``.
2. Attaches a ``TerminalTester`` to the simulated USART3.
3. Calls ``Emulation().StartAll()`` to start the virtual CPU.
4. Asserts on ``tester.WaitFor("LED ON", ...)`` etc.

For details on the Renode platform description and the pyrenode3 API used,
see :ref:`swil_windows_setup` and the source files
``tests/swil/conftest.py``, ``tests/swil/test_led_blink.py``.

---

Relationship to the BSP
------------------------

``apps/led_blink`` is the first application that actually exercised
``bsp/stm32h743_nucleo`` at runtime. Two pre-existing bugs in the BSP were
discovered and fixed during its development:

- **MPU background region ``DisableExec``:** The ``MPU_Config()`` call in
  ``system_stm32h7xx.c`` had ``MPU_INSTRUCTION_ACCESS_DISABLE`` on the
  catch-all 4 GB region, making all code non-executable (instant MemManage
  Fault at reset). The correct value for the background region is
  ``MPU_INSTRUCTION_ACCESS_ENABLE``.

- **FreeRTOS indirect routing:** The GCC_ARM_CM7 port requires SVC/PendSV
  handlers to be installed either directly (function pointer = FreeRTOS symbol)
  or indirectly (wrapper that branches to the FreeRTOS symbol). Because
  ``.isr_vector`` lives in flash (read-only), the vector table cannot be
  patched at runtime — indirect routing via ``freertos_hooks.c`` is mandatory,
  and ``configCHECK_HANDLER_INSTALLATION`` must be set to ``0`` to suppress
  the port's own Direct-Routing startup assertion.

See :ref:`bsp_freertos_wiring` for a full explanation of both.
