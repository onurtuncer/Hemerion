.. ------------------------------------------------------------------------------
.. Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _swil_windows_setup:

SWIL on Windows: WSL2 + Renode + pyrenode3
==========================================

Hemerion's SWIL tests use **pyrenode3** to host Renode in-process via
pythonnet/CoreCLR. This requires Renode's **.NET 8 / CoreCLR** Linux build.
The Windows Renode installer ships a .NET-Framework build that pyrenode3 cannot
load, so on a Windows development machine the test harness must run inside
**WSL2** (Windows Subsystem for Linux 2), which is also the environment the CI
pipeline uses (Ubuntu 24.04 + Renode container).

The firmware itself is still cross-compiled on the Windows host using the ARM
GNU Toolchain already installed there — only the Renode test harness runs in
WSL2. The WSL filesystem can access the Windows build output via ``/mnt/<drive>``.

.. contents:: On this page
   :local:
   :depth: 2

---

Prerequisites
-------------

- Windows 10 22H2 or Windows 11 (WSL2 requires a kernel with virtualisation support)
- ``winget`` or manual download access for the ARM GNU Toolchain (already
  required for ``cmake --preset renode-h743`` — see ``scripts/install-toolchain.ps1``)
- At least 4 GB free on the drive where you install the WSL2 distro

.. note::

   If your C: drive is low on space, install the WSL2 distro on another drive
   by passing ``--location`` to ``wsl --install`` (see step 1 below). The
   distro VHD grows as packages are installed; a clean Ubuntu 24.04 + Renode
   install uses roughly 1.5 GB.

---

Step 1: Install WSL2 (Ubuntu 24.04)
-------------------------------------

Open **PowerShell as administrator**:

.. code-block:: powershell

    # Install on D: (recommended if C: is tight):
    wsl --install -d Ubuntu-24.04 --location D:\WSL\Ubuntu-24.04

    # Or on the default drive:
    wsl --install -d Ubuntu-24.04

When the distro starts for the first time it prompts for a Unix username and
password. You can also enter as root without a password using:

.. code-block:: powershell

    wsl -d Ubuntu-24.04 -u root

After installation:

.. code-block:: bash

    # Inside WSL:
    sudo apt-get update && sudo apt-get upgrade -y

---

Step 2: Install Renode
-----------------------

Download the Debian package for the **CoreCLR** build (the ``.deb`` ships the
.NET 8 runtime and ``/opt/renode/bin/Renode.dll``):

.. code-block:: bash

    wget https://github.com/renode/renode/releases/download/v1.16.1/renode_1.16.1_amd64.deb
    sudo apt install -y ./renode_1.16.1_amd64.deb

Verify the install:

.. code-block:: bash

    renode --version     # prints "Renode v1.16.1 ..."
    ls /opt/renode/bin/  # should contain Renode.dll, libAntmicro.Renode.*.so, …

.. note::

   ``/usr/bin/renode`` is a shell launcher that calls
   ``dotnet /opt/renode/bin/Renode.dll``. pyrenode3 cannot use this script
   as its ``PYRENODE_BIN`` — it needs the directory of the actual DLL, which
   is ``/opt/renode/bin``. The environment variables in Step 4 handle this.

---

Step 3: Create the pyrenode3 virtual environment
-------------------------------------------------

.. code-block:: bash

    python3 -m venv ~/swilvenv
    ~/swilvenv/bin/pip install -r /mnt/d/Dev/Hemerion/tests/swil/requirements.txt

``requirements.txt`` pins pytest and a specific commit of pyrenode3 from
Antmicro's GitHub that is known to work with Renode 1.16.1:

.. code-block:: text

    pytest>=8.0
    pyrenode3 @ git+https://github.com/antmicro/pyrenode3.git@a80d364...

This step requires internet access (to clone pyrenode3 from GitHub). It takes
1–2 minutes on the first install.

---

Step 4: Set environment variables
----------------------------------

pyrenode3's loader uses two environment variables to locate Renode's binary
directory (``PYRENODE_BUILD_DIR``) and the output subdirectory within it
(``PYRENODE_BUILD_OUTPUT``):

.. code-block:: bash

    export PYRENODE_BUILD_DIR=/opt/renode
    export PYRENODE_BUILD_OUTPUT=bin

Add these to ``~/.bashrc`` to avoid setting them every session:

.. code-block:: bash

    echo 'export PYRENODE_BUILD_DIR=/opt/renode' >> ~/.bashrc
    echo 'export PYRENODE_BUILD_OUTPUT=bin'       >> ~/.bashrc
    source ~/.bashrc

---

Running SWIL tests
------------------

Build the firmware on the **Windows host** first. The WSL side does not
cross-compile — it only runs the test harness against the ELF the Windows
build produced:

.. code-block:: powershell

    # In PowerShell on Windows:
    cmake --build --preset renode-h743 --target led_blink

Then switch to WSL2 and run pytest:

.. code-block:: bash

    cd /mnt/d/Dev/Hemerion/tests/swil
    ~/swilvenv/bin/python3 -m pytest test_led_blink.py -v

Expected output:

.. code-block:: text

    platform linux -- Python 3.12.x, pytest-9.x, pluggy-1.x
    collected 1 item

    test_led_blink.py::test_led_blinks PASSED    [100%]

    1 passed in Xs

To run all SWIL tests:

.. code-block:: bash

    ~/swilvenv/bin/python3 -m pytest tests/swil/ -v

---

Why not ``ctest --preset test-swil``?
--------------------------------------

The ``test-swil`` CMake preset is designed for a single Linux environment
that has both the ARM GNU Toolchain and a CoreCLR Renode build — matching
the CI pipeline (Ubuntu 24.04 + Renode container). It configures,
cross-compiles, and runs tests in one step:

.. code-block:: bash

    # This is the CI flow (Linux only):
    cmake --preset test-swil
    cmake --build --preset test-swil
    ctest --preset test-swil -L swil

On a Windows dev machine this pipeline is intentionally split:
``cmake --preset renode-h743`` on Windows for the cross-compile (ARM toolchain
there), pytest on WSL for the Renode test harness (CoreCLR Renode there). The
``find_package(Renode)`` / ``find_package(Pyrenode3)`` gate in
``tests/swil/CMakeLists.txt`` detects this automatically — when configured on
Windows it silently skips the ``swil.led_blink`` CTest entry rather than
failing, since Windows Renode can't serve pyrenode3.

---

Troubleshooting
---------------

**``libMono.Unix.so`` symlink warning on startup**

.. code-block:: text

    WARNING:root:libMono.Unix.so is not in the expected location. Created symlink.

This is harmless — pyrenode3 creates a compatibility symlink on first use.
It appears once per install and disappears if you reinstall pyrenode3.

**WSL runs out of disk space during ``apt install``**

If ``dpkg`` throws I/O errors mid-install, the WSL distro's VHD is full or
the host drive the VHD lives on is full. Check with:

.. code-block:: bash

    df -h /

Then either free space on the host or unregister the distro and reinstall it
on a larger drive:

.. code-block:: powershell

    # On Windows PowerShell:
    wsl --unregister Ubuntu-24.04
    wsl --install -d Ubuntu-24.04 --location D:\WSL\Ubuntu-24.04

**``pyrenode3`` raises ``ValueError: invalid literal for int()`` reading PC**

The virtual CPU's ``PC`` property sometimes returns a class description
string instead of an integer when Renode's emulation stops abruptly (e.g.,
an unhandled fault or assertion). Wrap the read in a ``try/except`` or check
whether the emulation is still running before reading ``cpu.PC``.

**Test times out waiting for ``LED ON``**

The firmware may be hung in an early boot stage. Use the Renode monitor to
read the program counter while the test is running:

.. code-block:: bash

    # In another WSL terminal, or via Monitor().execute():
    sysbus.cpu PC
    # Then resolve with addr2line:
    arm-none-eabi-addr2line -e /path/to/led_blink -f -C <PC value>

Common causes are a missing sysbus Tag for a peripheral busy-wait (e.g.,
``PWR_D3CR_VOSRDY`` or ``PWR_CSR1_ACTVOSRDY``) or a ``configASSERT`` trap.
See :ref:`bsp_freertos_wiring` for the known cases that were fixed during
initial bring-up.
