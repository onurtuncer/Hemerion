# toolchains/

CMake toolchain files. A toolchain file tells CMake where to find the compiler, how to set system paths, and which flags to apply globally. Presets in `CMakePresets.json` reference these files by path — they are never included manually.

---

## Files

| File | Target | Compiler |
|---|---|---|
| `arm-none-eabi.cmake` | Cortex-M (STM32H7, F4) | `arm-none-eabi-gcc` ≥ 12 |
| `x86_64-linux.cmake` | Native Linux host | System GCC or Clang |
| `x86_64-windows-msvc.cmake` | Native Windows host | MSVC 2022 |

---

## `arm-none-eabi.cmake`

Sets `CMAKE_SYSTEM_NAME` to `Generic` (no OS) and `CMAKE_SYSTEM_PROCESSOR` to `arm`. Applies the baseline Cortex-M flags; BSPs add their MCU-specific flags (`-mcpu=cortex-m7 -mfpu=fpv5-d16 …`) via `target_compile_options` on the BSP interface target — they do not belong here.

```cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY      arm-none-eabi-objcopy)
set(CMAKE_SIZE         arm-none-eabi-size)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Baseline flags — no MCU specifics here; BSP adds -mcpu, -mfpu, -mfloat-abi
add_compile_options(
    -ffunction-sections
    -fdata-sections
    -fno-exceptions
    -fno-rtti
    -Wall -Wextra
)
add_link_options(
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    --specs=nosys.specs
)
```

---

## `x86_64-linux.cmake`

Minimal — mostly a marker so presets can reference a file rather than leaving `CMAKE_TOOLCHAIN_FILE` empty. Sets C++23 as the minimum standard.

```cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

---

## `x86_64-windows-msvc.cmake`

Used for `fmu-native` and `shm_bridge` builds on Windows. Sets `/std:c++latest`, disables `/permissive`, and adds `_WIN32_WINNT=0x0A00` for Windows 10 shared-memory APIs.

```cmake
set(CMAKE_SYSTEM_NAME Windows)
add_compile_options(/std:c++latest /permissive- /W4)
add_compile_definitions(_WIN32_WINNT=0x0A00 NOMINMAX)
```

---

## Adding a toolchain

1. Create `toolchains/<name>.cmake`.
2. Add a new preset in `CMakePresets.json` that sets `"toolchainFile": "${sourceDir}/toolchains/<name>.cmake"`.
3. If the new target requires a new BSP, add it under `bsp/` — see `bsp/README.md`.

Do not add MCU-specific flags (`-mcpu`, `-mfpu`) to a toolchain file. Those belong in the BSP so that one toolchain file covers all Cortex-M variants.
