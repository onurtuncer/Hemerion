# toolchains/

CMake toolchain files. A toolchain file tells CMake where to find the compiler, how to set system paths, and which flags to apply globally. Presets in `CMakePresets.json` reference these files by path — they are never included manually.

**A native toolchain file must not set `CMAKE_SYSTEM_NAME`.** CMake sets `CMAKE_CROSSCOMPILING` to `TRUE` whenever that variable is assigned from a toolchain file, *even when it is assigned the host's own name* — and the root `CMakeLists.txt` hard-errors on `CMAKE_CROSSCOMPILING` for `HEMERION_BUILD_FMU`, `_SIM` and `_EXAMPLES`. Both native files used to set it, which is why the native presets could not name a toolchain file at all until it was removed. Cross toolchain files (`arm-none-eabi.cmake`) set it, correctly.

---

## Files

| File | Target | Compiler | Named by |
|---|---|---|---|
| `arm-none-eabi.cmake` | Cortex-M (STM32H7, F4) | `arm-none-eabi-gcc` ≥ 12 | `cross-base` |
| `x86_64-native.cmake` | Native host (dispatcher) | — | `native-base` |
| `x86_64-linux.cmake` | Native Linux host | System GCC or Clang (CI pins gcc-14) | `x86_64-native.cmake` |
| `x86_64-windows-msvc.cmake` | Native Windows host | MSVC 2022, or a VS-bundled clang | `x86_64-native.cmake` |

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

## `x86_64-native.cmake`

The file `native-base` names, and so what `test-native`, `fmu-native` and `examples-native` all get. It holds no settings — it dispatches on `CMAKE_HOST_SYSTEM_NAME` to one of the two files below.

The indirection exists because a preset binds exactly one `toolchainFile`, while those presets are built under the same names on both Linux (gcc-14) and Windows (MSVC). The previous answer was to name no toolchain file at all, which kept the preset names portable at the cost of making both host files dead weight — nothing referenced them, so their flags applied to nothing.

`CMAKE_HOST_SYSTEM_NAME` is already populated when a toolchain file is read, so the branch is safe. `CMAKE_CXX_COMPILER_ID` and `MSVC` are *not* — compiler detection has not run yet — so per-compiler flags below are written as generator expressions, not `if(MSVC)`.

A host that is neither Linux nor Windows gets a `message(WARNING)` and the Linux file, which carries no host-specific flags. Loud, not fatal: an untested host still builds, but nobody gets to believe a host file was applied when none exists.

---

## `x86_64-linux.cmake`

Deliberately thin: C++23, no extensions, and no warning flags. Turning warnings on here would change how every native translation unit compiles across the whole tree — a policy change, not this file's business. Its job is to be the place those flags will go.

```cmake
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
```

---

## `x86_64-windows-msvc.cmake`

Adds `_WIN32_WINNT=0x0A00` for the Windows 10 `CreateFileMapping()`/`MapViewOfFile()` APIs `sim/shm_bridge` uses, and `NOMINMAX` to keep `<windows.h>`'s `min`/`max` macros away from headers calling `std::min`/`std::max`.

```cmake
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/W4>")
add_compile_definitions(_WIN32_WINNT=0x0A00 NOMINMAX)
```

`/W4` is guarded by compiler rather than host: a Visual Studio-bundled `clang++` targets `x86_64-pc-windows-msvc` through a GNU-style driver, where `/W4` is not a valid option. CI uses `cl.exe` via `ilammy/msvc-dev-cmd` and does get it.

`/std:c++latest` was removed. The root `CMakeLists.txt` sets `CMAKE_CXX_STANDARD 23` with extensions off, from which CMake emits the right `/std:` flag itself — and, on MSVC, `/permissive-` with it. Pinning `c++latest` here only fought that.

---

## "…was never applied" — the stale-cache guard

`CMAKE_TOOLCHAIN_FILE` is honoured only on a build tree's **first** configure. If a preset starts naming a toolchain file it did not name before, re-running `cmake --preset` against an existing build directory adopts none of it — and CMake still writes `CMAKE_TOOLCHAIN_FILE` into the cache, so the tree reads as correctly configured while compiling with none of that file's flags. The only native warning is a single "Manually-specified variables were not used by the project" line.

The root `CMakeLists.txt` turns that into a hard error. Every file here sets `HEMERION_TOOLCHAIN_APPLIED` as a **normal** variable — present on a healthy configure and on every reconfigure of a healthy tree, absent exactly when the cache was built without the file. A foreign toolchain (vcpkg, a distribution's own) does not set it and is deliberately not failed over it.

The remedy the error prints:

```bash
cmake --preset <name> --fresh
```

If you add a toolchain file to a preset that had none, expect every existing build directory for that preset to fail once, on purpose, until it is refreshed.

---

## Adding a toolchain

1. Create `toolchains/<name>.cmake`.
2. Add a new preset in `CMakePresets.json` that sets `"toolchainFile": "${sourceDir}/toolchains/<name>.cmake"`.
3. If the new target requires a new BSP, add it under `bsp/` — see `bsp/README.md`.

Do not add MCU-specific flags (`-mcpu`, `-mfpu`) to a toolchain file. Those belong in the BSP so that one toolchain file covers all Cortex-M variants.
