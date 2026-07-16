#pragma once

// ---------------------------------------------------------------------------
// hemerion/safety/flight_safe.hpp
//
// HEMERION_FLIGHT_SAFE_BEGIN / HEMERION_FLIGHT_SAFE_END
//
// Wrap any code region that must meet flight-software constraints.
// The macros push the current optimisation state, apply a safe profile,
// and pop back on END — so surrounding code is unaffected.
//
// Flags applied inside a flight-safe region:
//
//   -O2                   Predictable timing; never O0 in production firmware.
//   -fno-strict-overflow  No UB-based signed overflow folding.
//   -ffp-contract=off     No fused multiply-add; deterministic FP rounding.
//   -fno-omit-frame-pointer  Stack frames intact for fault handler unwinding.
//   -fno-tree-vectorize   No auto-vectorisation; byte-level correctness preserved.
//   -fno-aggressive-loop-optimizations  No loop transforms that assume no overflow.
//
// NOT applied (intentional omissions):
//   -fwrapv  — changes ABI; apply project-wide if needed, not per region.
//   -O0      — wrong for flight code; timing becomes unpredictable.
//
// Compiler support:
//   GCC (arm-none-eabi-gcc, x86_64-linux-gnu-gcc): #pragma GCC optimize / target
//   Clang (clang++, used for native test/FMU builds): #pragma clang optimize
//
//   Both compilers support #pragma GCC optimize on their common subset.
//   Clang accepts it for compatibility. Clang-specific fp pragmas are added
//   where GCC equivalents are insufficient.
//
//   MSVC: __pragma(optimize) — stubbed to no-op; MSVC is host-only (FMU/sim
//   builds) where these constraints are informational, not safety-critical.
//
// Usage:
//   #include "hemerion/safety/flight_safe.hpp"
//
//   HEMERION_FLIGHT_SAFE_BEGIN
//   // ... flight-critical code ...
//   HEMERION_FLIGHT_SAFE_END
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// GCC and Clang (covers arm-none-eabi-gcc and all native test compilers)
// ---------------------------------------------------------------------------
#if defined(__GNUC__) && !defined(_MSC_VER)

#define HEMERION_FLIGHT_SAFE_BEGIN                                                                                     \
  _Pragma("GCC push_options") _Pragma("GCC optimize(\"O2\")") _Pragma("GCC optimize(\"no-strict-overflow\")")          \
      _Pragma("GCC optimize(\"no-omit-frame-pointer\")") _Pragma("GCC optimize(\"no-tree-vectorize\")")                \
          _Pragma("GCC optimize(\"no-aggressive-loop-optimizations\")") _Pragma("GCC "                                 \
                                                                                "optimize(\"fp-contract="              \
                                                                                "off\")") static_assert(true,          \
                                                                                                        "HEMERION_"    \
                                                                                                        "FLIGHT_SAFE_" \
                                                                                                        "BEGIN — "   \
                                                                                                        "semicolon "   \
                                                                                                        "required")

#define HEMERION_FLIGHT_SAFE_END                                                                                       \
  _Pragma("GCC pop_options") static_assert(true, "HEMERION_FLIGHT_SAFE_END — semicolon required")

// ---------------------------------------------------------------------------
// MSVC — host-only builds (FMU / sim / Windows shm_bridge)
// Safety constraints are informational here; no-op stub avoids build errors.
// ---------------------------------------------------------------------------
#elif defined(_MSC_VER)

#define HEMERION_FLIGHT_SAFE_BEGIN static_assert(true, "HEMERION_FLIGHT_SAFE_BEGIN (MSVC stub)")

#define HEMERION_FLIGHT_SAFE_END static_assert(true, "HEMERION_FLIGHT_SAFE_END (MSVC stub)")

// ---------------------------------------------------------------------------
// Unknown compiler — warn loudly rather than silently doing nothing
// ---------------------------------------------------------------------------
#else
#error "hemerion/safety/flight_safe.hpp: unsupported compiler. \
Add pragmas for your toolchain or define HEMERION_SKIP_FLIGHT_SAFE_PRAGMAS."
#endif

// ---------------------------------------------------------------------------
// Escape hatch: define HEMERION_SKIP_FLIGHT_SAFE_PRAGMAS to unconditionally
// stub both macros. Useful when porting to a new toolchain before pragmas
// are validated. Never define this in production firmware builds.
// ---------------------------------------------------------------------------
#if defined(HEMERION_SKIP_FLIGHT_SAFE_PRAGMAS)
#undef HEMERION_FLIGHT_SAFE_BEGIN
#undef HEMERION_FLIGHT_SAFE_END
#define HEMERION_FLIGHT_SAFE_BEGIN static_assert(true, "flight-safe pragmas skipped")
#define HEMERION_FLIGHT_SAFE_END static_assert(true, "flight-safe pragmas skipped")
#endif