#!/usr/bin/env bash
# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Verifies the dev-environment prerequisites from README.md's "Build &
# Installation" table are present and new enough. Check-only: never installs
# or modifies anything. Exits non-zero if a tool required by every preset is
# missing or too old; SWIL-only tools (Renode, pyrenode3) only warn.
# ------------------------------------------------------------------------------
set -uo pipefail

RED=$'\033[0;31m'
YELLOW=$'\033[0;33m'
GREEN=$'\033[0;32m'
CYAN=$'\033[0;36m'
NC=$'\033[0m'

hard_failure=0

version_ge() {
  # version_ge FOUND REQUIRED -> 0 if FOUND >= REQUIRED
  [ "$(printf '%s\n%s\n' "$2" "$1" | sort -V | head -n1)" = "$2" ]
}

report() {
  local name="$1" command="$2" required="$3" pattern="$4" category="$5"

  if ! command -v "$command" >/dev/null 2>&1; then
    local color="$YELLOW"
    [ "$category" = "required" ] && { color="$RED"; hard_failure=1; }
    printf "%-18s %-10s %-34s [%s]${NC}\n" "$name" "--" "${color}MISSING${NC}" "$category"
    return
  fi

  local raw found
  raw=$("$command" --version 2>&1 || true)
  found=$(printf '%s' "$raw" | grep -oE "$pattern" | head -n1 | grep -oE '[0-9]+\.[0-9]+(\.[0-9]+)?' | head -n1)

  if [ -z "$found" ]; then
    printf "%-18s %-10s %-34s [%s]\n" "$name" "?" "${YELLOW}VERSION UNKNOWN${NC}" "$category"
    return
  fi

  if [ -n "$required" ]; then
    if version_ge "$found" "$required"; then
      printf "%-18s %-10s %-34s [%s]\n" "$name" "$found" "${GREEN}OK (>= $required)${NC}" "$category"
    else
      local color="$YELLOW"
      [ "$category" = "required" ] && { color="$RED"; hard_failure=1; }
      printf "%-18s %-10s %-34s [%s]\n" "$name" "$found" "${color}TOO OLD (need >= $required)${NC}" "$category"
    fi
  else
    printf "%-18s %-10s %-34s [%s]\n" "$name" "$found" "${GREEN}OK${NC}" "$category"
  fi
}

echo "${CYAN}Hemerion toolchain check (Linux)${NC}"
echo "==================================="
echo

echo "-- Always required --"
report "git"    "git"    ""      '[0-9]+\.[0-9]+\.[0-9]+'        "required"
report "cmake"  "cmake"  "3.26"  '[0-9]+\.[0-9]+\.[0-9]+'        "required"
report "ninja"  "ninja"  ""      '[0-9]+\.[0-9]+\.[0-9]+'        "required"

echo
echo "-- Cross-compiled firmware builds (renode-h743, cross-stm32f446, test-swil) --"
report "arm-none-eabi-gcc" "arm-none-eabi-gcc" "12.0" '[0-9]+\.[0-9]+\.[0-9]+' "cross builds"

echo
echo "-- Native / FMU builds (fmu-native, test-native) --"
if command -v gcc >/dev/null 2>&1 || command -v clang >/dev/null 2>&1; then
  cc_name="gcc"; command -v gcc >/dev/null 2>&1 || cc_name="clang"
  report "$cc_name" "$cc_name" "" '[0-9]+\.[0-9]+\.[0-9]+' "native builds"
else
  printf "%-18s %-10s %-34s [%s]\n" "gcc/clang" "--" "${RED}MISSING${NC}" "native builds"
  hard_failure=1
fi

echo
echo "-- SWIL simulation only (test-swil) --"
report "renode" "renode" "1.15" '[Vv]ersion[:[:space:]]+[0-9]+\.[0-9]+' "swil"
report "python3" "python3" "3.10" '[0-9]+\.[0-9]+\.[0-9]+' "swil"

if command -v python3 >/dev/null 2>&1; then
  pyrenode3_version=$(python3 -c "import importlib.metadata as m; print(m.version('pyrenode3'))" 2>/dev/null || true)
  if [ -n "$pyrenode3_version" ]; then
    printf "%-18s %-10s %-34s [%s]\n" "pyrenode3" "$pyrenode3_version" "${GREEN}OK${NC}" "swil"
  else
    printf "%-18s %-10s %-34s [%s]\n" "pyrenode3" "--" "${YELLOW}MISSING (pip install pyrenode3)${NC}" "swil"
  fi
fi

echo
echo "-- Git submodules (vendor/) --"
repo_root=$(git rev-parse --show-toplevel 2>/dev/null || true)
if [ -n "$repo_root" ]; then
  for sm in FreeRTOS-Kernel cmsis-core cmsis-device-f4 cmsis-device-h7 etl stm32f4xx-hal-driver stm32h7xx-hal-driver; do
    if [ -e "$repo_root/vendor/$sm/.git" ]; then
      printf "%-34s %s\n" "vendor/$sm" "${GREEN}OK${NC}"
    else
      printf "%-34s %s\n" "vendor/$sm" "${RED}NOT INITIALIZED (git submodule update --init --recursive)${NC}"
      hard_failure=1
    fi
  done
else
  echo "${YELLOW}Not inside a git repository -- skipping submodule check${NC}"
fi

echo
if [ "$hard_failure" -ne 0 ]; then
  echo "${RED}One or more required tools are missing, too old, or uninitialized. See README.md 'Build & Installation'.${NC}"
  exit 1
else
  echo "${GREEN}All required tools present. SWIL-only warnings (if any) don't block cross/native/test builds.${NC}"
  exit 0
fi
