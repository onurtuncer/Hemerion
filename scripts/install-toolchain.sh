#!/usr/bin/env bash
# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Installs missing dev-environment prerequisites via apt (Debian/Ubuntu) and
# pip. Prompts individually before each install unless -y/--yes is passed.
# Run scripts/check-toolchain.sh afterwards to verify.
#
# Usage:
#   scripts/install-toolchain.sh           # prompt before each install
#   scripts/install-toolchain.sh -y         # install everything without prompting
#
# Renode is installed via `renode-run` (pip, no sudo) instead of apt, since
# there is no official Renode apt repo -- renode-run downloads a portable
# build the same way Renode's own CI does.
# ------------------------------------------------------------------------------
set -uo pipefail

YES=0
if [ "${1:-}" = "-y" ] || [ "${1:-}" = "--yes" ]; then
  YES=1
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "apt-get not found -- this script only supports Debian/Ubuntu. Install prerequisites manually; see README.md 'Build & Installation'."
  exit 1
fi

confirm() {
  [ "$YES" -eq 1 ] && return 0
  read -r -p "Install $1 ? [y/N] " reply
  [ "$reply" = "y" ] || [ "$reply" = "Y" ]
}

apt_install() {
  local name="$1" pkg="$2" detect="$3" category="$4"
  if [ -n "$detect" ] && command -v "$detect" >/dev/null 2>&1; then
    echo "$name -- already on PATH, skipping [$category]"
    return
  fi
  echo "$name -- not found [$category]"
  if confirm "$name (apt package: $pkg)"; then
    sudo apt-get install -y "$pkg"
  else
    echo "Skipped $name."
  fi
}

echo "Hemerion toolchain installer (Linux / apt + pip)"
echo "================================================="
echo
echo "Refreshing apt package lists (sudo apt-get update)..."
sudo apt-get update

echo
echo "-- Always required --"
apt_install "git" "git" "git" "required"
apt_install "cmake" "cmake" "cmake" "required"
apt_install "ninja" "ninja-build" "ninja" "required"

echo
echo "-- Cross-compiled firmware builds --"
apt_install "arm-none-eabi-gcc" "gcc-arm-none-eabi" "arm-none-eabi-gcc" "cross builds"

echo
echo "-- Native / FMU builds --"
apt_install "build-essential (gcc/g++)" "build-essential" "gcc" "native builds"

echo
echo "-- SWIL simulation only --"
apt_install "python3" "python3" "python3" "swil"
apt_install "pip" "python3-pip" "pip3" "swil"

if command -v python3 >/dev/null 2>&1 && command -v pip3 >/dev/null 2>&1; then
  if python3 -c "import importlib.metadata as m; m.version('pyrenode3')" >/dev/null 2>&1; then
    echo "pyrenode3 -- already installed, skipping [swil]"
  elif confirm "pyrenode3 (pip package, for SWIL tests)"; then
    pip3 install --user pyrenode3
  else
    echo "Skipped pyrenode3."
  fi

  if command -v renode >/dev/null 2>&1; then
    echo "renode -- already on PATH, skipping [swil]"
  elif confirm "renode-run (pip package that downloads/manages a portable Renode build, for SWIL tests)"; then
    pip3 install --user renode-run
    "$HOME/.local/bin/renode-run" download || true
    echo "renode-run installed. Use 'renode-run exec ...' or add its install path to PATH; see https://github.com/antmicro/renode-run"
  else
    echo "Skipped Renode."
  fi
fi

echo
echo "Note: this Ubuntu's apt cmake/gcc-arm-none-eabi packages may be older than"
echo "the versions required by README.md's prerequisites table on older LTS"
echo "releases. If scripts/check-toolchain.sh still reports TOO OLD afterwards,"
echo "install a newer CMake via 'pip3 install --user cmake' or the Arm GNU"
echo "Toolchain tarball from developer.arm.com instead of the apt package."
echo
echo "Done. Run scripts/check-toolchain.sh to verify."
