#!/usr/bin/env bash
# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# apt-install.sh [apt-get install options...] <package>...
#
# apt-get update + install, retried. On 2026-11-19 the Ubuntu mirrors behind the
# hosted runners timed out or answered 503 for a large share of jobs, and a
# single apt-get attempt failed those jobs before anything was built (25 of the
# first 65 jobs of a step-7 batch). Each attempt has network timeouts so a
# stalled mirror fails instead of hanging; failed attempts are retried with a
# growing wait, which rides out an outage of a few minutes.
#
# Runs apt-get through sudo unless already root (container jobs). Arguments are
# passed to `apt-get install -y`, so options such as --no-install-recommends go
# first.
# ------------------------------------------------------------------------------
set -euo pipefail

if [ "$#" -eq 0 ]; then
  echo "usage: apt-install.sh [apt-get install options...] <package>..." >&2
  exit 2
fi

attempts=5
apt_opts=(-o Acquire::Retries=3 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30)
export DEBIAN_FRONTEND=noninteractive

sudo_cmd=()
if [ "$(id -u)" -ne 0 ]; then
  sudo_cmd=(sudo -E)
fi

for attempt in $(seq 1 "$attempts"); do
  if "${sudo_cmd[@]}" apt-get "${apt_opts[@]}" update -q \
    && "${sudo_cmd[@]}" apt-get "${apt_opts[@]}" install -y "$@"; then
    exit 0
  fi
  if [ "$attempt" -lt "$attempts" ]; then
    wait_s=$((attempt * 30))
    echo "::warning::apt-get failed (attempt ${attempt}/${attempts}); retrying in ${wait_s} s"
    sleep "$wait_s"
  fi
done

echo "::error::apt-get failed after ${attempts} attempts"
exit 1
