#!/usr/bin/env bash
# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# apt-install.sh [apt-get install options...] <package>...
#
# apt-get update + install, retried, with every network phase bounded.
#
# On 2026-11-19 the Ubuntu mirrors behind the hosted runners timed out or
# answered 503 for a large share of jobs, and a single apt-get attempt failed
# those jobs before anything was built (25 of the first 65 jobs of a step-7
# batch). On 2026-11-19/20 the docs job, whose TeX Live download is by far the
# largest, also hung three times with no output until its step timeout: the
# download trickled rather than stopped, which apt's Acquire::http::Timeout
# (time with *no* data) never catches.
#
# So the network work is split from the install:
#   1. `apt-get update` and `apt-get install --download-only`, each under a
#      wall-clock limit and retried with a growing wait. A stalled attempt is
#      killed and retried; killing a download is safe.
#   2. `apt-get install --no-download` from the local cache, unbounded and not
#      retried: it touches no network, and killing dpkg mid-install is not safe.
#
# Limits (seconds), overridable from the environment:
#   APT_UPDATE_TIMEOUT    (default 300)  per update attempt
#   APT_DOWNLOAD_TIMEOUT  (default 600)  per download attempt
#
# Runs apt-get through sudo unless already root (container jobs). The
# environment reaches apt-get through `env`, not `sudo -E`: Ubuntu 26.04's sudo
# ignores -E. Arguments are passed to `apt-get install -y`, so options such as
# --no-install-recommends or -qq go first.
# ------------------------------------------------------------------------------
set -euo pipefail

if [ "$#" -eq 0 ]; then
  echo "usage: apt-install.sh [apt-get install options...] <package>..." >&2
  exit 2
fi

attempts=5
update_timeout="${APT_UPDATE_TIMEOUT:-300}"
download_timeout="${APT_DOWNLOAD_TIMEOUT:-600}"
apt_opts=(-o Acquire::Retries=3 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30)

run_apt=(env DEBIAN_FRONTEND=noninteractive apt-get)
if [ "$(id -u)" -ne 0 ]; then
  run_apt=(sudo "${run_apt[@]}")
fi

fetched=0
for attempt in $(seq 1 "$attempts"); do
  if timeout --kill-after=30 "$update_timeout" "${run_apt[@]}" "${apt_opts[@]}" update -q \
    && timeout --kill-after=30 "$download_timeout" "${run_apt[@]}" "${apt_opts[@]}" install -y --download-only "$@"; then
    fetched=1
    break
  else
    status=$?
  fi
  if [ "$attempt" -lt "$attempts" ]; then
    wait_s=$((attempt * 30))
    if [ "$status" -eq 124 ] || [ "$status" -eq 137 ]; then
      reason="timed out"
    else
      reason="failed (exit ${status})"
    fi
    echo "::warning::apt-get update/download ${reason} (attempt ${attempt}/${attempts}); retrying in ${wait_s} s"
    sleep "$wait_s"
  fi
done

if [ "$fetched" -ne 1 ]; then
  echo "::error::apt-get update/download failed after ${attempts} attempts"
  exit 1
fi

"${run_apt[@]}" install -y --no-download "$@"
