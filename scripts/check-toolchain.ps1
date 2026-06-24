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

$hardFailure = $false

function Get-ToolVersion {
    param([string]$Command, [string]$Arg = "--version")
    $output = & $Command $Arg 2>&1 | Out-String
    return $output
}

function Test-MinVersion {
    param([string]$Found, [string]$Required)
    try {
        return ([version]$Found) -ge ([version]$Required)
    } catch {
        return $null
    }
}

function Report {
    param(
        [string]$Name,
        [string]$Command,
        [string]$RequiredVersion,
        [string]$VersionPattern,
        [string]$Category
    )
    $exists = Get-Command $Command -ErrorAction SilentlyContinue
    if (-not $exists) {
        $status = "MISSING"
        $color = if ($Category -eq "required") { "Red" } else { "Yellow" }
        if ($Category -eq "required") { $script:hardFailure = $true }
        Write-Host ("{0,-14} {1,-10} {2,-22} [{3}]" -f $Name, "--", $status, $Category) -ForegroundColor $color
        return
    }

    $raw = Get-ToolVersion -Command $Command
    if ($raw -match "was not found.*Microsoft Store") {
        # Windows App Execution Alias stub: Get-Command finds python.exe on PATH
        # even when no real interpreter is installed.
        $color = if ($Category -eq "required") { "Red" } else { "Yellow" }
        if ($Category -eq "required") { $script:hardFailure = $true }
        Write-Host ("{0,-14} {1,-10} {2,-22} [{3}]" -f $Name, "--", "MISSING (PATH has a Store alias stub only)", $Category) -ForegroundColor $color
        return
    }

    $found = $null
    if ($raw -match $VersionPattern) { $found = $Matches[1] }

    if (-not $found) {
        Write-Host ("{0,-14} {1,-10} {2,-22} [{3}]" -f $Name, "?", "VERSION UNKNOWN", $Category) -ForegroundColor Yellow
        return
    }

    if ($RequiredVersion) {
        $ok = Test-MinVersion -Found $found -Required $RequiredVersion
        if ($ok -eq $true) {
            Write-Host ("{0,-14} {1,-10} {2,-22} [{3}]" -f $Name, $found, "OK (>= $RequiredVersion)", $Category) -ForegroundColor Green
        } elseif ($ok -eq $false) {
            $color = if ($Category -eq "required") { "Red" } else { "Yellow" }
            if ($Category -eq "required") { $script:hardFailure = $true }
            Write-Host ("{0,-14} {1,-10} {2,-22} [{3}]" -f $Name, $found, "TOO OLD (need >= $RequiredVersion)", $Category) -ForegroundColor $color
        } else {
            Write-Host ("{0,-14} {1,-10} {2,-22} [{3}]" -f $Name, $found, "OK (version check skipped)", $Category) -ForegroundColor Green
        }
    } else {
        Write-Host ("{0,-14} {1,-10} {2,-22} [{3}]" -f $Name, $found, "OK", $Category) -ForegroundColor Green
    }
}

Write-Host "Hemerion toolchain check (Windows)" -ForegroundColor Cyan
Write-Host "===================================`n"

Write-Host "-- Always required --"
Report -Name "git"               -Command "git"               -RequiredVersion $null  -VersionPattern "(\d+\.\d+\.\d+)"        -Category "required"
Report -Name "cmake"             -Command "cmake"              -RequiredVersion "3.26" -VersionPattern "cmake version (\d+\.\d+)" -Category "required"
Report -Name "ninja"             -Command "ninja"              -RequiredVersion $null  -VersionPattern "(\d+\.\d+\.\d+)"        -Category "required"

Write-Host "`n-- Cross-compiled firmware builds (renode-h743, cross-stm32f446, test-swil) --"
Report -Name "arm-none-eabi-gcc" -Command "arm-none-eabi-gcc"  -RequiredVersion "12.0" -VersionPattern "\)\s+(\d+\.\d+)\."     -Category "cross builds"

Write-Host "`n-- Native / FMU builds (fmu-native, test-native) --"
$msvc = Get-Command "cl" -ErrorAction SilentlyContinue
if ($msvc) {
    Write-Host ("{0,-14} {1,-10} {2,-22} [{3}]" -f "msvc (cl)", "found", "OK", "native builds") -ForegroundColor Green
} else {
    Write-Host ("{0,-14} {1,-10} {2,-22} [{3}]" -f "msvc (cl)", "--", "MISSING (run from a VS dev shell)", "native builds") -ForegroundColor Yellow
}

Write-Host "`n-- SWIL simulation only (test-swil) --"
Report -Name "renode"            -Command "renode"             -RequiredVersion "1.15" -VersionPattern "[Vv]ersion[:\s]+(\d+\.\d+)" -Category "swil"
Report -Name "python"            -Command "python"             -RequiredVersion "3.10" -VersionPattern "Python (\d+\.\d+)"      -Category "swil"

$pyrenode3 = & python -c "import importlib.metadata as m; print(m.version('pyrenode3'))" 2>&1
if ($LASTEXITCODE -eq 0 -and $pyrenode3 -match "^\d") {
    Write-Host ("{0,-14} {1,-10} {2,-22} [{3}]" -f "pyrenode3", $pyrenode3, "OK", "swil") -ForegroundColor Green
} else {
    Write-Host ("{0,-14} {1,-10} {2,-22} [{3}]" -f "pyrenode3", "--", "MISSING (pip install pyrenode3)", "swil") -ForegroundColor Yellow
}

Write-Host "`n-- Git submodules (vendor/) --"
$repoRoot = git rev-parse --show-toplevel 2>$null
if ($repoRoot) {
    $submodules = @("FreeRTOS-Kernel", "cmsis-core", "cmsis-device-f4", "cmsis-device-h7", "etl", "stm32f4xx-hal-driver", "stm32h7xx-hal-driver")
    foreach ($sm in $submodules) {
        $path = Join-Path (Join-Path $repoRoot "vendor") $sm
        if (Test-Path (Join-Path $path ".git")) {
            Write-Host ("{0,-30} OK" -f "vendor/$sm") -ForegroundColor Green
        } else {
            Write-Host ("{0,-30} NOT INITIALIZED (git submodule update --init --recursive)" -f "vendor/$sm") -ForegroundColor Red
            $hardFailure = $true
        }
    }
} else {
    Write-Host "Not inside a git repository -- skipping submodule check" -ForegroundColor Yellow
}

Write-Host ""
if ($hardFailure) {
    Write-Host "One or more required tools are missing, too old, or uninitialized. See README.md 'Build & Installation'." -ForegroundColor Red
    exit 1
} else {
    Write-Host "All required tools present. SWIL-only warnings (if any) don't block cross/native/test builds." -ForegroundColor Green
    exit 0
}
