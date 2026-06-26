# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Installs missing dev-environment prerequisites via winget (Windows Package
# Manager, built into Windows 11 / recent Windows 10). Prompts individually
# before each install unless -Yes is passed. Run scripts/check-toolchain.ps1
# afterwards (in a NEW shell, so PATH updates take effect) to verify.
#
# Usage:
#   scripts/install-toolchain.ps1            # prompt before each install
#   scripts/install-toolchain.ps1 -Yes       # install everything without prompting
#   scripts/install-toolchain.ps1 -Heavy     # also offer MSVC Build Tools (~3-7 GB)
# ------------------------------------------------------------------------------

param(
    [switch]$Yes,
    [switch]$Heavy
)

if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
    Write-Host "winget was not found. Install 'App Installer' from the Microsoft Store, then re-run this script." -ForegroundColor Red
    exit 1
}

function Confirm-Install {
    param([string]$Description)
    if ($Yes) { return $true }
    $reply = Read-Host "Install $Description ? [y/N]"
    return ($reply -eq "y" -or $reply -eq "Y")
}

function Test-ToolWorking {
    param([string]$Command)
    if (-not (Get-Command $Command -ErrorAction SilentlyContinue)) { return $false }
    $raw = & $Command --version 2>&1 | Out-String
    # Windows App Execution Alias stubs (e.g. python.exe) resolve via Get-Command
    # but just redirect to the Microsoft Store instead of running anything.
    return -not ($raw -match "was not found")
}

function Test-AetherionInstalled {
    $hints = @($env:AETHERION_ROOT, "$env:ProgramFiles\Aetherion", "${env:ProgramFiles(x86)}\Aetherion") | Where-Object { $_ }
    foreach ($hint in $hints) {
        if (-not (Test-Path $hint)) { continue }
        if (Get-ChildItem -Path $hint -Recurse -Filter "AetherionConfig.cmake" -ErrorAction SilentlyContinue | Select-Object -First 1) { return $true }
        if (Test-Path (Join-Path $hint "include\Aetherion\Aetherion.h")) { return $true }
    }
    return $false
}

function Install-Aetherion {
    if (Test-AetherionInstalled) {
        Write-Host "Aetherion -- already installed, skipping [fmu co-simulation]" -ForegroundColor Green
        return
    }

    Write-Host "Aetherion -- not found [fmu co-simulation]" -ForegroundColor Yellow
    if (-not (Confirm-Install "Aetherion (FMU co-simulation library, latest GitHub release)")) {
        Write-Host "Skipped Aetherion." -ForegroundColor Yellow
        return
    }

    try {
        $release = Invoke-RestMethod -Uri "https://api.github.com/repos/onurtuncer/Aetherion/releases/latest" -Headers @{ "User-Agent" = "Hemerion-installer" }
    } catch {
        Write-Host "Could not query the Aetherion releases API: $_" -ForegroundColor Red
        return
    }

    $asset = $release.assets | Where-Object { $_.name -eq "Aetherion.msi" } | Select-Object -First 1
    if (-not $asset) {
        Write-Host "No Aetherion.msi asset found on the latest Aetherion release ($($release.tag_name)). Skipping." -ForegroundColor Red
        return
    }

    $msiPath = Join-Path $env:TEMP "Aetherion.msi"
    Write-Host "Downloading $($asset.browser_download_url)" -ForegroundColor Cyan
    Invoke-WebRequest -Uri $asset.browser_download_url -OutFile $msiPath

    Write-Host "Running: msiexec /i $msiPath /qn /norestart" -ForegroundColor Cyan
    $proc = Start-Process -FilePath "msiexec.exe" -ArgumentList @("/i", "`"$msiPath`"", "/qn", "/norestart") -Wait -PassThru
    if ($proc.ExitCode -ne 0) {
        Write-Host "Aetherion install failed (msiexec exit code $($proc.ExitCode))." -ForegroundColor Red
    }
    Remove-Item $msiPath -ErrorAction SilentlyContinue
}

function Install-WingetPackage {
    param([string]$Name, [string]$Id, [string]$DetectCommand, [string]$Category, [string[]]$ExtraArgs = @())

    if ($DetectCommand -and (Test-ToolWorking $DetectCommand)) {
        Write-Host "$Name -- already on PATH, skipping [$Category]" -ForegroundColor Green
        return
    }

    Write-Host "$Name -- not found [$Category]" -ForegroundColor Yellow
    if (-not (Confirm-Install "$Name (winget id: $Id)")) {
        Write-Host "Skipped $Name." -ForegroundColor Yellow
        return
    }

    $wingetArgs = @("install", "--id", $Id, "-e", "--source", "winget", "--accept-source-agreements", "--accept-package-agreements") + $ExtraArgs
    Write-Host "Running: winget $($wingetArgs -join ' ')" -ForegroundColor Cyan
    & winget @wingetArgs
}

Write-Host "Hemerion toolchain installer (Windows / winget)" -ForegroundColor Cyan
Write-Host "================================================`n"

Write-Host "-- Always required --"
Install-WingetPackage -Name "git"   -Id "Git.Git"           -DetectCommand "git"   -Category "required"
Install-WingetPackage -Name "cmake" -Id "Kitware.CMake"      -DetectCommand "cmake" -Category "required"
Install-WingetPackage -Name "ninja" -Id "Ninja-build.Ninja"  -DetectCommand "ninja" -Category "required"

Write-Host "`n-- Cross-compiled firmware builds --"
Install-WingetPackage -Name "arm-none-eabi-gcc" -Id "Arm.GnuArmEmbeddedToolchain" -DetectCommand "arm-none-eabi-gcc" -Category "cross builds"

Write-Host "`n-- SWIL simulation only --"
Install-WingetPackage -Name "python" -Id "Python.Python.3.12" -DetectCommand "python" -Category "swil"
Install-WingetPackage -Name "Renode" -Id "Renode.Renode"      -DetectCommand "renode" -Category "swil"

if (Test-ToolWorking "python") {
    & python -c "import importlib.metadata as m; m.version('pyrenode3')" 2>$null
    $hasPyrenode3 = ($LASTEXITCODE -eq 0)
    if ($hasPyrenode3) {
        Write-Host "pyrenode3 -- already installed, skipping [swil]" -ForegroundColor Green
    } elseif (Confirm-Install "pyrenode3 (pip package, for SWIL tests)") {
        & python -m pip install --user pyrenode3
    } else {
        Write-Host "Skipped pyrenode3." -ForegroundColor Yellow
    }
}

if ($Heavy) {
    Write-Host "`n-- Native / FMU builds (heavy, optional) --"
    Write-Host "MSVC Build Tools is a multi-GB download and installs system-wide." -ForegroundColor Yellow
    Install-WingetPackage -Name "MSVC Build Tools" -Id "Microsoft.VisualStudio.2022.BuildTools" -DetectCommand "cl" -Category "native builds" `
        -ExtraArgs @("--override", "--quiet --wait --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended")
} else {
    if (-not (Get-Command cl -ErrorAction SilentlyContinue)) {
        Write-Host "`nMSVC (cl) not found. Re-run with -Heavy to install Visual Studio Build Tools, or install Visual Studio manually." -ForegroundColor Yellow
    }
}

Write-Host "`n-- Aetherion (FMU co-simulation) --"
Install-Aetherion

Write-Host "`nDone. Open a NEW shell (so PATH updates apply) and run scripts/check-toolchain.ps1 to verify." -ForegroundColor Cyan
