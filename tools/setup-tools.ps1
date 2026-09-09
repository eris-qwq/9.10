[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$toolsRoot = Join-Path $projectRoot '.tools'
$toolchainRoot = Join-Path $toolsRoot 'arm-gnu-toolchain-14.3.rel1-mingw-w64-x86_64-arm-none-eabi'
$gcc = Join-Path $toolchainRoot 'bin\arm-none-eabi-gcc.exe'
$cmakeRoot = Join-Path $toolsRoot 'cmake-3.31.6-windows-x86_64'
$cmakeLocal = Join-Path $cmakeRoot 'bin\cmake.exe'
$ninjaRoot = Join-Path $toolsRoot 'ninja-win'
$ninjaLocal = Join-Path $ninjaRoot 'ninja.exe'

if (-not (Get-Command cmake -ErrorAction SilentlyContinue) -and -not (Test-Path $cmakeLocal)) {
    New-Item -ItemType Directory -Force -Path $toolsRoot | Out-Null
    $cmakeArchive = Join-Path $toolsRoot 'cmake-3.31.6-windows-x86_64.zip'
    Invoke-WebRequest -Uri 'https://github.com/Kitware/CMake/releases/download/v3.31.6/cmake-3.31.6-windows-x86_64.zip' -OutFile $cmakeArchive
    Expand-Archive -Path $cmakeArchive -DestinationPath $toolsRoot -Force
}
if (-not (Get-Command ninja -ErrorAction SilentlyContinue) -and -not (Test-Path $ninjaLocal)) {
    New-Item -ItemType Directory -Force -Path $toolsRoot | Out-Null
    $ninjaArchive = Join-Path $toolsRoot 'ninja-win.zip'
    Invoke-WebRequest -Uri 'https://github.com/ninja-build/ninja/releases/download/v1.12.1/ninja-win.zip' -OutFile $ninjaArchive
    Expand-Archive -Path $ninjaArchive -DestinationPath $ninjaRoot -Force
}

$env:STM32_CMAKE = if (Test-Path $cmakeLocal) { $cmakeLocal } else { (Get-Command cmake).Source }
$ninjaDir = if (Test-Path $ninjaLocal) { $ninjaRoot } else { Split-Path -Parent (Get-Command ninja).Source }
$env:Path = "$ninjaDir;$env:Path"

if (-not (Test-Path $gcc)) {
    $archive = Join-Path $toolsRoot 'arm-gnu-toolchain-14.3.rel1-mingw-w64-x86_64-arm-none-eabi.zip'
    $url = 'https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-mingw-w64-x86_64-arm-none-eabi.zip'
    New-Item -ItemType Directory -Force -Path $toolsRoot | Out-Null
    if (-not (Test-Path $archive)) {
        Invoke-WebRequest -Uri $url -OutFile $archive
    }
    Expand-Archive -Path $archive -DestinationPath $toolsRoot -Force
}

if (-not (Test-Path $gcc)) {
    throw "Arm GNU 工具链未正确安装：$gcc"
}

# STM32CubeProgrammer 需要先由 ST 官方安装程序安装；以下路径同时覆盖默认安装和环境变量。
$programmer = $env:STM32_PROGRAMMER_CLI
if (-not $programmer) {
    $candidate = 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    if (Test-Path $candidate) { $programmer = $candidate }
}
if (-not $programmer -or -not (Test-Path $programmer)) {
    throw '未找到 STM32CubeProgrammer。请安装 ST 官方 STM32CubeProgrammer 后重试，或设置环境变量 STM32_PROGRAMMER_CLI 指向 STM32_Programmer_CLI.exe。'
}
