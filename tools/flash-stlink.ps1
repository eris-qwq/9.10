[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Firmware
)

$ErrorActionPreference = 'Stop'
if (-not (Test-Path $Firmware)) { throw "找不到待下载固件：$Firmware" }

$programmer = $env:STM32_PROGRAMMER_CLI
if (-not $programmer) {
    $candidate = 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    if (Test-Path $candidate) { $programmer = $candidate }
}
if (-not $programmer -or -not (Test-Path $programmer)) {
    throw '未找到 STM32CubeProgrammer CLI。请设置 STM32_PROGRAMMER_CLI 或执行 tools/setup-tools.ps1 检查安装。'
}

Write-Host "下载固件：$Firmware"
& $programmer -c port=SWD freq=4000 -w $Firmware -v -rst
if ($LASTEXITCODE -ne 0) { throw "STM32 下载失败，退出码：$LASTEXITCODE" }
