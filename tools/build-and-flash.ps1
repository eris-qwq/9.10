[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot

. (Join-Path $PSScriptRoot 'setup-tools.ps1')

& $env:STM32_CMAKE --preset windows-release-flash -S $projectRoot
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $env:STM32_CMAKE --build --preset windows-release-flash --parallel
exit $LASTEXITCODE
