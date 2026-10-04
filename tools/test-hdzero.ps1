param([string]$Compiler = "gcc")
$ErrorActionPreference = "Stop"
$taskRoot = Split-Path -Parent $PSScriptRoot
Push-Location $taskRoot
try {
    New-Item -ItemType Directory -Force build | Out-Null
    & $Compiler -std=c11 -Wall -Wextra -Werror tests/hdzero_msp_test.c -o build/hdzero-msp-test.exe
    if ($LASTEXITCODE -ne 0) { throw "HDZero MSP test compilation failed" }
    & ./build/hdzero-msp-test.exe
    if ($LASTEXITCODE -ne 0) { throw "HDZero MSP test failed" }
    $taskSource = Get-Content src/drivers/osd/msp_displayport.c -Raw
    $taskSource = [regex]::Replace($taskSource, '(?m)^#include "[^"]+"\r?\n', '')
    [IO.File]::WriteAllText((Join-Path $taskRoot 'build/msp_displayport_under_test.inc'), $taskSource)
    & $Compiler -std=c11 -Wall -Wextra -Werror -Ibuild tests/msp_link_test.c -lm -o build/msp-link-test.exe
    if ($LASTEXITCODE -ne 0) { throw "MSP UART test compilation failed" }
    & ./build/msp-link-test.exe
    if ($LASTEXITCODE -ne 0) { throw "MSP UART test failed" }
} finally { Pop-Location }
