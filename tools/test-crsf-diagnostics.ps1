param([string]$Compiler = "gcc")
$ErrorActionPreference = "Stop"
$crsfRoot = Split-Path -Parent $PSScriptRoot
Push-Location $crsfRoot
try {
    New-Item -ItemType Directory -Path build -Force | Out-Null
    $crsfSource = Get-Content src/drivers/receiver/sbus.c -Raw
    $crsfStart = $crsfSource.IndexOf('static uint8_t crsf_crc8(')
    $crsfEnd = $crsfSource.IndexOf('void sbus_init(void)', $crsfStart)
    if ($crsfStart -lt 0 -or $crsfEnd -le $crsfStart) { throw "Missing CRSF decoder" }
    [IO.File]::WriteAllText((Join-Path $crsfRoot 'build/crsf_decode_under_test.inc'), $crsfSource.Substring($crsfStart, $crsfEnd - $crsfStart))
    & $Compiler -std=c11 -Wall -Wextra -Werror -Ibuild -Isrc/drivers/receiver tests/crsf_diagnostics_test.c -o build/crsf-diagnostics-test.exe
    if ($LASTEXITCODE -ne 0) { throw "CRSF diagnostics test compilation failed" }
    & ./build/crsf-diagnostics-test.exe
    if ($LASTEXITCODE -ne 0) { throw "CRSF diagnostics test failed" }
} finally { Pop-Location }
