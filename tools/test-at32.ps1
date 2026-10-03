param([string]$Compiler = "gcc")
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    New-Item -ItemType Directory -Path build -Force | Out-Null
    & $Compiler -std=c11 -Wall -Wextra -Wno-pointer-to-int-cast -Itests/at32_stubs -Isrc/drivers/motors tests/at32_dshot_test.c -lm -o build/at32-dshot-test.exe
    if ($LASTEXITCODE -ne 0) { throw "AT32 motor test compilation failed" }
    & ./build/at32-dshot-test.exe
    if ($LASTEXITCODE -ne 0) { throw "AT32 motor test failed" }
    & $Compiler -std=c11 -Wall -Wextra -Itests/flash_stubs -Isrc/storage tests/blackbox_flash_probe_test.c -o build/blackbox-flash-probe-test.exe
    if ($LASTEXITCODE -ne 0) { throw "Flash probe test compilation failed" }
    & ./build/blackbox-flash-probe-test.exe
    if ($LASTEXITCODE -ne 0) { throw "Flash probe test failed" }
} finally { Pop-Location }
