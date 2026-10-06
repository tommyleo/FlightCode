param([string]$Compiler = "gcc")
$ErrorActionPreference = "Stop"
$flyRoot = Split-Path -Parent $PSScriptRoot
Push-Location $flyRoot
try {
    New-Item -ItemType Directory -Path build -Force | Out-Null
    $flyStubs = Get-Content tests/foxeer_uart_test.c -Raw
    $flyStubs = $flyStubs.Substring(0, $flyStubs.IndexOf('#include "crsf_bind.h"'))
    [IO.File]::WriteAllText((Join-Path $flyRoot 'build/flywoo_uart_stubs.inc'), $flyStubs + "`n#include <string.h>`n")
    $flyHeader = Get-Content src/platform/board.h -Raw
    $flyStart = $flyHeader.IndexOf('#elif defined(BOARD_FLYWOOF405NANO)')
    $flyEnd = $flyHeader.IndexOf('#elif defined(BOARD_FOXEERF722V4)', $flyStart)
    $flyBlock = $flyHeader.Substring($flyStart, $flyEnd - $flyStart).Replace('#elif defined(BOARD_FLYWOOF405NANO)', '#if defined(BOARD_FLYWOOF405NANO)') + "`n#endif`n"
    [IO.File]::WriteAllText((Join-Path $flyRoot 'build/flywoo_board_under_test.inc'), $flyBlock)
    foreach ($flyPart in @(
        @('src/platform/board.c', 'bool board_receiver_uart_configure(', 'static void battery_adc_init(', 'flywoo_uart_under_test.inc'),
        @('src/storage/flight_settings.c', 'vtx_uart_status_t flight_settings_vtx_uart_status(', 'static bool vtx_valid(', 'flywoo_vtx_under_test.inc')
    )) {
        $flySource = Get-Content $flyPart[0] -Raw
        $flyStart = $flySource.IndexOf($flyPart[1])
        $flyEnd = $flySource.IndexOf($flyPart[2], $flyStart)
        [IO.File]::WriteAllText((Join-Path $flyRoot ('build/' + $flyPart[3])), $flySource.Substring($flyStart, $flyEnd - $flyStart))
    }
    $flySettings = Get-Content src/storage/flight_settings.c -Raw
    $flyStart = $flySettings.IndexOf('    /* Preserve tuning and the active OSD protocol')
    $flyEnd = $flySettings.IndexOf('#endif', $flyStart)
    [IO.File]::WriteAllText((Join-Path $flyRoot 'build/flywoo_migration_under_test.inc'), $flySettings.Substring($flyStart, $flyEnd - $flyStart))
    foreach ($flyBoard in @('FLYWOOF405NANO', 'FLYWOOF405NANO_ANALOG')) {
        & $Compiler -std=c11 -Wall -Wextra -Werror -Wno-unused-function -Ibuild -Isrc/platform -Isrc/drivers/receiver "-DBOARD_$flyBoard" tests/flywoo_receiver_test.c -o build/flywoo-receiver-test.exe
        if ($LASTEXITCODE -ne 0) { throw "Flywoo receiver test compilation failed: $flyBoard" }
        & ./build/flywoo-receiver-test.exe
        if ($LASTEXITCODE -ne 0) { throw "Flywoo receiver test failed: $flyBoard" }
    }
} finally { Pop-Location }
