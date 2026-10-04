param([string]$Compiler = "gcc")
$ErrorActionPreference = "Stop"
$taskRoot = Split-Path -Parent $PSScriptRoot
Push-Location $taskRoot
try {
    New-Item -ItemType Directory -Path build -Force | Out-Null
    $taskSource = Get-Content src/drivers/motors/dshot_f7.c -Raw
    $taskSource = [regex]::Replace($taskSource, '(?m)^#include "[^"]+"\r?\n', '')
    [IO.File]::WriteAllText((Join-Path $taskRoot 'build/foxeer_dshot_under_test.inc'), $taskSource)
    & $Compiler -std=c11 -Wall -Wextra -Wno-pointer-to-int-cast -Ibuild -Isrc/drivers/motors tests/foxeer_dshot_test.c -o build/foxeer-dshot-test.exe
    if ($LASTEXITCODE -ne 0) { throw "Foxeer DSHOT test compilation failed" }
    & ./build/foxeer-dshot-test.exe
    if ($LASTEXITCODE -ne 0) { throw "Foxeer DSHOT test failed" }
    $taskSource = Get-Content src/drivers/imu/imu.c -Raw
    $taskSource = [regex]::Replace($taskSource, '(?m)^#include "[^"]+"\r?\n', '')
    [IO.File]::WriteAllText((Join-Path $taskRoot 'build/foxeer_imu_under_test.inc'), $taskSource)
    & $Compiler -std=c11 -Wall -Wextra -Ibuild -Isrc/drivers/imu tests/foxeer_imu_test.c -lm -o build/foxeer-imu-test.exe
    if ($LASTEXITCODE -ne 0) { throw "Foxeer IMU test compilation failed" }
    & ./build/foxeer-imu-test.exe
    if ($LASTEXITCODE -ne 0) { throw "Foxeer IMU test failed" }
    foreach ($taskBoard in @('FOXEERF722V4', 'FOXEERH743')) {
        $taskDefinitions = Get-Content src/platform/board.h -Raw
        if ($taskBoard -eq 'FOXEERF722V4') {
            $taskDefinitions = Get-Content src/platform/foxeer_f722.h -Raw
        } else {
            $taskStart = $taskDefinitions.IndexOf('#elif defined(BOARD_FOXEERH743)')
            $taskEnd = $taskDefinitions.IndexOf('#elif ', $taskStart + 1)
            $taskDefinitions = $taskDefinitions.Substring($taskStart, $taskEnd - $taskStart)
        }
        $taskMask = [regex]::Match($taskDefinitions, '#define BOARD_UART_MASK (0x[0-9A-F]+U)').Groups[1].Value
        if (-not $taskMask) { throw "Missing $taskBoard UART mask" }
        $taskPlatform = if ($taskBoard -eq 'FOXEERF722V4') { '-DPLATFORM_STM32F7' } else { '-DPLATFORM_STM32H7' }
        & $Compiler -std=c11 -Wall -Wextra $taskPlatform "-DBOARD_$taskBoard" "-DBOARD_UART_MASK=$taskMask" -Isrc/platform tests/foxeer_uart_test.c -o "build/foxeer-uart-$taskBoard-test.exe"
        if ($LASTEXITCODE -ne 0) { throw "Foxeer UART test compilation failed" }
        & "./build/foxeer-uart-$taskBoard-test.exe"
        if ($LASTEXITCODE -ne 0) { throw "Foxeer UART test failed" }
    }
} finally { Pop-Location }
