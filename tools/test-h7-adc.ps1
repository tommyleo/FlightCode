param([string]$Compiler = "gcc")
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    New-Item -ItemType Directory -Path build -Force | Out-Null
    $source = Get-Content src/platform/board_h7.c -Raw
    # Import the actual SEQURE pin/channel definitions rather than mirroring them.
    $board = Get-Content src/platform/board.h -Raw
    $boardStart = $board.IndexOf('#elif defined(BOARD_SEQUREH7V2)')
    $boardEnd = $board.IndexOf('#elif ', $boardStart + 1)
    if ($boardStart -lt 0 -or $boardEnd -lt 0) { throw 'SEQURE board definitions not found' }
    $board = $board.Substring($boardStart, $boardEnd - $boardStart)
    $definitions = [regex]::Matches($board, '(?m)^#define (?:BATTERY_ADC_PORT|BATTERY_ADC_PIN|BATTERY_ADC_CHANNEL|BATTERY_VOLTAGE_DIVIDER|CURRENT_ADC_PORT|CURRENT_ADC_PIN|CURRENT_ADC_CHANNEL|CURRENT_METER_SCALE) .+$')
    if ($definitions.Count -ne 8) { throw 'Incomplete SEQURE ADC definitions' }
    [IO.File]::WriteAllText((Join-Path $root 'build/h7_adc_board_under_test.inc'), (($definitions | ForEach-Object { $_.Value }) -join "`n"))
    # Test actual initialization, sampler and getters, replacing hardware calls.
    $initStart = $source.IndexOf('static void battery_adc_init(void)')
    $initEnd = $source.IndexOf('void board_init(void)', $initStart)
    $globalsStart = $source.IndexOf('static uint32_t battery_adc_total;')
    $globalsEnd = $source.IndexOf('uint32_t board_micros(void)', $globalsStart)
    $bodyStart = $source.IndexOf('void board_battery_update(void)')
    $bodyEnd = $source.IndexOf('void board_buzzer_set(', $bodyStart)
    if ($initStart -lt 0 -or $initEnd -lt 0 -or $globalsStart -lt 0 -or $globalsEnd -lt 0 -or $bodyStart -lt 0 -or $bodyEnd -lt 0) { throw "ADC test source extraction failed" }
    $underTest = $source.Substring($initStart, $initEnd-$initStart) + $source.Substring($globalsStart, $globalsEnd-$globalsStart) + $source.Substring($bodyStart, $bodyEnd-$bodyStart)
    [IO.File]::WriteAllText((Join-Path $root 'build/h7_adc_under_test.inc'), $underTest)
    & $Compiler -std=c11 -Wall -Wextra -Ibuild tests/h7_adc_test.c -lm -o build/h7-adc-test.exe
    if ($LASTEXITCODE -ne 0) { throw "H7 ADC test compilation failed" }
    & ./build/h7-adc-test.exe
    if ($LASTEXITCODE -ne 0) { throw "H7 ADC test failed" }
} finally { Pop-Location }
