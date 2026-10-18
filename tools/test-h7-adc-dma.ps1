param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$board = Get-Content (Join-Path $root 'src\platform\board.h') -Raw
$start = $board.IndexOf('#elif defined(BOARD_SEQUREH7V2)')
$end = $board.IndexOf('#elif ', $start + 1)
if ($start -lt 0 -or $end -lt 0) { throw 'Missing SEQURE definitions' }
$definitions = [regex]::Matches($board.Substring($start,$end-$start), '(?m)^#define (?:BATTERY_ADC_PORT|BATTERY_ADC_PIN|BATTERY_ADC_CHANNEL|BATTERY_VOLTAGE_DIVIDER|CURRENT_ADC_PORT|CURRENT_ADC_PIN|CURRENT_ADC_CHANNEL|CURRENT_METER_SCALE) .+$')
if ($definitions.Count -ne 8) { throw 'Incomplete ADC board definitions' }
[IO.File]::WriteAllText((Join-Path $root 'build\h7_adc_board_under_test.inc'), (($definitions | ForEach-Object { $_.Value }) -join "`n"))
$source = Get-Content (Join-Path $root 'src\platform\board_h7.c') -Raw
$start = $source.IndexOf('static void battery_adc_init(void)')
$end = $source.IndexOf('void board_init(void)', $start)
if ($start -lt 0 -or $end -lt 0) { throw 'Missing ADC initialization' }
[IO.File]::WriteAllText((Join-Path $root 'build\h7_adc_init_under_test.inc'),$source.Substring($start,$end-$start))
$testOutput = Join-Path $root 'build\h7-adc-dma-test.exe'
& $Compiler -std=c11 -Wall -Wextra "-I$root\src\platform" "-I$root\build" (Join-Path $root 'tests\h7_adc_dma_test.c') -lm -o $testOutput
if ($LASTEXITCODE -ne 0) { throw 'H7 DMA test compilation failed' }
& $testOutput
if ($LASTEXITCODE -ne 0) { throw 'H7 DMA test failed' }
