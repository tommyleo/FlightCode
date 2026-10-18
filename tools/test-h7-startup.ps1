param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$source = Get-Content (Join-Path $root 'src\platform\board_h7.c') -Raw
$start = $source.IndexOf('void board_init(void)')
$end = $source.IndexOf('void board_peripherals_init(void)', $start)
if ($start -lt 0 -or $end -lt 0) { throw 'H7 startup extraction failed' }
[IO.File]::WriteAllText((Join-Path $root 'build\h7_startup_under_test.inc'), $source.Substring($start,$end-$start))
& $Compiler -std=c11 -Wall -Wextra -Werror "-I$root\build" (Join-Path $root 'tests\h7_startup_test.c') -o (Join-Path $root 'build\h7-startup-test.exe')
if ($LASTEXITCODE -ne 0) { throw 'H7 startup test compilation failed' }
& (Join-Path $root 'build\h7-startup-test.exe')
if ($LASTEXITCODE -ne 0) { throw 'H7 startup test failed' }
