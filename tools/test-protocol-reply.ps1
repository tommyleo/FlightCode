param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$source = Get-Content (Join-Path $root 'src\protocol\config_protocol.c') -Raw
$start = $source.IndexOf('static void reply(')
$end = $source.IndexOf('#define BACKUP_REPLY', $start)
if ($start -lt 0 -or $end -lt 0) { throw 'Reply extraction failed' }
[IO.File]::WriteAllText((Join-Path $root 'build\protocol_reply_under_test.inc'),$source.Substring($start,$end-$start))
& $Compiler -std=c11 -Wall -Wextra -Werror "-I$root\build" (Join-Path $root 'tests\protocol_reply_test.c') -o (Join-Path $root 'build\protocol-reply-test.exe')
if ($LASTEXITCODE -ne 0) { throw 'Protocol reply test compilation failed' }
& (Join-Path $root 'build\protocol-reply-test.exe')
if ($LASTEXITCODE -ne 0) { throw 'Protocol reply regression failed' }
