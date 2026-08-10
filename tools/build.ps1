param(
    [ValidateSet("MAMBAF411", "CLRACINGF4", "FLYWOOF405NANO", "All")]
    [string]$Board = "MAMBAF411",
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$cmake = Get-ChildItem "$env:USERPROFILE\.pico-sdk\cmake" -Filter cmake.exe -Recurse |
    Sort-Object FullName -Descending | Select-Object -First 1 -ExpandProperty FullName
$ninja = Get-ChildItem "$env:USERPROFILE\.pico-sdk\ninja" -Filter ninja.exe -Recurse |
    Sort-Object FullName -Descending | Select-Object -First 1 -ExpandProperty FullName

if (-not $cmake -or -not $ninja) {
    throw "CMake/Ninja non trovati. Installa STM32CubeCLT oppure il toolchain Pico."
}

$boards = if ($Board -eq "All") {
    @("MAMBAF411", "CLRACINGF4", "FLYWOOF405NANO")
} else {
    @($Board)
}

foreach ($selectedBoard in $boards) {
    $prefix = switch ($selectedBoard) {
        "MAMBAF411" { "" }
        "CLRACINGF4" { "clracingf4-" }
        "FLYWOOF405NANO" { "flywoof405nano-" }
    }
    $preset = $prefix + $Configuration.ToLowerInvariant()

    & $cmake --preset $preset "-DCMAKE_MAKE_PROGRAM=$ninja"
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $cmake --build --preset $preset
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

exit 0
