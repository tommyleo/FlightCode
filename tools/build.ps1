param(
    [ValidateSet("MAMBAF411", "CLRACINGF4", "FLYWOOF405NANO", "FLYWOOF405NANO_ANALOG", "HDZERO_HALO", "SEQUREH7V2", "All")]
    [string]$Board = "MAMBAF411",
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$cmakeCommand = Get-Command cmake.exe -ErrorAction SilentlyContinue
$ninjaCommand = Get-Command ninja.exe -ErrorAction SilentlyContinue
$cmake = if ($cmakeCommand) { $cmakeCommand.Source } else { $null }
$ninja = if ($ninjaCommand) { $ninjaCommand.Source } else { $null }

if (-not $cmake) {
    $cmake = Get-ChildItem "$env:ProgramFiles\CMake" -Filter cmake.exe -Recurse -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending | Select-Object -First 1 -ExpandProperty FullName
}

if (-not $ninja) {
    $ninja = Get-ChildItem "$env:ProgramFiles\Ninja" -Filter ninja.exe -Recurse -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending | Select-Object -First 1 -ExpandProperty FullName
}

# Keep compatibility with development-tool bundles already installed locally.
if (-not $cmake) {
    $cmake = Get-ChildItem "$env:USERPROFILE\.pico-sdk\cmake" -Filter cmake.exe -Recurse -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending | Select-Object -First 1 -ExpandProperty FullName
}

if (-not $ninja) {
    $ninja = Get-ChildItem "$env:USERPROFILE\.pico-sdk\ninja" -Filter ninja.exe -Recurse -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending | Select-Object -First 1 -ExpandProperty FullName
}

if (-not $cmake -or -not $ninja) {
    throw "CMake/Ninja non trovati. Installali e aggiungi i relativi eseguibili al PATH."
}

$boards = if ($Board -eq "All") {
    @("MAMBAF411", "CLRACINGF4", "FLYWOOF405NANO", "FLYWOOF405NANO_ANALOG", "HDZERO_HALO", "SEQUREH7V2")
} else {
    @($Board)
}

foreach ($selectedBoard in $boards) {
    $prefix = switch ($selectedBoard) {
        "MAMBAF411" { "" }
        "CLRACINGF4" { "clracingf4-" }
        "FLYWOOF405NANO" { "flywoof405nano-" }
        "FLYWOOF405NANO_ANALOG" { "flywoof405nano-analog-" }
        "HDZERO_HALO" { "hdzero-halo-" }
        "SEQUREH7V2" { "sequreh7v2-" }
    }
    $preset = $prefix + $Configuration.ToLowerInvariant()

    & $cmake --preset $preset "-DCMAKE_MAKE_PROGRAM=$ninja"
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $cmake --build --preset $preset
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

exit 0
