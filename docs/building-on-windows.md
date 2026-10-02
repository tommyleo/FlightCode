# Building on Windows

## Requirements

Use a 64-bit Windows PC with:

- Git for Windows;
- Windows PowerShell 5.1 or PowerShell 7;
- CMake and Ninja;
- the GNU Arm Embedded `arm-none-eabi` compiler, `objcopy` and `size`.

Install CMake, Ninja and the GNU Arm Embedded toolchain using their Windows
installers or a package manager. During installation, enable the option to add
each tool to `PATH` when available.

Verify the installation from a new PowerShell window:

```powershell
cmake --version
ninja --version
arm-none-eabi-gcc --version
arm-none-eabi-objcopy --version
```

All four commands must complete successfully before using the helper script.
An internet connection is required the first time a fresh build directory is
configured because CMake downloads the official CMSIS, STM32 HAL and USB
Device source dependencies. Subsequent incremental builds reuse the downloaded
copies stored inside that build directory.

## Get the source

Clone the repository and enter its directory:

```powershell
git clone https://github.com/tommyleo/FlightCode.git
Set-Location .\FlightCode
```

If the repository is already present, open PowerShell in its root directory,
the one containing `CMakePresets.json` and the `tools` folder.

## Build the firmware

Build one Release target:

```powershell
.\tools\build.ps1 -Board MAMBAF411
.\tools\build.ps1 -Board CLRACINGF4
.\tools\build.ps1 -Board FLYWOOF405NANO
.\tools\build.ps1 -Board FLYWOOF405NANO_ANALOG
.\tools\build.ps1 -Board HDZERO_HALO
.\tools\build.ps1 -Board SEQUREH7V2
```

Build every supported target:

```powershell
.\tools\build.ps1 -Board All
```

Release is the default configuration. Add `-Configuration Debug` when a debug
build is required:

```powershell
.\tools\build.ps1 -Board FLYWOOF405NANO -Configuration Debug
```

The script selects the appropriate CMake preset, creates or refreshes the
target-specific directory under `build\`, and generates `.elf`, `.bin`, `.hex`
and linker `.map` files. For example:

```text
build\flywoof405nano-release\FlightCode-FLYWOOF405NANO.bin
build\clracingf4-release\FlightCode-CLRACINGF4.bin
build\hdzero-halo-release\FlightCode-HDZERO_HALO.bin
```

The exact output path for every board is also listed on its dedicated board
page. Build directories are reusable; running the same command again performs
an incremental build.

## Troubleshooting

- **`CMake/Ninja non trovati`**: install the missing program, add its directory
  to `PATH`, then open a new PowerShell window.
- **`arm-none-eabi-gcc` not found**: add the GNU Arm Embedded toolchain `bin`
  directory to `PATH` and reopen PowerShell.
- **PowerShell blocks `build.ps1`**: allow scripts only for the current process
  and rerun the build:

  ```powershell
  Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
  ```

- **A stale build directory reports the wrong board or toolchain**: remove
  only that target's directory under `build\` and run the command again.

[Back to the main README](../README.md)
