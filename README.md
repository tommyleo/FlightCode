# FlightCode - MAMBAF411 e CLRacingF4

Firmware sperimentale Quad X rate mode per DIAT MAMBAF411 (STM32F411) e
CL Racing CLRacingF4 (STM32F405).

Il controllo gyro/PID gira a 8 kHz, pari alla massima frequenza dati utile
dell'MPU6000. DSHOT300 e' il protocollo ESC predefinito; dal Configurator si
possono selezionare anche OneShot125 e MultiShot.

- MCU STM32F411, core 96 MHz e USB CDC
- MPU6000 SPI1: CS PA4, SCK PA5, MISO PA6, MOSI PA7, orientamento CW180
- SBUS su USART1 RX PA10, inverter PB10, 100000 baud 8E2
- Motori: M1 PB3, M2 PB4, M3 PB6, M4 PB7
- LED PC13
- Buzzer attivo basso PB2, comandato da CH5 > 2000

La mappatura deriva dal target ufficiale Betaflight `DIAT/MAMBAF411`.
Canali radio: CH1 throttle, CH2 roll, CH3 pitch, CH4 yaw e
CH6 arm > 2000.

La CLRacingF4 usa la mappatura Betaflight `CLRA/CLRACINGF4`: MPU6000 su
SPI1 (CS PA4), SBUS su USART1 RX PA10 con inverter PC0, motori PB0, PB1,
PA3 e PA2, LED PB5 e buzzer attivo basso PB4.

## Compilazione Windows

Da PowerShell:

```powershell
cd C:\SvilST\FlightCode
# Una scheda (Debug)
.\tools\build.ps1 -Board MAMBAF411
.\tools\build.ps1 -Board CLRACINGF4

# Tutte le schede (Debug)
.\tools\build.ps1 -Board All

# Una o tutte in Release
.\tools\build.ps1 -Board CLRACINGF4 -Configuration Release
.\tools\build.ps1 -Board All -Configuration Release
```

Il firmware da caricare e':

- `build/debug/FlightCode-MAMBAF411.hex`
- `build/clracingf4-debug/FlightCode-CLRACINGF4.hex`

Le build Release sono rispettivamente in `build/release` e
`build/clracingf4-release`.

Il progetto puo' essere aperto direttamente in Visual Studio Code con le
estensioni CMake Tools, C/C++ e Cortex-Debug.

## Configuratore USB

Il firmware espone una porta seriale USB CDC con telemetria, canali SBUS,
uscite motori e configurazione PID. Dopo aver caricato il firmware:

1. riavviare la scheda normalmente, senza tenere premuto BOOT;
2. avviare `C:\SvilST\FlightCodeConfigurator\start-configurator.cmd`;
3. in Chrome o Edge premere **Connetti** e scegliere FlightCode.

I PID possono essere modificati o salvati solo a quad disarmato. L'ultimo
settore della flash interna e' riservato alle impostazioni persistenti.

## Sicurezza

Prima prova obbligatoriamente senza eliche. Verificare ordine motori, verso
motori, orientamento gyro, failsafe e comando di armamento.
