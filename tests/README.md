# tests

Host-side checks that the firmware does what the Python analysis says it does. Run them from this folder.

```
make               # build everything; checks that need no data
make equivalence   # uses ../datasets/raw_drives/, about a minute
make clean
```

| Folder | Check | Data needed |
|---|---|---|
| `esp32_compile_check/` | The ESP32 sketch type-checks against mock Arduino headers, with `USE_ML=1` and `USE_ML=0` | none |
| `stm32_pc/gps_test.c` | The NMEA parser decodes sample RMC and GGA sentences | none |
| `esp32_ml_equivalence/` | C features and forest vs the Python reference on every event window (453 of 453 classes agree, features within 0.01%) | `datasets/raw_drives` |
| `esp32_firmware_sim/` | The real `.ino` runs on the PC, fed sample by sample, with mocked MPU, Serial and time. It produces the same 453 events and labels as Python | `datasets/raw_drives` |
| `stm32_pc/pc_main.c` | The STM32 signal chain (`rs_core.c`, no HAL) on the PC. Its K/C/E/Q lines must equal the ESP32 simulation's | `datasets/raw_drives` |

All of these pass on the committed code and data. Not covered: the first compile with the real ESP32 Arduino core or STM32 HAL toolchain, and any test on the board itself.
