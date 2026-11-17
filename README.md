# RoadSense

RoadSense is a low-cost edge node for detecting road anomalies. It is rigidly mounted inside a car and detects potholes and speed breakers, grades road roughness, and geotags each event on the device, with no phone and no cloud. The hardware is an MPU6050 IMU and a NEO-6M GPS, running on an ESP32 or on an STM32F411 port.

Cars ride differently, so a fixed threshold tuned on one car floods another with false detections. RoadSense handles this with **Adaptive Baseline Calibration (ABC)**: every threshold is μ + kσ of that vehicle's own normal-road signal, measured at start-up and adapted slowly while driving. The detection constants were tuned on one car and carried unchanged to two others.

First-author paper accepted at **ICEdge 2026**, IISc Bangalore. See [`docs/ICEdge2026.md`](docs/ICEdge2026.md).

## Pipeline

```
 MPU6050 accel + gyro (100 Hz)        NEO-6M GPS (RMC/GGA)
                 │                          │
                 ▼                          ▼
 ┌─ Edge node: ESP32 (firmware/esp32) or STM32F411 (firmware/stm32) ─┐
 │  gravity alignment → jerk, heave (3 Hz LP vertical), 1 s roughness │
 │  ABC: μ + kσ per signal, per vehicle                               │
 │  trigger → 1.4 s event window → classify                           │
 │     rules: heave duration (ICEdge)  |  random forest (extension)   │
 │  roughness tier GREEN / YELLOW / RED                                │
 └────────────────────────────────────────────────────────────────────┘
                 │  serial log: raw R, events E, forest Q, tags M, calibration C
                 ▼
 datasets/   3 vehicles, same route, passenger key-tags as ground truth
                 │
                 ▼
 edge_ml/    offline replay → cross-vehicle evaluation → forest training
                 │  export_model.py → rs_ml_model.h
                 ▼
 firmware/   same forest on ESP32 and STM32
 tests/      C == Python, firmware sim == Python, STM32 == ESP32
```

## Repository layout

| Folder | Stage | Contents |
|---|---|---|
| [`firmware/esp32/RoadSense_v4_ML/`](firmware/esp32/RoadSense_v4_ML) | Edge node | ESP32 Arduino sketch. `USE_ML 0` gives the v3 rule detector used in the paper; `USE_ML 1` adds the on-device forest |
| [`firmware/stm32/`](firmware/stm32) | Edge node | STM32F411 Black Pill port (CubeIDE, HAL): bare-C signal chain, MPU6050 driver, DMA NMEA parser, USB CDC output |
| [`datasets/`](datasets) | Data | Raw 100 Hz drive logs from three vehicles, plus the event-window and whole-drive feature datasets |
| [`edge_ml/`](edge_ml) | Analysis | Python replay of the firmware, cross-vehicle evaluation, ML experiments, model export, results |
| [`tests/`](tests) | Verification | Host builds and equivalence tests for both firmwares (`make`) |
| [`docs/`](docs) | Docs | Signal chain and output protocol ([`architecture.md`](docs/architecture.md)), paper scope ([`ICEdge2026.md`](docs/ICEdge2026.md)) |

## Results

Three drives on the same route: hatchback, sedan and SUV. Ground truth comes from passenger key-tags (`b` breaker, `p`/`r` surface defect). A detection counts as a hit if it starts between 5 s before and 1.5 s after a tag.

**ICEdge configuration** (v3 rules on the ESP32, parameters tuned on the hatchback, frozen for the other two cars, pooled over all three drives):

| Thresholds | Events | Untagged events | Breaker recall | Breaker precision | Class accuracy |
|---|---|---|---|---|---|
| **ABC** (μ + kσ per vehicle) | 86 | 33 | 0.83 (15 of 18) | 0.56 | 0.92 |
| Fixed, carried over from the hatchback | 598 | 497 | 0.78 | 0.07–0.08 per car | 0.71 |

The sedan and SUV have 2–2.6× the hatchback's calibrated jerk noise. With fixed thresholds they flood the log with false events: 228 and 268, against 31 and 25 with ABC. The roughness tier flagged 89–100% of the rough-patch tags on each car.

**Extension: on-device random forest** (leave-one-vehicle-out: each car is scored by a model that never saw it):

| Method | Tags found | Untagged events | Breaker recall | Surface recall |
|---|---|---|---|---|
| Rules, looser trigger (k_L = 4) | 0.81 | 131 | 0.67 | 0.71 |
| Event filter forest, P(NONE) < 0.6 | 0.81 | 108 | 0.64 | 0.74 |
| Whole-drive forest, ABC-normalised | **0.88** | 133 | **0.76** | **0.81** |

With raw (un-normalised) features, the same forest does worse on unseen cars and is inconsistent between them. That is independent evidence for ABC. Full tables and caveats: [`edge_ml/RESULTS_ML.md`](edge_ml/RESULTS_ML.md), [`edge_ml/RESULTS_wholedrive.md`](edge_ml/RESULTS_wholedrive.md).

The deployed forest has 40 trees, depth 6 and 1968 nodes. It takes about 29 KB of flash and 3.2 KB of RAM, and runs once per event, not once per sample.

## Status

- The v3 rule firmware (ESP32) produced all the drive data and the paper results.
- The v4 forest firmware (ESP32) and the STM32 port are verified in simulation: fed the logged drives sample by sample, they reproduce the Python pipeline event for event. **Neither has run on hardware yet.**
- The sample is small: 18 breakers in total, so per-car breaker figures move in steps of about 17 points.

## Quick start

**ESP32:** open `firmware/esp32/RoadSense_v4_ML/RoadSense_v4_ML.ino` in the Arduino IDE with the ESP32 core and TinyGPSPlus installed. Wire the MPU6050 to SDA 21 / SCL 22 and the GPS to RX 16 / TX 17. Log at 460800 baud with CoolTerm and press `c` to calibrate on a smooth stretch.

**STM32:** see [`firmware/stm32/README.md`](firmware/stm32/README.md).

**Analysis:**
```
pip install -r edge_ml/requirements.txt
cd edge_ml
python3 eval_3vehicles.py ../datasets/raw_drives/{hatchback,sedan,suv}.txt --params best_3veh.json --out results/paper
python3 ml_experiment.py && python3 make_ml_figure.py
```

**Tests:**
```
cd tests && make               # builds, compile checks, GPS parser
make equivalence               # regenerates the model and checks C, sim and STM32 against Python
```

## Hardware

| Part | ESP32 node | STM32 node |
|---|---|---|
| MCU | ESP32 dev board | STM32F411 Black Pill |
| IMU | MPU6050, I2C SDA 21 / SCL 22, 100 Hz, ±4 g, DLPF 44 Hz | MPU6050 on I2C1 (PB6 SCL, PB7 SDA) |
| GPS | NEO-6M, UART2 RX 16 / TX 17, 9600 baud | NEO-6M on USART1 RX (PA10), circular DMA |
| Output | USB serial 460800 baud + LittleFS backup | USB CDC |
| Tagging | Keys in CoolTerm or a button on GPIO4 | Keys in CoolTerm |

---
Anvay Sinkar, Vishwakarma Institute of Technology, Pune
