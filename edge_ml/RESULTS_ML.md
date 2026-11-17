# RoadSense ML extension: event filter results and ESP32 deployment

## What was built

Option 1 is an ML filter on a loose trigger. The trigger runs at K_LOCAL = 2 instead of 6. This catches almost every tagged anomaly (98%), but it also produces 351 untagged events. A random forest then looks at each event window and decides NONE, BREAKER or SURFACE. NONE events are dropped.

Option 2 tests whether the features need ABC normalisation. The same forest is trained twice:

- **raw:** amplitudes in g, with durations and hit counts measured against fixed thresholds (the hatchback's values)
- **norm:** amplitudes divided by each vehicle's own calibrated noise, with the per-vehicle thresholds

Both sets share 7 dimensionless features: local contrast ratio, jerk-peak timing, four spectral band fractions of vertical acceleration, and the first heave sign.

The evaluation is leave-one-vehicle-out. Each drive is predicted by a model trained only on the other two drives. Scoring uses the paper's protocol: tags are the ground truth, the match window is [tag − 5 s, tag + 1.5 s], and p and r tags count as SURFACE. The cut-off P(NONE) < 0.5 was fixed before testing.

## Main result (pooled over the 3 held-out drives)

| | events | untagged | tags found | breaker recall | breaker precision | surface recall | class acc. |
|---|---|---|---|---|---|---|---|
| Rules, K_LOCAL = 6 (paper) | 86 | 33 | 0.51 | 0.83 | 0.56 | 0.40 | 0.92 |
| Rules, K_LOCAL = 5 | 133 | 62 | 0.68 | 0.78 | 0.44 | 0.56 | 0.87 |
| Rules, K_LOCAL = 4 | 215 | 131 | 0.81 | 0.67 | 0.36 | 0.71 | 0.87 |
| ML forest, raw features | 198 | 118 | 0.77 | 0.70 | 0.48 | 0.65 | 0.86 |
| **ML forest, norm features** | 156 | 78 | 0.75 | 0.63 | 0.55 | 0.67 | 0.89 |
| ML forest, norm, cut-off 0.6 | 192 | 108 | 0.81 | 0.64 | 0.53 | 0.74 | 0.90 |

The forest rows are the mean of 5 random seeds. The full curves are in `results/ml_operating_points.png`, and the per-vehicle tables are in `results/ml_summary.txt`.

What the numbers say:

1. **The ML filter helps, modestly.** At cut-off 0.6 it finds the same share of tags as the rules at K_LOCAL = 4 (0.81). It does this with 108 untagged events instead of 131, a higher surface recall (0.74 vs 0.71) and a much higher breaker precision (0.53 vs 0.36).
2. **Breaker recall drops.** The ML filter reaches 0.63–0.64, against 0.83 for the paper's rule. Each training fold has only 12 breakers, and the heave-duration rule (D_h ≥ 450 ms) is hard to beat with that little data. With more breaker examples this should improve. For now the rule label is the safer choice for breakers.
3. **Normalisation matters (option 2).** The norm curve sits above the raw curve along its whole length. The gap is largest at the strict end: at about 50 untagged events, norm finds 0.67 of tags and raw only 0.46. The raw model also ranks below the plain rules, so ML on raw units does not transfer between cars, while ML on ABC units does. This is ML evidence for the calibration idea.
4. **Per-vehicle results swing a lot** (see `ml_summary.txt`). With 6 breakers and 24–34 surface tags per drive, a single event changes a percentage by 3–17 points. Treat these as preliminary results.

Caveats:

- The trigger constants (K_EVENT and the others) were tuned on the hatchback, so the hatchback fold is not fully unseen.
- Untagged events are not all false. Many are real small undulations nobody pressed a key for, so precision is a lower bound. Training leaves out unmatched candidates close to a tag (35–39 per drive), because their true label is unknown. Test drives are always scored on all candidates.
- The deployed model is trained on all three drives. Its expected performance on a new car is the leave-one-vehicle-out figure above, not an in-sample figure.

## ESP32 deployment (edge computing)

It runs on the ESP32. The sketch is in `firmware/esp32/RoadSense_v4_ML/`.

| | |
|---|---|
| `RoadSense_v4_ML.ino` | v3 plus the ML filter. `#define USE_ML 0` gives exactly v3 again |
| `rs_ml_features.h` | the 21 features computed on the device from the existing 1.4 s event window |
| `rs_ml_model.h` | the generated forest: 40 trees, depth 6, 1968 nodes, about 29 KB of flash |
| `AdaptiveBaseline.h` | unchanged |

Cost:

- **RAM:** about 3.2 KB for the event window buffer.
- **Compute:** runs once per event, not per sample. On a PC it takes about 40 µs per event. On the ESP32 it should take around 1–2 ms (single-precision float only). The real time is printed on every `Q` line and in `#STATUS` (`s` key), so check it on the first drive.
- **Output:** the E line now carries the ML label (NONE for a dropped event). A new `Q,ms,pBreaker,pNone,pSurface,us` line follows each event. NONE events stay out of the 10 Hz summary and the map. R lines are unchanged, so every offline tool still works.
- **Tuning:** `ML_KEEP_PNONE` (default 0.5) is the recall-versus-false-alarm dial. Raising it to 0.6 gives the 0.81-recall operating point.

How it was verified:

1. **C features and forest match Python.** `tests/esp32_ml_equivalence/test_ml.cpp` checks all 453 windows. Features agree to within 0.01%, and the predicted class is identical in 453 of 453 windows.
2. **The real sketch matches the Python pipeline.** `tests/esp32_firmware_sim/` runs the actual `.ino` on a PC, fed sample by sample from the three CoolTerm logs, with the mocks standing in for MPU, Serial and time. It produces the same 453 events as Python with the same labels in all 453. The largest probability difference is 0.06, from float32 vs float64 signal paths, and no label changes because of it.
3. **Compilation.** Both `USE_ML 1` and `USE_ML 0` compile cleanly against mock Arduino headers. The ESP32 Arduino core was not available offline here, so do the first real compile in the Arduino IDE.

I have not tested this on the hardware. Before trusting it on the road, flash it, do one short drive and compare the Q-line timings and the event counts with the offline run (`compare_firmware.py`).

## Reproduce

From `edge_ml/`, with the three raw drives in `../datasets/raw_drives/`:

```
python3 ml_experiment.py      # options 1 + 2, leave-one-vehicle-out, writes results/
python3 make_ml_figure.py     # results/ml_operating_points.png
python3 export_model.py       # rs_ml_model.h for ESP32 + STM32, results/test_vectors.txt
cd ../tests && make equivalence   # C forest vs Python, firmware sim vs Python, STM32 vs ESP32
```
