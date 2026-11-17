# datasets

Three drives over the same urban route in Pune, one each in a hatchback, a sedan and an SUV. They were logged with the v3 firmware: ESP32 + MPU6050 at 100 Hz + NEO-6M, rigidly mounted. Ground truth is a passenger pressing a key when the car crosses an anomaly: `b` breaker, `p` pothole, `r` rough patch.

| Drive | Breaker tags | Surface tags (p + r) |
|---|---|---|
| Hatchback | 6 | 24 |
| Sedan | 6 | 34 |
| SUV | 6 | 28 |

## `raw_drives/`

The raw CoolTerm captures, one file per drive. Every other dataset here is derived from them.

| File | Raw 100 Hz samples | Tags (`M` lines) | Calibrations | Analysed |
|---|---|---|---|---|
| `hatchback.txt` | 86,698 | 30 | 2 | 848 s |
| `sedan.txt` | 81,068 | 42 | 2 (first aborted) | 802 s |
| `suv.txt` | 85,970 | 36 | 2 | 841 s |

Line formats (`R`, `E`, `M`, `K`, `C`, 10 Hz summary) are documented in [`docs/architecture.md`](../docs/architecture.md). The tools analyse each drive from its final calibration onward. Repeated presses of the same tag key within a short gap count as one tag.

**Cleaning.** The original captures contain serial noise. Only the following was removed:

- the ESP32 boot-ROM bytes before `#BOOT`
- 17 lines where a burst of `0xFF` bytes had corrupted the line (9 hatchback, 8 sedan), mostly partly overwritten `R` lines
- two ESP-IDF LittleFS "no free space" messages (SUV)
- blank lines

Line endings were converted from CRLF to LF. The analysis tools already skipped all of these lines. To confirm, the full pipeline was run on both the original and the cleaned logs: every output (events, features, forest, summaries, window datasets, firmware simulation) is byte-identical between the two, and identical to the results committed in this repo.

The logs include GPS positions along the test route.

## `event_windows/ml_dataset.csv`

One row per candidate event from the loose trigger (`K_LOCAL = 2`): 453 events across the three drives. Written by `edge_ml/ml_experiment.py`.

| Columns | Meaning |
|---|---|
| `vehicle, t_ms, lat, lon` | drive, trigger time on the device clock, GPS position |
| `rule_label` | what the v3 rule classifier says |
| `y` | label from the tags: `BREAKER`, `SURFACE` or `NONE` |
| `ambiguous` | 1 = untagged but close to a tag. Left out of training, kept for testing |
| `jpk … span_abs` | raw features: g units, fixed thresholds |
| `ratio_local, t_peak_ms, bf_*, first_sign` | dimensionless features (in both sets) |
| `jpk_z … span` | ABC-normalised features: divided by the vehicle's own σ, per-vehicle thresholds |

## `wholedrive_windows/<car>_windows_1p4s_hop0p2s.csv`

Every 1.4 s window across the analysed part of each drive, starting every 0.2 s, with no trigger. That gives 4220, 4004 and 4196 windows. Written by `edge_ml/wd_dataset.py`.

| Columns | Meaning |
|---|---|
| `t_start, t_anchor, lat, lon` | window start, anchor time, position |
| feature columns | as above, raw and normalised. Normalisation uses the baseline as it stood at the window start, so nothing inside the window leaks in |
| `tier_frac_green` | fraction of the window in the GREEN roughness tier |
| `label` | `NONE`, `SURFACE`, `BREAKER` or `IGNORE` (inside a tag's span but not its strongest window; left out of training) |
| `tag_index` | which tag the window belongs to, or −1 |

The labels are generated automatically from the tags and have not been hand-checked. Many untagged events are real small undulations that nobody tagged, so precision figures are lower bounds.
