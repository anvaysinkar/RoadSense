# ICEdge 2026

**RoadSense**, first-author paper by Anvay Sinkar, accepted at ICEdge 2026, IISc Bangalore (17–19 December 2026). The proceedings citation will be added here once it is published.

## What the paper covers

| In the paper | Where it is in this repo |
|---|---|
| ESP32 + MPU6050 + NEO-6M edge node | `firmware/esp32/RoadSense_v4_ML/` built with `USE_ML 0` (the v3 rule detector) |
| Adaptive Baseline Calibration | `AdaptiveBaseline.h`; offline replay in `edge_ml/roadsense_analyze.py` |
| Rule classifier (heave duration, hit count) | same sketch, `USE_ML 0`; parameters in `edge_ml/best_3veh.json` |
| Cross-vehicle validation: tuned on a hatchback, tested unchanged on a sedan and an SUV over the same route | `edge_ml/eval_3vehicles.py` |
| Comparison against fixed thresholds carried over from the tuning car | `edge_ml/eval_3vehicles.py` (static baseline) |

Pooled over the three drives, the rule detector with ABC found 15 of 18 speed breakers (recall 0.83), with breaker precision 0.56 and class accuracy 0.92, from 86 events (33 untagged). Using the same pipeline with the hatchback's thresholds fixed in absolute units produced 598 events (497 untagged), with breaker precision of 0.07–0.08 on each car.

To reproduce, from `edge_ml/`:
```
python3 eval_3vehicles.py ../datasets/raw_drives/hatchback.txt ../datasets/raw_drives/sedan.txt \
        ../datasets/raw_drives/suv.txt --params best_3veh.json --out results/paper
```

## Work after the paper

The following are not part of the paper:

- The on-device random forest (v4)
- The whole-drive classifier
- The STM32F411 port

Results are in `edge_ml/RESULTS_ML.md` and `edge_ml/RESULTS_wholedrive.md`.
