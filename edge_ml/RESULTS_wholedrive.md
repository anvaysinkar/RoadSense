# RoadSense whole-drive classifier: results

## Task

- **Input:** a 1.4 s window of the 100 Hz signals.
- **Output:** smooth, surface defect (pothole or undulation) or speed breaker.
- **Scoring:** per real anomaly. Positive windows are merged into events, and each event is matched to a passenger tag if it starts between 5 s before and 1.5 s after the tag, which is the paper's rule. Evaluation is leave-one-vehicle-out: train on two cars, test on the third. No detector or trigger is used, so the model sees the whole drive.

## Step 1: dataset (`<car>_windows_1p4s_hop0p2s.csv`)

| Drive | Windows | NONE | SURFACE | BREAKER | IGNORE | Tags |
|---|---|---|---|---|---|---|
| Hatchback | 4220 | 3349 | 64 | 18 | 789 | 30 |
| Sedan | 4004 | 2852 | 96 | 18 | 1038 | 40 |
| SUV | 4196 | 3217 | 81 | 18 | 880 | 34 |

- **Windows:** 1.4 s long, starting every 0.2 s across the analysed part of each drive.
- **Features:** 21 per window, in raw units and ABC-normalised. Normalised features use the car's baseline as it stood at the start of the window, so nothing from inside the window leaks into the normalisation.
- **Labels:** each tag owns the windows whose anchor falls in [tag − 5 s, tag + 1.5 s]. The strongest window in that span (heave for a breaker, jerk for a surface defect) and its two neighbours are labelled positive. The rest of the span is IGNORE and is left out of training. Every other window is NONE.

## Steps 2 to 4: baselines and models (pooled over the three test drives)

The comparison is made at the same false-alarm budget as the rule detector at k_L = 4, which gives 131 untagged events:

| Method | Untagged | Tags found | Breaker recall | Breaker precision | Surface recall | Class accuracy |
|---|---|---|---|---|---|---|
| Rules, k_L = 4 | 131 | 0.81 | 0.67 | 0.36 | 0.71 | 0.87 |
| One feature (jerk z ≥ 6) | 124 | 0.78 | 0.00 | 0.00 | 0.79 | 0.84 |
| **Forest, normalised** | 133 | **0.88** | **0.76** | **0.52** | **0.81** | **0.91** |
| Forest, raw | 130 | 0.78 | 0.81 | 0.77 | 0.72 | 0.95 |
| Boosting, normalised | 95 | 0.79 | 0.72 | 0.45 | 0.73 | 0.93 |
| Boosting, raw | 131 | 0.79 | 0.89 | 0.70 | 0.72 | 0.95 |
| Logistic regression, normalised | 114 | 0.84 | 0.56 | 0.33 | 0.80 | 0.91 |

At the strict end, with a budget of 40 untagged events (about the paper's rule at k_L = 6):

| Method | Untagged | Tags found | Breaker recall | Breaker precision |
|---|---|---|---|---|
| Rules, k_L = 6 | 33 | 0.51 | 0.83 | 0.56 |
| Forest, normalised | 36 | 0.62 | 0.54 | 0.71 |

The forest and boosting rows are the mean of 3 random seeds. The full threshold sweeps are in `wd_summary.txt`.

## Step 5: what the results show

1. **The whole-drive forest beats the rule detector.** With the same number of false alarms (about 130), the normalised forest finds 88% of tagged anomalies against 81% for the rules. Surface recall rises from 0.71 to 0.81, breaker recall from 0.67 to 0.76, breaker precision from 0.36 to 0.52, and class accuracy from 0.87 to 0.91.
2. **The normalised model is more consistent across cars.** The raw models look strong on breakers when the three drives are pooled, but per car they are unstable. At the same budget, the raw forest logs only 16 events on the hatchback, finding 47% of its tags, and 121 on the SUV, 90 of them untagged. The normalised forest stays at 66 to 86 events per car and finds 85 to 90% of tags on every car. A fixed scale suits one car and not another, which is exactly the problem ABC was designed for. The right-hand panel of `wd_results.png` shows this.
3. **One feature isn't enough.** A single jerk threshold finds surface defects but no breakers. The heave-based features matter, and they rank at the top of the feature importance: heave duration first, then heave spread and low-frequency vertical energy.
4. **At the strict end, the trade-off is mixed.** At about 35 false alarms, the forest finds more anomalies than the rule (0.62 vs 0.51) but fewer breakers (0.54 vs 0.83). The physics rule for breakers (heave duration ≥ 450 ms) remains hard to beat at very low false-alarm levels.

## Caveats

- **Small sample:** 18 breakers in total, so per-car breaker figures move in steps of 17 points (one breaker out of six).
- **Precision is a lower bound:** many untagged events are real small undulations nobody tagged.
- **Positive windows are chosen automatically:** the strongest response inside each tag's span. This is a reasonable choice, but the labels are not hand-checked.
- **No threshold tuned on test data:** the tables report whole sweeps and compare at matched false-alarm budgets.

## Files

- `wd_dataset.py`: builds the windows and labels (step 1). Output goes to `results/wholedrive/`; the published copies are in `../datasets/wholedrive_windows/`.
- `wd_experiment.py`: runs the baselines, models and leave-one-vehicle-out evaluation (steps 2 to 5).
- `wd_figure.py`: draws `results/wholedrive/wd_results.png`.
- `results/wholedrive/wd_summary.txt`: the full output.
- `results/wholedrive/wd_curves.json`: every operating point, with per-vehicle numbers.
- `full_dataset.py`: optional export of every 100 Hz sample and every 1 s window of each drive.
