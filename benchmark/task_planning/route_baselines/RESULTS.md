# Results — Route baselines (N=1, matched to each of the 12 routes)

Computed directly from each route's `task_planning_metrics.json` (`per_task[].makespan_s`). See `PROCEDURE.md` for how these were run.

| Scenario | Route | n | Mean (s) | Stdev (s) | Min (s) | Max (s) |
|---|---|---|---|---|---|---|
| Bottleneck | bottleneck_1 → bottleneck_3 | 5 | 52.6 | 21.0 | 39.8 | 89.9 |
| Bottleneck | sharedlane_3 → loop_1 | 5 | 121.2 | 21.5 | 104.7 | 155.4 |
| Bottleneck | bottleneck_3 → sharedlane_3 | 5 | 87.9 | 24.8 | 73.0 | 132.1 |
| Crossing | charger_1 → bottleneck_1 | 5 | 170.6 | 37.4 | 103.8 | 189.7 |
| Crossing | crossing_1 → loop_4 | 5 | 58.0 | 14.4 | 33.7 | 72.2 |
| Crossing | crossing_2 → bottleneck_3 | 5 | 171.9 | 33.3 | 113.8 | 192.7 |
| Head-on | charger_1 → bottleneck_3 | 5 | 193.4 | 42.2 | 119.1 | 222.6 |
| Head-on | crossing_1 → bottleneck_3 | 5 | 166.1 | 39.7 | 95.2 | 186.1 |
| Head-on | crossing_2 → bottleneck_1 | 5 | 146.8 | 31.7 | 91.5 | 170.0 |
| Shared Lane | sharedlane_1 → sharedlane_3 | 5 | 79.4 | 7.7 | 65.6 | 83.2 |
| Shared Lane | sharedlane_3 → sharedlane_1 | 5 | 91.1 | 13.4 | 81.2 | 114.3 |
| Shared Lane | charger_1 → sharedlane_2 | 5 | 116.8 | 22.8 | 76.5 | 133.1 |


Pooled per scenario (3 routes × 5 repeats = 15), matches the report's "Mean makespan by scenario" table:

| Scenario | n | Mean (s) | Stdev (s) | Min (s) | Max (s) |
|---|---|---|---|---|---|
| Bottleneck | 15 | 87.2 | 35.7 | 39.8 | 155.4 |
| Crossing | 15 | 133.5 | 61.9 | 33.7 | 192.7 |
| Head-on | 15 | 168.8 | 40.5 | 91.5 | 222.6 |
| Shared Lane | 15 | 95.8 | 21.9 | 65.6 | 133.1 |

Each route's baseline is compared only against its own Actual (N=3) makespan — see the official scenario dirs (`bottleneck_FLEET0N_TRAF03_CONFIG01`, etc.) and the report's "Over Baseline by Route" chart for the paired comparison.
