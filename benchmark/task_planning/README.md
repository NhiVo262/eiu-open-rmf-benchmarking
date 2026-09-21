# Task Planning benchmark data

Data and scripts for Feature Claim #1 (Task Planning): dispatch success rate,
planning latency, and makespan. Full results and analysis are in the
Milestone 2 report; this folder holds the underlying data.

## Structure

| Folder | Content |
|---|---|
| `clearpath_FLEET01_TRAF00_CONFIG01/` | N=1, no contention, `charger_1 → crossing_1`, 5 repeats |
| `bottleneck_FLEET0N_TRAF03_CONFIG01/` | N=3, official run (`run_20260918_1356_N3`), 5 repeats × 3 tasks |
| `crossing_FLEET02_TRAF01_CONFIG01/` | N=3, official run (`run_20260919_0109_N3`), 5 repeats × 3 tasks |
| `headon_FLEET0N_TRAF04_CONFIG01/` | N=3, official run (`run_20260919_0322_N3`), 5 repeats × 3 tasks |
| `sharedlane_FLEET0N_TRAF02_CONFIG01/` | N=3, official run (`run_20260919_0431_N3`), 5 repeats × 3 tasks |
| `baselines_matched/` | N=1 baseline for each of the 12 routes used across the 4 N=3 scenarios above, 5 repeats each |
| `scripts/` | Collection (`run_*.py/.sh`) and analysis (`analyze_*.py`) scripts |

## Reproducing

See `PROCEDURE.md` for how each dataset was collected and how to
re-run/verify it.
