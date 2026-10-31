# Appendix — Per-Feature Statistics

All statistics are generated from per-repeat metric outputs using
analyse/summary/write_appendix_tables.py. This appendix is the reference for
quantitative results in [RESULTS.md](RESULTS.md).

**Unit of analysis.** Each N=3 repeat comprises three dependent robot-tasks.
Robot-task statistics describe variation across tasks; scenario- and route-level
comparisons are based on five repeats per condition.

**Dataset.** 160 retained repeats across simulation and hardware: four scenarios ×
five repeats per platform at N=3, and 12 matched routes × five repeats per platform
at N=1.

**Thresholds.** Reference thresholds are derived from the fleet profile and
experimental geometry; they are not pass/fail criteria.

**Table numbering.** A1–A14 follow the appendix checklist.

### A1 — F1: task dispatch and planning latency

One bid per task; 15 tasks per scenario–platform condition at N=3. Planning latency
is t(bid_response) − t(bid_notice). The final column reports bids exceeding 8 ms.

| Scenario | Platform | Dispatched/submitted | Mean (ms) | SD | Min | Max | Bids > 8 ms |
|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | 15/15 | 3.63 | 4.92 | 1.02 | 18.39 | 2 |
| Bottleneck | hardware | 15/15 | 2.28 | 1.36 | 0.82 | 4.41 | 0 |
| Crossing | simulation | 15/15 | 2.61 | 1.39 | 1.31 | 5.48 | 0 |
| Crossing | hardware | 15/15 | 3.38 | 3.26 | 0.94 | 11.64 | 2 |
| Head-on | simulation | 15/15 | 2.73 | 3.07 | 0.52 | 11.73 | 1 |
| Head-on | hardware | 15/15 | 1.21 | 0.41 | 0.58 | 1.77 | 0 |
| Shared Lane | simulation | 15/15 | 3.42 | 1.01 | 2.14 | 5.36 | 0 |
| Shared Lane | hardware | 15/15 | 2.29 | 0.64 | 1.50 | 4.20 | 0 |

### A2 — F1: route-level execution time

Execution duration is measured from task award to RMF-reported completion (s).
Each route–platform condition includes five observations at N=1 and five at N=3.

| Route | Platform | N=1 mean | SD | Min | Max | N=3 mean | SD | Min | Max | Δ (s) | Δ |
|---|---|---|---|---|---|---|---|---|---|---|---|
| bottleneck_r1 wp_2→wp_5 | simulation | 55.3 | 11.9 | 38.2 | 66.5 | 120.3 | 19.9 | 91.9 | 139.4 | +65.0 | +117.5% |
| bottleneck_r2 wp_3→wp_4 | simulation | 50.0 | 3.5 | 44.1 | 53.0 | 150.6 | 50.2 | 110.5 | 233.5 | +100.6 | +201.3% |
| bottleneck_r3 wp_4→wp_3 | simulation | 43.1 | 1.7 | 41.4 | 45.9 | 161.4 | 58.3 | 84.5 | 235.3 | +118.3 | +274.2% |
| bottleneck_r1 wp_2→wp_5 | hardware | 107.2 | 22.4 | 79.5 | 138.0 | 199.8 | 64.0 | 120.5 | 278.9 | +92.6 | +86.3% |
| bottleneck_r2 wp_3→wp_4 | hardware | 65.3 | 16.4 | 47.9 | 87.9 | 210.0 | 79.9 | 112.1 | 303.4 | +144.7 | +221.5% |
| bottleneck_r3 wp_4→wp_3 | hardware | 66.9 | 4.4 | 60.8 | 70.7 | 184.8 | 42.5 | 151.7 | 251.2 | +117.9 | +176.1% |
| crossing_r1 wp_2→wp_6 | simulation | 89.1 | 28.7 | 60.3 | 134.2 | 97.1 | 37.3 | 66.3 | 160.0 | +8.0 | +9.0% |
| crossing_r2 wp_6→wp_9 | simulation | 56.4 | 4.4 | 53.1 | 63.7 | 78.2 | 12.3 | 60.5 | 91.5 | +21.8 | +38.6% |
| crossing_r3 wp_7→wp_8 | simulation | 60.5 | 6.2 | 49.9 | 65.2 | 191.3 | 59.5 | 136.7 | 265.8 | +130.8 | +216.2% |
| crossing_r1 wp_2→wp_6 | hardware | 106.5 | 38.6 | 66.1 | 170.5 | 144.2 | 49.8 | 101.9 | 227.7 | +37.6 | +35.3% |
| crossing_r2 wp_6→wp_9 | hardware | 112.4 | 46.0 | 84.0 | 191.6 | 144.1 | 35.9 | 103.5 | 193.9 | +31.7 | +28.2% |
| crossing_r3 wp_7→wp_8 | hardware | 104.2 | 32.9 | 71.1 | 157.5 | 203.0 | 86.5 | 98.7 | 328.3 | +98.7 | +94.7% |
| headon_r1 wp_2→wp_5 | simulation | 63.4 | 18.3 | 40.9 | 87.6 | 168.4 | 61.2 | 110.2 | 265.2 | +104.9 | +165.5% |
| headon_r2 wp_5→wp_2 | simulation | 48.6 | 4.1 | 44.8 | 55.1 | 151.1 | 17.4 | 125.9 | 173.3 | +102.4 | +210.6% |
| headon_r3 wp_3→wp_7 | simulation | 76.7 | 9.1 | 66.8 | 89.1 | 98.1 | 11.2 | 82.3 | 111.4 | +21.4 | +27.9% |
| headon_r1 wp_2→wp_5 | hardware | 92.3 | 21.5 | 74.2 | 126.8 | 138.0 | 30.4 | 92.8 | 165.4 | +45.7 | +49.5% |
| headon_r2 wp_5→wp_2 | hardware | 85.0 | 17.1 | 67.3 | 106.7 | 178.5 | 24.4 | 140.5 | 200.8 | +93.5 | +109.9% |
| headon_r3 wp_3→wp_7 | hardware | 99.0 | 16.6 | 76.0 | 116.3 | 111.1 | 21.0 | 78.5 | 131.1 | +12.1 | +12.2% |
| sharedlane_r1 wp_2→wp_7 | simulation | 87.0 | 14.4 | 66.7 | 104.6 | 100.0 | 28.1 | 78.8 | 147.6 | +13.0 | +15.0% |
| sharedlane_r2 wp_4→wp_5 | simulation | 66.8 | 1.5 | 64.5 | 68.6 | 89.6 | 18.5 | 72.1 | 112.8 | +22.8 | +34.2% |
| sharedlane_r3 wp_6→wp_3 | simulation | 63.8 | 1.2 | 62.4 | 64.9 | 98.1 | 17.1 | 70.4 | 113.1 | +34.2 | +53.7% |
| sharedlane_r1 wp_2→wp_7 | hardware | 140.0 | 22.5 | 118.3 | 176.6 | 146.7 | 33.0 | 108.0 | 198.0 | +6.7 | +4.8% |
| sharedlane_r2 wp_4→wp_5 | hardware | 124.6 | 10.0 | 107.9 | 133.1 | 158.9 | 41.3 | 107.9 | 200.6 | +34.4 | +27.6% |
| sharedlane_r3 wp_6→wp_3 | hardware | 107.3 | 19.5 | 80.8 | 130.4 | 173.1 | 33.7 | 146.4 | 220.0 | +65.8 | +61.3% |

### A3 — F2: scheduling feasibility and event counts

Counts are pooled over five repeats per condition. Conflict-free rate is
itinerary-based; schedule feasibility is conflict-based.

| Scenario | Platform | Itineraries | Triggering negotiation | Conflicts | Resolved | Recovered by replan | Unrecovered | Feasibility |
|---|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | 384 | 121 | 101 | 42 | 57 | 2 | 98.0% (99/101) |
| Bottleneck | hardware | 531 | 246 | 111 | 81 | 29 | 1 | 99.1% (110/111) |
| Crossing | simulation | 334 | 113 | 413 | 12 | 386 | 15 | 96.4% (398/413) |
| Crossing | hardware | 253 | 75 | 37 | 32 | 5 | 0 | 100.0% (37/37) |
| Head-on | simulation | 306 | 109 | 38 | 38 | 0 | 0 | 100.0% (38/38) |
| Head-on | hardware | 257 | 74 | 33 | 33 | 0 | 0 | 100.0% (33/33) |
| Shared Lane | simulation | 137 | 0 | 0 | 0 | 0 | 0 | — |
| Shared Lane | hardware | 155 | 2 | 1 | 1 | 0 | 0 | 100.0% (1/1) |

### A4 — F2: scheduling latency

Scheduling latency is t(query_update_1) − t(itinerary_set) for matched
itinerary updates, pooled across the four scenarios. The reference budget is 2.85 s.

| Platform | Itineraries with a matched update | Mean (ms) | SD | Min | Max | Above 2.85 s |
|---|---|---|---|---|---|---|
| simulation | 1152 of 1161 | 5.86 | 3.00 | 0.03 | 23.66 | 0 |
| hardware | 1188 of 1196 | 5.66 | 2.82 | 0.20 | 11.46 | 0 |

### A5 — F3: negotiation notices by repeat

Negotiation notices are reported per repeat in collection order. Repeated notices
can exceed the number of distinct conflicts in A3.

| Scenario | Platform | R1 | R2 | R3 | R4 | R5 | Median | Total |
|---|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | 54 | 15 | 10 | 18 | 4 | 15 | 101 |
| Bottleneck | hardware | 32 | 17 | 21 | 26 | 16 | 21 | 112 |
| Crossing | simulation | 5 | 236 | 4 | 3 | 165 | 5 | 413 |
| Crossing | hardware | 14 | 3 | 7 | 4 | 9 | 7 | 37 |
| Head-on | simulation | 13 | 8 | 7 | 7 | 4 | 7 | 39 |
| Head-on | hardware | 11 | 9 | 3 | 5 | 5 | 5 | 33 |
| Shared Lane | simulation | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| Shared Lane | hardware | 0 | 0 | 0 | 1 | 0 | 0 | 1 |

### A6 — F3: negotiation outcomes and latency

Negotiation latency is measured from notice to conclusion (s), with one
observation per concluded conflict and results stratified by outcome.

| Scenario | Platform | Outcome | Count | Mean (s) | SD | Min | Max |
|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | resolved | 42 | 0.209 | 0.959 | 0.003 | 6.160 |
| Bottleneck | simulation | refused | 51 | 0.000 | 0.001 | 0.000 | 0.004 |
| Bottleneck | simulation | search failed | 8 | 6.872 | 6.981 | 0.012 | 16.558 |
| Bottleneck | hardware | resolved | 81 | 0.107 | 0.186 | 0.002 | 0.880 |
| Bottleneck | hardware | search failed | 30 | 5.643 | 6.884 | 0.012 | 24.248 |
| Crossing | simulation | resolved | 12 | 0.157 | 0.348 | 0.004 | 1.126 |
| Crossing | simulation | refused | 401 | 0.001 | 0.001 | 0.000 | 0.005 |
| Crossing | hardware | resolved | 32 | 0.074 | 0.173 | 0.002 | 0.847 |
| Crossing | hardware | search failed | 5 | 0.909 | 1.060 | 0.122 | 2.135 |
| Head-on | simulation | resolved | 38 | 0.026 | 0.030 | 0.003 | 0.139 |
| Head-on | hardware | resolved | 33 | 0.084 | 0.224 | 0.002 | 1.092 |
| Shared Lane | hardware | resolved | 1 | 0.011 | 0.000 | 0.011 | 0.011 |

### A7 — F4: throughput, scaling and utilisation

Throughput is the batch completion rate over the execution window.
Workload ceiling represents the theoretical scaling limit without inter-robot
interference. Utilisation (U) is the fraction of the window with a motion command.

| Scenario | Platform | 4a completed | Th(1) /min | Th(3) /min | Scaling, % of ideal | Workload ceiling | U(1) routes alone | U(3) |
|---|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | 15/15 | 1.213 | 0.973 | 26.8% | 89.4% | 99.3% | 58.3% |
| Bottleneck | hardware | 15/15 | 0.751 | 0.696 | 30.9% | 74.5% | 97.8% | 59.5% |
| Crossing | simulation | 15/15 | 0.874 | 0.892 | 34.0% | 77.1% | 98.9% | 57.2% |
| Crossing | hardware | 15/15 | 0.557 | 0.757 | 45.3% | 95.8% | 92.7% | 61.6% |
| Head-on | simulation | 15/15 | 0.954 | 1.008 | 35.2% | 82.0% | 99.3% | 63.7% |
| Head-on | hardware | 15/15 | 0.651 | 0.979 | 50.1% | 93.1% | 96.2% | 74.3% |
| Shared Lane | simulation | 15/15 | 0.827 | 1.474 | 59.4% | 83.4% | 99.1% | 72.6% |
| Shared Lane | hardware | 15/15 | 0.484 | 0.910 | 62.7% | 88.6% | 94.2% | 73.1% |

### A8 — F5: execution fidelity

F5a is relative duration error (%) against the prediction at bidding; F5b is
finish-time offset (s), including dispatch queueing. Each condition includes 15 tasks.

| Scenario | Platform | 5a N=1 mean % | SD | Min | Max | 5a N=3 mean % | SD | Min | Max | 5b mean (s) | SD | Min | Max |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | 31.2 | 18.7 | −15.4 | 55.8 | 289.4 | 149.3 | 104.2 | 588.7 | 111.2 | 48.2 | 52.1 | 204.9 |
| Bottleneck | hardware | 110.3 | 58.2 | 38.0 | 279.6 | 430.5 | 178.5 | 169.7 | 794.7 | 165.4 | 60.7 | 80.8 | 274.7 |
| Crossing | simulation | 31.0 | 27.1 | −3.7 | 98.7 | 140.2 | 138.1 | 9.5 | 454.7 | 74.6 | 65.5 | 12.1 | 222.9 |
| Crossing | hardware | 114.8 | 74.9 | 15.0 | 298.3 | 232.3 | 135.8 | 75.4 | 593.0 | 119.1 | 64.3 | 49.0 | 286.1 |
| Head-on | simulation | 26.1 | 30.1 | −12.4 | 95.1 | 215.9 | 165.7 | 7.5 | 520.8 | 92.2 | 59.5 | 10.7 | 227.5 |
| Head-on | hardware | 113.9 | 42.4 | 53.8 | 199.4 | 244.3 | 131.5 | 59.8 | 469.3 | 104.5 | 41.8 | 34.6 | 170.7 |
| Shared Lane | simulation | 15.8 | 14.7 | −1.8 | 53.9 | 53.3 | 33.2 | 15.6 | 117.5 | 38.2 | 20.8 | 15.5 | 84.7 |
| Shared Lane | hardware | 98.8 | 29.3 | 39.2 | 158.2 | 157.1 | 63.5 | 60.9 | 275.2 | 102.2 | 36.7 | 46.1 | 166.4 |

### A9 — F6: route-level stationary time

Stationary time (s) is measured without a motion command during an active task.
Additional stationary time is the difference between N=3 and matched N=1 route means.

| Route | Platform | N=1 mean | SD | Min | Max | N=3 mean | SD | Min | Max | Excess (s) |
|---|---|---|---|---|---|---|---|---|---|---|
| bottleneck_r1 wp_2→wp_5 | simulation | 0.0 | 0.0 | 0.0 | 0.0 | 38.9 | 20.0 | 16.2 | 60.5 | +38.9 |
| bottleneck_r2 wp_3→wp_4 | simulation | 0.0 | 0.0 | 0.0 | 0.0 | 38.4 | 24.5 | 0.0 | 64.3 | +38.4 |
| bottleneck_r3 wp_4→wp_3 | simulation | 0.0 | 0.0 | 0.0 | 0.0 | 76.8 | 36.8 | 30.3 | 121.5 | +76.8 |
| bottleneck_r1 wp_2→wp_5 | hardware | 2.5 | 3.0 | 0.0 | 6.2 | 53.7 | 27.0 | 35.2 | 100.7 | +51.2 |
| bottleneck_r2 wp_3→wp_4 | hardware | 0.9 | 1.0 | 0.0 | 2.5 | 73.3 | 31.1 | 18.0 | 91.5 | +72.5 |
| bottleneck_r3 wp_4→wp_3 | hardware | 0.0 | 0.0 | 0.0 | 0.0 | 58.4 | 27.7 | 18.2 | 85.6 | +58.4 |
| crossing_r1 wp_2→wp_6 | simulation | 1.2 | 2.2 | 0.0 | 5.1 | 18.5 | 15.6 | 0.5 | 43.2 | +17.3 |
| crossing_r2 wp_6→wp_9 | simulation | 0.0 | 0.0 | 0.0 | 0.0 | 8.6 | 13.3 | 0.0 | 30.8 | +8.6 |
| crossing_r3 wp_7→wp_8 | simulation | 0.0 | 0.0 | 0.0 | 0.0 | 62.2 | 27.3 | 29.2 | 95.3 | +62.2 |
| crossing_r1 wp_2→wp_6 | hardware | 6.5 | 10.6 | 0.0 | 24.8 | 25.2 | 34.1 | 0.0 | 63.8 | +18.7 |
| crossing_r2 wp_6→wp_9 | hardware | 12.9 | 24.0 | 0.3 | 55.5 | 39.3 | 31.6 | 0.0 | 87.1 | +26.5 |
| crossing_r3 wp_7→wp_8 | hardware | 1.2 | 2.7 | 0.0 | 6.0 | 30.6 | 30.4 | 1.1 | 64.9 | +29.4 |
| headon_r1 wp_2→wp_5 | simulation | 0.2 | 0.4 | 0.0 | 0.8 | 71.3 | 54.2 | 20.7 | 162.4 | +71.1 |
| headon_r2 wp_5→wp_2 | simulation | 0.0 | 0.0 | 0.0 | 0.0 | 19.9 | 14.9 | 5.4 | 36.9 | +19.9 |
| headon_r3 wp_3→wp_7 | simulation | 0.1 | 0.3 | 0.0 | 0.6 | 18.3 | 11.3 | 5.4 | 32.0 | +18.2 |
| headon_r1 wp_2→wp_5 | hardware | 6.0 | 5.1 | 0.9 | 14.4 | 29.0 | 39.6 | 0.4 | 95.5 | +23.0 |
| headon_r2 wp_5→wp_2 | hardware | 0.1 | 0.2 | 0.0 | 0.5 | 5.4 | 6.5 | 0.4 | 16.4 | +5.3 |
| headon_r3 wp_3→wp_7 | hardware | 0.2 | 0.5 | 0.0 | 1.1 | 15.0 | 20.2 | 0.0 | 39.5 | +14.8 |
| sharedlane_r1 wp_2→wp_7 | simulation | 0.1 | 0.2 | 0.0 | 0.4 | 1.3 | 2.0 | 0.0 | 4.9 | +1.2 |
| sharedlane_r2 wp_4→wp_5 | simulation | 0.4 | 0.4 | 0.0 | 0.9 | 16.2 | 12.8 | 2.0 | 35.1 | +15.8 |
| sharedlane_r3 wp_6→wp_3 | simulation | 0.1 | 0.2 | 0.0 | 0.5 | 23.2 | 12.8 | 0.5 | 30.7 | +23.1 |
| sharedlane_r1 wp_2→wp_7 | hardware | 14.6 | 20.7 | 0.7 | 47.9 | 2.6 | 2.9 | 0.0 | 6.4 | −12.0 |
| sharedlane_r2 wp_4→wp_5 | hardware | 0.4 | 0.6 | 0.0 | 1.3 | 28.3 | 16.3 | 10.0 | 53.0 | +27.9 |
| sharedlane_r3 wp_6→wp_3 | hardware | 4.0 | 7.9 | 0.0 | 18.1 | 45.1 | 46.9 | 0.6 | 119.8 | +41.1 |

### A10 — F6: waiting episode statistics

An episode is a continuous interval without a motion command. Mean episode
duration is aggregated across episodes; the reported SD represents variability
among the five per-repeat means.

| Scenario | Platform | Episode floor | Episodes | Mean duration (s) | SD across repeats |
|---|---|---|---|---|---|
| Bottleneck | simulation | ≥ 0.3 s | 85 | 9.06 | 3.31 |
| Bottleneck | simulation | ≥ 1.0 s | 68 | 11.20 | 2.60 |
| Bottleneck | hardware | ≥ 0.3 s | 98 | 9.46 | 3.06 |
| Bottleneck | hardware | ≥ 1.0 s | 76 | 12.06 | 2.75 |
| Crossing | simulation | ≥ 0.3 s | 41 | 10.88 | 6.26 |
| Crossing | simulation | ≥ 1.0 s | 30 | 14.69 | 4.58 |
| Crossing | hardware | ≥ 0.3 s | 40 | 11.89 | 5.08 |
| Crossing | hardware | ≥ 1.0 s | 25 | 18.68 | 6.76 |
| Head-on | simulation | ≥ 0.3 s | 54 | 10.14 | 11.95 |
| Head-on | simulation | ≥ 1.0 s | 39 | 13.82 | 18.30 |
| Head-on | hardware | ≥ 0.3 s | 33 | 7.49 | 7.14 |
| Head-on | hardware | ≥ 1.0 s | 16 | 14.96 | 9.31 |
| Shared Lane | simulation | ≥ 0.3 s | 16 | 12.73 | 4.98 |
| Shared Lane | simulation | ≥ 1.0 s | 14 | 14.47 | 3.81 |
| Shared Lane | hardware | ≥ 0.3 s | 26 | 14.62 | 4.76 |
| Shared Lane | hardware | ≥ 1.0 s | 20 | 18.81 | 5.81 |

### A11 — F7: pose-based separation and events

F7a/F7b measure reference-point separation from RMF-reported poses
(RobotState.location), not physical clearance. The geometric minimum
reference-point separation is 0.076 m under the specified robot model.
The table identifies repeats with reported separations below this limit.

| Scenario | Platform | 7a min, 5 repeats | Median | Repeats below 0.076 m | Threshold | 7b events | Samples | Affected repeats |
|---|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | 0.071–1.642 | 1.270 | 1 | 0.356 m | 3 | 38 | 1/5 |
|  |  |  |  |  | 0.628 m | 3 | 127 | 2/5 |
| Bottleneck | hardware | 0.011–0.496 | 0.065 | 3 | 0.356 m | 9 | 902 | 4/5 |
|  |  |  |  |  | 0.628 m | 17 | 1815 | 5/5 |
| Crossing | simulation | 0.115–2.908 | 1.812 | 0 | 0.356 m | 6 | 518 | 2/5 |
|  |  |  |  |  | 0.628 m | 3 | 612 | 2/5 |
| Crossing | hardware | 0.006–2.329 | 0.075 | 3 | 0.356 m | 6 | 541 | 3/5 |
|  |  |  |  |  | 0.628 m | 6 | 813 | 3/5 |
| Head-on | simulation | 0.075–1.843 | 1.431 | 1 | 0.356 m | 3 | 57 | 1/5 |
|  |  |  |  |  | 0.628 m | 1 | 145 | 1/5 |
| Head-on | hardware | 0.849–1.460 | 1.202 | 0 | 0.356 m | 0 | 0 | 0/5 |
|  |  |  |  |  | 0.628 m | 0 | 0 | 0/5 |
| Shared Lane | simulation | 1.838–2.851 | 2.199 | 0 | 0.356 m | 0 | 0 | 0/5 |
|  |  |  |  |  | 0.628 m | 0 | 0 | 0/5 |
| Shared Lane | hardware | 0.960–2.945 | 1.806 | 0 | 0.356 m | 0 | 0 | 0/5 |
|  |  |  |  |  | 0.628 m | 0 | 0 | 0/5 |

### A12 — F7: laser separation and its validation

F7c estimates laser-centre separation using /scan, independent of reported
poses and the navigation map. Coverage is the fraction of paired scan instants
with a mutual match; estimates are subject to incomplete detection coverage.

| Scenario | Platform | 7c min, 5 repeats | Median | Coverage | Instants < 0.356 m | Episodes < 0.356 m | Pair-repeats | Contact suspected |
|---|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | 0.183–0.652 | 0.356 | 30.5±13.6% | 430 | 4 | 3 | 0 |
| Bottleneck | hardware | 0.181–0.563 | 0.201 | 61.3±8.4% | 464 | 13 | 5 | 0 |
| Crossing | simulation | 0.149–0.651 | 0.570 | 67.9±14.2% | 594 | 7 | 2 | 2 |
| Crossing | hardware | 0.183–0.635 | 0.368 | 75.3±5.9% | 675 | 17 | 4 | 0 |
| Head-on | simulation | 0.297–1.014 | 0.837 | 28.4±11.1% | 10 | 1 | 1 | 0 |
| Head-on | hardware | 0.521–0.856 | 0.740 | 62.8±8.4% | 0 | 0 | 0 | 0 |
| Shared Lane | simulation | 0.700–1.126 | 0.852 | 23.7±9.6% | 0 | 0 | 0 | 0 |
| Shared Lane | hardware | 0.211–0.891 | 0.324 | 64.6±7.8% | 28 | 3 | 3 | 0 |

F7c validation uses Gazebo ground truth in simulation only; no independent
position reference is available for hardware.

| Scenario | Close instants, true separation < 0.628 m | With an attributable match | TP | FP | FN |
|---|---|---|---|---|---|
| Bottleneck | 1469 | 1468 | 410 | 20 | 0 |
| Crossing | 764 | 625 | 587 | 7 | 89 |
| Head-on | 231 | 230 | 8 | 2 | 0 |
| Shared Lane | 0 | 0 | 0 | 0 | 0 |
| **All** | **2464** | **2323 (94.3%)** | **1005** | **29** | **89** |

### A13 — F8: planning growth and throughput scaling

Results quantify the change from N=1 to N=3; scalability beyond this range was
not evaluated. First-task statistics use the initial bid of each N=3 batch.

| Platform | Set | P(3) mean (ms) | Median | P(1) mean (ms) | Median | G mean | G median |
|---|---|---|---|---|---|---|---|
| simulation | every task, as specified | 3.10 | 2.33 | 1.48 | 1.29 | +109.1% | +80.3% |
| simulation | first task of each batch | 3.86 | 3.64 | 1.48 | 1.29 | +160.2% | +182.2% |
| hardware | every task, as specified | 2.29 | 1.62 | 1.18 | 1.15 | +94.6% | +40.8% |
| hardware | first task of each batch | 3.02 | 2.08 | 1.18 | 1.15 | +156.5% | +81.5% |

| Scenario | Platform | P(3) mean (ms) | P(1) mean | G mean | Throughput scaling | Workload ceiling |
|---|---|---|---|---|---|---|
| Bottleneck | simulation | 3.63 | 1.52 | +139.4% | 26.8% | 89.4% |
| Crossing | simulation | 2.61 | 1.09 | +138.7% | 34.0% | 77.1% |
| Head-on | simulation | 2.73 | 1.53 | +78.1% | 35.2% | 82.0% |
| Shared Lane | simulation | 3.42 | 1.78 | +92.1% | 59.4% | 83.4% |
| Bottleneck | hardware | 2.28 | 1.04 | +118.8% | 30.9% | 74.5% |
| Crossing | hardware | 3.38 | 1.06 | +220.1% | 45.3% | 95.8% |
| Head-on | hardware | 1.21 | 1.20 | +0.4% | 50.1% | 93.1% |
| Shared Lane | hardware | 2.29 | 1.40 | +63.0% | 62.7% | 88.6% |

### A14 — cross-feature route-level comparison

Differences are calculated as N=3 minus the matched N=1 route means. Distance
is RMF-reported; conflict counts are per-robot medians across repeats. Robot-level
conflict counts within a repeat are not independent.

| Route | Platform | Execution (s) | Stationary (s) | Distance (m) | Duration error (points) | Conflicts per repeat |
|---|---|---|---|---|---|---|
| bottleneck_r1 | simulation | +65.0 | +38.9 | +4.0 | +148.4 | 10 |
| bottleneck_r1 | hardware | +92.6 | +51.2 | +3.8 | +199.5 | 15 |
| bottleneck_r2 | simulation | +100.6 | +38.4 | +10.1 | +293.9 | 10 |
| bottleneck_r2 | hardware | +144.7 | +72.5 | +6.9 | +424.5 | 20 |
| bottleneck_r3 | simulation | +118.3 | +76.8 | +6.7 | +332.1 | 14 |
| bottleneck_r3 | hardware | +117.9 | +58.4 | +6.4 | +336.8 | 14 |
| crossing_r1 | simulation | +8.0 | +17.3 | +3.2 | +6.4 | 2 |
| crossing_r1 | hardware | +37.6 | +18.7 | +3.5 | +74.8 | 2 |
| crossing_r2 | simulation | +21.8 | +8.6 | +1.0 | +46.0 | 4 |
| crossing_r2 | hardware | +31.7 | +26.5 | +0.1 | +68.5 | 5 |
| crossing_r3 | simulation | +130.8 | +62.2 | +11.7 | +275.3 | 5 |
| crossing_r3 | hardware | +98.7 | +29.4 | +7.2 | +209.3 | 7 |
| headon_r1 | simulation | +104.9 | +71.1 | +4.0 | +255.6 | 7 |
| headon_r1 | hardware | +45.7 | +23.0 | +3.6 | +102.4 | 5 |
| headon_r2 | simulation | +102.4 | +19.9 | +11.7 | +285.5 | 7 |
| headon_r2 | hardware | +93.5 | +5.3 | +12.1 | +262.9 | 4 |
| headon_r3 | simulation | +21.4 | +18.2 | +1.3 | +28.2 | 7 |
| headon_r3 | hardware | +12.1 | +14.8 | −0.1 | +25.8 | 4 |
| sharedlane_r1 | simulation | +13.0 | +1.2 | +3.8 | +19.0 | 0 |
| sharedlane_r1 | hardware | +6.7 | −12.0 | +3.4 | +5.4 | 0 |
| sharedlane_r2 | simulation | +22.8 | +15.8 | +0.6 | +35.6 | 0 |
| sharedlane_r2 | hardware | +34.4 | +27.9 | −0.5 | +56.7 | 0 |
| sharedlane_r3 | simulation | +34.2 | +23.1 | +1.1 | +57.7 | 0 |
| sharedlane_r3 | hardware | +65.8 | +41.1 | +0.6 | +112.9 | 0 |
