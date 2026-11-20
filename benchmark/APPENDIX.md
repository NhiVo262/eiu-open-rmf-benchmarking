# Appendix — Per-Feature Statistics

Detailed F1–F8 results for the 160 retained repeats (40 N=3 scenario repeats and 120 N=1 route baselines) across simulation and hardware. Tables use five repeats per condition; N=3 robot-tasks within a repeat are dependent. Source data: data/. See [RESULTS.md](RESULTS.md) for the main findings and [METRIC_DESIGN.md](METRIC_DESIGN.md) for metric definitions.

### A1 — F1: Task Dispatch and Planning Latency

All 120 N=3 task dispatches succeeded (15 per scenario–platform condition). Planning latency remained below 8 ms for every matched bid; hardware bid-response coverage was 10–11 of 15 tasks per condition. Latency statistics use matched bids only.

| Scenario | Platform | Dispatched/submitted | Bids matched | Mean (ms) | SD | Min | Max | Bids > 8 ms |
|---|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | 15/15 | 15/15 | 1.68 | 0.52 | 0.81 | 2.79 | 0 |
| Bottleneck | hardware | 15/15 | 11/15 | 1.32 | 0.32 | 0.91 | 1.73 | 0 |
| Crossing | simulation | 15/15 | 15/15 | 2.20 | 1.13 | 1.08 | 4.06 | 0 |
| Crossing | hardware | 15/15 | 10/15 | 1.47 | 0.36 | 1.19 | 2.36 | 0 |
| Head-on | simulation | 15/15 | 15/15 | 1.36 | 0.58 | 0.53 | 2.16 | 0 |
| Head-on | hardware | 15/15 | 11/15 | 1.28 | 0.47 | 0.71 | 1.82 | 0 |
| Shared Lane | simulation | 15/15 | 15/15 | 3.07 | 0.99 | 1.83 | 5.41 | 0 |
| Shared Lane | hardware | 15/15 | 11/15 | 2.37 | 0.78 | 1.61 | 3.88 | 0 |

### A2 — F1: Route-Level Execution Time

N=3 execution time exceeded N=1 on most routes, with the largest increase at Bottleneck R1 in simulation (+168.5 s; +492.0%). Crossing R1 and Shared Lane R1 were slightly faster at N=3 on hardware. Execution time = t(completed) − t(award); Δ compares route means and excludes dispatcher queueing.

| Route | Platform | N=1 mean | SD | Min | Max | N=3 mean | SD | Min | Max | Δ (s) | Δ (%) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Bottleneck R1 | simulation | 34.2 | 3.4 | 30.9 | 39.7 | 202.7 | 87.6 | 50.4 | 258.5 | +168.5 | +492.0% |
| Bottleneck R1 | hardware | 55.2 | 7.2 | 45.6 | 61.3 | 124.6 | 30.2 | 99.9 | 168.3 | +69.4 | +125.7% |
| Bottleneck R2 | simulation | 44.3 | 14.4 | 29.9 | 63.8 | 146.8 | 44.1 | 87.2 | 183.5 | +102.5 | +231.2% |
| Bottleneck R2 | hardware | 44.9 | 8.5 | 37.2 | 55.5 | 96.7 | 39.9 | 63.3 | 150.2 | +51.7 | +115.2% |
| Bottleneck R3 | simulation | 35.9 | 2.9 | 32.7 | 38.9 | 124.4 | 44.4 | 47.3 | 157.4 | +88.5 | +246.3% |
| Bottleneck R3 | hardware | 69.8 | 5.8 | 61.4 | 77.5 | 208.7 | 77.9 | 134.2 | 302.4 | +138.9 | +199.0% |
| Crossing R1 | simulation | 50.4 | 0.6 | 49.6 | 51.2 | 51.1 | 0.5 | 50.4 | 51.7 | +0.7 | +1.3% |
| Crossing R1 | hardware | 102.8 | 56.7 | 54.1 | 195.8 | 88.7 | 37.0 | 53.9 | 146.1 | -14.1 | -13.7% |
| Crossing R2 | simulation | 60.6 | 6.2 | 51.1 | 68.4 | 63.1 | 9.3 | 52.5 | 73.5 | +2.4 | +4.0% |
| Crossing R2 | hardware | 64.3 | 10.4 | 56.1 | 80.3 | 81.2 | 17.0 | 63.2 | 108.0 | +17.0 | +26.4% |
| Crossing R3 | simulation | 70.9 | 8.9 | 58.4 | 82.7 | 154.1 | 25.3 | 131.7 | 194.5 | +83.3 | +117.5% |
| Crossing R3 | hardware | 61.5 | 5.7 | 53.6 | 68.4 | 203.6 | 52.9 | 163.9 | 296.2 | +142.1 | +231.0% |
| Head-on R1 | simulation | 44.6 | 17.8 | 30.6 | 74.2 | 101.8 | 45.8 | 37.0 | 163.0 | +57.1 | +128.0% |
| Head-on R1 | hardware | 61.3 | 14.1 | 42.4 | 78.0 | 137.7 | 21.4 | 107.5 | 161.8 | +76.3 | +124.4% |
| Head-on R2 | simulation | 41.8 | 2.9 | 39.6 | 46.6 | 112.8 | 6.9 | 102.1 | 120.2 | +71.1 | +170.2% |
| Head-on R2 | hardware | 64.5 | 9.4 | 55.0 | 78.7 | 143.7 | 30.8 | 90.7 | 165.9 | +79.2 | +122.9% |
| Head-on R3 | simulation | 71.3 | 4.5 | 65.4 | 76.4 | 127.8 | 102.9 | 69.4 | 311.1 | +56.5 | +79.2% |
| Head-on R3 | hardware | 81.5 | 17.2 | 57.3 | 105.6 | 96.6 | 39.2 | 62.4 | 146.1 | +15.1 | +18.5% |
| Shared Lane R1 | simulation | 66.9 | 9.2 | 61.3 | 83.0 | 78.1 | 9.0 | 63.9 | 86.6 | +11.2 | +16.7% |
| Shared Lane R1 | hardware | 113.6 | 34.6 | 79.7 | 168.7 | 112.7 | 19.8 | 93.3 | 134.7 | -0.9 | -0.8% |
| Shared Lane R2 | simulation | 64.5 | 5.2 | 57.2 | 71.6 | 80.8 | 5.5 | 73.3 | 87.8 | +16.3 | +25.3% |
| Shared Lane R2 | hardware | 113.5 | 7.6 | 103.2 | 123.0 | 142.2 | 37.5 | 99.9 | 191.9 | +28.7 | +25.2% |
| Shared Lane R3 | simulation | 73.9 | 7.6 | 64.6 | 85.8 | 102.0 | 13.2 | 90.1 | 123.0 | +28.1 | +38.1% |
| Shared Lane R3 | hardware | 99.7 | 27.5 | 68.0 | 144.0 | 122.2 | 14.5 | 104.0 | 142.8 | +22.5 | +22.6% |

### A3 — F2: Scheduling Feasibility and Event Counts

Bottleneck showed the lowest scheduling feasibility (39.1% simulation; 79.4% hardware); Crossing and Head-on reached 99.5–100%. Shared Lane had no classified conflicts. Feasibility = (Resolved + Recovered by replan) / Conflicts; itinerary counts and classified conflict versions are different units.

| Scenario | Platform | Itineraries | Triggering negotiation | Conflicts | Resolved | Recovered by replan | Unrecovered | Feasibility |
|---|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | 178 | 41 | 115 | 8 | 37 | 70 | 39.1% |
| Bottleneck | hardware | 296 | 118 | 97 | 33 | 44 | 20 | 79.4% |
| Crossing | simulation | 672 | 372 | 189 | 183 | 5 | 1 | 99.5% |
| Crossing | hardware | 255 | 67 | 32 | 32 | 0 | 0 | 100.0% |
| Head-on | simulation | 177 | 47 | 23 | 22 | 1 | 0 | 100.0% |
| Head-on | hardware | 123 | 23 | 8 | 8 | 0 | 0 | 100.0% |
| Shared Lane | simulation | 86 | 0 | 0 | 0 | 0 | 0 | N/A |
| Shared Lane | hardware | 98 | 0 | 0 | 0 | 0 | 0 | N/A |

### A4 — F2: Scheduling Latency

Matched itinerary-update latency averaged 5.13 ms in simulation and 5.69 ms on hardware, with no observations above the 2.85 s reference budget. Latency = t(query_update_1) − t(itinerary_set).

| Platform | Itineraries with a matched update | Mean (ms) | SD | Min | Max | Above 2.85 s |
|---|---|---|---|---|---|---|
| simulation | 1107 | 5.13 | 3.04 | 0.17 | 10.90 | 0 |
| hardware | 762 | 5.69 | 2.84 | 0.15 | 10.54 | 0 |

### A5 — F3: Negotiation Notices by Repeat

Negotiation activity was highly uneven across repeats: Bottleneck simulation R4 recorded 109 notices and Crossing simulation R1 recorded 165. Shared Lane recorded none. Notice totals equal the classified conflict-version counts in A3 for this dataset.

| Scenario | Platform | R1 | R2 | R3 | R4 | R5 | Median | Total |
|---|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | 1 | 2 | 1 | 109 | 2 | 2 | 115 |
| Bottleneck | hardware | 11 | 44 | 18 | 12 | 12 | 12 | 97 |
| Crossing | simulation | 165 | 1 | 20 | 1 | 2 | 2 | 189 |
| Crossing | hardware | 2 | 1 | 1 | 2 | 26 | 2 | 32 |
| Head-on | simulation | 4 | 3 | 8 | 5 | 3 | 4 | 23 |
| Head-on | hardware | 1 | 1 | 3 | 2 | 1 | 1 | 8 |
| Shared Lane | simulation | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| Shared Lane | hardware | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

### A6 — F3: Negotiation Outcomes and Latency

Most Crossing negotiations resolved quickly (mean 0.028 s in simulation; 0.123 s on hardware). Hardware Bottleneck recorded 64 search-failed outcomes, with a maximum latency of 24.454 s. Counts and extrema are conflict-level; mean and SD use unweighted means from repeats containing the outcome. Latency runs from the first notice to conclusion.

| Scenario | Platform | Outcome | Count | Mean (s) | SD | Min | Max |
|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | resolved | 8 | 0.031 | 0.014 | 0.007 | 0.058 |
| Bottleneck | simulation | refused | 107 | 0.001 | 0.000 | -0.000 | 0.005 |
| Bottleneck | hardware | resolved | 33 | 0.057 | 0.073 | 0.002 | 0.560 |
| Bottleneck | hardware | search failed | 64 | 6.777 | 5.174 | 0.056 | 24.454 |
| Crossing | simulation | resolved | 183 | 0.028 | 0.013 | 0.001 | 0.428 |
| Crossing | simulation | search failed | 6 | 2.001 | 0.000 | 2.001 | 2.002 |
| Crossing | hardware | resolved | 32 | 0.123 | 0.172 | 0.002 | 0.405 |
| Head-on | simulation | resolved | 22 | 0.146 | 0.132 | 0.002 | 1.476 |
| Head-on | simulation | search failed | 1 | 12.018 | 0.000 | 12.018 | 12.018 |
| Head-on | hardware | resolved | 8 | 0.186 | 0.074 | 0.003 | 0.300 |

### A7 — F4: Throughput, Scaling and Utilisation

All 15 N=3 tasks completed per scenario–platform condition. Throughput scaling ranged from 17.6% of ideal (Bottleneck simulation) to 71.2% (Shared Lane hardware). Scaling = Th(3) / [3 × Th(1)]; Th(1) omits inter-route travel, making reported scaling a lower bound. Workload ceiling = Σd / (3 × max d) using mean route durations; utilisation is moving time / observation-window time.

| Scenario | Platform | 4a completed | Th(1) /min | Th(3) /min | Scaling, % of ideal | Workload ceiling | U(1) routes alone | U(3) |
|---|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | 15/15 | 1.572 | 0.829 | 17.6% | 86.1% | 99.4% | 47.0% |
| Bottleneck | hardware | 15/15 | 1.059 | 0.820 | 25.8% | 81.1% | 99.1% | 45.7% |
| Crossing | simulation | 15/15 | 0.989 | 1.094 | 36.9% | 85.6% | 99.4% | 48.9% |
| Crossing | hardware | 15/15 | 0.787 | 0.840 | 35.5% | 74.1% | 96.1% | 58.8% |
| Head-on | simulation | 15/15 | 1.141 | 1.072 | 31.3% | 73.7% | 99.3% | 55.2% |
| Head-on | hardware | 15/15 | 0.868 | 1.120 | 43.0% | 84.8% | 97.8% | 68.5% |
| Shared Lane | simulation | 15/15 | 0.877 | 1.600 | 60.9% | 92.6% | 99.4% | 65.1% |
| Shared Lane | hardware | 15/15 | 0.551 | 1.177 | 71.2% | 95.9% | 96.9% | 69.3% |

### A8 — F5: Execution Fidelity

N=3 duration prediction error was largest in Bottleneck (336.1% simulation; 348.9% hardware), substantially above its N=1 baseline. Hardware N=3 fidelity statistics cover 10–11 matched tasks per condition; the remaining tasks lacked recorded bid responses. F5a is relative duration error; F5b is actual-minus-predicted finish-time offset.

| Scenario | Platform | 5a N=1 mean % | SD | Min | Max | 5a N=3 mean % | SD | Min | Max | 5b mean (s) | SD | Min | Max |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | 11.8 | 27.8 | -11.6 | 88.4 | 336.1 | 195.4 | 40.0 | 640.3 | 125.7 | 66.2 | 18.5 | 228.6 |
| Bottleneck | hardware | 66.7 | 37.8 | 11.0 | 131.5 | 348.9 | 233.9 | 87.7 | 796.9 | 123.2 | 78.8 | 34.8 | 273.9 |
| Crossing | simulation | 27.0 | 21.6 | 4.2 | 73.4 | 86.8 | 104.6 | 3.7 | 310.3 | 46.5 | 49.9 | 6.8 | 152.1 |
| Crossing | hardware | 61.9 | 76.0 | 14.0 | 313.0 | 196.8 | 155.3 | 31.3 | 517.7 | 99.6 | 74.4 | 20.3 | 253.4 |
| Head-on | simulation | 13.9 | 31.6 | -14.2 | 112.3 | 161.4 | 123.0 | -8.9 | 365.8 | 70.6 | 61.4 | -1.6 | 240.0 |
| Head-on | hardware | 76.7 | 32.7 | 19.1 | 125.3 | 216.1 | 138.2 | 26.2 | 374.3 | 86.9 | 45.9 | 18.2 | 136.1 |
| Shared Lane | simulation | 16.6 | 14.8 | -5.3 | 47.9 | 46.4 | 25.1 | 10.7 | 109.1 | 32.5 | 14.6 | 11.2 | 69.2 |
| Shared Lane | hardware | 86.5 | 42.7 | 19.2 | 191.9 | 113.8 | 46.3 | 61.3 | 210.7 | 73.8 | 29.0 | 40.7 | 135.3 |

### A9 — F6: Route-Level Stationary Time

Bottleneck had the largest additional stationary times on several routes; Bottleneck R1 simulation increased by 92.8 s and R3 hardware by 88.1 s. Two hardware routes showed decreases: Crossing R1 (−8.7 s) and Shared Lane R1 (−8.2 s). Δ stationary time = N=3 route mean − matched N=1 mean; it does not identify why a robot stopped.

| Route | Platform | N=1 mean | SD | Min | Max | N=3 mean | SD | Min | Max | Δ stationary time (s) |
|---|---|---|---|---|---|---|---|---|---|---|
| Bottleneck R1 | simulation | 0.0 | 0.0 | 0.0 | 0.0 | 92.8 | 56.3 | 5.1 | 146.3 | +92.8 |
| Bottleneck R1 | hardware | 0.3 | 0.7 | 0.0 | 1.5 | 17.9 | 27.3 | 5.5 | 66.7 | +17.6 |
| Bottleneck R2 | simulation | 0.1 | 0.2 | 0.0 | 0.4 | 76.2 | 69.7 | 0.0 | 133.1 | +76.1 |
| Bottleneck R2 | hardware | 0.1 | 0.3 | 0.0 | 0.7 | 25.8 | 21.7 | 3.8 | 50.1 | +25.7 |
| Bottleneck R3 | simulation | 0.0 | 0.0 | 0.0 | 0.0 | 8.8 | 5.4 | 0.0 | 14.9 | +8.8 |
| Bottleneck R3 | hardware | 0.3 | 0.4 | 0.0 | 0.8 | 88.4 | 68.3 | 31.1 | 206.6 | +88.1 |
| Crossing R1 | simulation | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | +0.0 |
| Crossing R1 | hardware | 10.1 | 21.9 | 0.0 | 49.2 | 1.4 | 3.1 | 0.0 | 6.8 | -8.7 |
| Crossing R2 | simulation | 0.0 | 0.0 | 0.0 | 0.0 | 6.0 | 8.2 | 0.9 | 20.5 | +6.0 |
| Crossing R2 | hardware | 0.0 | 0.0 | 0.0 | 0.0 | 5.5 | 0.1 | 5.3 | 5.5 | +5.5 |
| Crossing R3 | simulation | 0.4 | 0.3 | 0.0 | 0.8 | 42.8 | 36.1 | 0.7 | 99.0 | +42.4 |
| Crossing R3 | hardware | 0.0 | 0.0 | 0.0 | 0.0 | 29.7 | 25.2 | 0.4 | 62.5 | +29.7 |
| Head-on R1 | simulation | 0.2 | 0.5 | 0.0 | 1.2 | 33.3 | 30.6 | 0.0 | 70.9 | +33.0 |
| Head-on R1 | hardware | 0.4 | 0.4 | 0.0 | 1.0 | 27.6 | 20.7 | 3.0 | 51.4 | +27.2 |
| Head-on R2 | simulation | 0.0 | 0.0 | 0.0 | 0.0 | 0.1 | 0.1 | 0.0 | 0.3 | +0.1 |
| Head-on R2 | hardware | 0.1 | 0.3 | 0.0 | 0.6 | 1.2 | 2.1 | 0.0 | 4.8 | +1.0 |
| Head-on R3 | simulation | 0.1 | 0.1 | 0.0 | 0.3 | 37.6 | 61.6 | 0.9 | 147.3 | +37.6 |
| Head-on R3 | hardware | 3.1 | 6.3 | 0.0 | 14.5 | 16.8 | 33.6 | 0.0 | 76.7 | +13.7 |
| Shared Lane R1 | simulation | 0.0 | 0.0 | 0.0 | 0.0 | 0.1 | 0.3 | 0.0 | 0.6 | +0.1 |
| Shared Lane R1 | hardware | 8.2 | 18.1 | 0.0 | 40.5 | 0.0 | 0.0 | 0.0 | 0.0 | -8.2 |
| Shared Lane R2 | simulation | 0.2 | 0.2 | 0.0 | 0.4 | 22.3 | 5.3 | 16.1 | 30.3 | +22.1 |
| Shared Lane R2 | hardware | 0.1 | 0.2 | 0.0 | 0.5 | 27.4 | 10.1 | 16.8 | 43.6 | +27.3 |
| Shared Lane R3 | simulation | 0.0 | 0.0 | 0.0 | 0.0 | 20.4 | 4.2 | 16.5 | 25.4 | +20.4 |
| Shared Lane R3 | hardware | 0.3 | 0.4 | 0.0 | 0.8 | 30.8 | 0.6 | 30.1 | 31.7 | +30.5 |

### A10 — F6: Waiting Episode Statistics

Bottleneck produced the longest mean qualifying waiting episodes (33.22 s simulation at the 0.3 s floor). Episodes occurred in every repeat at both floors. Counts are pooled; duration mean and SD are calculated across the five per-repeat means, not weighted by episode count.

| Scenario | Platform | Episode floor | Episodes | Mean duration (s) | SD across repeats |
|---|---|---|---|---|---|
| Bottleneck | simulation | 0.3 s | 28 | 33.22 | 25.59 |
| Bottleneck | simulation | 1.0 s | 21 | 39.96 | 26.60 |
| Bottleneck | hardware | 0.3 s | 46 | 16.48 | 11.53 |
| Bottleneck | hardware | 1.0 s | 40 | 18.11 | 10.47 |
| Crossing | simulation | 0.3 s | 16 | 15.82 | 10.69 |
| Crossing | simulation | 1.0 s | 11 | 22.56 | 8.58 |
| Crossing | hardware | 0.3 s | 19 | 9.15 | 5.77 |
| Crossing | hardware | 1.0 s | 15 | 12.09 | 8.64 |
| Head-on | simulation | 0.3 s | 27 | 13.06 | 6.50 |
| Head-on | simulation | 1.0 s | 24 | 14.54 | 6.06 |
| Head-on | hardware | 0.3 s | 19 | 11.87 | 8.98 |
| Head-on | hardware | 1.0 s | 13 | 13.90 | 9.59 |
| Shared Lane | simulation | 0.3 s | 20 | 11.00 | 2.43 |
| Shared Lane | simulation | 1.0 s | 15 | 14.22 | 1.24 |
| Shared Lane | hardware | 0.3 s | 34 | 9.40 | 2.86 |
| Shared Lane | hardware | 1.0 s | 24 | 13.16 | 3.72 |

### A11 — F7: Pose-Based Separation and Events

Reported pose separation fell below the model-derived 0.076 m reference minimum in seven repeats, including four hardware repeats. These are RMF-reported reference-point separations, not verified physical clearances; waypoint snapping can produce a reported 0.000 m even when robots remain separated.

| Scenario | Platform | 7a min, 5 repeats | Median | Repeats below 0.076 m | Threshold | 7b events | Samples | Affected repeats |
|---|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | 0.187, 0.467, 1.100, 0.085, 0.458 | 0.458 | 0 | 0.356 m | 5 | 42 | 2/5 |
| Bottleneck | hardware | 0.131, 0.000, 0.000, 0.000, 0.408 | 0.000 | 3 | 0.356 m | 5 | 2700 | 4/5 |
| Crossing | simulation | 0.000, 0.000, 2.640, 0.666, 0.000 | 0.000 | 3 | 0.356 m | 3 | 199 | 3/5 |
| Crossing | hardware | 2.982, 0.000, 0.376, 0.925, 2.412 | 0.925 | 1 | 0.356 m | 1 | 817 | 1/5 |
| Head-on | simulation | 0.054, 0.353, 0.452, 0.176, 0.930 | 0.353 | 1 | 0.356 m | 4 | 86 | 3/5 |
| Head-on | hardware | 1.661, 1.647, 1.146, 0.918, 1.545 | 1.545 | 0 | 0.356 m | 0 | 0 | 0/5 |
| Shared Lane | simulation | 2.455, 1.934, 2.274, 2.001, 1.759 | 2.001 | 0 | 0.356 m | 0 | 0 | 0/5 |
| Shared Lane | hardware | 1.719, 1.693, 2.045, 1.896, 1.716 | 1.719 | 0 | 0.356 m | 0 | 0 | 0/5 |

### A12 — F7: Laser Separation and Its Validation

LiDAR-based separation coverage ranged from 23.3% to 78.0% across conditions. Threshold events were observed in several scenarios, but low coverage limits interpretation of unobserved periods; Contact suspected does not confirm physical collision. Minimum values are per-repeat lower bounds from /scan, pooled across three robot pairs.

| Scenario | Platform | 7c min, 5 repeats | Median | Coverage | Instants < 0.356 m | Episodes < 0.356 m | Pair-repeats | Contact suspected |
|---|---|---|---|---|---|---|---|---|
| Bottleneck | simulation | 0.149, 0.261, 0.779, 0.184, 0.149 | 0.184 | 34.4% | 136 | 8 | 15 | 2/5 |
| Bottleneck | hardware | 0.197, 0.189, 0.215, 0.181, 0.365 | 0.197 | 70.8% | 1307 | 13 | 15 | 0/5 |
| Crossing | simulation | 0.371, 0.293, 0.972, 0.358, 0.278 | 0.358 | 65.4% | 53 | 2 | 15 | 0/5 |
| Crossing | hardware | 0.533, 0.181, 0.358, 0.309, 0.361 | 0.358 | 78.0% | 532 | 12 | 15 | 0/5 |
| Head-on | simulation | 0.149, 0.151, 0.204, 0.171, 0.149 | 0.151 | 25.3% | 401 | 11 | 15 | 3/5 |
| Head-on | hardware | 0.824, 0.663, 0.185, 0.437, 0.755 | 0.663 | 61.9% | 13 | 1 | 15 | 0/5 |
| Shared Lane | simulation | 1.084, 0.462, 0.750, 0.307, 0.852 | 0.750 | 23.3% | 7 | 1 | 15 | 0/5 |
| Shared Lane | hardware | 0.547, 0.651, 0.219, 1.012, 0.185 | 0.547 | 63.8% | 31 | 3 | 15 | 0/5 |

Simulation validation: Of 1,441 ground-truth close instants (<0.628 m), 1,169 had an attributable laser match (81.1%). At the separate 0.356 m classification threshold, TP = 560, FP = 37 and FN = 141 (precision 93.8%; recall 79.9%). Unmatched true-close instants count as false negatives; these classifications are not confirmed collisions. The two thresholds use different denominators.

| Scenario | Close instants, true separation < 0.628 m | With an attributable match | TP | FP | FN |
|---|---|---|---|---|---|
| Bottleneck | 331 | 232 | 124 | 12 | 37 |
| Crossing | 310 | 308 | 47 | 6 | 0 |
| Head-on | 748 | 577 | 385 | 16 | 104 |
| Shared Lane | 52 | 52 | 4 | 3 | 0 |
| **All** | **1441** | **1169 (81.1%)** | **560** | **37** | **141** |

### A13 — F8: Planning Growth and Throughput Scaling

N=3 planning latency increased relative to N=1 in most comparisons; overall mean growth was +80.6% in simulation and +13.9% on hardware. The first bid of each N=3 batch showed higher growth (+142.1% and +70.9%, respectively). Scalability beyond N=3 was not tested.

| Platform | Set | P(3) mean (ms) | Median | P(1) mean (ms) | Median | G mean | G median |
|---|---|---|---|---|---|---|---|
| simulation | all bids | 2.08 | 1.81 | 1.15 | 1.10 | +80.6% | +64.6% |
| simulation | first bid per batch | 2.79 | 2.47 | 1.15 | 1.10 | +142.1% | +124.9% |
| hardware | all bids | 1.61 | 1.58 | 1.42 | 1.25 | +13.9% | +26.9% |
| hardware | first bid per batch | 2.42 | 1.73 | 1.42 | 1.25 | +70.9% | +38.5% |

| Scenario | Platform | P(3) mean (ms) | P(1) mean | G mean | Throughput scaling | Workload ceiling |
|---|---|---|---|---|---|---|
| Bottleneck | simulation | 1.68 | 1.07 | +57.4% | 17.6% | 86.1% |
| Crossing | simulation | 2.20 | 1.03 | +114.5% | 36.9% | 85.6% |
| Head-on | simulation | 1.36 | 1.13 | +21.1% | 31.3% | 73.7% |
| Shared Lane | simulation | 3.08 | 1.39 | +121.9% | 60.9% | 92.6% |
| Bottleneck | hardware | 1.31 | 1.02 | +28.9% | 25.8% | 81.1% |
| Crossing | hardware | 1.47 | 1.23 | +20.0% | 35.5% | 74.1% |
| Head-on | hardware | 1.28 | 1.61 | -20.6% | 43.0% | 84.8% |
| Shared Lane | hardware | 2.37 | 1.80 | +31.6% | 71.2% | 95.9% |

### A14 — Cross-Feature Route-Level Comparison

Bottleneck combined large execution-time increases, stationary delays and duration-prediction errors. Shared Lane had no classified conflicts despite measurable waiting on some routes. Values are N=3 route means; Δ duration error is N=3 minus matched N=1 in percentage points. Hardware Crossing R1 has no valid F5 comparison (—). Conflicts per repeat are scenario-level, repeated across routes.

| Route | Platform | Execution (s) | Stationary (s) | Distance (m) | Δ duration error (pp) | Conflicts per repeat |
|---|---|---|---|---|---|---|
| Bottleneck R1 | simulation | 202.7 | 92.8 | 16.0 | +423 | 23.0 |
| Bottleneck R1 | hardware | 124.6 | 17.9 | 14.0 | +254 | 19.4 |
| Bottleneck R2 | simulation | 146.8 | 76.2 | 10.5 | +292 | 23.0 |
| Bottleneck R2 | hardware | 96.7 | 25.8 | 8.0 | +153 | 19.4 |
| Bottleneck R3 | simulation | 124.4 | 8.8 | 18.1 | +258 | 23.0 |
| Bottleneck R3 | hardware | 208.7 | 88.4 | 15.0 | +411 | 19.4 |
| Crossing R1 | simulation | 51.1 | 0.0 | 12.9 | +0 | 37.8 |
| Crossing R1 | hardware | 88.7 | 1.4 | 16.0 | — | 6.4 |
| Crossing R2 | simulation | 63.1 | 6.0 | 9.2 | +5 | 37.8 |
| Crossing R2 | hardware | 81.2 | 5.5 | 10.2 | +31 | 6.4 |
| Crossing R3 | simulation | 154.1 | 42.8 | 17.7 | +174 | 37.8 |
| Crossing R3 | hardware | 203.6 | 29.7 | 20.6 | +292 | 6.4 |
| Head-on R1 | simulation | 101.8 | 33.3 | 10.0 | +163 | 4.6 |
| Head-on R1 | hardware | 137.7 | 27.6 | 12.4 | +269 | 1.6 |
| Head-on R2 | simulation | 112.8 | 0.1 | 17.5 | +205 | 4.6 |
| Head-on R2 | hardware | 143.7 | 1.2 | 17.7 | +227 | 1.6 |
| Head-on R3 | simulation | 127.8 | 37.6 | 14.8 | +74 | 4.6 |
| Head-on R3 | hardware | 96.6 | 16.8 | 11.2 | +26 | 1.6 |
| Shared Lane R1 | simulation | 78.1 | 0.1 | 12.6 | +19 | 0.0 |
| Shared Lane R1 | hardware | 112.7 | 0.0 | 13.9 | -35 | 0.0 |
| Shared Lane R2 | simulation | 80.8 | 22.3 | 10.4 | +24 | 0.0 |
| Shared Lane R2 | hardware | 142.2 | 27.4 | 14.1 | +42 | 0.0 |
| Shared Lane R3 | simulation | 102.0 | 20.4 | 11.4 | +46 | 0.0 |
| Shared Lane R3 | hardware | 122.2 | 30.8 | 11.9 | +33 | 0.0 |
