# Benchmark Results - Open-RMF Multi-Robot Coordination

## 1. Evaluation Dataset

The evaluation covers four scenarios (Bottleneck, Crossing, Head-on and Shared Lane) on simulation and hardware. Each platform includes five N=3 repeats per scenario (three robot-tasks per repeat) and five N=1 baseline repeats for each of 12 matched routes. The retained dataset comprises 160 repeats. Results are descriptive; individual robot-tasks within a repeat are not independent experimental repeats.

The tables below report the principal results. Detailed per-route statistics, standard deviations, ranges and event distributions are provided in [APPENDIX.md](APPENDIX.md). Metric definitions and observation sources are specified in [METRIC_DESIGN.md](METRIC_DESIGN.md).

## 2. F1 - Task Planning and Execution

All 60 N=3 robot-tasks on each platform were dispatched and reported as completed by RMF in the retained dataset. Mean bid latency ranged from 1.21 to 3.63 ms across scenario-platform conditions; pooled means were 3.10 ms in simulation and 2.29 ms on hardware.

Execution duration relative to the matched N=1 baseline

| Scenario | Simulation: N=1 → N=3 (s) | Added (%) | Hardware: N=1 → N=3 (s) | Added (%) |
|---|---|---|---|---|
| Bottleneck | 49.5 → 144.1 | +191.3 | 79.8 → 198.2 | +148.3 |
| Crossing | 68.7 → 122.2 | +77.9 | 107.7 → 163.8 | +52.0 |
| Head-on | 62.9 → 139.2 | +121.2 | 92.1 → 142.5 | +54.8 |
| Shared Lane | 72.5 → 95.9 | +32.2 | 123.9 → 159.6 | +28.7 |

Execution duration increased on all 24 route-platform comparisons. Bottleneck showed the greatest mean increase, while Shared Lane showed the smallest on both platforms. The 100% completion figure applies only to retained repeats, not all collection attempts.

## 3. F2 - Traffic Scheduling

| Scenario | Platform | Conflict-free itineraries (%) | Resolved / Replanned / Unrecovered conflicts | Schedule feasibility (%) |
|---|---|---|---|---|
| Bottleneck | Simulation | 68.5 | 42 / 57 / 2 | 98.0 |
| Bottleneck | Hardware | 53.7 | 81 / 29 / 1 | 99.1 |
| Crossing | Simulation | 66.2 | 12 / 386 / 15 | 96.4 |
| Crossing | Hardware | 70.4 | 32 / 5 / 0 | 100.0 |
| Head-on | Simulation | 64.4 | 38 / 0 / 0 | 100.0 |
| Head-on | Hardware | 71.2 | 33 / 0 / 0 | 100.0 |
| Shared Lane | Simulation | 100.0 | 0 / 0 / 0 | N/A |
| Shared Lane | Hardware | 98.7 | 1 / 0 / 0 | 100.0 |

Conflict-free rate is itinerary-based; schedule feasibility is conflict-based. Shared Lane recorded only one conflict across both platforms, limiting its feasibility assessment. Matched scheduling-update latency averaged 5.86 ms in simulation and 5.66 ms on hardware; the largest observed latencies were 23.66 ms and 11.46 ms, respectively.

## 4. F3 - Traffic Negotiation

| Scenario | Platform | Median notices / repeat | Resolved | Refused | Search failed |
|---|---|---|---|---|---|
| Bottleneck | Simulation | 15 | 42 | 51 | 8 |
| Bottleneck | Hardware | 21 | 81 | 0 | 30 |
| Crossing | Simulation | 5 | 12 | 401 | 0 |
| Crossing | Hardware | 7 | 32 | 0 | 5 |
| Head-on | Simulation | 7 | 38 | 0 | 0 |
| Head-on | Hardware | 5 | 33 | 0 | 0 |
| Shared Lane | Simulation | 0 | 0 | 0 | 0 |
| Shared Lane | Hardware | 0 | 1 | 0 | 0 |

Mean resolution latency for resolved negotiations was 0.126 s in simulation and 0.094 s on hardware. All resolved negotiations finished within the 15 s timeout. Simulation Crossing contained 401 refused negotiations concentrated in two repeats. Negotiation notices and distinct conflicts may differ because one conflict can generate repeated notices.

## 5. F4 - Fleet Throughput and Utilisation

| Scenario | Platform | Batch Throughput N=3 (tasks/min) | Scaling vs ideal 3× (%) | Workload ceiling (%) | Utilisation N=3 (%) |
|---|---|---|---|---|---|
| Bottleneck | Simulation | 0.973 | 26.8 | 89.4 | 58.3 |
| Bottleneck | Hardware | 0.696 | 30.9 | 74.5 | 59.5 |
| Crossing | Simulation | 0.892 | 34.0 | 77.1 | 57.2 |
| Crossing | Hardware | 0.757 | 45.3 | 95.8 | 61.6 |
| Head-on | Simulation | 1.008 | 35.2 | 82.0 | 63.7 |
| Head-on | Hardware | 0.979 | 50.1 | 93.1 | 74.3 |
| Shared Lane | Simulation | 1.474 | 59.4 | 83.4 | 72.6 |
| Shared Lane | Hardware | 0.910 | 62.7 | 88.6 | 73.1 |

Throughput scaling ranged from 26.8% to 62.7% of ideal threefold scaling. Both platforms ranked the scenarios Bottleneck, Crossing, Head-on and Shared Lane from lowest to highest scaling. Utilisation reflects commanded motion, not independently measured physical motion on hardware.

## 6. F5 - Execution Fidelity

Mean relative duration error (%)

| Scenario | Simulation N=1 | Simulation N=3 | Hardware N=1 | Hardware N=3 |
|---|---|---|---|---|
| Bottleneck | +31.2 | +289.4 | +110.3 | +430.5 |
| Crossing | +31.0 | +140.2 | +114.8 | +232.3 |
| Head-on | +26.1 | +215.9 | +113.9 | +244.3 |
| Shared Lane | +15.8 | +53.3 | +98.8 | +157.1 |

Duration underestimation was present at N=1 and increased at N=3 for every matched route. The hardware N=1 scenario means (+98.8% to +114.8%) exceeded simulation N=1 means (+15.8% to +31.2%). Finish-time offset, including dispatcher queueing, is reported separately in the appendix.

## 7. F6 - Stationary Time

Additional stationary time, N=3 minus matched N=1 mean (s / robot-task)

| Scenario | Simulation | Hardware |
|---|---|---|
| Bottleneck | +51.4 | +60.7 |
| Crossing | +29.3 | +24.9 |
| Head-on | +36.4 | +14.4 |
| Shared Lane | +13.4 | +19.0 |

Bottleneck produced the largest additional stationary time on both platforms. Stationary time was determined from cmd_vel and cannot be attributed exclusively to traffic negotiation. Episode-duration statistics and route-level values are provided in the appendix.

## 8. F7 - Inter-Robot Separation

F7a/F7b use distances between RMF-reported reference poses (RobotState.location); they do not directly measure physical clearance. F7c estimates laser-centre separation from /scan and was validated against Gazebo ground truth in simulation only.

| Scenario | Platform | F7a minimum range across repeats (m) | Pose-Based Threshold Events (below 0.356 m) | F7c minimum range (m) |
|---|---|---|---|---|
| Bottleneck | Simulation | 0.071–1.642 | 3 | 0.183–0.652 |
| Bottleneck | Hardware | 0.011–0.496 | 9 | 0.181–0.563 |
| Crossing | Simulation | 0.115–2.908 | 6 | 0.149–0.651 |
| Crossing | Hardware | 0.006–2.329 | 6 | 0.183–0.635 |
| Head-on | Simulation | 0.075–1.843 | 3 | 0.297–1.014 |
| Head-on | Hardware | 0.849–1.460 | 0 | 0.521–0.856 |
| Shared Lane | Simulation | 1.838–2.851 | 0 | 0.700–1.126 |
| Shared Lane | Hardware | 0.960–2.945 | 0 | 0.211–0.891 |

Simulation validation of F7c at the 0.356 m threshold yielded 1,005 true positives, 29 false positives and 89 false negatives (97.2% precision; 91.9% recall at the instant level), over the 2,464 instants whose true separation was below 0.628 m, of which 94.3% had an attributable match. Hardware has no independent physical position reference; hardware separation estimates remain unvalidated. Some pose-based separations are below the minimum geometrically possible reference-point distance of 0.076 m.

## 9. F8 - Fleet-Size Comparison

| Platform | N=1 mean planning latency (ms) | N=3 mean (ms) | Growth (%) | N=3 throughput scaling range (% of ideal) |
|---|---|---|---|---|
| Simulation | 1.48 | 3.10 | +109.1 | 26.8–59.4 |
| Hardware | 1.18 | 2.29 | +94.6 | 30.9–62.7 |

The evaluation includes only N=1 and N=3. These results quantify the observed change between two fleet sizes; they do not establish a scalability curve or breaking point.

## 10. Cross Feature Findings

- Bottleneck showed the greatest scenario-level execution overhead and additional stationary time on both platforms.
- Execution overhead varied considerably among routes within the same scenario. For example, additional time on Head-on R2 was +102.4 s in simulation and +93.5 s on hardware, with approximately 12 m of additional RMF-reported travel on both.
- Shared Lane produced additional stationary time despite almost no recorded traffic conflicts, indicating that conflict counts alone do not capture stationary behaviour.
- Simulation and hardware differed in duration-prediction error, negotiation outcomes and scenario-level stationary time. The causes of these differences were not isolated.

## 11. Interpretation Scope

Results describe retained repeats with fixed initial placement and one task per robot. Five repeats were collected per condition; no statistical significance testing was performed. N=1 and N=3 are the only evaluated fleet sizes. Robot-task observations within one N=3 repeat are not statistically independent. Hardware has no external position ground truth. Performance differences are descriptive and do not establish causal attribution.

Detailed statistics are retained in [APPENDIX.md](APPENDIX.md); hardware and simulation configuration is described in [SYSTEM_UNDER_TEST.md](SYSTEM_UNDER_TEST.md), and scenario geometry in [SCENARIO_DESIGN.md](SCENARIO_DESIGN.md).
