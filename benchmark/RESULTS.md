# Benchmark Results — Open-RMF Multi-Robot Coordination

## 1. Evaluation Dataset

The retained dataset includes 160 repeats and 240 robot-tasks across simulation and hardware: five repeats for each of four N=3 scenarios and 12 matched N=1 routes per platform. Results are descriptive; robot-tasks within an N=3 repeat are not independent observations. Detailed statistics are in [APPENDIX.md](APPENDIX.md); definitions are in [METRIC_DESIGN.md](METRIC_DESIGN.md).

## 2. F1 — Task Planning and Execution

All 120 robot-tasks per platform were dispatched and reported completed by RMF in retained repeats. Mean bid latency was 2.08 ms in simulation (60 matched bids) and 1.61 ms on hardware (43 of 60 matched bids) for N=3.

Execution duration relative to the matched N=1 baseline

| Scenario | Simulation: N=1 → N=3 (s) | Added (%) | Hardware: N=1 → N=3 (s) | Added (%) |
|---|---|---|---|---|
| Bottleneck | 38.2 → 158.0 | +313.9 | 56.6 → 143.3 | +153.1 |
| Crossing | 60.6 → 89.4 | +47.5 | 76.2 → 124.5 | +63.4 |
| Head-on | 52.6 → 114.1 | +117.1 | 69.1 → 126.0 | +82.3 |
| Shared Lane | 68.4 → 87.0 | +27.1 | 108.9 → 125.7 | +15.4 |

Execution time increased in all eight scenario–platform means and 22 of 24 route comparisons. Bottleneck had the largest increase; Shared Lane had the smallest on both platforms. The two decreases were hardware Crossing R1 (−14.1 s) and Shared Lane R1 (−0.9 s). Completion figures exclude discarded attempts.

## 3. F2 — Traffic Scheduling

| Scenario | Platform | Conflict-free itineraries (%) | Resolved / Replanned / Unrecovered conflicts | Schedule feasibility (%) |
|---|---|---|---|---|
| Bottleneck | Simulation | 77.0 | 8 / 37 / 70 | 39.1 |
| Bottleneck | Hardware | 60.1 | 33 / 44 / 20 | 79.4 |
| Crossing | Simulation | 44.6 | 183 / 5 / 1 | 99.5 |
| Crossing | Hardware | 73.7 | 32 / 0 / 0 | 100.0 |
| Head-on | Simulation | 73.4 | 22 / 1 / 0 | 100.0 |
| Head-on | Hardware | 81.3 | 8 / 0 / 0 | 100.0 |
| Shared Lane | Simulation | 100.0 | 0 / 0 / 0 | N/A |
| Shared Lane | Hardware | 100.0 | 0 / 0 / 0 | N/A |

Bottleneck had the most unrecovered conflicts: 70/115 in simulation and 20/97 on hardware; simulation Crossing had 1/189. Shared Lane had no classified conflicts (feasibility: N/A). Mean scheduling-update latency was 5.13 ms in simulation and 5.69 ms on hardware. Conflict-free rate uses itineraries; feasibility uses classified conflicts.

## 4. F3 — Traffic Negotiation

| Scenario | Platform | Median notices / repeat | Resolved | Refused | Search failed |
|---|---|---|---|---|---|
| Bottleneck | Simulation | 2 | 8 | 107 | 0 |
| Bottleneck | Hardware | 12 | 33 | 0 | 64 |
| Crossing | Simulation | 2 | 183 | 0 | 6 |
| Crossing | Hardware | 2 | 32 | 0 | 0 |
| Head-on | Simulation | 4 | 22 | 0 | 1 |
| Head-on | Hardware | 1 | 8 | 0 | 0 |
| Shared Lane | Simulation | 0 | 0 | 0 | 0 |
| Shared Lane | Hardware | 0 | 0 | 0 | 0 |

Resolved negotiations averaged 0.052 s in simulation (n=213) and 0.062 s on hardware (n=73); every resolved negotiation concluded within the 15 s timeout. Bottleneck differed notably: 107 refusals in simulation versus 64 search failures on hardware.

## 5. F4 — Fleet Throughput and Utilisation

| Scenario | Platform | Batch Throughput N=3 (tasks/min) | Scaling vs ideal 3× (%) | Workload ceiling (%) | Utilisation N=3 (%) |
|---|---|---|---|---|---|
| Bottleneck | Simulation | 0.829 | 17.6 | 86.1 | 47.0 |
| Bottleneck | Hardware | 0.820 | 25.8 | 81.1 | 45.7 |
| Crossing | Simulation | 1.094 | 36.9 | 85.6 | 48.9 |
| Crossing | Hardware | 0.840 | 35.5 | 74.1 | 58.8 |
| Head-on | Simulation | 1.072 | 31.3 | 73.7 | 55.2 |
| Head-on | Hardware | 1.120 | 43.0 | 84.8 | 68.5 |
| Shared Lane | Simulation | 1.600 | 60.9 | 92.6 | 65.1 |
| Shared Lane | Hardware | 1.177 | 71.2 | 95.9 | 69.3 |

Scaling reached 60.9% (simulation) and 71.2% (hardware) in Shared Lane, versus 17.6% and 25.8% in Bottleneck. Utilisation measures commanded motion, not independently verified physical motion.

## 6. F5 — Execution Fidelity

Mean relative duration error (%)

| Scenario | Simulation N=1 | Simulation N=3 | Hardware N=1 | Hardware N=3 |
|---|---|---|---|---|
| Bottleneck | +11.8 | +336.1 | +66.7 | +348.9 |
| Crossing | +27.0 | +86.8 | +61.9 | +196.8 |
| Head-on | +13.9 | +161.4 | +76.7 | +216.1 |
| Shared Lane | +16.6 | +46.4 | +86.5 | +113.8 |

Mean duration underestimation increased from N=1 to N=3 in every scenario. Bottleneck had the largest N=3 errors (+336.1% simulation; +348.9% hardware). Hardware N=1 errors exceeded simulation across all scenarios. N=3 estimates use 15 tasks per simulation condition and 10–11 with matched bids per hardware condition; finish-time offsets are in the appendix.

## 7. F6 — Stationary Time

Additional stationary time, N=3 minus matched N=1 mean (s / robot-task)

| Scenario | Simulation | Hardware |
|---|---|---|
| Bottleneck | +59.3 | +43.8 |
| Crossing | +16.1 | +8.8 |
| Head-on | +23.6 | +14.0 |
| Shared Lane | +14.2 | +16.6 |

Bottleneck had the largest added stationary time (+59.3 s simulation; +43.8 s hardware). The smallest increases occurred in Shared Lane (simulation) and Crossing (hardware). These command-based stops are not exclusively attributable to traffic negotiation.

## 8. F7 — Inter-Robot Separation

F7a/F7b measure RMF-reported pose separation, not physical clearance. F7c estimates laser-centre separation from /scan; independent validation is available only in simulation.

| Scenario | Platform | F7a minimum range across repeats (m) | Pose-Based Threshold Events (below 0.356 m) | F7c minimum range (m) |
|---|---|---|---|---|
| Bottleneck | Simulation | 0.085–1.100 | 5 | 0.149–0.779 |
| Bottleneck | Hardware | 0.000–0.408 | 5 | 0.181–0.365 |
| Crossing | Simulation | 0.000–2.640 | 3 | 0.278–0.972 |
| Crossing | Hardware | 0.000–2.982 | 1 | 0.181–0.533 |
| Head-on | Simulation | 0.054–0.930 | 4 | 0.149–0.204 |
| Head-on | Hardware | 0.918–1.661 | 0 | 0.185–0.824 |
| Shared Lane | Simulation | 1.759–2.455 | 0 | 0.307–1.084 |
| Shared Lane | Hardware | 1.693–2.045 | 0 | 0.185–1.012 |

At the 0.356 m threshold, simulation F7c validation achieved 93.8% precision (560 TP, 37 FP) and 79.9% recall (141 FN). Attribution coverage was 81.1% (1,169/1,441) at the wider 0.628 m gate. Hardware estimates lack independent position-ground-truth validation.

Waypoint-coordinate merging produced 0.000 m reported separation in three simulation Crossing and four hardware repeats, despite a 0.387 m Gazebo ground-truth separation for the cited simulation instant. Such reported pose minima must not be interpreted as confirmed physical contact.

## 9. F8 — Fleet-Size Comparison

| Platform | N=1 mean planning latency (ms) | N=3 mean (ms) | Growth (%) | N=3 throughput scaling range (% of ideal) |
|---|---|---|---|---|
| Simulation | 1.15 | 2.08 | +80.6 | 17.6–60.9 |
| Hardware | 1.42 | 1.61 | +13.9 | 25.8–71.2 |

Planning latency rose 80.6% in simulation and 13.9% on hardware from N=1 to N=3. Hardware had higher throughput scaling in three scenarios; Crossing was the exception (35.5% hardware versus 36.9% simulation). Only N=1 and N=3 were evaluated; no scaling curve is inferred.

## 10. Cross Feature Findings

- Bottleneck had the greatest execution overhead, added stationary time and feasibility shortfall on both platforms.
- Crossing varied substantially by route: execution overhead ranged from +0.7 to +83.3 s in simulation and −14.1 to +142.1 s on hardware.
- Shared Lane had no classified conflicts but still showed added stationary time (+14.2 s simulation; +16.6 s hardware).
- Simulation and hardware differed in prediction errors, negotiation outcomes and planning growth; causes were not isolated.

## 11. Interpretation Scope

Results cover retained repeats only (five per condition), with fixed initial placement and one task per robot. max_merge_waypoint_distance was 0.25 m on both platforms. No significance testing or causal attribution is claimed; N=3 robot-tasks are dependent, and hardware has no independent position ground truth. See [SYSTEM_UNDER_TEST.md](SYSTEM_UNDER_TEST.md), [SCENARIO_DESIGN.md](SCENARIO_DESIGN.md) and [APPENDIX.md](APPENDIX.md).
