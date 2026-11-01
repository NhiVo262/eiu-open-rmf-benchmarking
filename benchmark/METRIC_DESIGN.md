# Metric Specification - F1 - F8

This document defines the benchmark metrics, calculation rules, and measurement sources. Experimental conditions are documented in [SYSTEM_UNDER_TEST.md](SYSTEM_UNDER_TEST.md); results and detailed statistics are in [RESULTS.md](RESULTS.md) and [APPENDIX.md](APPENDIX.md).

## 1. Measurement Conventions

- Platforms: Simulation and hardware; fleet sizes: N=1 and N=3.

- Aggregation: Five repeats per scenario or matched baseline route on each platform. Robot-tasks from one N=3 repeat are not independent repeats.

- Task timestamps: t_submit from /task_api_requests; t_award from /rmf_task/dispatch_request; t_completed from the rmf_websocket task state (completed).

- Planning and traffic latencies: Differences in recorded message receive timestamps unless specified otherwise.

- Incomplete observations: Missing terminal states or missing required signals are not assigned zero duration; valid observation counts accompany aggregate statistics.

- Statistics: Mean, standard deviation, minimum and maximum for durations; event counts and medians where specified. The observation unit is defined for each metric.

Reference parameters

| Parameter | Value |
|---|---|
| RMF footprint radius | 0.178 m |
| RMF vicinity radius | 0.450 m |
| RMF conflict separation | 0.628 m |
| Maximum linear speed | 0.220 m/s |
| RMF angular speed model | 1.82 rad/s |
| Nav2 angular speed limit | 1.00 rad/s |
| Negotiation timeout | 15 s |
| Bid window: N=3 / N=1 | 5 s / 2 s |
| Separation thresholds | 0.356 m / 0.628 m |

## 2. F1 - Task Planning and Execution

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F1a — Dispatch success | `100 × N(first dispatch_ack success=true) / N(submitted tasks)` | Task | /rmf_task/dispatch_ack, task submissions |
| F1b — Bid latency | `t(bid_response) − t(bid_notice)` | Bid | /rmf_task/bid_notice, /rmf_task/bid_response |
| F1c — Batch makespan | `max(t_completed) − min(t_submit)` | N=3 repeat | Task submission and terminal-state timestamps |

F1c components:

| Quantity | Definition | Unit |
|---|---|---|
| Queueing time | `t_award − t_submit` | Task |
| Execution duration | `t_completed − t_award` | Task |
| End-to-end task duration | `t_completed − t_submit` | Task |
| Matched execution overhead | `D_exec(N=3) − D_exec(N=1)`; also expressed relative to `D_exec(N=1)` | Route, platform |

F1a is reported for retained tasks. F1c baseline comparisons use matched-route execution durations, excluding dispatcher queueing. The F1a reference target is ≥ 95% dispatch success; no fixed bid-latency threshold is specified.

## 3. F2 - Traffic Scheduling

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F2a-i — Conflict-free itinerary rate | `100 × (N_itineraries − N_triggering_negotiation) / N_itineraries` | Itinerary | /rmf_traffic/itinerary_set, /rmf_traffic/negotiation_notice |
| F2a-ii — Schedule feasibility | `100 × (N_resolved + N_recovered) / N_conflicts` | Distinct conflict | Negotiation outcomes, itinerary updates |
| F2b — Schedule update latency | `t(first matching query_update_1) − t(itinerary_set)` | Matched itinerary | /rmf_traffic/itinerary_set, schedule mirror updates |

A resolved conflict has negotiation_conclusion.resolved=true. A recovered conflict has resolved=false followed by a participant's new itinerary within 5.0 s. Remaining concluded conflicts are unrecovered. Distinct conflicts are keyed by conflict_version. The feasibility denominator is the number of classified conflicts; where this is zero, feasibility is N/A.

F2b matches participant and itinerary version. Its reference budget is `0.628 / 0.220 = 2.85 s`. It measures schedule propagation, not bid computation.

## 4. F3 - Traffic Negotiation

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F3a — Negotiation notices | count(negotiation_notice) | Repeat | /rmf_traffic/negotiation_notice |
| F3b — Negotiation latency | `t(negotiation_conclusion) − t(first notice for conflict_version)` | Concluded conflict | Negotiation notices and conclusions |

F3a reports per-repeat counts and their median. F3b separates resolved, refused (resolved=false with matching refusal), and search-failed (resolved=false without matching refusal) outcomes. The resolution-time mean is calculated over resolved negotiations only; the configured negotiation timeout is 15 s.

## 5. F4 - Fleet Throughput and Utilisation

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F4a — Task completion | `100 × N(tasks with completed state) / N(submitted tasks)` | Task | rmf_websocket task state |
| Batch throughput | `N_completed / Σ T_batch`, in tasks/min | Scenario across repeats | Task award and completion timestamps |
| Throughput scaling | `100 × Th(3) / (3 × Th(1))` | Scenario, platform | N=3 batches and matched N=1 routes |
| Workload ceiling | `100 × Σ d_r / (3 × max_r d_r)`, where `d_r` is mean N=1 route execution duration | Scenario, platform | Matched N=1 routes |
| F4b — Robot utilisation | `100 × Σ T_moving / Σ T_window` | Robot-window, pooled across repeats | Per-robot /cmd_vel |

For N=3, `T_batch = max(t_completed) − min(t_award)` within each repeat. For N=1, throughput is computed from the aggregate execution durations of the same three routes run separately. F4b uses the common per-repeat window `[min(t_award), max(t_completed)]` for all three robots.

A robot is commanded to move when its most recent /cmd_vel sample is no older than 0.25 s and `|v| ≥ 0.005 m/s` or `|ω| ≥ 0.05 rad/s`. Hardware utilisation is command-based, not a direct measurement of physical motion.

## 6. F5 - Execution Fidelity

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F5a — Relative duration error | `100 × (D_actual − D_planned) / D_planned` | Task | Winning bid and task completion |
| F5b — Finish-time offset | `t_completed − t_estimated_finish` | Task | Winning bid and task completion |

`D_planned = t_estimated_finish − t_bid_response`; `D_actual = t_completed − t_award`. The winning bid_response.proposal.finish_time provides the planned finish. Simulation timestamps are aligned using /clock; hardware uses wall time. F5a is evaluated for N=1 and N=3 on matched routes. F5b includes dispatch queueing.

## 7. F6 - Stationary Time

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F6a — Stationary duration | Sum of stationary-episode durations within [t_award, t_completed] | Robot-task | Per-robot /cmd_vel |
| F6b — Stationary episode duration | Mean duration of qualifying stationary episodes | Episode | Per-robot /cmd_vel |
| Additional stationary time | `F6a(N=3) − F6a(N=1)` on the same route | Route, platform | N=3 and matched N=1 recordings |

Stationary episodes contain no qualifying motion command for at least 0.3 s; episode statistics are also reported with a 1.0 s minimum. Motion thresholds and command freshness follow F4b. Stationary duration is not restricted to a map region and is not exclusively attributed to traffic coordination.

## 8. F7 - Inter-Robot Separation

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F7a — Minimum reported separation | `min_(t,i < j) ‖p_i(t) − p_j(t)‖` | Repeat | /fleet_states, RobotState.location |
| F7b — Separation threshold events | Number of contiguous below-threshold frame sequences per robot pair | Pair-event | /fleet_states, RobotState.location |
| F7c — LiDAR-derived separation | Matched mutual /scan range plus LDS head geometry offset | Robot pair, repeat | Per-robot /scan |

F7a and F7b use RMF-reported reference-point poses, not physical ground truth. F7b reports event counts and below-threshold sample counts at 0.356 m and 0.628 m. One event is a contiguous sequence of threshold violations for the same pair.

F7c is a supplementary measurement of laser-centre separation. A candidate mutual match requires a 0.02–0.12 m scan cluster on each robot and range disagreement ≤ 0.045 m. The reported lower-bound estimate is `D = min(r_1,r_2) + 0.029 m`, with a geometry interval extending to +0.034 m. Measurements at the 0.120 m sensor range floor are censored. F7c uses 0.356 m for threshold reporting and was validated against Gazebo ground truth in simulation; hardware has no independent physical position reference.

## 9. F8 - Fleet-Size Comparison

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F8a — Planning-latency growth | `100 × (P(3) − P(1)) / P(1)`, where `P(N)` is mean F1b bid latency | Bid, pooled by fleet size | F1b |
| F8b — Task throughput | Th(N) and throughput scaling from F4 | Scenario, platform | F4 throughput outputs |

F8a is reported for all N=3 bids and separately for the first bid of each N=3 repeat. The average change per added robot between N=1 and N=3 is `F8a / 2`; it is not a fitted scaling rate. Only two fleet sizes were evaluated, so no scalability curve or breaking point is estimated.

## 10. Measurement Scope

- Results are descriptive and based on retained repeats only.

- F2 and F3 represent RMF traffic and negotiation records, not independent physical collision observations.

- F4b and F6 are based on commanded velocities.

- F7a/F7b use reported robot poses; only simulation has independent position ground truth. Hardware F7c is unvalidated against an external position reference.

- The N=1 baseline contains no multi-robot conflicts or negotiations. F8 compares N=1 with N=3 only.
