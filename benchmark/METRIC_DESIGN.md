# Metric Specification — F1–F8

Definitions, calculation rules, and data sources for the Open-RMF benchmark. See [SYSTEM_UNDER_TEST.md](SYSTEM_UNDER_TEST.md) for experimental conditions and [RESULTS.md](RESULTS.md) / [APPENDIX.md](APPENDIX.md) for results.

## 1. Measurement Conventions

- Platforms and fleet sizes: Simulation and hardware; N=1 and N=3.
- Repeats: Five per scenario or matched baseline route on each platform. Robot-tasks within an N=3 repeat are dependent observations.
- Timestamps: t_submit from /task_api_requests; t_award from /rmf_task/dispatch_request; t_completed from the RMF WebSocket task state completed. Planning and traffic latencies use recorded message receive timestamps unless noted.
- Missing data: Missing terminal states or required signals are not treated as zero duration. Aggregates report valid observation counts.
- Statistics: Mean, standard deviation (SD), minimum and maximum for durations; counts and medians where specified. Observation units are listed per metric.

Reference Parameters

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

## 2. F1 — Task Planning and Execution

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F1a — Dispatch success | 100 × N(first dispatch_ack success=true) / N(submitted tasks) | Task | /rmf_task/dispatch_ack, task submissions |
| F1b — Bid latency | t(bid_response) − t(bid_notice) | Bid | /rmf_task/bid_notice, /rmf_task/bid_response |
| F1c — Batch makespan | max(t_completed) − min(t_submit) | N=3 repeat | Task submission and terminal-state timestamps |

| F1c component | Definition | Observation unit |
|---|---|---|
| Queueing time | t_award − t_submit | Task |
| Execution duration | t_completed − t_award | Task |
| End-to-end task duration | t_completed − t_submit | Task |
| Matched execution overhead | D_exec(N=3) − D_exec(N=1); also relative to D_exec(N=1) | Route, platform |

F1a uses retained tasks (reference target: ≥95%). Matched-route F1c comparisons exclude dispatcher queueing. No fixed F1b latency threshold is specified.

## 3. F2 — Traffic Scheduling

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F2a-i — Conflict-free itinerary rate | 100 × (N_itineraries − N_triggering_negotiation) / N_itineraries | Itinerary | /rmf_traffic/itinerary_set, /rmf_traffic/negotiation_notice |
| F2a-ii — Schedule feasibility | 100 × (N_resolved + N_recovered) / N_conflicts | Distinct conflict | Negotiation outcomes, itinerary updates |
| F2b — Schedule update latency | t(first matching query_update_1) − t(itinerary_set) | Matched itinerary | /rmf_traffic/itinerary_set, schedule mirror updates |

Conflicts are keyed by conflict_version: resolved means negotiation_conclusion.resolved=true; recovered means resolved=false followed by a participant's new itinerary within 5.0 s; other concluded conflicts are unrecovered. Feasibility is N/A when no conflict is classified.

F2b matches participant and itinerary version. Its reference budget is 0.628 / 0.220 = 2.85 s; it measures schedule propagation, not bid computation.

## 4. F3 — Traffic Negotiation

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F3a — Negotiation notices | count(negotiation_notice) | Repeat | /rmf_traffic/negotiation_notice |
| F3b — Negotiation latency | t(negotiation_conclusion) − t(first notice for conflict_version) | Concluded conflict | Negotiation notices and conclusions |

F3a reports repeat counts and their median. F3b classifies outcomes as resolved, refused (resolved=false with matching refusal), or search-failed (resolved=false without matching refusal). Resolution-time mean uses resolved negotiations only. Configured negotiation timeout: 15 s.

## 5. F4 — Fleet Throughput and Utilisation

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F4a — Task completion | 100 × N(tasks with completed state) / N(submitted tasks) | Task | RMF WebSocket task state |
| Batch throughput | N_completed / Σ T_batch (tasks/min) | Scenario across repeats | Task award and completion timestamps |
| Throughput scaling | 100 × Th(3) / (3 × Th(1)) | Scenario, platform | N=3 batches and matched N=1 routes |
| Workload ceiling | 100 × Σ d_r / (3 × max_r d_r); d_r = mean N=1 route execution duration | Scenario, platform | Matched N=1 routes |
| F4b — Robot utilisation | 100 × Σ T_moving / Σ T_window | Robot-window, pooled across repeats | Per-robot /cmd_vel |

For N=3, T_batch = max(t_completed) − min(t_award) per repeat. For N=1, throughput uses aggregate execution durations of the same three routes run separately. Th(1) excludes inter-route travel, making the reported throughput scaling a conservative lower-bound estimate. F4b uses the common N=3 window [min(t_award), max(t_completed)] for all three robots.

A robot counts as commanded to move when its latest /cmd_vel is ≤0.25 s old and either |v| ≥ 0.005 m/s or |ω| ≥ 0.05 rad/s. Hardware utilisation measures commands, not physical motion.

## 6. F5 — Execution Fidelity

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F5a — Relative duration error | 100 × (D_actual − D_planned) / D_planned | Task | Winning bid and task completion |
| F5b — Finish-time offset | t_completed − t_estimated_finish | Task | Winning bid and task completion |

D_planned = t_estimated_finish − t_bid_response; D_actual = t_completed − t_award. Planned finish comes from the winning bid_response.proposal.finish_time. Simulation timestamps are aligned using /clock; hardware uses wall time. F5a compares matched routes at N=1 and N=3; F5b includes dispatch queueing.

## 7. F6 — Stationary Time

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F6a — Stationary duration | Sum of stationary-episode durations within [t_award, t_completed] | Robot-task | Per-robot /cmd_vel |
| F6b — Stationary episode duration | Mean duration of qualifying stationary episodes | Episode | Per-robot /cmd_vel |
| Additional stationary time | F6a(N=3) − F6a(N=1) on the same route | Route, platform | N=3 and matched N=1 recordings |

Episodes require at least 0.3 s without a qualifying motion command; results also use a 1.0 s minimum. Motion thresholds and command freshness follow F4b. Stops are not restricted to a map region or attributed exclusively to traffic coordination.

## 8. F7 — Inter-Robot Separation

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F7a — Minimum reported separation | min_(t,i<j) ‖p_i(t) − p_j(t)‖ | Repeat | /fleet_states, RobotState.location |
| F7b — Separation threshold events | Count of contiguous below-threshold frame sequences per robot pair | Pair-event | /fleet_states, RobotState.location |
| F7c — LiDAR-derived separation | Matched mutual /scan range plus LDS head geometry offset | Robot pair, repeat | Per-robot /scan |

F7a/F7b use RMF-reported reference-point poses, not physical ground truth. F7b counts events and below-threshold samples at 0.356 m and 0.628 m; an event is one contiguous threshold-violation sequence for a pair.

F7c estimates laser-centre separation. Mutual matches require a 0.02–0.12 m scan cluster on each robot and range disagreement ≤ 0.045 m. The lower bound is D = min(r_1,r_2) + 0.029 m, with a geometry interval extending to +0.034 m. Measurements at the 0.120 m sensor range floor are censored. F7c reports the 0.356 m threshold and is validated with Gazebo ground truth in simulation only; hardware has no independent position reference.

## 9. F8 — Fleet-Size Comparison

| Metric | Definition | Observation unit | Source |
|---|---|---|---|
| F8a — Planning-latency growth | 100 × (P(3) − P(1)) / P(1); P(N) = mean F1b bid latency | Bid, pooled by fleet size | F1b |
| F8b — Task throughput | Th(N) and throughput scaling from F4 | Scenario, platform | F4 throughput outputs |

F8a covers all N=3 bids and, separately, the first bid of each N=3 repeat. Average change per added robot between N=1 and N=3 is F8a / 2, not a fitted scaling rate. No scalability curve or breaking point is inferred from two fleet sizes.

## 10. Measurement Scope

- Descriptive statistics use retained repeats only.
- F2/F3 describe RMF traffic records and negotiations, not independently observed collisions.
- F4b/F6 measure commanded velocities.
- F7a/F7b use reported poses. Independent position ground truth is available only in simulation; hardware F7c has no external position-reference validation.
- N=1 baselines have no multi-robot conflicts or negotiations; F8 compares N=1 and N=3 only.
