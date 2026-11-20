# Benchmark Scenario Design

## 1. Scope

The benchmark evaluates Open-RMF concurrent task execution and traffic coordination across four N=3 workloads: Bottleneck, Crossing, Head-on, and Shared Lane. Each workload specifies one route per robot, with robots initially positioned at their respective route starts. This setup evaluates coordination under predefined workloads, not unconstrained task allocation or load balancing.

## 2. Navigation Graph

The indoor graph has 9 waypoints (wp_1–wp_9) and 9 undirected edges (18 directed lanes):

- wp_1–wp_2, wp_2–wp_3, wp_2–wp_4
- wp_3–wp_5, wp_4–wp_6, wp_5–wp_7
- wp_6–wp_7, wp_6–wp_8, wp_7–wp_9

A six-waypoint loop (wp_2 → wp_4 → wp_6 → wp_7 → wp_5 → wp_3 → wp_2) connects three terminals (wp_1, wp_8, wp_9). The shortest edge is wp_2–wp_3 (1.67 m).

## 3. Scenario Configuration

Routes T1–T3 are assigned to Robot 1–3 as the intended configuration. Distances are calculated from the navigation graph.

| Scenario | T1 — Robot 1 | T2 — Robot 2 | T3 — Robot 3 | Shared resource |
|---|---|---|---|---|
| Bottleneck | wp_2 → wp_3 → wp_5 (7.08 m) | wp_3 → wp_2 → wp_4 (6.82 m) | wp_4 → wp_2 → wp_3 (6.82 m) | Opposing traversal of wp_2–wp_3 |
| Crossing | wp_2 → wp_4 → wp_6 (10.07 m) | wp_6 → wp_7 → wp_9 (9.64 m) | wp_7 → wp_6 → wp_8 (9.61 m) | Junction wp_6 and opposing traversal of wp_6–wp_7 |
| Head-on | wp_2 → wp_3 → wp_5 (7.08 m) | wp_5 → wp_3 → wp_2 (7.08 m) | wp_3 → wp_5 → wp_7 (10.16 m) | Opposing traversal of shared segments |
| Shared Lane | wp_2 → wp_3 → wp_5 → wp_7 (11.83 m) | wp_4 → wp_2 → wp_3 → wp_5 (12.22 m) | wp_6 → wp_4 → wp_2 → wp_3 (11.74 m) | Same-direction traversal of wp_2–wp_3 |

Lane configuration: Bottleneck closes wp_6–wp_7; the other scenarios use the unmodified graph. Crossing combines junction sharing with opposing lane traversal rather than representing an intersection-only case.

## 4. Experimental Setup

| Parameter | Configuration |
|---|---|
| Platforms | Gazebo simulation and physical TurtleBot3 robots |
| Fleet sizes | N=3 concurrent workloads; N=1 matched-route baselines |
| Initial poses | First waypoint of each route, facing its second waypoint |
| Intended assignment | Robot 1 → T1, Robot 2 → T2, Robot 3 → T3; actual awards checked in execution logs |
| Repetitions | 5 per N=3 scenario/platform; 5 per N=1 route/platform |
| Retained dataset | 160 repeats: 40 N=3 scenario repeats and 120 N=1 baseline repeats |

Robots are repositioned before each repeat. Matched N=1 runs provide route-level references for execution and stationary time.

## 5. Interpretation Limits

- Assignment: Route-start placement favours the intended robot–task pairing; observed dispatch does not establish optimal or unconstrained allocation.
- Contention: Shared graph resources indicate possible contention, not independently verified temporal overlap in every repeat.
- Route length: Head-on T3 is 3.08 m longer than T1/T2, limiting direct raw-duration comparisons across these routes.
- Position reference: Gazebo ground truth is available in simulation; hardware lacks independent external position ground truth.
- Reproducibility: Exact topology and lane settings must be checked against artefacts from the collected runs.

## 6. References

| Resource | Location |
|---|---|
| Scenario definitions | [collect/hardware/run_hardware_scenarios.sh](collect/hardware/run_hardware_scenarios.sh) (SCENARIOS_TABLE) |
| Collection procedure | [collect/hardware/PROCEDURE.md](collect/hardware/PROCEDURE.md) |
| Metric specification | [METRIC_DESIGN.md](METRIC_DESIGN.md) |
| Navigation graph | install/tb3_fleet/share/tb3_fleet/maps/world_tb3/nav_graphs/0.yaml |
| Results | [RESULTS.md](RESULTS.md) |
