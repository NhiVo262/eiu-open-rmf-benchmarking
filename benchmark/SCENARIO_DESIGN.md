# Benchmark Scenario Design

## 1. Objective and Scope

The benchmark evaluates Open-RMF task execution and traffic coordination for a fleet of three robots (N=3) under four predefined workloads: Bottleneck, Crossing, Head-on, and Shared Lane. Each workload assigns one task route to each robot, with initial poses placed at the corresponding route starts.

This configuration primarily evaluates concurrent route execution and coordination. It does not provide an unconstrained assessment of task allocation or load balancing.

## 2. Navigation Graph

The indoor navigation graph contains 9 waypoints (wp_1–wp_9) and 9 undirected edges (18 directed lanes). Its edges are:

- wp_1–wp_2, wp_2–wp_3, wp_2–wp_4
- wp_3–wp_5, wp_4–wp_6, wp_5–wp_7
- wp_6–wp_7, wp_6–wp_8, wp_7–wp_9

The graph comprises a six-waypoint loop (wp_2 → wp_4 → wp_6 → wp_7 → wp_5 → wp_3 → wp_2) and three terminal waypoints (wp_1, wp_8, wp_9). The shortest edge is wp_2–wp_3 (1.67 m).

## 3. Scenario Configuration

Each scenario defines three task routes (T1–T3). Route length is calculated from the navigation graph.

| Scenario | T1 — Robot 1 | T2 — Robot 2 | T3 — Robot 3 | Primary shared resource |
|---|---|---|---|---|
| Bottleneck | wp_2 → wp_3 → wp_5 (7.08 m) | wp_3 → wp_2 → wp_4 (6.82 m) | wp_4 → wp_2 → wp_3 (6.82 m) | wp_2–wp_3, traversed in both directions |
| Crossing | wp_2 → wp_4 → wp_6 (10.07 m) | wp_6 → wp_7 → wp_9 (9.64 m) | wp_7 → wp_6 → wp_8 (9.61 m) | Junction wp_6 and opposing movements on wp_6–wp_7 |
| Head-on | wp_2 → wp_3 → wp_5 (7.08 m) | wp_5 → wp_3 → wp_2 (7.08 m) | wp_3 → wp_5 → wp_7 (10.16 m) | Opposing use of shared route segments |
| Shared Lane | wp_2 → wp_3 → wp_5 → wp_7 (11.83 m) | wp_4 → wp_2 → wp_3 → wp_5 (12.22 m) | wp_6 → wp_4 → wp_2 → wp_3 (11.74 m) | Same-direction use of wp_2–wp_3 |

For Bottleneck, the wp_6–wp_7 lane is closed. All other scenarios use the unmodified graph. Crossing combines a shared junction with opposing lane traversal; it is not a pure intersection-only workload.

## 4. Experimental Procedure

- Platforms: Simulation and physical TurtleBot3 hardware.
- Fleet sizes: N=3 for concurrent scenarios; N=1 for route-matched baselines.
- Initial condition: Each robot starts at the first waypoint of its assigned route and faces the second waypoint.
- Assignment: Robot 1 executes T1, Robot 2 executes T2, and Robot 3 executes T3 as the intended configuration; actual awards are verified from execution logs.
- Repetitions: Five repeats per scenario and platform at N=3; five repeats per route and platform at N=1.
- Dataset: 160 repeats in total across both platforms (40 scenario repeats and 120 baseline repeats).

Robots are repositioned before each repeat. A matching N=1 run provides the reference for route-level execution-time and stationary-time comparisons.

## 5. Interpretation and Limitations

1. Route-dependent assignment: Initial placement at route starts strongly favours the corresponding robots. Dispatch results must not be interpreted as evidence of optimal or unconstrained allocation.
2. Potential versus observed contention: Shared graph edges or waypoints establish potential resource contention. Actual temporal overlap and resource occupancy were not independently reconstructed for every repeat.
3. Route comparability: Head-on T3 is 3.08 m longer than T1/T2; raw completion times across these routes are therefore not directly comparable without accounting for route differences.
4. Measurement reference: Simulation includes Gazebo ground truth. Hardware has no independent external position-tracking reference; RMF-reported poses do not establish physical ground truth.
5. Configuration provenance: Graph topology and lane configuration must be checked against the artefacts retained for the collected runs before claiming exact reproduction.

## 6. Implementation References

- Scenario definitions: collect/hardware/run_hardware_scenarios.sh (SCENARIOS_TABLE)
- Collection procedure: collect/hardware/PROCEDURE.md
- Metric definitions: METRIC_DESIGN.md
- Navigation graph: install/tb3_fleet/share/tb3_fleet/maps/world_tb3/nav_graphs/0.yaml
- Evaluation results: RESULTS.md
