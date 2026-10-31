# System Under Test

## 1. System Overview

The benchmark evaluates an Open-RMF multi-robot fleet on simulation and hardware platforms. Three TurtleBot3 robots execute navigation tasks through ROS 2 Nav2 and a custom fleet adapter. Both platforms use the same fleet profile and navigation graph.

The navigation graph contains 9 waypoints (wp_1–wp_9) and 18 directed lanes. Scenario routes, initial placements, and lane closures are specified in [SCENARIO_DESIGN.md](SCENARIO_DESIGN.md).

## 2. Experimental Configuration

| Parameter | Configuration |
|---|---|
| Platforms | Gazebo simulation; physical TurtleBot3 fleet |
| Fleet sizes | N=1 (baseline); N=3 (concurrent workload) |
| Scenarios | Bottleneck, Crossing, Head-on, Shared Lane |
| Navigation graph | 9 waypoints; 18 directed lanes |
| RMF footprint radius | 0.178 m |
| RMF vicinity radius | 0.45 m |
| Nav2 local controller | dwb_core::DWBLocalPlanner |
| Hardware command limits (observed) | Linear: 0.220 m/s; angular: 1.000 rad/s |
| /scan frequency (observed) | Simulation: 9.72–9.95 Hz; hardware: 9.60–10.32 Hz |
| /fleet_states frequency (observed) | 10 Hz on both platforms |
| Task bid window | N=3: 5.0 s; N=1: 2.0 s |

The simulation N=3 collection recorded a real-time factor of 0.999.

## 3. Navigation and Task Execution

All robots use Nav2 for local navigation. The fleet adapter reports robot states to Open-RMF, forwards navigation commands, and requests replanning after navigation failures. Unsuccessful navigation goals may be retried without a fixed retry limit.

Each N=3 repeat contains three tasks. Robots are initially placed at the respective route starts, and each robot executes one task in the retained workload. Task auctions are processed sequentially; queueing time is measured separately from execution duration.

The configuration evaluates task dispatch and concurrent route execution, not unconstrained task allocation or load balancing.

## 4. Dataset and Measurement Sources

| Condition | Per platform | Both platforms |
|---|---|---|
| N=3: 4 scenarios × 5 repeats | 20 repeats | 40 repeats |
| N=1: 12 routes × 5 repeats | 60 repeats | 120 repeats |
| Total | 80 repeats | 160 repeats |

Five repeats are retained per scenario at N=3 and per route at N=1. N=3 repeats contain three robot-tasks; N=1 repeats contain one robot-task.

The collection records task events, robot states, navigation signals, traffic coordination events, and feature-specific metrics. Gazebo ground-truth poses are available only in simulation. Hardware has no independent position reference.

Retained repeats pass recording and applicable physics checks. N=3 repeats additionally require all three tasks to reach the RMF completed state. Completion statistics apply to retained repeats, not all attempted runs.

## 5. Evaluation Scope

- Task allocation: Route-start placement favours predefined robot–route assignments; allocation optimality is outside the evaluation scope.
- Localisation: RMF-reported poses are available on both platforms; independent pose validation is available only in simulation.
- Task completion: The completed state is reported by RMF and does not independently verify physical goal arrival on hardware.
- Execution duration: Bid queueing is reported separately from execution time because the N=1 and N=3 bid windows differ.
- Measurement definitions: Metric-specific data sources, aggregation units, and validation procedures are defined in [METRIC_DESIGN.md](METRIC_DESIGN.md).

Results are reported in [RESULTS.md](RESULTS.md).
