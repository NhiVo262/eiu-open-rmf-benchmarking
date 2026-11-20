# System Under Test

## 1. System Overview

The benchmark evaluates an Open-RMF fleet of three TurtleBot3 robots in Gazebo simulation and on physical hardware. Both platforms use ROS 2 Nav2, a custom fleet adapter, the same fleet profile, and a navigation graph with 9 waypoints and 18 directed lanes. Routes, initial placements, and lane closures are defined in [SCENARIO_DESIGN.md](SCENARIO_DESIGN.md).

## 2. Experimental Configuration

| Parameter | Configuration |
|---|---|
| Platforms | Gazebo simulation; physical TurtleBot3 fleet |
| Fleet sizes | N=1 (baseline); N=3 (concurrent workload) |
| Scenarios | Bottleneck, Crossing, Head-on, Shared Lane |
| Navigation graph | 9 waypoints; 18 directed lanes |
| RMF footprint / vicinity radius | 0.178 m / 0.450 m |
| Nav2 local controller | dwb_core::DWBLocalPlanner |
| Hardware command limits (observed) | Linear: 0.220 m/s; angular: 1.000 rad/s |
| /scan frequency (observed) | Simulation: 9.78–10.00 Hz; hardware: 8.99–11.38 Hz |
| /fleet_states frequency (observed) | Simulation: 9.89–10.00 Hz; hardware: 9.87–10.00 Hz |
| Task bid window | N=3: 5.0 s; N=1: 2.0 s |
| Waypoint merge radius | 0.25 m on both platforms |

Simulation N=3 real-time factor: 0.979–1.000 across 20 repeats (mean 0.998). Both platforms set max_merge_waypoint_distance to 0.25 m, matching the Nav2 goal tolerance rather than the RMF default of 1e-3 m. Within this radius, RMF reports the robot at the waypoint.

## 3. Navigation and Task Execution

Nav2 handles local navigation. The custom fleet adapter reports robot states, forwards navigation commands, and requests replanning after navigation failures. Failed goals may be retried without a fixed retry limit.

Each N=3 repeat submits three tasks, with robots initially positioned at the corresponding route starts. Each robot executes one task in the retained workload. Task auctions are processed sequentially, and dispatch queueing is measured separately from execution duration.

This setup evaluates task dispatch and concurrent route execution, not unconstrained allocation or load balancing.

## 4. Dataset and Measurement

| Condition | Per platform | Both platforms |
|---|---|---|
| N=3: 4 scenarios × 5 repeats | 20 | 40 |
| N=1: 12 routes × 5 repeats | 60 | 120 |
| Total retained repeats | 80 | 160 |

Each N=3 repeat contains three robot-tasks; each N=1 repeat contains one. Recordings include task events, robot states, navigation signals, and traffic coordination events, with feature-specific metrics generated during analysis.

Retained repeats pass recording and applicable physics checks. N=3 repeats additionally require all three tasks to reach the RMF completed state. Completion rates therefore refer to retained repeats, not all collection attempts.

Gazebo ground-truth poses are available in simulation only; the hardware dataset has no independent position reference.

## 5. Evaluation Limits

- Allocation: Initial placement favours the predefined robot–route assignments; allocation optimality is not evaluated.
- Localisation: RMF-reported poses are available on both platforms, but independent pose validation is limited to simulation.
- Completion: RMF completed does not independently establish physical goal arrival on hardware.
- Timing: Dispatch queueing is separate from execution duration because N=1 and N=3 use different bid windows.

Metric definitions and measurement rules: [METRIC_DESIGN.md](METRIC_DESIGN.md). Results: [RESULTS.md](RESULTS.md).
