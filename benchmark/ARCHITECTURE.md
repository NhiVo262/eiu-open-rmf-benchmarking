# Benchmark Architecture

This document describes the directory structure, component responsibilities, data flow, and dataset organization of src/benchmark/. Experimental conditions, metric definitions, and results are documented separately in [SYSTEM_UNDER_TEST.md] [METRIC_DESIGN.md](METRIC_DESIGN.md), and [RESULTS.md](RESULTS.md).

## 1. Directory Structure

```text
src/benchmark/
├── collect/
│   ├── rmf/             Task submission and RMF task-state recording
│   ├── simulation/      Gazebo collection runners and physics-state recording
│   └── hardware/        Physical-robot collection runners and readiness checks
├── analyse/
│   ├── <feature>/       F1–F7 metric analysers
│   ├── common/          Shared analysis utilities
│   ├── validity/        Measurement and dataset-quality assessment
│   └── summary/         F8, aggregation, and report tables
├── data/
│   ├── simulation/      Simulation recordings and metrics
│   └── hardware/        Hardware recordings and metrics
└── diagnostics/         Supplementary diagnostic outputs
    ├── simulation/
    └── hardware/
```

## 2. Component Responsibilities

| Component | Responsibilities | Principal entry points |
|---|---|---|
| RMF collection | Submit patrol tasks and record task states | rmf/run_benchmark_concurrent.py, rmf/record_api_state.py |
| Simulation collection | Run N=1 and N=3 workloads; capture Gazebo poses | simulation/run_simulation_baselines.sh, simulation/run_simulation_scenarios.sh, simulation/record_ground_truth_poses.py |
| Hardware collection | Run N=1 and N=3 workloads; verify communication and capture readiness | hardware/run_hardware_baselines.sh, hardware/run_hardware_scenarios.sh, hardware/check_hardware_preflight.py |
| Metric analysis | Compute feature-specific metrics from recorded data | analyse/<feature>/analyze_*.py |
| Validity assessment | Assess recorded data and measurement validity | analyse/validity/analyze_*.py |
| Aggregation and reporting | Compute fleet-size comparisons and aggregate statistics | analyse/fleet_coordination/analyze_fleet_scaling.py, analyse/summary/analyze_scalability.py |

Hardware collection also uses hardware/check_n3_coordination.py to assess N=3 repeat eligibility, hardware/check_zenoh_routes.py to check robot action routes, and hardware/hold_robot_topics.py to maintain topic availability through Zenoh.

## 3. Data Flow

```text
Simulation (Gazebo + TurtleBot3)        Hardware (TurtleBot3 + Nav2)
                 \                       /
                  \                     /
                   Collection runners
                           |
                ROS 2 bags + RMF task states
                  (+ Gazebo poses in simulation)
                           |
                Per-repeat metric analysers
                           |
              Feature metrics (F1–F7) + checks
                           |
               Collection-level aggregation
                           |
               F4 scaling / F8 comparison
                           |
                  RESULTS.md / APPENDIX.md
```

Collection runners manage task submission, recording, and repeat acceptance. Analysers process retained recordings into per-repeat metric files; aggregation scripts produce cross-repeat and cross-platform statistics. Independent pose ground truth is available in simulation, but not in the evaluated hardware dataset.

## 4. Metric Organization

| Feature | Analysis directory or script | Principal measurements |
|---|---|---|
| F1 — Task Planning | analyse/task_planning/ | Dispatch success, bid latency, execution duration |
| F2 — Traffic Scheduling | analyse/traffic_scheduling/ | Conflict-free itinerary rate, schedule feasibility, scheduling latency |
| F3 — Negotiation | analyse/negotiation/ | Negotiation notices, outcomes, latency |
| F4 — Fleet Coordination | analyse/fleet_coordination/ | Task completion, batch throughput, scaling, command-based utilisation |
| F5 — Execution Fidelity | analyse/execution_fidelity/ | Planned-versus-actual duration error, finish offset |
| F6 — Congestion Handling | analyse/congestion_handling/ | Stationary time and episode duration from cmd_vel |
| F7 — Inter-Robot Separation | analyse/safety/ | RMF-reported pose separation (7a/7b) and LiDAR-derived separation (7c) |
| F8 — Fleet-Size Comparison | analyse/summary/analyze_scalability.py | Planning-latency growth and N=1-to-N=3 throughput comparison |

For F7, analyse/safety/analyze_safety_tracker.py computes separation and threshold events from /fleet_states (RobotState.location); analyse/safety/analyze_safety_lidar.py estimates laser-centre separation from /scan. Simulation ground truth supports measurement validation, including analyse/validity/analyze_lidar_attribution.py.

## 5. Collection and Dataset Organization

| Platform | Fleet size | Runner | Workload |
|---|---|---|---|
| Simulation | N=1 | simulation/run_simulation_baselines.sh | 12 routes × 5 repeats |
| Simulation | N=3 | simulation/run_simulation_scenarios.sh | 4 scenarios × 5 repeats |
| Hardware | N=1 | hardware/run_hardware_baselines.sh | 12 routes × 5 repeats |
| Hardware | N=3 | hardware/run_hardware_scenarios.sh | 4 scenarios × 5 repeats |

The retained dataset contains 160 repeats across both platforms. Each N=3 repeat contains three robot-tasks; each N=1 repeat contains one.

```text
data/<platform>/
├── baselines/<route>/run_<timestamp>/
│   ├── repeat_1/ ... repeat_5/
│   └── api_state_attempt<N>.jsonl
└── scenarios/<scenario>/run_<timestamp>/
    ├── repeat_1/ ... repeat_5/
    ├── api_state_attempt<N>.jsonl
    └── repeats_summary.json
```

Each retained repeat_N/ contains a ROS 2 bag and applicable <feature>_metrics.json outputs. Run-level files and session logs are generated where supported. Collection-level outputs include fleet_scaling_metrics.json, scalability_metrics.json, and simulation/lidar_attribution_metrics.json.

## 6. Related Documentation

| Document | Scope |
|---|---|
| [SYSTEM_UNDER_TEST.md](SYSTEM_UNDER_TEST.md) | Fleet, platform configuration, and measurement limitations |
| [SCENARIO_DESIGN.md](SCENARIO_DESIGN.md) | Navigation graph and scenario workloads |
| [METRIC_DESIGN.md](METRIC_DESIGN.md) | Measurement definitions, sources, and aggregation units |
| collect/simulation/PROCEDURE.md | Simulation data collection |
| collect/hardware/PROCEDURE.md | Hardware data collection |
| analyse/<feature>/PROCEDURE.md | Feature-specific analysis procedures, where available |
| [RESULTS.md](RESULTS.md) | Principal experimental results |
| [APPENDIX.md](APPENDIX.md) | Detailed per-feature statistics |
