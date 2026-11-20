# Benchmark Architecture

This document outlines the structure, components, data flow, and dataset organization of src/benchmark/. Experimental setup, measurement definitions, and findings are documented in [SYSTEM_UNDER_TEST.md](SYSTEM_UNDER_TEST.md), [METRIC_DESIGN.md](METRIC_DESIGN.md), and [RESULTS.md](RESULTS.md).

## 1. Directory Structure

```text
src/benchmark/
├── collect/
│   ├── rmf/             Task submission and RMF task-state recording
│   ├── simulation/      Gazebo collection and ground-truth recording
│   └── hardware/        Physical-robot collection and preflight checks
├── analyse/
│   ├── <feature>/       F1–F7 feature analyses
│   ├── common/          Shared analysis utilities
│   ├── validity/        Measurement and dataset-quality checks
│   └── summary/         F8 and cross-repeat summaries
├── data/
│   ├── simulation/      Simulation recordings and metrics
│   └── hardware/        Hardware recordings and metrics
└── diagnostics/
    ├── simulation/
    └── hardware/
```

## 2. Components

| Component | Responsibility | Main entry points |
|---|---|---|
| RMF collection | Submit patrol tasks and capture RMF task states | collect/rmf/run_benchmark_concurrent.py, record_api_state.py |
| Simulation collection | Execute N=1/N=3 workloads and record Gazebo poses | collect/simulation/run_simulation_baselines.sh, run_simulation_scenarios.sh, record_ground_truth_poses.py |
| Hardware collection | Execute N=1/N=3 workloads and check system readiness | collect/hardware/run_hardware_baselines.sh, run_hardware_scenarios.sh, check_hardware_preflight.py |
| Feature analysis | Compute F1–F7 metrics from recordings | analyse/<feature>/analyze_*.py |
| Validity checks | Assess measurement and dataset quality | analyse/validity/analyze_*.py |
| Aggregation | Compare repeats, platforms, and fleet sizes | analyse/fleet_coordination/analyze_fleet_scaling.py, analyse/summary/analyze_scalability.py |

Hardware support scripts include check_n3_coordination.py (N=3 eligibility), check_zenoh_routes.py (action-route availability), and hold_robot_topics.py (Zenoh topic availability).

## 3. Data Flow

```text
Simulation (Gazebo + TurtleBot3)       Hardware (TurtleBot3 + Nav2)
                 \                         /
                  \                       /
                    Collection runners
                            |
               ROS 2 bags + RMF API states
                  + Gazebo poses (simulation)
                            |
                  Per-repeat analysers
                            |
                    F1–F7 metrics
                            |
                 Validity and aggregation
                            |
                    F4 / F8 comparisons
                            |
                  RESULTS.md / APPENDIX.md
```

Collection runners manage submission, recording, and repeat acceptance. Analysers produce per-repeat metrics; aggregation scripts summarize retained runs. Independent pose ground truth is available in simulation, not in the evaluated hardware dataset.

## 4. Metric Organization

| Feature | Analysis location | Principal measurements |
|---|---|---|
| F1 — Task Planning | analyse/task_planning/ | Dispatch success, bid latency, execution duration |
| F2 — Traffic Scheduling | analyse/traffic_scheduling/ | Conflict-free itinerary rate, feasibility, scheduling latency |
| F3 — Negotiation | analyse/negotiation/ | Notices, outcomes, latency |
| F4 — Fleet Coordination | analyse/fleet_coordination/ | Completion, throughput, scaling, command-based utilisation |
| F5 — Execution Fidelity | analyse/execution_fidelity/ | Predicted-versus-actual duration error, finish offset |
| F6 — Congestion Handling | analyse/congestion_handling/ | Command-based stationary time and waiting episodes |
| F7 — Inter-Robot Separation | analyse/safety/ | RMF-reported pose separation (7a/7b), LiDAR-derived separation (7c) |
| F8 — Fleet-Size Comparison | analyse/summary/analyze_scalability.py | Planning-latency growth and N=1-to-N=3 throughput comparison |

F7 uses analyze_safety_tracker.py for /fleet_states pose separation and analyze_safety_lidar.py for /scan-based laser-centre separation. Simulation validation includes analyse/validity/analyze_lidar_attribution.py.

## 5. Dataset Organization

| Platform | Fleet size | Collection runner | Retained workload |
|---|---|---|---|
| Simulation | N=1 | simulation/run_simulation_baselines.sh | 12 routes × 5 repeats |
| Simulation | N=3 | simulation/run_simulation_scenarios.sh | 4 scenarios × 5 repeats |
| Hardware | N=1 | hardware/run_hardware_baselines.sh | 12 routes × 5 repeats |
| Hardware | N=3 | hardware/run_hardware_scenarios.sh | 4 scenarios × 5 repeats |

Dataset: 160 retained repeats across both platforms. Each N=1 repeat contains one robot-task; each N=3 repeat contains three dependent robot-tasks.

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

Retained repeat_N/ directories contain ROS 2 bags and applicable feature-metric JSON files. Run-level logs and summaries vary by collection workflow. Aggregated outputs include fleet_scaling_metrics.json, scalability_metrics.json, and simulation/lidar_attribution_metrics.json.

## 6. Related Documentation

| Document | Content |
|---|---|
| [SYSTEM_UNDER_TEST.md](SYSTEM_UNDER_TEST.md) | Platforms, fleet configuration, and measurement limitations |
| [SCENARIO_DESIGN.md](SCENARIO_DESIGN.md) | Navigation graph and workloads |
| [METRIC_DESIGN.md](METRIC_DESIGN.md) | Metric definitions, data sources, and aggregation |
| [collect/simulation/PROCEDURE.md](collect/simulation/PROCEDURE.md) | Simulation collection procedure |
| [collect/hardware/PROCEDURE.md](collect/hardware/PROCEDURE.md) | Hardware collection procedure |
| analyse/<feature>/PROCEDURE.md | Feature analysis procedures, where available |
| [RESULTS.md](RESULTS.md) | Experimental findings |
| [APPENDIX.md](APPENDIX.md) | Detailed statistics |
