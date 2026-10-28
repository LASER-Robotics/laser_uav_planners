# LASER UAV Planners

Gitman collection of the planning backends used by the LASER UAV stack, following
`laser_uav_lio-s`. This repository aggregates ROS packages; it does not build a
planner library or expose a compatibility ROS package named `laser_uav_planners`.

| Backend | ROS package | Role |
| --- | --- | --- |
| PMM | `pmm_planner` | Minimum-time trajectories for agile flight |
| MINCO | `super_planner` | Minimum-snap trajectories using the existing MINCO-S4 solver |
| SUPER | `super_planner` | Trajectories with obstacle avoidance |

ROG-Map is provided by the separate `laser_uav_mappings` collection. SUPER retains
its ROS dependency on `rog_map` and also provides `mars_quadrotor_msgs`. MINCO uses
the existing SUPER library without another copy of its solver or a separate
`minco` package.

## Install

From the root of this repository:

```bash
gitman install
```

Gitman stores repositories in `.gitman/` and links them under `ros_packages/`.
Every dependency uses `dev/separate_planning_and_control`. The PMM integration on
that branch must be published to `LASER-Robotics/pmm_uav_planner` before a fresh
remote installation can obtain it.

From the workspace root, install the mapping collection too:

```bash
gitman install --root src/laser_uav_mappings
colcon build --packages-up-to laser_uav_managers
```

Select `planner: pmm`, `planner: minco` or `planner: super` in the planning-manager
setup YAML before starting. Each simulation session retains its own setup.
Control and planning launches remain separate; simulation time is inferred from
`REAL_UAV=false`.

The PMM backend returns raw states and relative times. Only `control_manager`
interpolates reference horizons for its control timestep.
