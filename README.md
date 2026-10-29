# LASER UAV Planners

`laser_uav_planners` collects the trajectory-planning backends used by the LASER UAV System (LUS). Gitman downloads the implementations and links them under `ros_packages/`.

The `planning_manager` node in `laser_uav_managers` selects and runs the backend. This collection has no ROS node or planner executable of its own.

## Backends

| Backend | ROS package | Role |
| --- | --- | --- |
| PMM | `pmm_planner` | Minimum-time trajectories for agile flight. |
| MINCO | `super_planner` | Smooth minimum-snap waypoint trajectories using the exported MINCO-S4 solver. |
| SUPER | `super_planner` | Obstacle-aware planning and replanning with ROG-Map. |

MINCO and PMM do not perform collision checking. SUPER needs the separate `laser_uav_mappings` collection, which provides `rog_map`. Its message interfaces are supplied by `mars_quadrotor_msgs` in the SUPER repository.

## Install and build

From this repository:

```bash
gitman install
```

Dependencies and their revisions are listed in [.gitman.yml](.gitman.yml). Checkouts are stored in `.gitman/`; `ros_packages/` contains links to them. Make the component ROS packages available in your workspace before building.

From a workspace with the LASER packages and their dependencies:

```bash
colcon build --packages-up-to laser_uav_managers
source install/setup.bash
```

## Configuration

Set `planner` in the planning-manager YAML before starting:

```yaml
/**/**:
  ros__parameters:
    planner: minco
```

Accepted values are `minco`, `pmm` and `super`. The default is MINCO. The manager does not switch backends while running, and simulation sessions can supply their own planning-manager YAML.

PMM tuning lives in its `params/<UAV_TYPE>.yaml`. MINCO limits and output sampling are configured through `minco.*` in the planning-manager YAML. SUPER has its own configuration selected by `super_config`; it also needs odometry, a point cloud and TF connecting their frames.

The planning manager receives the vehicle calibration and publishes `laser_msgs/msg/ReferenceHorizon`. Control interpolates that reference onto its own prediction timestep. Planner optimization and control-loop timing are configured separately.

## Interfaces

For navigation, send goals to `/<UAV_NAME>/planning_manager/goto` or `goto_relative` (`laser_msgs/msg/PoseWithHeading`), or waypoints to `trajectory_path` (`laser_msgs/msg/TrajectoryPath`). The manager publishes the reference horizon and mission diagnostics.

See [LASER UAV Managers](https://github.com/LASER-Robotics/laser_uav_managers) for the full node interfaces, mission behavior and launch arguments. Backend details are in the [PMM README](ros_packages/pmm_planner/README.md) and [SUPER README](ros_packages/super_planner/README.md).
