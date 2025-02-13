<h1> LASER UAV Planning of Minimum-Time Trajectories </h1>

<h2> Dependencies: </h2>
<p>

- [UAV Custom Msgs](https://github.com/Renataavln/uav_custom_msgs.git)

- [Eigen](https://gitlab.com/libeigen/eigen)

</p>

<h2> How to install Eigen:</h2>

<p>
Put [Eigen](https://gitlab.com/libeigen/eigen) folder in your <pre> /usr/include/ </pre>
</p>

### Run server:
```
ros2 run laser_uav_pmt_trajectory laser_uav_pmt_trajectory
```

### Run client with UAV model:
```
ros2 topic pub /uav_parameters uav_custom_msgs/msg/UAVParameters "
maximum_acceleration_norm: 34.32
maximum_velocity: 90.0
gravitational_acceleration: 9.8066
drag_coefficients: [0.28, 0.35, 0.7]
drag: true
precision: 0.01
max_iter: 20
first_run_alpha: 10.0
first_run_alpha_reduction_factor: 0.2
first_run_alpha_min_threshold: 0.001
first_run_max_iter: 30
second_run_alpha: 35.0
second_run_alpha_reduction_factor: 0.1
second_run_alpha_min_threshold: 0.01
second_run_max_iter: 10
dt_precision: 0.001
debug: false
sampled_trajectory: true
sampling_step: 0.5
sampled_trajectory_file: 'sampled_trajectory.csv'
"
```


### Run client with path:
```
ros2 topic pub /waypoints uav_custom_msgs/msg/Waypoints "
start_position: [0, 0, 0]
start_velocity: [0, 0, 0]
end_position: [5, 5, 2.5]
end_velocity: [0, 0, 0]
waypoints: [0, 10, 0, 0, 10, 5, 10, 0, 5, 0, 0, 0]
"
```

### Return:
```
ros2 topic echo /trajectory
```


```
timestamp vector
position_x vector
position_y vector
position_z vector
velocity_x vector
velocity_y vector
velocity_z vector
acceleration_x vector
acceleration_y vector
acceleration_z vector
```
