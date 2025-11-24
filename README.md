# LASER UAV Planners

This package provides **C++ classes** that implement trajectory and motion planning algorithms for the Laser UAV System (LUS). It generates safe, optimized paths.

## Overview and Algorithms

The main objective of this package is to receive a goal and, based on the vehicle's current state, generate a smooth trajectory.

### Agile Planner
-   **Description:** A class that use an optimal motion planning algorithm modified for our specific use case. It uses a **point-of-mass model [PMM] with thrust constraints** to incorporate vehicle limitations while remaining lightweight enough for real-time computation.

-   **Reference:**
    K. Teissing, M. Novosad, R. Penicka and M. Saska, "Real-Time Planning of Minimum-Time Trajectories for Agile UAV Flight," in IEEE Robotics and Automation Letters, vol. 9, no. 11, pp. 10351-10358, Nov. 2024, doi: 10.1109/LRA.2024.3471388.

-   **Configurable Parameters:**
    ```yaml
    agile_planner:
      multirotor_parameters:
        # This is a = F / M. F is maximum total thrust available
        max_accel: 30.0 

        # This is the maximum speed
        max_vel: 16.0

        # This is default speed to goto
        default_vel: 1.0
        
        absolute_maximum_angular_accel: 5.0 # rad/s²

      ltd_opt:
        use_drag: false
        thrust_decomp_acc_precision: 0.01
        thrust_decomp_max_iter: 20

      first_vel_opt:
        alpha: 10.0
        alpha_reduction_factor: 0.2
        alpha_min_threshold: 0.001
        max_iter: 30

      second_vel_opt:
        run: true
        alpha: 35.0
        alpha_reduction_factor: 0.1
        alpha_min_threshold: 0.01
        max_iter: 10

      time:
        dt_precision: 0.001
        sampling_step: 0.0095 # DON'T CHANGE
    ```
