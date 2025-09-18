# LASER UAV Planners

This package provides **C++ classes** that implement trajectory and motion planning algorithms for the Laser UAV System (LUS). It generates safe, optimized paths.

## Overview and Algorithms

The main objective of this package is to receive a goal and, based on the vehicle's current state, generate a smooth trajectory.

### Agile Planner
-   **Description:** A class that implements an optimal motion planning algorithm for our specific use case. It uses a **center-of-mass model with thrust constraints** to incorporate vehicle limitations while remaining lightweight enough for real-time computation.
-   **Reference:**
    ```bibtex
    @article{teissing2024pmm,
        author = "Teissing, Krystof and Novosad, Matej and Penicka, Robert and Saska, Martin",
        doi = "10.1109/LRA.2024.3471388",
        journal = "IEEE Robotics and Automation Letters",
        number = 11,
        pages = "10351-10358",
        pdf = "[https://arxiv.org/pdf/2409.16074.pdf](https://arxiv.org/pdf/2409.16074.pdf)",
        title = "Real-Time Planning of Minimum-Time Trajectories for Agile UAV Flight",
        url = "[https://ieeexplore.ieee.org/document/10700666](https://ieeexplore.ieee.org/document/10700666)",
        volume = 9,
        year = 2024
    }
    ```
-   **Configurable Parameters:**
    ```yaml
    agile_planner:
      quadrotor_parameters:
        # This is a = F / M. F is maximum total thrust available
        max_acc: 20.0 

        # This is the maximum speed
        max_vel: 1.0

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
        dt_precision: 0.001 # DON'T CHANGE
        sampling_step: 0.008 # DON'T CHANGE
    ```
