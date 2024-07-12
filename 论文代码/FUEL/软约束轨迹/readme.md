
# Trajectory Planning with Soft Constraints

This MATLAB project demonstrates how to perform trajectory planning with both hard and soft constraints. The example involves generating a smooth trajectory from a start point to an end point while avoiding obstacles.

## Table of Contents
- [Trajectory Planning with Soft Constraints](#trajectory-planning-with-soft-constraints)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Requirements](#requirements)
  - [Usage](#usage)
  - [Files](#files)
  - [Functions](#functions)
    - [`costFunction`](#costfunction)
    - [`nonlinearConstraints`](#nonlinearconstraints)
    - [Main Script (`trajectory_planning.m`)](#main-script-trajectory_planningm)
  - [Customization](#customization)
  - [Troubleshooting](#troubleshooting)

## Introduction
Trajectory planning is a key aspect of robotics and autonomous systems. This project provides an example of how to use MATLAB's optimization toolbox to plan a trajectory that avoids obstacles (hard constraints) and aims to be as smooth as possible (soft constraints).

## Requirements
- MATLAB R2017 or later
- Optimization Toolbox

## Usage
1. Clone or download this repository to your local machine.
2. Open MATLAB and navigate to the directory containing the files.
3. Run the main script `trajectory_planning.m` to execute the trajectory planning and visualize the results.

## Files
- `trajectory_planning.m`: The main script that sets up the problem, performs optimization, and plots the results.
- `README.md`: This README file.

## Functions
### `costFunction`
Calculates the cost of a given trajectory based on the following criteria:
- **Velocity smoothness**: Ensures the trajectory is smooth by penalizing large changes in position between consecutive points.
- **Obstacle penalty**: Penalizes trajectories that come too close to obstacles.

### `nonlinearConstraints`
Defines the hard constraints for the optimization problem:
- Ensures that the trajectory does not intersect with defined obstacles.

### Main Script (`trajectory_planning.m`)
The main script performs the following steps:
1. **Initialization**: Sets up the start and end points, obstacle position and radius, and the number of points in the trajectory.
2. **Initial Trajectory**: Generates an initial straight-line trajectory from the start to the end point.
3. **Optimization**: Uses the `fmincon` function to optimize the trajectory considering both the cost function and nonlinear constraints.
4. **Visualization**: Plots the initial and optimized trajectories along with the obstacle, start, and end points.

## Customization
You can customize the trajectory planning by modifying the following parameters in the main script:
- `start_point` and `end_point`: Coordinates of the start and end points of the trajectory.
- `obstacle_center` and `obstacle_radius`: Position and size of the obstacle.
- `num_points`: Number of points in the trajectory, affecting the resolution of the path.

You can also adjust the weights in the `costFunction` to prioritize smoothness or obstacle avoidance differently:
```matlab
cost = 10 * velocity_smoothness + 50 * obstacle_penalty;
```

## Troubleshooting
- **Optimization not converging**: Ensure that the `optimset` parameters are correctly configured. Increasing the number of points (`num_points`) can improve the resolution but may increase computation time.
- **Trajectory intersects the obstacle**: Verify that the hard constraints are correctly applied in the `nonlinearConstraints` function. Adjust the obstacle penalty weight in the `costFunction` to enforce stricter avoidance.

If you encounter any issues or have further questions, please feel free to open an issue or contact the project maintainer.



