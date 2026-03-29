# Dynamic Window Approach

A standalone, header-based modern C++ implementation of the **Dynamic Window Approach (DWA)** — a local path planning algorithm for mobile robots that avoids obstacles while navigating toward a goal.

![Demo](avoidance_example.gif)

## Overview

DWA samples candidate velocities within a dynamically computed feasible window (constrained by the robot's kinematic limits and acceleration bounds), simulates short trajectories for each candidate, and selects the velocity that minimizes a weighted cost over three criteria:

| Cost | Description |
|------|-------------|
| **Heading** | Penalizes angular deviation from the goal at the end of the predicted trajectory |
| **Clearance** | Penalizes proximity to obstacles (returns `∞` on collision) |
| **Velocity** | Penalizes low linear speed, encouraging forward progress |

Supports both **circular** and **rectangular** robot footprints.

## Files

```
src/
  dwa.hpp   — class definition, data structures, configuration
  dwa.cpp   — algorithm implementation
```

No external dependencies. Drop the two files into your project and include `dwa.hpp`.

## Quick Start

```cpp
#include "dwa.hpp"

// Use default configuration
DynamicWindowApproach dwa;

// Or provide a custom configuration
DynamicWindowApproach::Configuration config = DynamicWindowApproach::DEFAULT_CONFIGURATION;
config.maxSpeed = 1.0f;
config.robot_type = RobotType::RECTANGLE;
DynamicWindowApproach dwa(config);

// Set up inputs
Pose  current_pose     = {0.0, 0.0, 0.0};   // x, y, yaw [m, m, rad]
Velocity current_vel   = {0.0f, 0.0f};       // linear [m/s], angular [rad/s]
Point goal             = {5.0f, 5.0f};       // x, y [m]
PointCloud obstacles;
obstacles.points.push_back({1.5f, 1.5f});    // obstacle positions or LiDAR points

// Run one planning step
Velocity cmd = dwa.planning(current_pose, current_vel, goal, obstacles);
// cmd.linearVelocity  → commanded linear  velocity [m/s]
// cmd.angularVelocity → commanded angular velocity [rad/s]
```

Call `planning()` at each control loop iteration with the robot's current state.

## Configuration

All parameters are in `DynamicWindowApproach::Configuration`. Default values:

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `minSpeed` | `-0.5` | m/s | Minimum linear velocity (allows reversing) |
| `maxSpeed` | `0.6` | m/s | Maximum linear velocity |
| `maxYawrate` | `3.2` | rad/s | Maximum angular velocity |
| `maxAccel` | `10.0` | m/s² | Maximum linear acceleration |
| `maxdYawrate` | `40.0` | rad/s² | Maximum angular acceleration |
| `velocityResolution` | `0.1` | m/s | Linear velocity sampling resolution |
| `yawrateResolution` | `0.1` | rad/s | Angular velocity sampling resolution |
| `dt` | `0.05` | s | Simulation time step |
| `safety_distance` | `0.5` | m | Minimum clearance to maintain from obstacles |
| `predictTime` | `1.0` | s | Prediction horizon length |
| `robot_type` | `CIRCLE` | — | `RobotType::CIRCLE` or `RobotType::RECTANGLE` |
| `robot_radius` | `0.8` | m | Robot radius (circle mode) |
| `robot_length` | `1.0` | m | Robot length (rectangle mode) |
| `robot_width` | `0.9` | m | Robot width (rectangle mode) |
| `heading_weight` | `0.15` | — | Weight for heading cost |
| `velocity_weight` | `1.0` | — | Weight for velocity cost |
| `clearance_weight` | `1.0` | — | Weight for clearance cost |

## Tuning Notes

**Prediction horizon (`predictTime`)** is the most critical parameter. The heading cost is evaluated only at the **last pose** of the predicted trajectory, so the horizon must be long enough for the robot to clear obstacles and be pointing toward the goal by the end of the simulation. A horizon that is too short can cause the robot to get stuck behind obstacles.

**Cost weights** control the trade-off between speed, safety, and goal alignment. Increase `clearance_weight` for more conservative behavior near obstacles; increase `heading_weight` to prioritize pointing toward the goal.

## License

See [LICENSE](LICENSE).
