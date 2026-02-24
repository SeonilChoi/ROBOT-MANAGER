# ROBOT MANAGER

The Robot Manager is a software framework for managing, planning, and controlling robot motion.
It provides a modular architecture that separates robot management, motion planning, scheduling, and control.

---

## Core Classes

### Planner

The `Planner` class generates motions or paths by computing trajectories that connect a start point and a target point.

#### Functions

- `plan(start, target, obstacle_state)`: Generates a trajectory connecting start and target. Returns true if a path was found. The trajectory is stored and can be read via `get_trajectory()`.
- `reset()`: Resets the start point, target point, and any stored trajectory.
- `is_valid(state, obstacle_state)`: Checks if the robot at the given state collides with obstacles or own links.
- `get_trajectory()`: Returns the last planned trajectory (valid only after a successful `plan()`).

#### Planner implementations

- `RrtPlanner`: RRT-based motion planner.
- `PrmPlanner`: PRM-based motion planner.
- `StompPlanner`: STOMP-based motion planner.

---