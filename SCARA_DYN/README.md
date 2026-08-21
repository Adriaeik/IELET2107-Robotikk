# SCARA dynamics example

[Back to the repository overview](../README.md)

This folder contains a small closed-loop simulation of a three-DOF SCARA
robot. Two revolute joints position the arm in the horizontal plane, while a
prismatic third joint moves the end effector vertically.

## Run the example

From the repository root in MATLAB:

```matlab
addpath("SCARA_DYN")
open("SCARA_DYN/example.mlx")
```

Run the complete live script. It integrates the model, animates the resulting
motion, and prints the final end-effector position and roll-pitch-yaw
orientation.

The animation requires Robotics System Toolbox. The `ode45` integration and
the dynamics function use base MATLAB.

## What the live script does

The simulated state is

```text
x = [q; Dq]
q = [theta1; theta2; d3]
```

where `theta1` and `theta2` are in radians and `d3` is in metres.
The example then:

1. Defines the initial joint positions and velocities.
2. Defines a desired joint state.
3. Creates diagonal proportional and derivative gain matrices.
4. Adds `[0; 0; 5]` as gravity compensation on the prismatic axis.
5. Integrates `SCARA_dynamics` from 0 to 3 seconds with `ode45`.
6. Passes the position part of the solution to `SCARA_animation`.

The controller used by the example is

```text
u = Kp * (q_des - q) + Kd * (Dq_des - Dq) + [0; 0; 5]
```

The animation constructs a `rigidBodyTree`, shows every tenth solver sample,
and calculates the final pose of `link3`. Orientation is extracted with a
ZYX Euler decomposition and printed in roll, pitch, yaw order.

## Files

| File | Role |
| --- | --- |
| [`example.mlx`](example.mlx) | Initial conditions, target, controller, integration, and animation call |
| [`SCARA_dynamics.m`](SCARA_dynamics.m) | State derivative using the hard-coded inertia, velocity-coupling, and gravity terms |
| [`SCARA_animation.m`](SCARA_animation.m) | Robot geometry, animation, and final end-effector pose |

The easiest parameters to experiment with are `q0`, `q_des`, `Kp`,
`Kd`, and `tspan` in the live script. Robot geometry, plot limits, and the
dynamic model are currently hard-coded in the two function files.

## Model note

`SCARA_dynamics.m` currently computes
`DDq = M\(-C*q - g + u)`. A common manipulator model instead uses the
velocity vector in this term, `C*Dq`. The README describes the implementation
as it exists; confirm whether `C*q` is intentional before using the result as
a quantitative dynamics model.
