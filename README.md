# robotics_control_challenge

ROS 2 package (Python, `ament_python`) developed for **Robotics Control Challenge 4.1** of the *TE3001B – Fundamentación de Robótica* course (Tecnológico de Monterrey, Campus Monterrey). It implements a trajectory-tracking system for the **xArm Lite 6**, comparing two controllers — **Cartesian PD** and **Computed Torque Control (CTC)** — while injecting artificial perturbations (sinusoidal and Gaussian) to evaluate tracking robustness and stability. Inverse kinematics is solved with **MoveIt 2** (`pymoveit2`).

**Team: RJ CREW**

| Student | ID |
|---|---|
| Jose Eduardo Sanchez Martinez | A01738476 |
| Josue Ureña Valencia | A01738940 |
| Rafael André Gamiz Salazar | A00838280 |
| César Arellano Arellano | A00839373 |

Instructor: Nezih Nieto Gutiérrez

## Package contents

```
robotics_control_challenge/
├── xarm_perturbations_ctc/
│   ├── moveit_position.py       # Node: reads waypoints.csv, solves IK via MoveIt2, publishes desired pose/joints
│   ├── pd_ctc_controller.py     # Node: dual-mode controller (Cartesian PD / joint-space CTC), keyboard control
│   ├── perturbation_injector.py # Node: injects sine/gaussian perturbations on /servo_server/delta_joint_cmds
│   ├── waypoints.csv            # Cartesian waypoints (x,y,z) defining the trajectory
│   └── __init__.py
├── resource/xarm_perturbations_ctc
├── test/
├── package.xml
├── setup.py / setup.cfg
├── Graficas_generador/                       # Plotting scripts for the logged tracking data
├── PD_CTC_Controller_csv/                    # Example tracking CSVs (PD/CTC, with/without perturbations)
└── Robotics Control Challenge 4.1_Equipo5.pdf # Full lab report
```

## How it works

The system is made up of three ROS 2 nodes and a waypoint file, described in detail in `Robotics Control Challenge 4.1_Equipo5.pdf`:

```
waypoints.csv
      │
      ▼
moveit_position
(solves IK, publishes desired pose)
      │
      ▼
pd_ctc_controller
(PD / CTC controller)
      │
      ▼
MoveIt Servo
(robot execution layer)
      ▲
      │
perturbation_injector
(injects perturbations)
```

1. **Waypoint generator + inverse kinematics** — `moveit_position` reads `waypoints.csv`, solves IK for each point via `pymoveit2`, and publishes both the Cartesian target (`/posicion_deseada`, `geometry_msgs/Point`) and the joint target (`/q_deseada`, `sensor_msgs/JointState`).

   The trajectory traces a square motion in the XY plane with vertical movement in Z, simulating a drill that has to make 4 holes, forming a 0.2 m × 0.2 m square. Work area:

   ```
         y
         ↑
   0.2 ──●────────●
        │        │
        │        │
   0.0 ──●────────●
         0.15    0.35 → x
   ```

   Each waypoint is processed — publishing both the Cartesian and joint targets — every 10 seconds, which also serves as the dwell period used to evaluate steady-state error.

2. **PD / CTC controller** — `pd_ctc_controller` implements two selectable control laws:
   - **Cartesian PD**: `v = Kp*e + Kd*de`, tracking `/posicion_deseada` with the end-effector pose read from TF (`link_base → link_eef`), published as `geometry_msgs/TwistStamped` on `/servo_server/delta_twist_cmds`.
   - **Computed Torque Control (CTC)**: nonlinear joint-space control `τ = M(q)v + C(q,q̇) + G(q)`, with `v = q̈d + Kd(q̇d − q̇) + Kp(qd − q)`, tracking `/q_deseada` along a quintic joint trajectory generated between consecutive waypoints. Robot dynamics are approximated with `inertia_matrix(q)`, `coriolis_torque(q, qd)` and `gravity_torque(q)`, and commands are published as `control_msgs/JointJog` on `/servo_server/delta_joint_cmds`.

   Toggle between modes with `c` (CTC ⇄ PD) and pause/resume with `p`, from the terminal running `pd_ctc_controller`. **Both shortcuts require an X11 session** — see [Requirements](#requirements).

3. **Perturbation injection** — `perturbation_injector` independently publishes a sine or Gaussian perturbation on the `/servo_server/delta_joint_cmds` topic.

`pd_ctc_controller` also logs to `tracking_data_lite6.csv` the full run history — `time`, `x_d/y_d/z_d`, `x/y/z`, `vx_cmd/vy_cmd/vz_cmd`, `q1..q6`, `qd1..qd6`, `qref1..qref6`, `cmd1..cmd6`, `mode`, `perturbation` — used for RMSE analysis and controller comparison. See `Graficas_generador/` for the plotting scripts and `PD_CTC_Controller_csv/` for example runs.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- [`xarm_ros2`](https://github.com/xArm-Developer/xarm_ros2) (UFACTORY), with xArm Lite 6 support and the `xarm_moveit_config` / `xarm_moveit_servo` packages
- `pymoveit2` (used by `moveit_position` to solve the IK)
- Python dependencies: `numpy`, `pynput` (`sudo apt install python3-pynput` or via `rosdep`)

**Keyboard control requires X11:** `pd_ctc_controller` uses `pynput` to capture `p` (pause/resume) and `c` (toggle CTC/PD) globally from the terminal. `pynput` needs an **X11** session to hook the keyboard; under **Wayland** the keys aren't intercepted at all and just get typed as plain text into the terminal, with no effect on the node. Check your session with:

```bash
echo $XDG_SESSION_TYPE
```

If it prints `wayland`, log out and pick **"Ubuntu on Xorg"** at the login screen (instead of the default session) before logging back in.

## Workspace installation / setup

Follow the workspace installation/setup guide (cloning `xarm_ros2`, `rosdep`, `colcon build`, etc.) described in [lite6_demo_moveit/README.md](https://github.com/Jose05M/lite6_demo_moveit/blob/main/README.md).

Once the workspace is set up, place (or clone) this `robotics_control_challenge` repository inside `~/xarm_ws/src/` and build it:

```bash
cd ~/xarm_ws/
colcon build --packages-select xarm_perturbations_ctc
source install/setup.bash
```

## How to launch it

### 1. Bring up MoveIt + Servo

**Option A — physical robot:**

Make sure the xArm Lite 6 is connected and reachable, then run its configuration in two separate terminals:

```bash
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py robot_ip:=192.168.1.179
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.179
```

Adjust `robot_ip` to the arm controller's actual IP, and always keep the emergency stop button within reach.

**Option B — simulation (no physical robot):**

If the arm isn't available, use the simulated/fake-hardware equivalent instead (no `robot_ip` needed):

```bash
ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py
ros2 launch xarm_moveit_servo lite6_moveit_servo_fake.launch.py
```

This opens RViz with the robot model ready to receive MoveIt Servo commands. It's the recommended way to validate that the node pipeline works end-to-end before testing on the real robot.

Either option leaves running:

- the robot model
- the TF tree
- MoveIt Servo
- the inverse kinematics solver

Then move the robot to the predefined starting pose, so every test run starts from identical initial conditions and to avoid MoveIt IK solver errors:

| Joint | Value (°) |
|---|---|
| joint1 | 44 |
| joint2 | 24 |
| joint3 | 78 |
| joint4 | -2 |
| joint5 | 55 |
| joint6 | 45 |

In RViz: **MotionPlanning** panel → **Joints** tab, and move each joint slider to the values above.

### 2. Run the controller

In another terminal:

```bash
ros2 run xarm_perturbations_ctc pd_ctc_controller
```

The controller:

- reads the desired Cartesian positions
- reads the desired joint states
- computes the control law (PD/CTC)
- sends commands to the robot through MoveIt Servo

> ⚠️ **CTC-in-simulation note:** the dynamics model used by the CTC controller (`inertia_matrix`, `coriolis_torque`, `gravity_torque` in `pd_ctc_controller.py`) is an approximation tuned empirically against the **real robot's** observed behavior, not an exact physical model. Simulated (`fake`) hardware has no real inertia or friction — it applies velocity/torque commands almost instantly, without the damping the real robot provides. Because of this, CTC can look more abrupt in simulation (fast initial jumps) or fail to fully converge to the waypoints if the initial error saturates `tau_limit`.

> **Switching between PD and CTC:** press `c` on that terminal to toggle between controllers, and `p` to pause/resume (requires X11 — see [Requirements](#requirements)).

### 3. Run the waypoint generator

In another terminal:

```bash
ros2 run xarm_perturbations_ctc moveit_position
```

Publishes:

```
/posicion_deseada
/q_deseada
```

### 4. (Optional) Inject perturbations

To test the controllers' behavior under perturbations while the trajectory is running, in a third terminal:

**Sinusoidal** — simulates periodic vibration:

```
v = A sin(2πft)
```

Tunable via `sine_freq_hz`, `sine_amp_joint`, `sine_axis`:

```bash
ros2 run xarm_perturbations_ctc perturbation_injector \
--ros-args \
-p mode:=sine \
-p sine_freq_hz:=8 \
-p sine_amp_joint:=0.1 \
-p sine_axis:=2
```

**Gaussian noise** — random perturbation:

```
v ~ N(0, σ)
```

```bash
ros2 run xarm_perturbations_ctc perturbation_injector \
--ros-args \
-p mode:=gaussian \
-p noise_std_joint:=0.0001
```

## Evaluation metric

Performance is evaluated using **Root Mean Square Error (RMSE)**:

```
RMSE = sqrt( (1/N) Σ (x_d − x)^2 )
```

## Known notes / limitations

- **Keyboard control and Wayland:** `pd_ctc_controller` depends on `pynput` to capture `p`/`c` from the terminal. Under a Wayland session this doesn't work at all (keys are typed as plain text instead of switching the node's mode) — use an X11 session instead, see [Requirements](#requirements).
- **CTC tuned for the real robot:** `inertia_matrix`, `coriolis_torque` and `gravity_torque` aren't an exact physical model of the Lite 6, they're an approximation calibrated against the real robot's observed behavior. On simulated (`fake`) hardware — with no real inertia — CTC can saturate `tau_limit` and converge less cleanly; see the note in step 2 of [How to launch it](#how-to-launch-it). PD mode doesn't depend on this model and is more reliable for validating the pipeline in simulation.
- **Perturbation and controller share a topic:** `perturbation_injector` and `pd_ctc_controller` both publish independently to `/servo_server/delta_joint_cmds` — the commands aren't summed, whichever arrives last wins. On the real robot this is visible as trembling while the perturbation is active, which is the intended way to observe the disturbance. In `fake` simulation, the lack of real inertia/damping can make this look calmer during the perturbation and produce a comparatively abrupt correction once it stops — a simulation artifact, not a change in the underlying control design.
- **`waypoints.csv` path:** `moveit_position` resolves it relative to its own file location (`os.path.dirname(__file__)`), so it always reads the `waypoints.csv` packaged alongside the node, regardless of the directory `ros2 run` is launched from.

## Technologies used

ROS 2
MoveIt 2
MoveIt Servo
Python
NumPy
pymoveit2

## Project goal

Evaluate the robustness of robotic controllers under perturbations by comparing:

- Cartesian PD control
- Computed Torque Control (CTC)

on trajectory-tracking tasks with the xArm Lite 6.
