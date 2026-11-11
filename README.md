# multi_quadcopter_control

ROS (C++) simulator for heterogeneous multirotor swarms flying in close proximity, used in
*"Disturbance-Aware Hybrid Trajectory Generation and Control for Multirotor Swarms"*.

It includes:

- quadcopter rigid-body dynamics with first-order actuator lags and translational drag,
- an analytical propeller downwash model with pink-noise (1/f) turbulence,
- a hybrid trajectory generator (cubic spline, minimum jerk and minimum snap segments) with
  continuous re-planning when a new waypoint set arrives, and a trajectory-aligned, low-pass
  filtered yaw angle,
- a hierarchical position / attitude controller with three mid-level options
  (PID + downwash feedforward, PID without feedforward, PID + nonlinear disturbance observer),
- MATLAB scripts that produce the figures and tables of the manuscript.

## Requirements

- ROS Noetic (`roscpp`, `rviz`, `tf2_ros`, `xacro`, `mavros_msgs`, ...; see `package.xml`)
- Eigen 3, Boost (odeint)
- MATLAB R2020a or newer for the plotting scripts (`readtable`, `tiledlayout`, `exportgraphics`)

## Build

```bash
cd ~/catkin_ws/src
git clone -b ros https://github.com/ahmetsduru/multi_quadcopter_control.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Running the simulation

```bash
roslaunch multi_quadcopter_control sequential_sim.launch
```

`sequential_sim.launch` starts a single node (`multi_sequential_sim`) that simulates the whole
swarm, plus RViz. Every time step (`dt` = 2 ms) runs the components in a fixed order:

1. trajectory generator (20 Hz)
2. downwash / turbulence disturbance (500 Hz)
3. mid-level (position) controller (`mid_level_controller/dt`, 100 Hz)
4. low-level (attitude) controller (`low_level_controller/dt`, 250 Hz)
5. logging
6. dynamics integration (Runge-Kutta Cash-Karp 5(4), 500 Hz)

Time is a step counter (`t = k * dt`), not the wall clock, so **a run is deterministic**: running it
twice gives bit-identical log files on the same machine and build, independent of CPU load or
`real_time_factor`. The turbulence noise is seeded with `noise_seed`.

### Controller options

Select the mid-level controller with the `controller` argument. All three use the same
hierarchical PID (same gains, reference-angle computation, thrust projection and limits); they
differ only in the disturbance compensation term `f_comp` in

```
f_des = Kp e_p + Ki ∫e_p + Kd e_v + m a_des + m g - f_comp
```

| `controller` | `f_comp` | Description |
|---|---|---|
| `pid_ff` (default) | `f_dw` | Modelled downwash as feedforward (proposed controller) |
| `pid` | `0` | No disturbance compensation (ablation) |
| `ndob` | `Δf` | Disturbance estimated by a nonlinear disturbance observer |

`controller:=se3_ff` runs the geometric tracking controller on SE(3) of Lee, Leok and McClamroch
(CDC 2010): thrust `f = F_des · R e3`, desired attitude from `F_des` and the yaw reference, desired
angular velocity from `dF_des` (using the measured acceleration), and the attitude law
`M = -k_R e_R - k_Ω e_Ω + Ω×JΩ - J(Ω̂ Rᵀ R_d Ω_d - Rᵀ R_d Ω̇_d)` (`Ω̇_d` by finite difference,
`ψ̇_d = 0`). It uses the same downwash feedforward `f_dw` as `pid_ff`; `controller:=se3` is the same
controller without feedforward. All controllers share the same vehicle model (mass, inertia, actuator
lags, drag), rates and thrust limits; only the controller gains differ. The SE(3) gains are in the
`se3_mid_level_controller` / `se3_low_level_controller` sections of the parameter files.

Gain tuning (PID and SE(3), same procedure): the position gains `(kp_xy, kd_xy)` were selected on a
common grid as the set with the lowest mean RMSE over 8 turbulence seeds among the sets that stay
stable, with no change in the vertical order of the UAVs, under all ±30 % perturbations of the x/y
and z gains. `kp_z = 15`, `kd_z = 5` for both; the attitude loops use the same gains
(`kp = 0.02`, `kd = 0.01`) and no integral terms. Result: PID `kp_xy = 4.0`, `kd_xy = 0.4`;
SE(3) `kp_xy = 5.0`, `kd_xy = 0.4`.

```bash
roslaunch multi_quadcopter_control sequential_sim.launch controller:=pid_ff
roslaunch multi_quadcopter_control sequential_sim.launch controller:=pid
roslaunch multi_quadcopter_control sequential_sim.launch controller:=ndob
roslaunch multi_quadcopter_control sequential_sim.launch controller:=se3
roslaunch multi_quadcopter_control sequential_sim.launch controller:=se3_ff
```

Without disturbance (`disturbance_enabled:=false`) `pid_ff` and `pid` give identical results.
`ndob` gives practically the same result as well (its observer, updated with an explicit Euler
step at the mid-level rate, produces only a very small estimate from unmodelled effects). At a low
mid-level rate this discretization error grows: at 20 Hz it increases the tracking error by about
10 % without disturbance.

- `pid_ff` uses only the deterministic downwash part of the model; the turbulence is not
  compensated and acts directly on the plant.
- `ndob` uses the observer (per axis, world frame, explicit Euler at the mid-level rate)

  ```
  Δf = z + L m v
  ż  = -L Δf - L (f_thrust - m g)
  ```

  where `f_thrust` is the previous thrust command passed through a first-order filter with the
  actuator time constant (`tau_thrust`) and projected with the measured attitude. The gains
  `L = diag(lv_x, lv_y, lv_z)` are read from `mid_level_controller/lv_x`, `lv_y`, `lv_z` in the
  drone's parameter file (defaults 2.0, 2.0, 2.5 rad/s). The estimate is logged (`ndob_f.*`) and
  published on `/droneN/ndob_estimated_disturbance`.

### Other launch arguments

| Argument | Default | Description |
|---|---|---|
| `rviz` | `true` | Start RViz |
| `real_time_factor` | `1.0` | Playback speed: 1 = real time, 2 = twice as fast, 0 = as fast as possible (results are identical) |
| `sim_duration` | `-1.0` | Stop after this many seconds; `-1` = when all trajectories end + `end_hold_time` |
| `end_hold_time` | `2.0` | Extra simulated time after the last trajectory ends (s) |
| `shutdown_on_finish` | `false` | Shut the node down when the simulation ends (otherwise the final state stays in RViz) |
| `disturbance_enabled` | `true` | Enable the downwash / turbulence disturbance |
| `noise_seed` | `42` | Turbulence noise seed; `-1` = random seed on every run |
| `max_vertical_range` | `-1.0` | Ignore downwash interactions farther than this vertical distance (m); `-1` = no limit |
| `psi_filter_beta` | `0.05` | Low-pass coefficient β of the trajectory-aligned yaw angle |
| `trajectory_time_scale` | `1.0` | Multiplies all waypoint times; < 1 gives faster, more aggressive trajectories (0.5 = twice as fast) |
| `log_dir` | `<package>/log` | Folder for the log files |

Downwash model parameters (`c_coup`, `r_lat`, `z0`, `sigma`, `turb_intensity`, `disturbance_rate`)
and visualization / logging rates (`viz_rate`, `path_rate`, `log_decimation`) are node parameters
set inside `sequential_sim.launch`.

Fast run without RViz:

```bash
roslaunch multi_quadcopter_control sequential_sim.launch rviz:=false real_time_factor:=0 shutdown_on_finish:=true
```

At the end the node prints the RMS and maximum position error of every drone.

## Configuration

| File | Content |
|---|---|
| `config/quadcopter_params_1.yaml`, `quadcopter_params_2.yaml` | Mass, inertia, drag, actuator time constants, controller gains and rates |
| `config/trajectory_manager_droneN.yaml` | Waypoint sets flown by drone N (in order) and yaw mode |
| `config/all_trajectories.yaml` | Waypoint sets: points, times and method (`cubic_spline`, `natural_cubic_spline`, `minimum_jerk`, `minimum_snap`) |
| `launch/sequential_sim.launch` | Drones, initial positions, simulation options and downwash parameters |

## Logs

One file per drone, `log/droneN_log.txt` (CSV, one row per time step), with the columns

```
time, a_pos.xyz, a_vel.xyz, a_acc.xyz, a_ang_vel.xyz, a_eul_ang.xyz,
r_pos.xyz, r_vel.xyz, r_acc.xyz, r_eul_ang.xyz, r_torq.xyz, dist_f.xyz, [ndob_f.xyz,] r_thr, r_psi
```

`a_` = actual, `r_` = reference / command, `dist_f` = disturbance force on the drone (world frame),
`ndob_f` = NDOB estimate (only with `controller:=ndob`). Positions include the drone's initial offset.

Each run also writes `run_info.txt` to the log folder (controller, seed, time scale, date and the
per-drone RMS printed in the terminal). `scripts/main.m` prints which run each log folder belongs to
and warns if the RMS computed from the logs differs from `run_info.txt` or the file is missing.

## Reproducing the figures

1. Run the simulations (each writes to its own folder). All three in one go:

   ```bash
   rosrun multi_quadcopter_control run_paper_sims.sh
   ```

   or one by one:

   ```bash
   roscd multi_quadcopter_control
   # proposed controller -> log/
   roslaunch multi_quadcopter_control sequential_sim.launch rviz:=false real_time_factor:=0 shutdown_on_finish:=true controller:=pid_ff
   # ablation, no feedforward (Fig. 8, Table 2) -> log_no_feedforward/
   roslaunch multi_quadcopter_control sequential_sim.launch rviz:=false real_time_factor:=0 shutdown_on_finish:=true controller:=pid log_dir:=$(pwd)/log_no_feedforward
   # yaw filter beta = 0.1 (Fig. S6) -> log_beta_0.1/
   roslaunch multi_quadcopter_control sequential_sim.launch rviz:=false real_time_factor:=0 shutdown_on_finish:=true psi_filter_beta:=0.1 log_dir:=$(pwd)/log_beta_0.1
   ```

2. Run `scripts/main.m` in MATLAB. It writes the EPS figures and `Table1.csv` / `Table2.csv` to
   `scripts/EPS_Results/` and all figures to `scripts/all_drones_plots_fixed.pdf`.
   Fig. 2 and Table 1 also need the logs of the SE(3) geometric controller: set `geo_log_dir` at
   the top of `main.m`. Figures whose input folder is missing are skipped with a warning.
   Line plots use every 5th sample (100 Hz) to keep the vector EPS files small; metrics and bar
   plots use the full 500 Hz data.

   `main.m` also computes formation indices (Fig. F1, `Table3_formation_<controller>.csv`) for neighbouring
   UAVs (desired distance below `neighbor_radius` = 3 m): the relative distance error
   `e_d,ij = ||p_i - p_j|| - ||p_i^ref - p_j^ref||` and the formation error
   `e_ij = (p_i - p_j) - (p_i^ref - p_j^ref)`, each as an RMS over the neighbour pairs (`E_d`, `E_F`).
   Table 3 is a Q1-Q5 matrix per controller, computed while the pair are neighbours:
   upper triangle = RMS formation error of the pair with its dominant direction, `(xy)` horizontal or
   `(z)` vertical; lower triangle = RMS relative distance error with its signed mean in parentheses,
   `+` the pair is farther apart than desired, `-` closer than desired. The RMS values carry no
   direction, the parenthesised entries do. Since `e_d,ij` is the component of `e_ij` along the line
   joining the pair, the lower entry never exceeds the upper one.

3. Fig. S2 (downwash force and velocity field) is computed from the model by
   `scripts/fig_S2_downwash_field.m`.

## Distributed (legacy) version

`launch/multiple_quadrotor.launch` runs the original multi-node version, in which every component
of every drone is a separate ROS node (`waypoint_server`, `trajectory_generator`, `mid_level_cont`,
`low_level_cont`, `eom`, `rviz_data_handler`, `relative_disturbance_generator`, `data_logger`).
The nodes run on independent wall-clock timers, so results vary slightly from run to run, and the
mid-level controller has no disturbance feedforward. Use `sequential_sim.launch` for reproducible results.

## Repository structure

```
config/        drone parameters and waypoint sets
include/       headers (sim/: ROS-free simulation core)
launch/        sequential_sim.launch (recommended), multiple_quadrotor.launch (legacy)
rviz/          RViz configuration
scripts/       MATLAB plotting scripts (main.m, fig_S2_downwash_field.m, legacy_plots.m)
src/sim/       simulation core: dynamics, controllers, trajectory sequencer, downwash model, logger
src/           sequential_sim_node.cpp and the legacy distributed nodes
urdf/, meshes/ quadcopter models for RViz
```
