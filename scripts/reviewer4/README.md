# Reviewer 4: trajectory feasibility, planner comparison, computation time

`run_r4_sims.sh` runs the simulations, measures the planner timings and writes three figures to
`scripts/Figures/reviewer4/` (SVG and CSV with the plotted numbers):

```bash
bash $(rospack find multi_quadcopter_control)/scripts/reviewer4/run_r4_sims.sh
```

Only FigR3 (computation time on this computer, no ROS needed; g++, Eigen3 and python3 with numpy,
pandas and matplotlib are enough):

```bash
bash scripts/reviewer4/measure_timing.sh
```

It measures 200 repetitions (pass another number as the first argument), writes the CPU, core count,
memory, OS and compiler to `log/r4_traj_eval/hardware.txt` and draws
`scripts/Figures/reviewer4/FigR3_computation_time.svg` with the CPU name in the figure. Close other heavy
programs during the measurement and keep a laptop on mains power.

| Figure | Content | Runs |
|---|---|---|
| FigR1_feasibility | ‖f_thrust‖ / f_thrust,max, roll and pitch of all UAVs against the limits (f_thrust,max, ±0.6 rad); peak values when all segment durations are scaled by μ | `log/r4_scale_<μ>`: PD+FF, downwash on, hybrid trajectory, `trajectory_time_scale:=μ` (μ = 1 is the paper run) |
| FigR2_segment_comparison | Time histories for one UAV (Q2 by default, `R4_DRONE`): position error, reference acceleration with the largest jump, jerk with ∫‖j‖²dt, snap (impulses at jerk jumps marked with * in the summary), tilt, thrust deviation and torque; a summary value per planner in each panel | `log/r4_<planner>`: PD+FF, downwash on; hybrid (paper run) or every segment planned with one method, re-planning at the same instants |
| FigR3_computation_time | Computation time of every 50 ms tick over the mission: median of 200 runs for the four planners, median and maximum for the hybrid planner; re-planning ticks marked | `log/r4_traj_eval/<planner>/timing.csv` |

Notes:

- The limits are the ones in the code: `angle_limit = 0.6` rad clamps the roll and pitch references
  (`src/sim/controllers.cpp`); f_thrust,max is `max_thrust` in `config/quadcopter_params_*.yaml`
  (15.905 N for Q1, 17.848 N for Q2–Q5), the limit that clamps the total thrust command.
- Acceleration, jerk and snap are the analytic derivatives of the planned polynomials, sampled at 20 Hz.
  Jumps are one-sided limits at the polynomial knots; at a re-planning instant the old plan's value is
  taken from `replan.csv` (the new plan overwrites it in the same tick). Jumps below 0.05 are
  treated as numerical noise.
- In the single-method runs each trajectory keeps the re-planning segment of its hybrid method
  (`switch_segment` in the trajectory yaml; cubic spline 3, the others 5), so all four runs re-plan at
  the same instants and fly the same waypoints over the same total time.
- `traj_eval.cpp` runs `TrajectorySequencer` (the same planner code as the simulator) without ROS in a
  single thread, 200 times per UAV. `hardware.txt` records the CPU, compiler and flags; the figure
  prints its first line.
- `make_specs.py` writes the traj_eval inputs and copies of `config/all_trajectories.yaml` with every
  `method:` replaced and `switch_segment` added; `run_r4_sims.sh` passes them via the `trajectories_file`
  argument of `launch/sequential_sim.launch`.
