# Paper figures (SVG)

Figures of the manuscript and the supplementary material, generated from the simulation runs of
`scripts/run_paper_sims.sh` (current code and parameters, downwash from the instantaneous thrust of
the source drone, `noise_seed` 42) with the same layout,
colours and line widths as `scripts/main.m`. Text is converted to outlines, so the files look the
same on every computer.

| File | Figure |
|---|---|
| Fig1.svg | Positions, with zoom insets |
| Fig2.svg | Position error: PD, PD+NDOB, PD+FF |
| Fig3.svg | Velocities |
| Fig4.svg | Accelerations |
| Fig5.svg | Euler angles |
| Fig6.svg | Downwash forces and NDOB estimate |
| Fig7.svg | Thrust |
| Fig8.svg | Ablation: PD+FF, PD, SE(3)+FF, SE(3) (Q2, Q3) |
| Fig9.svg | NDOB estimate (z) and compensation comparison |
| FigF1_formation_error.svg | Formation indices |
| FigS2.svg | Downwash force and velocity field |
| FigS5.svg | Hybrid trajectory of Q1 |
| FigS6.svg | Orientation error, beta comparison |
| FigS7.svg | Fit of Eq. (8) to the measured downwash data (`scripts/fig_S7_S8.py`) |
| FigS8.svg | Wake overlap limit of two side-by-side sources (`scripts/fig_S7_S8.py`) |
| FigR1_feasibility.svg | Reviewer 4: thrust and attitude against the limits, time-scale sweep |
| FigR2_segment_comparison.svg | Reviewer 4: hybrid vs single-method planners over time, Q2 (acceleration, jerk, snap impulses, error, tilt, thrust, torque) |
| FigR3_computation_time.svg | Reviewer 4: measured computation time of every 50 ms tick, regeneration marked (measured with `scripts/reviewer4/measure_timing.sh` on an Intel Core i5-10300H, Ubuntu 20.04, 200 repetitions) |

The FigR* files come from `scripts/reviewer4/run_r4_sims.sh`. These files are not regenerated automatically; `scripts/main.m` writes its own SVGs to
`scripts/Figures/<main_ctrl>/`.
