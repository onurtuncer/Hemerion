## Step 7 batch: case 12, W20 8 m/s, Aetherion 0.18.2 (49e75b814534e6d58c9216a52df2fb7151dfd2f9), Linux

12 run(s): 12 passed every check, 12 have an EKFReplay summary.
Hemerion: `85d305224d10ae7a9bc59a2c4bd5d4702a2a3794`.
Aetherion: `49e75b814534e6d58c9216a52df2fb7151dfd2f9`.

Second-half figures as EKFReplay prints them (ANEES of the nav core, /9; NIS per dimension).

| run | case | seed | turbulence_w20_mps | turbulence_seed | stop_s | passed | duration_s | imu_steps | gnss_pos_updates | gnss_vel_updates | baro_updates | fixes_skipped | anees_nav | nis_gnss_pos | nis_gnss_vel | nis_baro | rms_position_m | rms_velocity_mps | rms_tilt_deg | rms_yaw_deg | cosim_wall_s | convention |
|---|---|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---|---|
| step7-case12-seed-2 | 12 | 2 | 8 | 2 | 200 | yes | 200 | 20000 | 0 | 0 | 9999 | 2000 | 276.917 |  |  | 0.443975 | 3827.74 | 51.0319 | 1.70251 | 18.8762 | 200.1 | right-invariant |
| step7-case12-seed-3 | 12 | 3 | 8 | 3 | 200 | yes | 199.99 | 19999 | 0 | 0 | 9999 | 2000 | 2.8069 |  |  | 0.453296 | 998.657 | 7.89138 | 0.445625 | 4.29626 | 200.1 | right-invariant |
| step7-case12-seed-4 | 12 | 4 | 8 | 4 | 200 | yes | 199.99 | 19999 | 0 | 0 | 9999 | 2000 | 1858.58 |  |  | 0.441386 | 5602.98 | 78.6592 | 2.98794 | 19.7435 | 219.4 | right-invariant |
| step7-case12-seed-5 | 12 | 5 | 8 | 5 | 200 | yes | 199.99 | 19999 | 0 | 0 | 9999 | 2000 | 2.01981e+07 |  |  | 5690.86 | 107572 | 1251.38 | 70.6137 | 104.859 | 218.7 | right-invariant |
| step7-case12-seed-6 | 12 | 6 | 8 | 6 | 200 | yes | 199.99 | 19999 | 0 | 0 | 9999 | 2000 | 893.322 |  |  | 0.450542 | 2210.87 | 42.3148 | 2.57286 | 1.04156 | 217.7 | right-invariant |
| step7-case12-seed-9 | 12 | 9 | 8 | 9 | 200 | yes | 199.99 | 19999 | 0 | 0 | 10000 | 2000 | 1.30633e+06 |  |  | 5.68611 | 32663.2 | 292.886 | 19.6605 | 107.628 | 274.4 | right-invariant |
| step7-case12-seed-10 | 12 | 10 | 8 | 10 | 200 | yes | 199.99 | 19999 | 0 | 0 | 10000 | 2000 | 1.09486e+08 |  |  | 1342.63 | 481773 | 3594.68 | 83.8473 | 34.3366 | 272.4 | right-invariant |
| step7-case12-seed-12 | 12 | 12 | 8 | 12 | 200 | yes | 199.99 | 19999 | 0 | 0 | 10000 | 2000 | 160.19 |  |  | 0.428017 | 896.139 | 22.5063 | 1.93262 | 10.9635 | 219.1 | right-invariant |
| step7-case12-seed-15 | 12 | 15 | 8 | 15 | 200 | yes | 199.99 | 19999 | 0 | 0 | 9999 | 2000 | 6592.64 |  |  | 0.50863 | 6877.7 | 53.082 | 3.28778 | 40.5218 | 219.6 | right-invariant |
| step7-case12-seed-16 | 12 | 16 | 8 | 16 | 200 | yes | 199.99 | 19999 | 0 | 0 | 10000 | 2000 | 670.686 |  |  | 0.432065 | 5959.43 | 75.8058 | 2.94199 | 70.5723 | 272.4 | right-invariant |
| step7-case12-seed-18 | 12 | 18 | 8 | 18 | 200 | yes | 199.99 | 19999 | 0 | 0 | 9999 | 2000 | 684.221 |  |  | 0.42749 | 3152.4 | 49.4933 | 2.60874 | 8.8763 | 200.1 | right-invariant |
| step7-case12-seed-19 | 12 | 19 | 8 | 19 | 200 | yes | 199.99 | 19999 | 0 | 0 | 9999 | 2000 | 283618 |  |  | 8.06796 | 16964.9 | 225.075 | 10.0154 | 106.527 | 200.1 | right-invariant |

Over the runs that report each figure (finite values only; std is the sample one):

| figure | n | mean | std | min | max |
|---|---:|---:|---:|---:|---:|
| duration_s | 12 | 199.991 | 0.00288675 | 199.99 | 200 |
| imu_steps | 12 | 19999.1 | 0.288675 | 19999 | 20000 |
| gnss_pos_updates | 12 | 0 | 0 | 0 | 0 |
| gnss_vel_updates | 12 | 0 | 0 | 0 | 0 |
| baro_updates | 12 | 9999.33 | 0.492366 | 9999 | 10000 |
| fixes_skipped | 12 | 2000 | 0 | 2000 | 2000 |
| anees_nav | 12 | 1.09404e+07 | 3.15661e+07 | 2.8069 | 1.09486e+08 |
| nis_baro | 12 | 587.569 | 1652.71 | 0.42749 | 5690.86 |
| rms_position_m | 12 | 55708.3 | 137489 | 896.139 | 481773 |
| rms_velocity_mps | 12 | 478.734 | 1039.93 | 7.89138 | 3594.68 |
| rms_tilt_deg | 12 | 16.8847 | 28.8113 | 0.445625 | 83.8473 |
| rms_yaw_deg | 12 | 44.0202 | 42.039 | 1.04156 | 107.628 |
