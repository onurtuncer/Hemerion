## Step 7 batch: case 12, W20 8 m/s, Aetherion 0.18.2 (49e75b814534e6d58c9216a52df2fb7151dfd2f9), Linux

8 run(s): 8 passed every check, 8 have an EKFReplay summary.
Hemerion: `1927309f7f22a4ab9ecd280a3ec762f7119d40fb`.
Aetherion: `49e75b814534e6d58c9216a52df2fb7151dfd2f9`.

Second-half figures as EKFReplay prints them (ANEES of the nav core, /9; NIS per dimension).

| run | case | seed | turbulence_w20_mps | turbulence_seed | stop_s | passed | duration_s | imu_steps | gnss_pos_updates | gnss_vel_updates | baro_updates | fixes_skipped | anees_nav | nis_gnss_pos | nis_gnss_vel | nis_baro | rms_position_m | rms_velocity_mps | rms_tilt_deg | rms_yaw_deg | cosim_wall_s | convention |
|---|---|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---|---|
| step7-case12-seed-1 | 12 | 1 | 8 | 1 | 200 | yes | 199.99 | 19999 | 0 | 0 | 10000 | 2000 | 157968 |  |  | 21.6247 | 14695 | 74.2631 | 10.1517 | 33.9171 | 211.4 | right-invariant |
| step7-case12-seed-7 | 12 | 7 | 8 | 7 | 200 | yes | 199.99 | 19999 | 0 | 0 | 9999 | 2000 | 59.3695 |  |  | 0.441645 | 2170.66 | 29.6662 | 1.34801 | 13.5755 | 200.1 | right-invariant |
| step7-case12-seed-8 | 12 | 8 | 8 | 8 | 200 | yes | 199.99 | 19999 | 0 | 0 | 9999 | 2000 | 55.6961 |  |  | 0.453306 | 2414.12 | 31.5644 | 1.18456 | 20.7022 | 200.1 | right-invariant |
| step7-case12-seed-11 | 12 | 11 | 8 | 11 | 200 | yes | 199.99 | 19999 | 0 | 0 | 9999 | 2000 | 198.031 |  |  | 0.448399 | 1971.41 | 31.9686 | 1.5886 | 8.66798 | 200.1 | right-invariant |
| step7-case12-seed-13 | 12 | 13 | 8 | 13 | 200 | yes | 199.99 | 19999 | 0 | 0 | 9999 | 2000 | 29.2064 |  |  | 0.449988 | 2006.98 | 25.5057 | 1.02604 | 9.12061 | 200.1 | right-invariant |
| step7-case12-seed-14 | 12 | 14 | 8 | 14 | 200 | yes | 200 | 20000 | 0 | 0 | 10000 | 2000 | 3133.87 |  |  | 0.467349 | 5292.1 | 71.0192 | 3.14636 | 60.8601 | 275.8 | right-invariant |
| step7-case12-seed-17 | 12 | 17 | 8 | 17 | 200 | yes | 199.99 | 19999 | 0 | 0 | 9999 | 2000 | 948.588 |  |  | 0.433309 | 3889.88 | 58.2547 | 2.27561 | 8.0506 | 200.1 | right-invariant |
| step7-case12-seed-20 | 12 | 20 | 8 | 20 | 200 | yes | 199.99 | 19999 | 0 | 0 | 9999 | 2000 | 3.28205e+06 |  |  | 523.408 | 31276.7 | 467.264 | 50.2465 | 114.086 | 200.1 | right-invariant |

Over the runs that report each figure (finite values only; std is the sample one):

| figure | n | mean | std | min | max |
|---|---:|---:|---:|---:|---:|
| duration_s | 8 | 199.991 | 0.00353553 | 199.99 | 200 |
| imu_steps | 8 | 19999.1 | 0.353553 | 19999 | 20000 |
| gnss_pos_updates | 8 | 0 | 0 | 0 | 0 |
| gnss_vel_updates | 8 | 0 | 0 | 0 | 0 |
| baro_updates | 8 | 9999.25 | 0.46291 | 9999 | 10000 |
| fixes_skipped | 8 | 2000 | 0 | 2000 | 2000 |
| anees_nav | 8 | 430555 | 1.15349e+06 | 29.2064 | 3.28205e+06 |
| nis_baro | 8 | 68.4658 | 183.974 | 0.433309 | 523.408 |
| rms_position_m | 8 | 7964.61 | 10339.2 | 1971.41 | 31276.7 |
| rms_velocity_mps | 8 | 98.6882 | 150.201 | 25.5057 | 467.264 |
| rms_tilt_deg | 8 | 8.87092 | 16.9878 | 1.02604 | 50.2465 |
| rms_yaw_deg | 8 | 33.6225 | 37.1525 | 8.0506 | 114.086 |
