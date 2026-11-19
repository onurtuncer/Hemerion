## Step 7 batch: case 11, W20 8 m/s, Aetherion 0.18.2 (49e75b814534e6d58c9216a52df2fb7151dfd2f9), Linux

1 run(s): 1 passed every check, 1 have an EKFReplay summary.
Hemerion: `1927309f7f22a4ab9ecd280a3ec762f7119d40fb`.
Aetherion: `49e75b814534e6d58c9216a52df2fb7151dfd2f9`.

Second-half figures as EKFReplay prints them (ANEES of the nav core, /9; NIS per dimension).

| run | case | seed | turbulence_w20_mps | turbulence_seed | stop_s | passed | duration_s | imu_steps | gnss_pos_updates | gnss_vel_updates | baro_updates | fixes_skipped | anees_nav | nis_gnss_pos | nis_gnss_vel | nis_baro | rms_position_m | rms_velocity_mps | rms_tilt_deg | rms_yaw_deg | cosim_wall_s | convention |
|---|---|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| step7-case11-seed-5 | 11 | 5 | 8 | 5 | 200 | yes | 199.99 | 19999 | 2000 | 2000 | 9999 | 0 | 0.813587 | 1.01693 | 1.04141 | 0.114368 | 0.224213 | 0.021467 | 0.116666 | 0.668225 | 205.9 | right-invariant |

Over the runs that report each figure (finite values only; std is the sample one):

| figure | n | mean | std | min | max |
|---|---:|---:|---:|---:|---:|
| duration_s | 1 | 199.99 | nan | 199.99 | 199.99 |
| imu_steps | 1 | 19999 | nan | 19999 | 19999 |
| gnss_pos_updates | 1 | 2000 | nan | 2000 | 2000 |
| gnss_vel_updates | 1 | 2000 | nan | 2000 | 2000 |
| baro_updates | 1 | 9999 | nan | 9999 | 9999 |
| fixes_skipped | 1 | 0 | nan | 0 | 0 |
| anees_nav | 1 | 0.813587 | nan | 0.813587 | 0.813587 |
| nis_gnss_pos | 1 | 1.01693 | nan | 1.01693 | 1.01693 |
| nis_gnss_vel | 1 | 1.04141 | nan | 1.04141 | 1.04141 |
| nis_baro | 1 | 0.114368 | nan | 0.114368 | 0.114368 |
| rms_position_m | 1 | 0.224213 | nan | 0.224213 | 0.224213 |
| rms_velocity_mps | 1 | 0.021467 | nan | 0.021467 | 0.021467 |
| rms_tilt_deg | 1 | 0.116666 | nan | 0.116666 | 0.116666 |
| rms_yaw_deg | 1 | 0.668225 | nan | 0.668225 | 0.668225 |
