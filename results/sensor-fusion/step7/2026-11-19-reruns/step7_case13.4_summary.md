## Step 7 batch: case 13.4, W20 8 m/s, Aetherion 0.18.2 (49e75b814534e6d58c9216a52df2fb7151dfd2f9), Linux

4 run(s): 4 passed every check, 4 have an EKFReplay summary.
Hemerion: `1927309f7f22a4ab9ecd280a3ec762f7119d40fb`.
Aetherion: `49e75b814534e6d58c9216a52df2fb7151dfd2f9`.

Second-half figures as EKFReplay prints them (ANEES of the nav core, /9; NIS per dimension).

| run | case | seed | turbulence_w20_mps | turbulence_seed | stop_s | passed | duration_s | imu_steps | gnss_pos_updates | gnss_vel_updates | baro_updates | fixes_skipped | anees_nav | nis_gnss_pos | nis_gnss_vel | nis_baro | rms_position_m | rms_velocity_mps | rms_tilt_deg | rms_yaw_deg | cosim_wall_s | convention |
|---|---|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| step7-case13.4-seed-1 | 13.4 | 1 | 8 | 1 | 240 | yes | 239.99 | 23999 | 2400 | 2400 | 11999 | 0 | 2.24345 | 0.99928 | 0.975553 | 0.115184 | 0.150733 | 0.0296696 | 0.314994 | 0.38289 | 263.2 | right-invariant |
| step7-case13.4-seed-2 | 13.4 | 2 | 8 | 2 | 240 | yes | 239.99 | 23999 | 2400 | 2400 | 11999 | 0 | 1.90691 | 1.00945 | 1.03118 | 0.115039 | 0.152263 | 0.0311654 | 0.242899 | 0.756872 | 263.1 | right-invariant |
| step7-case13.4-seed-5 | 13.4 | 5 | 8 | 5 | 240 | yes | 239.99 | 23999 | 2400 | 2400 | 12000 | 0 | 1.71587 | 1.01656 | 1.0425 | 0.112639 | 0.195521 | 0.0301008 | 0.223585 | 0.834732 | 335.1 | right-invariant |
| step7-case13.4-seed-8 | 13.4 | 8 | 8 | 8 | 240 | yes | 240 | 24000 | 2400 | 2400 | 11999 | 0 | 1.40247 | 0.98033 | 1.03882 | 0.114496 | 0.231226 | 0.0304146 | 0.174132 | 0.510582 | 240.2 | right-invariant |

Over the runs that report each figure (finite values only; std is the sample one):

| figure | n | mean | std | min | max |
|---|---:|---:|---:|---:|---:|
| duration_s | 4 | 239.993 | 0.005 | 239.99 | 240 |
| imu_steps | 4 | 23999.2 | 0.5 | 23999 | 24000 |
| gnss_pos_updates | 4 | 2400 | 0 | 2400 | 2400 |
| gnss_vel_updates | 4 | 2400 | 0 | 2400 | 2400 |
| baro_updates | 4 | 11999.2 | 0.5 | 11999 | 12000 |
| fixes_skipped | 4 | 0 | 0 | 0 | 0 |
| anees_nav | 4 | 1.81718 | 0.352139 | 1.40247 | 2.24345 |
| nis_gnss_pos | 4 | 1.0014 | 0.0157381 | 0.98033 | 1.01656 |
| nis_gnss_vel | 4 | 1.02201 | 0.0313303 | 0.975553 | 1.0425 |
| nis_baro | 4 | 0.114339 | 0.0011717 | 0.112639 | 0.115184 |
| rms_position_m | 4 | 0.182436 | 0.0385883 | 0.150733 | 0.231226 |
| rms_velocity_mps | 4 | 0.0303376 | 0.000630735 | 0.0296696 | 0.0311654 |
| rms_tilt_deg | 4 | 0.238903 | 0.0584116 | 0.174132 | 0.314994 |
| rms_yaw_deg | 4 | 0.621269 | 0.210579 | 0.38289 | 0.834732 |
