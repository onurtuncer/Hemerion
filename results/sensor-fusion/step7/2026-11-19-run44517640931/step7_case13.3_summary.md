## Step 7 batch: case 13.3, W20 8 m/s, Aetherion 0.18.2 (49e75b814534e6d58c9216a52df2fb7151dfd2f9), Linux

9 run(s): 9 passed every check, 9 have an EKFReplay summary.
Hemerion: `85d305224d10ae7a9bc59a2c4bd5d4702a2a3794`.
Aetherion: `49e75b814534e6d58c9216a52df2fb7151dfd2f9`.

Second-half figures as EKFReplay prints them (ANEES of the nav core, /9; NIS per dimension).

| run | case | seed | turbulence_w20_mps | turbulence_seed | stop_s | passed | duration_s | imu_steps | gnss_pos_updates | gnss_vel_updates | baro_updates | fixes_skipped | anees_nav | nis_gnss_pos | nis_gnss_vel | nis_baro | rms_position_m | rms_velocity_mps | rms_tilt_deg | rms_yaw_deg | cosim_wall_s | convention |
|---|---|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| step7-case13.3-seed-2 | 13.3 | 2 | 8 | 2 | 240 | yes | 239.99 | 23999 | 2400 | 2400 | 11999 | 0 | 1.34794 | 1.00941 | 1.03164 | 0.115076 | 0.151647 | 0.0312438 | 0.159042 | 0.878451 | 240.1 | right-invariant |
| step7-case13.3-seed-5 | 13.3 | 5 | 8 | 5 | 240 | yes | 239.99 | 23999 | 2400 | 2400 | 12000 | 0 | 1.30855 | 1.01656 | 1.04221 | 0.112644 | 0.195844 | 0.0299249 | 0.170355 | 0.400484 | 329.5 | right-invariant |
| step7-case13.3-seed-6 | 13.3 | 6 | 8 | 6 | 240 | yes | 239.99 | 23999 | 2400 | 2400 | 11999 | 0 | 1.31284 | 1.01146 | 1.03444 | 0.116187 | 0.283526 | 0.0314889 | 0.123217 | 0.196877 | 240.1 | right-invariant |
| step7-case13.3-seed-7 | 13.3 | 7 | 8 | 7 | 240 | yes | 239.99 | 23999 | 2400 | 2400 | 11999 | 0 | 2.08237 | 0.992792 | 1.01325 | 0.112703 | 0.21104 | 0.0298352 | 0.293718 | 0.488861 | 264.9 | right-invariant |
| step7-case13.3-seed-8 | 13.3 | 8 | 8 | 8 | 240 | yes | 239.99 | 23999 | 2400 | 2400 | 11999 | 0 | 1.1899 | 0.980417 | 1.03812 | 0.114497 | 0.231217 | 0.0300513 | 0.151611 | 0.139936 | 264.7 | right-invariant |
| step7-case13.3-seed-10 | 13.3 | 10 | 8 | 10 | 240 | yes | 239.99 | 23999 | 2400 | 2400 | 11999 | 0 | 1.08092 | 1.01247 | 1.05488 | 0.113526 | 0.151655 | 0.0302496 | 0.141954 | 0.257012 | 240.1 | right-invariant |
| step7-case13.3-seed-11 | 13.3 | 11 | 8 | 11 | 240 | yes | 239.99 | 23999 | 2400 | 2400 | 11999 | 0 | 1.92528 | 1.03599 | 1.0017 | 0.117428 | 0.165934 | 0.0327316 | 0.260044 | 0.383211 | 262.1 | right-invariant |
| step7-case13.3-seed-17 | 13.3 | 17 | 8 | 17 | 240 | yes | 239.99 | 23999 | 2400 | 2400 | 11999 | 0 | 1.54914 | 0.977506 | 1.00781 | 0.11236 | 0.263664 | 0.0324485 | 0.189469 | 0.228437 | 240.2 | right-invariant |
| step7-case13.3-seed-18 | 13.3 | 18 | 8 | 18 | 240 | yes | 240 | 24000 | 2400 | 2400 | 11999 | 0 | 1.67226 | 1.00089 | 1.03747 | 0.111371 | 0.211345 | 0.0314695 | 0.222471 | 0.455445 | 240.1 | right-invariant |

Over the runs that report each figure (finite values only; std is the sample one):

| figure | n | mean | std | min | max |
|---|---:|---:|---:|---:|---:|
| duration_s | 9 | 239.991 | 0.00333333 | 239.99 | 240 |
| imu_steps | 9 | 23999.1 | 0.333333 | 23999 | 24000 |
| gnss_pos_updates | 9 | 2400 | 0 | 2400 | 2400 |
| gnss_vel_updates | 9 | 2400 | 0 | 2400 | 2400 |
| baro_updates | 9 | 11999.1 | 0.333333 | 11999 | 12000 |
| fixes_skipped | 9 | 0 | 0 | 0 | 0 |
| anees_nav | 9 | 1.49658 | 0.338857 | 1.08092 | 2.08237 |
| nis_gnss_pos | 9 | 1.00417 | 0.018476 | 0.977506 | 1.03599 |
| nis_gnss_vel | 9 | 1.02906 | 0.0176001 | 1.0017 | 1.05488 |
| nis_baro | 9 | 0.113977 | 0.0019774 | 0.111371 | 0.117428 |
| rms_position_m | 9 | 0.207319 | 0.0469384 | 0.151647 | 0.283526 |
| rms_velocity_mps | 9 | 0.0310493 | 0.0010937 | 0.0298352 | 0.0327316 |
| rms_tilt_deg | 9 | 0.190209 | 0.0573414 | 0.123217 | 0.293718 |
| rms_yaw_deg | 9 | 0.380968 | 0.222396 | 0.139936 | 0.878451 |
