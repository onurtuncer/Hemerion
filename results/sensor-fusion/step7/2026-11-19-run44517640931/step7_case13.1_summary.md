## Step 7 batch: case 13.1, W20 8 m/s, Aetherion 0.18.2 (49e75b814534e6d58c9216a52df2fb7151dfd2f9), Linux

6 run(s): 6 passed every check, 6 have an EKFReplay summary.
Hemerion: `85d305224d10ae7a9bc59a2c4bd5d4702a2a3794`.
Aetherion: `49e75b814534e6d58c9216a52df2fb7151dfd2f9`.

Second-half figures as EKFReplay prints them (ANEES of the nav core, /9; NIS per dimension).

| run | case | seed | turbulence_w20_mps | turbulence_seed | stop_s | passed | duration_s | imu_steps | gnss_pos_updates | gnss_vel_updates | baro_updates | fixes_skipped | anees_nav | nis_gnss_pos | nis_gnss_vel | nis_baro | rms_position_m | rms_velocity_mps | rms_tilt_deg | rms_yaw_deg | cosim_wall_s | convention |
|---|---|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| step7-case13.1-seed-2 | 13.1 | 2 | 8 | 2 | 60 | yes | 59.99 | 5999 | 600 | 600 | 2999 | 0 | 0.928029 | 0.997079 | 0.999946 | 0.118319 | 0.158182 | 0.0296218 | 0.123972 | 0.422066 | 65.3 | right-invariant |
| step7-case13.1-seed-6 | 13.1 | 6 | 8 | 6 | 60 | yes | 59.99 | 5999 | 600 | 600 | 2999 | 0 | 1.26964 | 1.0237 | 1.0003 | 0.118221 | 0.182077 | 0.0309506 | 0.17177 | 1.25978 | 66.2 | right-invariant |
| step7-case13.1-seed-7 | 13.1 | 7 | 8 | 7 | 60 | yes | 59.99 | 5999 | 600 | 600 | 3000 | 0 | 1.64055 | 1.05117 | 1.00415 | 0.118365 | 0.206792 | 0.0410516 | 0.145755 | 0.709634 | 82 | right-invariant |
| step7-case13.1-seed-13 | 13.1 | 13 | 8 | 13 | 60 | yes | 59.99 | 5999 | 600 | 600 | 2999 | 0 | 1.62686 | 0.985831 | 1.04566 | 0.108585 | 0.292119 | 0.0335415 | 0.143515 | 1.04559 | 65.9 | right-invariant |
| step7-case13.1-seed-15 | 13.1 | 15 | 8 | 15 | 60 | yes | 60 | 6000 | 600 | 600 | 3000 | 0 | 0.974031 | 0.955505 | 1.0537 | 0.124657 | 0.182515 | 0.0331134 | 0.11814 | 0.321083 | 82.7 | right-invariant |
| step7-case13.1-seed-18 | 13.1 | 18 | 8 | 18 | 60 | yes | 59.99 | 5999 | 600 | 600 | 3000 | 0 | 1.41278 | 0.938876 | 0.980259 | 0.113648 | 0.108199 | 0.0306459 | 0.208548 | 0.761737 | 85.6 | right-invariant |

Over the runs that report each figure (finite values only; std is the sample one):

| figure | n | mean | std | min | max |
|---|---:|---:|---:|---:|---:|
| duration_s | 6 | 59.9917 | 0.00408248 | 59.99 | 60 |
| imu_steps | 6 | 5999.17 | 0.408248 | 5999 | 6000 |
| gnss_pos_updates | 6 | 600 | 0 | 600 | 600 |
| gnss_vel_updates | 6 | 600 | 0 | 600 | 600 |
| baro_updates | 6 | 2999.5 | 0.547723 | 2999 | 3000 |
| fixes_skipped | 6 | 0 | 0 | 0 | 0 |
| anees_nav | 6 | 1.30865 | 0.310039 | 0.928029 | 1.64055 |
| nis_gnss_pos | 6 | 0.992027 | 0.0417705 | 0.938876 | 1.05117 |
| nis_gnss_vel | 6 | 1.014 | 0.0289797 | 0.980259 | 1.0537 |
| nis_baro | 6 | 0.116966 | 0.00539934 | 0.108585 | 0.124657 |
| rms_position_m | 6 | 0.188314 | 0.0608536 | 0.108199 | 0.292119 |
| rms_velocity_mps | 6 | 0.0331541 | 0.0041508 | 0.0296218 | 0.0410516 |
| rms_tilt_deg | 6 | 0.15195 | 0.0335606 | 0.11814 | 0.208548 |
| rms_yaw_deg | 6 | 0.753315 | 0.3579 | 0.321083 | 1.25978 |
