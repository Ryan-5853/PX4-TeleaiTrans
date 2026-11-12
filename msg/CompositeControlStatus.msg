uint64 timestamp
uint64 timestamp_sample
uint32 state_age_us
float32[4] requested_wrench
float32[2] side_thrust
float32 servo_scale
float32[2] yaw_scale
float32[4] attitude
float32[3] angular_velocity
float32[4] target_attitude
float32[3] target_angular_velocity
float32 vertical_acceleration
float32 target_vertical_acceleration
float32[4] achieved_wrench
float32[4] residual_wrench
# collective=1, roll=2, pitch grids=4, yaw A=8, yaw B=16
uint8 saturation_flags
float32[2] achieved_side_thrust

# Same-cycle C/FRD control trace. Errors: actual - target; attitude uses log(q_target^-1*q).
float32[3] attitude_error
float32[3] rate_error
float32[3] attitude_integral
float32[3] moment_attitude
float32[3] moment_rate
float32[3] moment_integral
float32[3] acceleration_b
float32 filtered_vertical_acceleration
float32 acceleration_error
float32 acceleration_integral
float32 acceleration_pi_p
float32 acceleration_pi_i
float32 corrected_vertical_acceleration
float32 bounded_vertical_acceleration
float32 raw_thrust
float32 allocated_roll_moment
float32[2] installation_yaw
float32 arm_m
float32 loop_dt_s
float32 zero_throttle_threshold
# trace_flags: bit0 trace available, bit1 zero-throttle reset
uint32 trace_flags

# Collective consumed in this control cycle (not the latest independently sampled reference).
float32 collective_input
