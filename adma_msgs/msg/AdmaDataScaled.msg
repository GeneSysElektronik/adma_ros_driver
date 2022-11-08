# adma static header
string genesys_id
string header_version
uint32 format_id
string format_version
uint32 serial_number
uint32 alias

# adma dynamic header
uint32 config_id
uint32 config_format
uint32 config_version
uint32 config_size
uint32 byte_offset
uint32 slice_size
uint32 slice_data

# status
int32 status_gnss_mode
int32 status_stand_still
int32 status_skidding
int32 status_external_vel_out
int32 status_trig_gnss
int32 status_signal_in3
int32 status_signal_in2
int32 status_signal_in1
int32 status_alignment
int32 status_ahrs_ins
int32 status_dead_reckoning
int32 status_sync_lock
int32 status_evk_activ
int32 status_evk_estimates
int32 status_heading_executed
int32 status_config_changed
int32 status_tilt
int32 status_pos
int8 status_count
int32 status_kalmanfilter_settled
int32 status_kf_lat_stimulated
int32 status_kf_long_stimulated
int32 status_kf_steady_state
int32 status_speed

# error and warning
string error_hardware
string error_misc1
string error_misc2
string error_misc3
string warn_gnss
string warn_misc1
string error_hw_sticky

# sensors body acc in g
float64 acc_body_hr_x
float64 acc_body_hr_y
float64 acc_body_hr_z

# sensors body rate in deg/s
float64 rate_body_hr_x
float64 rate_body_hr_y
float64 rate_body_hr_z

# rates body
# rates in deg/s
float64 rate_body_x
float64 rate_body_y
float64 rate_body_z

# rates horz
# rates in deg/s
float64 rate_hor_x
float64 rate_hor_y
float64 rate_hor_z

# accelerations body
# acc in g
float64 acc_body_x
float64 acc_body_y
float64 acc_body_z

# acceleration horizontal
# acc in g
float64 acc_hor_x
float64 acc_hor_y
float64 acc_hor_z

# accelerations body poi1
# acc in g
float64 acc_body_x_1
float64 acc_body_y_1
float64 acc_body_z_1

# accelerations body poi2
# acc in g
float64 acc_body_x_2
float64 acc_body_y_2
float64 acc_body_z_2

# accelerations body poi3
# acc in g
float64 acc_body_x_3
float64 acc_body_y_3
float64 acc_body_z_3

# accelerations body poi4
# acc in g
float64 acc_body_x_4
float64 acc_body_y_4
float64 acc_body_z_4

# accelerations body poi5
# acc in g
float64 acc_body_x_5
float64 acc_body_y_5
float64 acc_body_z_5

# accelerations body poi6
# acc in g
float64 acc_body_x_6
float64 acc_body_y_6
float64 acc_body_z_6

# accelerations body poi7
# acc in g
float64 acc_body_x_7
float64 acc_body_y_7
float64 acc_body_z_7

# accelerations body poi8
# acc in g
float64 acc_body_x_8
float64 acc_body_y_8
float64 acc_body_z_8

# acceleration horizontal poi1
# acc in g
float64 acc_hor_x_1
float64 acc_hor_y_1
float64 acc_hor_z_1

# acceleration horizontal poi2
# acc in g
float64 acc_hor_x_2
float64 acc_hor_y_2
float64 acc_hor_z_2

# acceleration horizontal poi3
# acc in g
float64 acc_hor_x_3
float64 acc_hor_y_3
float64 acc_hor_z_3

# acceleration horizontal poi4
# acc in g
float64 acc_hor_x_4
float64 acc_hor_y_4
float64 acc_hor_z_4

# acceleration horizontal poi5
# acc in g
float64 acc_hor_x_5
float64 acc_hor_y_5
float64 acc_hor_z_5

# acceleration horizontal poi6
# acc in g
float64 acc_hor_x_6
float64 acc_hor_y_6
float64 acc_hor_z_6

# acceleration horizontal poi7
# acc in g
float64 acc_hor_x_7
float64 acc_hor_y_7
float64 acc_hor_z_7

# acceleration horizontal poi8
# acc in g
float64 acc_hor_x_8
float64 acc_hor_y_8
float64 acc_hor_z_8

# external velocity analog
#  vel in m/s
float64 ext_vel_an_x
float64 ext_vel_an_y

# external velocity digital pulses in m/s
float64 ext_vel_dig_x
float64 ext_vel_dig_y
int16 ext_vel_dig_pulses_x
int16 ext_vel_dig_pulses_y


# externaal velocity corrected in m/s
float64 ext_vel_x_corrected
float64 ext_vel_y_corrected

# miscellaneous
# invpath radius in 1/m
float64 inv_path_radius
# side slip in degrees
float64 side_slip_angle
# dis_trav in m
float64 dist_trav

# miscellaneous_poi1
float64 inv_path_radius_1
float64 side_slip_angle_1
float64 dist_trav_1

# miscellaneous_poi2
float64 inv_path_radius_2
float64 side_slip_angle_2
float64 dist_trav_2

# miscellaneous_poi3
float64 inv_path_radius_3
float64 side_slip_angle_3
float64 dist_trav_3

# miscellaneous_poi4
float64 inv_path_radius_4
float64 side_slip_angle_4
float64 dist_trav_4

# miscellaneous_poi5
float64 inv_path_radius_5
float64 side_slip_angle_5
float64 dist_trav_5

# miscellaneous_poi6
float64 inv_path_radius_6
float64 side_slip_angle_6
float64 dist_trav_6

# miscellaneous_poi7
float64 inv_path_radius_7
float64 side_slip_angle_7
float64 dist_trav_7

# miscellaneous_poi8
float64 inv_path_radius_8
float64 side_slip_angle_8
float64 dist_trav_8

# triggers 1 and 2
# triggers in mu.s
uint16 trig_rising_1
uint16 trig_falling_1
uint16 trig_rising_2
uint16 trig_falling_2

# triggers 3 and 4
# triggers in mu.s
uint16 trig_rising_3
uint16 trig_falling_3
uint16 trig_rising_4
uint16 trig_falling_4

# system data
# systemta in mu.s
uint16 system_ta
# systemtemp in deg. c
float64 system_temp
# systemtimesinceinit in s
uint16 system_time_since_init
# system dsp load in %
float64 system_dsp_load

# gnss position absolute
# position in deg
float64 gnss_lat_abs
float64 gnss_lon_abs

# gnss position relative
# position in m
float64 gnss_pos_rel_x
float64 gnss_pos_rel_y

# gnss epe
#  error in m
float64 gnss_stddev_lat
float64 gnss_stddev_lon
float64 gnss_stddev_height

# gnss velocity frame
# vel in m/s
float64 gnss_vel_frame_x
float64 gnss_vel_frame_y
float64 gnss_vel_frame_z
# latency in s
float64 gnss_vel_latency

# gnss eve
# error in m/s
float64 gnss_stddev_vel_x
float64 gnss_stddev_vel_y
float64 gnss_stddev_vel_z

# gnss time utc
# gnsstimemsec in ms
uint32 gnss_time_msec
# gnsstimeweek in week
uint16 gnss_time_week
# gnsstrigger in mu.s
uint16 gnss_trigger

# gnss auxdata 1
# gnssdiffage in s
float64 gnss_diffage
int8 gnss_sats_used
int8 gnss_sats_visible
int8 gnss_sats_dualant_used
int8 gnss_sats_dualant_visible

# gnss auxdata 2
# gnsslogdelay in ta
int8 gnss_log_delay
# gnssreceiverload in %
float64 gnss_receiver_load

string gnss_base_nr

# ins angle and gnss cog
# all units in deg
float64 ins_roll
float64 ins_pitch
float64 ins_yaw
float64 gnss_cog

# gnss height(msl)
float64 gnss_height
float64 undulation

# gnss dualant time utc
# gnssdualanttimemsec in ms
uint32 gnss_dualant_time_msec
# gnssdualanttimeweek in week
uint16 gnss_dualant_time_week

# gnss dualant angle
# all units in deg
float64 gnss_dualant_heading
float64 gnss_dualant_pitch

# gnss dualant angle ete
# all units in deg
float64 gnss_dualant_stddev_heading
float64 gnss_dualant_stddev_pitch
float32 gnss_dualant_stddev_heading_hr
float32 gnss_dualant_stddev_pitch_hr

# ins position height (msl)
float64 ins_height

# ins position height poi1 and poi2 (msl)
float64 ins_height_1
float64 ins_height_2

# ins position height poi3 and poi4 (msl)
float64 ins_height_3
float64 ins_height_4

# ins position height poi5 and poi6 (msl)
float64 ins_height_5
float64 ins_height_6

# ins position height poi7 and poi8 (msl)
float64 ins_height_7
float64 ins_height_8

# ins time utc
# instimemsec in ms
uint32 ins_time_msec
# instimeweek in week
uint16 ins_time_week
# leapseconds in s
int16 leap_seconds

# ins position absolute
# position in deg
float64 ins_lat_abs
float64 ins_lon_abs

# ins position relative
# position in m
float64 ins_pos_rel_x
float64 ins_pos_rel_y

# ins position absolute pos1
# position in deg
float64 ins_lat_abs_1
float64 ins_lon_abs_1

# ins position relative pos1
# position in m
float64 ins_pos_rel_x_1
float64 ins_pos_rel_y_1

# ins position absolute pos2
# position in deg
float64 ins_lat_abs_2
float64 ins_lon_abs_2

# ins position relative pos2
# position in m
float64 ins_pos_rel_x_2
float64 ins_pos_rel_y_2

# ins position absolute pos3
# position in deg
float64 ins_lat_abs_3
float64 ins_lon_abs_3

# ins position relative pos3
# position in m
float64 ins_pos_rel_x_3
float64 ins_pos_rel_y_3

# ins position absolute pos4
# position in deg
float64 ins_lat_abs_4
float64 ins_lon_abs_4

# ins position relative pos4
# position in m
float64 ins_pos_rel_x_4
float64 ins_pos_rel_y_4

# ins position absolute pos5
# position in deg
float64 ins_lat_abs_5
float64 ins_lon_abs_5

# ins position relative pos5
# position in m
float64 ins_pos_rel_x_5
float64 ins_pos_rel_y_5

# ins position absolute pos6
# position in deg
float64 ins_lat_abs_6
float64 ins_lon_abs_6

# ins position relative pos6
# position in m
float64 ins_pos_rel_x_6
float64 ins_pos_rel_y_6

# ins position absolute pos7
# position in deg
float64 ins_lat_abs_7
float64 ins_lon_abs_7

# ins position relative pos7
# position in m
float64 ins_pos_rel_x_7
float64 ins_pos_rel_y_7

# ins position absolute pos8
# position in deg
float64 ins_lat_abs_8
float64 ins_lon_abs_8

# ins position relative pos8
# position in m
float64 ins_pos_rel_x_8
float64 ins_pos_rel_y_8

# ins velocity horizontal
# velocity in m/s
float64 ins_vel_hor_x
float64 ins_vel_hor_y
float64 ins_vel_hor_z

# ins velocity frame
# velocity in m/s
float64 ins_vel_frame_x
float64 ins_vel_frame_y
float64 ins_vel_frame_z

# ins velocity horizontal poi1
# velocity in m/s
float64 ins_vel_hor_x_1
float64 ins_vel_hor_y_1
float64 ins_vel_hor_z_1

# ins velocity horizontal poi2
# velocity in m/s
float64 ins_vel_hor_x_2
float64 ins_vel_hor_y_2
float64 ins_vel_hor_z_2

# ins velocity horizontal poi3
# velocity in m/s
float64 ins_vel_hor_x_3
float64 ins_vel_hor_y_3
float64 ins_vel_hor_z_3

# ins velocity horizontal poi4
# velocity in m/s
float64 ins_vel_hor_x_4
float64 ins_vel_hor_y_4
float64 ins_vel_hor_z_4

# ins velocity horizontal poi5
# velocity in m/s
float64 ins_vel_hor_x_5
float64 ins_vel_hor_y_5
float64 ins_vel_hor_z_5

# ins velocity horizontal poi6
# velocity in m/s
float64 ins_vel_hor_x_6
float64 ins_vel_hor_y_6
float64 ins_vel_hor_z_6

# ins velocity horizontal poi7
# velocity in m/s
float64 ins_vel_hor_x_7
float64 ins_vel_hor_y_7
float64 ins_vel_hor_z_7

# ins velocity horizontal poi8
# velocity in m/s
float64 ins_vel_hor_x_8
float64 ins_vel_hor_y_8
float64 ins_vel_hor_z_8

# ins epe
# error in m
float64 ins_stddev_lat
float64 ins_stddev_long
float64 ins_stddev_height

# ins eve and ins ete
#  std vel in m/s
float64 ins_stddev_vel_x
float64 ins_stddev_vel_y
float64 ins_stddev_vel_z
# std ete in deg
float64 ins_stddev_roll
float64 ins_stddev_pitch
float64 ins_stddev_yaw

# analog in1
# analog in v
float64 an1
float64 an2
float64 an3
float64 an4

# kalman filter status
# all uints in %
uint8 kf_lat_stimulated
uint8 kf_long_stimulated
uint8 kf_steady_state

# gnss receiver status
uint32 gnss_receiver_error
uint32 gnss_receiver_status


# mobileeye time comparison
float64 time_msec
float64 time_nsec