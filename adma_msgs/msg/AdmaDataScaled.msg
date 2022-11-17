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

# sensors body acc in g
geometry_msgs/Vector3 acc_body_hr

# sensors body rate in deg/s
geometry_msgs/Vector3 rate_body_hr

# rates body in deg/s
geometry_msgs/Vector3 rate_body

# rates hor in deg/s
geometry_msgs/Vector3 rate_hor

# accelerations body in g
geometry_msgs/Vector3 acc_body

# acceleration horizontal in g
geometry_msgs/Vector3 acc_hor

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
float64 gnss_long_abs

# gnss position relative
# position in m
float64 gnss_pos_rel_x
float64 gnss_pos_rel_y

# gnss epe
#  error in m
float64 gnss_stddev_lat
float64 gnss_stddev_long
float64 gnss_stddev_height

# gnss velocity frame in m/s
geometry_msgs/Vector3 gnss_vel_frame
# latency in s
float64 gnss_vel_latency

# gnss eve error in m/s
geometry_msgs/Vector3 gnss_stddev_vel

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
float64 ins_long_abs

# ins position relative
# position in m
float64 ins_pos_rel_x
float64 ins_pos_rel_y

# ins velocity horizontal in m/s
geometry_msgs/Vector3 ins_vel_hor

# ins velocity frame in m/s
geometry_msgs/Vector3 ins_vel_frame

# ins epe
# error in m
float64 ins_stddev_lat
float64 ins_stddev_long
float64 ins_stddev_height

# ins eve and ins ete std vel in m/s
geometry_msgs/Vector3 ins_stddev_vel
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

# gnss receiver status
uint32 gnss_receiver_error
uint32 gnss_receiver_status

# mobileeye time comparison
float64 time_msec
float64 time_nsec

# list of POIs
POI poi_1
POI poi_2
POI poi_3
POI poi_4
POI poi_5
POI poi_6
POI poi_7
POI poi_8

ByteEW error_warnings
States states