# This message contains all decoded state bitfields that are included in the byte protocol

# status_byte_0 (2100)
uint8 status_gnss_mode                  # 2000
bool status_standstill                 # 2001
bool status_skidding                   # 2002
bool status_external_vel_out           # 2004

# status_byte_1 (2101)
bool status_trig_gnss                  # 2005
bool status_signal_in3                 # 2006
bool status_signal_in2                 # 2007
bool status_signal_in1                 # 2008
bool status_alignment                  # 2009
bool status_ahrs_ins                   # 2010
bool status_dead_reckoning             # 2011
bool status_synclock                   # 2012

# status_byte_2 (2102)
bool status_evk_activ                  # 2013
bool status_evk_estimates              # 2014
bool status_heading_executed           # 2015
bool status_config_changed             # 2016
uint8 status_tilt                       # 2017
uint8 status_pos                        # 2018

uint8 status_count

# status_byte_4
bool status_kalmanfilter_settled       # 2020
bool status_kf_lat_stimulated          # 2021
bool status_kf_long_stimulated         # 2022
bool status_kf_steady_state            # 2023
uint8 status_speed                      # 2024

# status_byte_5
uint8 status_robot                      # 2027

