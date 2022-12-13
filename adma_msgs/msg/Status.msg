# This message contains all decoded state bitfields that are included in the byte protocol

# status_byte_0 (2100)
uint8 status_gnss_mode                  # 2000
uint8 status_standstill                 # 2001
uint8 status_skidding                   # 2002
uint8 status_external_vel_out           # 2004

# status_byte_1 (2101)
uint8 status_trig_gnss                  # 2005
uint8 status_signal_in3                 # 2006
uint8 status_signal_in2                 # 2007
uint8 status_signal_in1                 # 2008
uint8 status_alignment                  # 2009
uint8 status_ahrs_ins                   # 2010
uint8 status_dead_reckoning             # 2011
uint8 status_synclock                   # 2012

# status_byte_2 (2102)
uint8 status_evk_activ                  # 2013
uint8 status_evk_estimates              # 2014
uint8 status_heading_executed           # 2015
uint8 status_config_changed             # 2016
uint8 status_tilt                       # 2017
uint8 status_pos                        # 2018

uint8 status_count

# status_byte_4
uint8 status_kalmanfilter_settled       # 2020
uint8 status_kf_lat_stimulated          # 2021
uint8 status_kf_long_stimulated         # 2022
uint8 status_kf_steady_state            # 2023
uint8 status_speed                      # 2024

# status_byte_5
uint8 status_robot                      # 2027

