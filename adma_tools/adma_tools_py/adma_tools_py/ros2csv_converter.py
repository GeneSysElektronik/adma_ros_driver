import rclpy
from rclpy.node import Node
from adma_ros_driver_msgs.msg import AdmaDataScaled, AdmaStatus
import message_filters

class Ros2CSVConverter(Node):

    def __init__(self):
        super().__init__("ros2csvConverter")  
        
        self.filename = "recorded_data.csv"
        self.sub_adma_data_scaled = message_filters.Subscriber(self, AdmaDataScaled, "/genesys/adma/data_scaled")
        self.sub_adma_error_warning = message_filters.Subscriber(self, AdmaStatus, "/genesys/adma/status")

        self.ts = message_filters.TimeSynchronizer([self.sub_adma_data_scaled, self.sub_adma_error_warning], 10)
        self.ts.registerCallback(self.msg_cb)

        self.i = 1

    def msg_cb(self, admaMsg : AdmaDataScaled, ewMsg : AdmaStatus):  
        #Open File and write Data
        if self.i > 1:
            file = open(self.filename,"a")

            file.write("\n")
            file.write(f"{admaMsg.genesys_id}, {admaMsg.header_version}, {admaMsg.format_id}, {admaMsg.format_version},{admaMsg.serial_number}, {admaMsg.alias},\
            {admaMsg.config_id}, {admaMsg.config_format}, {admaMsg.config_version}, {admaMsg.config_size},{admaMsg.byte_offset}, {admaMsg.slice_size}, {admaMsg.slice_data},\
            {admaMsg.acc_body_hr.x}, {admaMsg.acc_body_hr.y}, {admaMsg.acc_body_hr.z},\
            {admaMsg.rate_body_hr.x}, {admaMsg.rate_body_hr.y}, {admaMsg.rate_body_hr.z},\
            {admaMsg.rate_body.x}, {admaMsg.rate_body.y}, {admaMsg.rate_body.z},\
            {admaMsg.rate_hor.x}, {admaMsg.rate_hor.y}, {admaMsg.rate_hor.z},\
            {admaMsg.acc_body.x}, {admaMsg.acc_body.y}, {admaMsg.acc_body.z},\
            {admaMsg.acc_hor.x}, {admaMsg.acc_hor.y}, {admaMsg.acc_hor.z},\
            {admaMsg.ext_vel_an_x}, {admaMsg.ext_vel_an_y},\
            {admaMsg.ext_vel_dig_x}, {admaMsg.ext_vel_dig_y}, {admaMsg.ext_vel_dig_pulses_x}, {admaMsg.ext_vel_dig_pulses_y},\
            {admaMsg.ext_vel_x_corrected}, {admaMsg.ext_vel_y_corrected},\
            {admaMsg.inv_path_radius}, {admaMsg.side_slip_angle}, {admaMsg.dist_trav},\
            {admaMsg.trig_rising_1}, {admaMsg.trig_falling_1}, {admaMsg.trig_rising_2}, {admaMsg.trig_falling_2},\
            {admaMsg.trig_rising_3}, {admaMsg.trig_falling_3}, {admaMsg.trig_rising_4}, {admaMsg.trig_falling_4},\
            {admaMsg.system_ta}, {admaMsg.system_temp}, {admaMsg.system_time_since_init}, {admaMsg.system_dsp_load},\
            {admaMsg.gnss_lat_abs}, {admaMsg.gnss_long_abs}, {admaMsg.gnss_pos_rel_x}, {admaMsg.gnss_pos_rel_y},\
            {admaMsg.gnss_stddev_lat}, {admaMsg.gnss_stddev_long}, {admaMsg.gnss_stddev_height},\
            {admaMsg.gnss_vel_frame.x}, {admaMsg.gnss_vel_frame.y}, {admaMsg.gnss_vel_frame.z}, {admaMsg.gnss_vel_latency},\
            {admaMsg.gnss_stddev_vel.x}, {admaMsg.gnss_stddev_vel.y}, {admaMsg.gnss_stddev_vel.z},\
            {admaMsg.gnss_time_msec}, {admaMsg.gnss_time_week}, {admaMsg.gnss_trigger},\
            {admaMsg.gnss_diffage}, {admaMsg.gnss_sats_used}, {admaMsg.gnss_sats_visible}, {admaMsg.gnss_sats_dualant_used}, {admaMsg.gnss_sats_dualant_visible},\
            {admaMsg.gnss_log_delay}, {admaMsg.gnss_receiver_load}, {admaMsg.gnss_base_nr},\
            {admaMsg.ins_roll}, {admaMsg.ins_pitch},{admaMsg.ins_yaw}, {admaMsg.gnss_cog},\
            {admaMsg.gnss_height}, {admaMsg.undulation}, {admaMsg.gnss_dualant_time_msec}, {admaMsg.gnss_dualant_time_week},\
            {admaMsg.gnss_dualant_heading}, {admaMsg.gnss_dualant_pitch},\
            {admaMsg.gnss_dualant_stddev_heading}, {admaMsg.gnss_dualant_stddev_pitch}, {admaMsg.gnss_dualant_stddev_heading_hr}, {admaMsg.gnss_dualant_stddev_pitch_hr},\
            {admaMsg.ins_height}, {admaMsg.ins_time_msec}, {admaMsg.ins_time_week}, {admaMsg.leap_seconds},\
            {admaMsg.ins_lat_abs}, {admaMsg.ins_long_abs}, {admaMsg.ins_pos_rel_x}, {admaMsg.ins_pos_rel_y},\
            {admaMsg.ins_vel_hor.x}, {admaMsg.ins_vel_hor.y}, {admaMsg.ins_vel_hor.z},\
            {admaMsg.ins_vel_frame.x}, {admaMsg.ins_vel_frame.y}, {admaMsg.ins_vel_frame.z},\
            {admaMsg.ins_stddev_lat}, {admaMsg.ins_stddev_long}, {admaMsg.ins_stddev_height},\
            {admaMsg.ins_stddev_vel.x}, {admaMsg.ins_stddev_vel.y}, {admaMsg.ins_stddev_vel.z},\
            {admaMsg.ins_stddev_roll}, {admaMsg.ins_stddev_pitch}, {admaMsg.ins_stddev_yaw},\
            {admaMsg.an1}, {admaMsg.an2}, {admaMsg.an3}, {admaMsg.an4},\
            {admaMsg.gnss_receiver_error}, {admaMsg.gnss_receiver_status},\
            {admaMsg.time_msec}, {admaMsg.time_nsec},\
            {admaMsg.poi_1.acc_body.x}, {admaMsg.poi_1.acc_body.y}, {admaMsg.poi_1.acc_body.z},\
            {admaMsg.poi_1.acc_hor.x}, {admaMsg.poi_1.acc_hor.y}, {admaMsg.poi_1.acc_hor.z},\
            {admaMsg.poi_1.inv_path_radius}, {admaMsg.poi_1.side_slip_angle}, {admaMsg.poi_1.dist_trav}, {admaMsg.poi_1.ins_height},\
            {admaMsg.poi_1.ins_lat_abs}, {admaMsg.poi_1.ins_lon_abs}, {admaMsg.poi_1.ins_pos_rel_x}, {admaMsg.poi_1.ins_pos_rel_y},\
            {admaMsg.poi_1.ins_vel_hor.x}, {admaMsg.poi_1.ins_vel_hor.y}, {admaMsg.poi_1.ins_vel_hor.z},\
            {admaMsg.poi_2.acc_body.x}, {admaMsg.poi_2.acc_body.y}, {admaMsg.poi_2.acc_body.z},\
            {admaMsg.poi_2.acc_hor.x}, {admaMsg.poi_2.acc_hor.y}, {admaMsg.poi_2.acc_hor.z},\
            {admaMsg.poi_2.inv_path_radius}, {admaMsg.poi_2.side_slip_angle}, {admaMsg.poi_2.dist_trav}, {admaMsg.poi_2.ins_height},\
            {admaMsg.poi_2.ins_lat_abs}, {admaMsg.poi_2.ins_lon_abs}, {admaMsg.poi_2.ins_pos_rel_x}, {admaMsg.poi_2.ins_pos_rel_y},\
            {admaMsg.poi_2.ins_vel_hor.x}, {admaMsg.poi_2.ins_vel_hor.y}, {admaMsg.poi_2.ins_vel_hor.z},\
            {admaMsg.poi_3.acc_body.x}, {admaMsg.poi_3.acc_body.y}, {admaMsg.poi_3.acc_body.z},\
            {admaMsg.poi_3.acc_hor.x}, {admaMsg.poi_3.acc_hor.y}, {admaMsg.poi_3.acc_hor.z},\
            {admaMsg.poi_3.inv_path_radius}, {admaMsg.poi_3.side_slip_angle}, {admaMsg.poi_3.dist_trav}, {admaMsg.poi_3.ins_height},\
            {admaMsg.poi_3.ins_lat_abs}, {admaMsg.poi_3.ins_lon_abs}, {admaMsg.poi_3.ins_pos_rel_x}, {admaMsg.poi_3.ins_pos_rel_y},\
            {admaMsg.poi_3.ins_vel_hor.x}, {admaMsg.poi_3.ins_vel_hor.y}, {admaMsg.poi_3.ins_vel_hor.z},\
            {admaMsg.poi_4.acc_body.x}, {admaMsg.poi_4.acc_body.y}, {admaMsg.poi_4.acc_body.z},\
            {admaMsg.poi_4.acc_hor.x}, {admaMsg.poi_4.acc_hor.y}, {admaMsg.poi_4.acc_hor.z},\
            {admaMsg.poi_4.inv_path_radius}, {admaMsg.poi_4.side_slip_angle}, {admaMsg.poi_4.dist_trav}, {admaMsg.poi_4.ins_height},\
            {admaMsg.poi_4.ins_lat_abs}, {admaMsg.poi_4.ins_lon_abs}, {admaMsg.poi_4.ins_pos_rel_x}, {admaMsg.poi_4.ins_pos_rel_y},\
            {admaMsg.poi_4.ins_vel_hor.x}, {admaMsg.poi_4.ins_vel_hor.y}, {admaMsg.poi_4.ins_vel_hor.z},\
            {admaMsg.poi_5.acc_body.x}, {admaMsg.poi_5.acc_body.y}, {admaMsg.poi_5.acc_body.z},\
            {admaMsg.poi_5.acc_hor.x}, {admaMsg.poi_5.acc_hor.y}, {admaMsg.poi_5.acc_hor.z},\
            {admaMsg.poi_5.inv_path_radius}, {admaMsg.poi_5.side_slip_angle}, {admaMsg.poi_5.dist_trav}, {admaMsg.poi_5.ins_height},\
            {admaMsg.poi_5.ins_lat_abs}, {admaMsg.poi_5.ins_lon_abs}, {admaMsg.poi_5.ins_pos_rel_x}, {admaMsg.poi_5.ins_pos_rel_y},\
            {admaMsg.poi_5.ins_vel_hor.x}, {admaMsg.poi_5.ins_vel_hor.y}, {admaMsg.poi_5.ins_vel_hor.z},\
            {admaMsg.poi_6.acc_body.x}, {admaMsg.poi_6.acc_body.y}, {admaMsg.poi_6.acc_body.z},\
            {admaMsg.poi_6.acc_hor.x}, {admaMsg.poi_6.acc_hor.y}, {admaMsg.poi_6.acc_hor.z},\
            {admaMsg.poi_6.inv_path_radius}, {admaMsg.poi_6.side_slip_angle}, {admaMsg.poi_6.dist_trav}, {admaMsg.poi_6.ins_height},\
            {admaMsg.poi_6.ins_lat_abs}, {admaMsg.poi_6.ins_lon_abs}, {admaMsg.poi_6.ins_pos_rel_x}, {admaMsg.poi_6.ins_pos_rel_y},\
            {admaMsg.poi_6.ins_vel_hor.x}, {admaMsg.poi_6.ins_vel_hor.y}, {admaMsg.poi_6.ins_vel_hor.z},\
            {admaMsg.poi_7.acc_body.x}, {admaMsg.poi_7.acc_body.y}, {admaMsg.poi_7.acc_body.z},\
            {admaMsg.poi_7.acc_hor.x}, {admaMsg.poi_7.acc_hor.y}, {admaMsg.poi_7.acc_hor.z},\
            {admaMsg.poi_7.inv_path_radius}, {admaMsg.poi_7.side_slip_angle}, {admaMsg.poi_7.dist_trav}, {admaMsg.poi_7.ins_height},\
            {admaMsg.poi_7.ins_lat_abs}, {admaMsg.poi_7.ins_lon_abs}, {admaMsg.poi_7.ins_pos_rel_x}, {admaMsg.poi_7.ins_pos_rel_y},\
            {admaMsg.poi_7.ins_vel_hor.x}, {admaMsg.poi_7.ins_vel_hor.y}, {admaMsg.poi_7.ins_vel_hor.z},\
            {admaMsg.poi_8.acc_body.x}, {admaMsg.poi_8.acc_body.y}, {admaMsg.poi_8.acc_body.z},\
            {admaMsg.poi_8.acc_hor.x}, {admaMsg.poi_8.acc_hor.y}, {admaMsg.poi_8.acc_hor.z},\
            {admaMsg.poi_8.inv_path_radius}, {admaMsg.poi_8.side_slip_angle}, {admaMsg.poi_8.dist_trav}, {admaMsg.poi_8.ins_height},\
            {admaMsg.poi_8.ins_lat_abs}, {admaMsg.poi_8.ins_lon_abs}, {admaMsg.poi_8.ins_pos_rel_x}, {admaMsg.poi_8.ins_pos_rel_y},\
            {admaMsg.poi_8.ins_vel_hor.x}, {admaMsg.poi_8.ins_vel_hor.y}, {admaMsg.poi_8.ins_vel_hor.z},\
            {admaMsg.status.status_gnss_mode}, {admaMsg.status.status_standstill}, {admaMsg.status.status_skidding}, {admaMsg.status.status_external_vel_out},\
            {admaMsg.status.status_trig_gnss}, {admaMsg.status.status_signal_in3}, {admaMsg.status.status_signal_in2}, {admaMsg.status.status_signal_in1},\
            {admaMsg.status.status_alignment}, {admaMsg.status.status_ahrs_ins}, {admaMsg.status.status_dead_reckoning}, {admaMsg.status.status_synclock},\
            {admaMsg.status.status_evk_activ}, {admaMsg.status.status_evk_estimates}, {admaMsg.status.status_heading_executed}, {admaMsg.status.status_config_changed},\
            {admaMsg.status.status_tilt}, {admaMsg.status.status_pos}, {admaMsg.status.status_count},\
            {admaMsg.status.status_kalmanfilter_settled}, {admaMsg.status.status_kf_lat_stimulated}, {admaMsg.status.status_kf_long_stimulated}, {admaMsg.status.status_kf_steady_state},\
            {admaMsg.status.status_speed}, {admaMsg.status.status_robot},\
            {ewMsg.error_warnings.error_gyro_hw}, {ewMsg.error_warnings.error_accel_hw}, {ewMsg.error_warnings.error_ext_speed_hw}, {ewMsg.error_warnings.error_gnss_hw},\
            {ewMsg.error_warnings.error_data_bus_checksum},{ewMsg.error_warnings.error_eeprom}, {ewMsg.error_warnings.error_xmit}, {ewMsg.error_warnings.error_cmd},\
            {ewMsg.error_warnings.error_data_bus}, {ewMsg.error_warnings.error_can_bus}, {ewMsg.error_warnings.error_num},\
            {ewMsg.error_warnings.error_temp_warning}, {ewMsg.error_warnings.error_reduced_accuracy}, {ewMsg.error_warnings.error_range_max},\
            {ewMsg.error_warnings.warn_gnss_no_solution}, {ewMsg.error_warnings.warn_gnss_vel_ignored}, {ewMsg.error_warnings.warn_gnss_pos_ignored}, {ewMsg.error_warnings.warn_gnss_unable_to_cfg},\
            {ewMsg.error_warnings.warn_speed_off}, {ewMsg.error_warnings.warn_gnss_dualant_ignored},\
            {ewMsg.error_warnings.error_hw_sticky}\
            ")
            file.close()
        else:
            #define headers
            file = open(self.filename,"w")
            file.write(f"genesys_id, header_version, format_id, format_version, serial_number, alias,\
            config_id, config_format, config_version, config_size, byte_offset, slice_size, slice_data,\
            Acc_Body_HR_X, Acc_Body_HR_Y, Acc_Body_HR_Z,\
            Rate_Body_HR_X, Rate_Body_HR_Y, Rate_Body_HR_Z,\
            Rate_Body_X, Rate_Body_Y, Rate_Body_Z,\
            Rate_Hor_X, Rate_Hor_Y, Rate_Hor_Z,\
            Acc_Body_X, Acc_Body_Y, Acc_Body_Z,\
            Acc_Hor_X, Acc_Hor_Y, Acc_Hor_Z,\
            Ext_Vel_An_X, Ext_Vel_An_Y,\
            Ext_Vel_Dig_X, Ext_Vel_Dig_Y, Ext_Vel_Dig_Pulses_X, Ext_Vel_Dig_Pulses_Y, Ext_Vel_X_Corrected, Ext_Vel_Y_Corrected,\
            Inv_Path_Radius, Side_Slip_Angle, Dist_Trav,\
            Trig_Rising_1, Trig_Falling_1, Trig_Rising_2, Trig_Falling_2, Trig_Rising_3, Trig_Falling_3, Trig_Rising_4, Trig_Falling_4,\
            System_Ta, System_Temp, System_Time_Since_Init, System_Dsp_Load,\
            GNSS_Lat_Abs, GNSS_Long_Abs, GNSS_Pos_Rel_X, GNSS_Pos_Rel_Y,\
            GNSS_Stddev_Lat, GNSS_Stddev_Long, GNSS_Stddev_Height,\
            GNSS_Vel_Frame_X, GNSS_Vel_Frame_Y, GNSS_Vel_Frame_Z, GNSS_Vel_Latency,\
            GNSS_Stddev_Vel_X, GNSS_Stddev_Vel_Y, GNSS_Stddev_Vel_Z,\
            GNSS_Time_msec, GNSS_Time_Week, GNSS_Trigger,\
            GNSS_Diffage, GNSS_Sats_Used, GNSS_Sats_Visible, GNSS_Sats_Dualant_Used, GNSS_Sats_Dualant_Visible,\
            GNSS_Log_Delay, GNSS_Receiver_Load, GNSS_BaseNr,\
            INS_Roll, INS_Pitch, INS_Yaw, GNSS_COG,\
            GNSS_Height, Undulation, GNSS_Dualant_Time_msec, GNSS_Dualant_Time_Week,\
            GNSS_Dualant_Heading, GNSS_Dualant_Pitch,\
            GNSS_Dualant_Stddev_Heading, GNSS_Dualant_Stddev_Pitch, GNSS_Dualant_Stddev_Heading_HR, GNSS_Dualant_Stddev_Pitch_HR,\
            INS_Height, INS_Time_msec, INS_Time_week, Leap_Seconds,\
            INS_Lat_Abs, INS_Long_Abs, INS_Pos_Rel_X, INS_Pos_Rel_Y,\
            INS_Vel_Hor_X, INS_Vel_Hor_Y, INS_Vel_Hor_Z,\
            INS_Vel_Frame_X, INS_Vel_Frame_Y, INS_Vel_Frame_Z,\
            INS_Stddev_Lat, INS_Stddev_Long, INS_Stddev_Height,\
            INS_Stddev_Vel_X, INS_Stddev_Vel_Y, INS_Stddev_Vel_Z,\
            INS_Stddev_Roll, INS_Stddev_Pitch, INS_Stddev_Yaw,\
            AN1, AN2, AN3, AN4,\
            GNSS_Receiver_Error, GNSS_Receiver_Status,\
            Time_msec, Time_nsec,\
            Acc_Body_X_POI1, Acc_Body_Y_POI1, Acc_Body_Z_POI1,\
            Acc_Hor_X_POI1, Acc_Hor_Y_POI1, Acc_Hor_Z_POI1,\
            Inv_Path_Radius_POI1, Side_Slip_Angle_POI1, Dist_Trav_POI1, INS_Height_POI1,\
            INS_Lat_Abs_POI1, INS_Long_Abs_POI1, INS_Pos_Rel_X_POI1, INS_Pos_Rel_Y_POI1,\
            INS_Vel_Hor_X_POI1, INS_Vel_Hor_Y_POI1, INS_Vel_Hor_Z_POI1,\
            Acc_Body_X_POI2, Acc_Body_Y_POI2, Acc_Body_Z_POI2,\
            Acc_Hor_X_POI2, Acc_Hor_Y_POI2, Acc_Hor_Z_POI2,\
            Inv_Path_Radius_POI2, Side_Slip_Angle_POI2, Dist_Trav_POI2, INS_Height_POI2,\
            INS_Lat_Abs_POI2, INS_Long_Abs_POI2, INS_Pos_Rel_X_POI2, INS_Pos_Rel_Y_POI2,\
            INS_Vel_Hor_X_POI2, INS_Vel_Hor_Y_POI2, INS_Vel_Hor_Z_POI2,\
            Acc_Body_X_POI3, Acc_Body_Y_POI3, Acc_Body_Z_POI3,\
            Acc_Hor_X_POI3, Acc_Hor_Y_POI3, Acc_Hor_Z_POI3,\
            Inv_Path_Radius_POI3, Side_Slip_Angle_POI3, Dist_Trav_POI3, INS_Height_POI3,\
            INS_Lat_Abs_POI3, INS_Long_Abs_POI3, INS_Pos_Rel_X_POI3, INS_Pos_Rel_Y_POI3,\
            INS_Vel_Hor_X_POI3, INS_Vel_Hor_Y_POI3, INS_Vel_Hor_Z_POI3,\
            Acc_Body_X_POI4, Acc_Body_Y_POI4, Acc_Body_Z_POI4,\
            Acc_Hor_X_POI4, Acc_Hor_Y_POI4, Acc_Hor_Z_POI4,\
            Inv_Path_Radius_POI4, Side_Slip_Angle_POI4, Dist_Trav_POI4, INS_Height_POI4,\
            INS_Lat_Abs_POI4, INS_Long_Abs_POI4, INS_Pos_Rel_X_POI4, INS_Pos_Rel_Y_POI4,\
            INS_Vel_Hor_X_POI4, INS_Vel_Hor_Y_POI4, INS_Vel_Hor_Z_POI4,\
            Acc_Body_X_POI5, Acc_Body_Y_POI5, Acc_Body_Z_POI5,\
            Acc_Hor_X_POI5, Acc_Hor_Y_POI5, Acc_Hor_Z_POI5,\
            Inv_Path_Radius_POI5, Side_Slip_Angle_POI5, Dist_Trav_POI5, INS_Height_POI5,\
            INS_Lat_Abs_POI5, INS_Long_Abs_POI5, INS_Pos_Rel_X_POI5, INS_Pos_Rel_Y_POI5,\
            INS_Vel_Hor_X_POI5, INS_Vel_Hor_Y_POI5, INS_Vel_Hor_Z_POI5,\
            Acc_Body_X_POI6, Acc_Body_Y_POI6, Acc_Body_Z_POI6,\
            Acc_Hor_X_POI6, Acc_Hor_Y_POI6, Acc_Hor_Z_POI6,\
            Inv_Path_Radius_POI6, Side_Slip_Angle_POI6, Dist_Trav_POI6, INS_Height_POI6,\
            INS_Lat_Abs_POI6, INS_Long_Abs_POI6, INS_Pos_Rel_X_POI6, INS_Pos_Rel_Y_POI6,\
            INS_Vel_Hor_X_POI6, INS_Vel_Hor_Y_POI6, INS_Vel_Hor_Z_POI6,\
            Acc_Body_X_POI7, Acc_Body_Y_POI7, Acc_Body_Z_POI7,\
            Acc_Hor_X_POI7, Acc_Hor_Y_POI7, Acc_Hor_Z_POI7,\
            Inv_Path_Radius_POI7, Side_Slip_Angle_POI7, Dist_Trav_POI7, INS_Height_POI7,\
            INS_Lat_Abs_POI7, INS_Long_Abs_POI7, INS_Pos_Rel_X_POI7, INS_Pos_Rel_Y_POI7,\
            INS_Vel_Hor_X_POI7, INS_Vel_Hor_Y_POI7, INS_Vel_Hor_Z_POI7,\
            Acc_Body_X_POI8, Acc_Body_Y_POI8, Acc_Body_Z_POI8,\
            Acc_Hor_X_POI8, Acc_Hor_Y_POI8, Acc_Hor_Z_POI8,\
            Inv_Path_Radius_POI8, Side_Slip_Angle_POI8, Dist_Trav_POI8, INS_Height_POI8,\
            INS_Lat_Abs_POI8, INS_Long_Abs_POI8, INS_Pos_Rel_X_POI8, INS_Pos_Rel_Y_POI8,\
            INS_Vel_Hor_X_POI8, INS_Vel_Hor_Y_POI8, INS_Vel_Hor_Z_POI8,\
            Status_GNSS_Mode, Status_Standstill, Status_Skidding, Status_External_Vel_Out,\
            Status_Trig_GNSS, Status_Signal_IN3, Status_Signal_IN2, Status_Signal_IN1,\
            Status_Alignment, Status_AHRS_INS, Status_Dead_Reckoning, Status_SyncLock,\
            Status_EVK_Activ, Status_EVK_Estimates, Status_Heading_Executed, Status_Config_Changed,\
            Status_Tilt, Status_Pos, Status_Count,\
            Status_Kalmanfilter_Settled, Status_KF_Lat_Stimulated, Status_KF_Long_Stimulated, Status_KF_steady_state,\
            Status_Speed, Status_Robot,\
            Error_Gyro_HW, Error_Accel_HW, Error_Ext_Speed_HW, Error_GNSS_HW,\
            Error_Data_Bus_Checksum, Error_Eeprom, Error_Xmit, Error_Cmd,\
            Error_Data_Bus, Error_CAN_Bus, Error_Num,\
            Error_Temp_Warning, Error_Reduced_Accuracy, Error_Range_Max,\
            Warn_GNSS_No_Solution, Warn_GNSS_Vel_Ignored, Warn_GNSS_Pos_Ignored, Warn_GNSS_Unable_To_Cfg,\
            Warn_Speed_Off, Warn_GNSS_Dualant_Ignored,\
            Error_HW_Sticky\
            ")                                                        

        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    l = Ros2CSVConverter()
    while rclpy.ok():
        rclpy.spin(l)


if __name__ == "__main__":
    main()


