import rclpy
from rclpy.node import Node
from adma_msgs.msg import AdmaDataScaled, AdmaStatus
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
            acc_body_hr_x, acc_body_hr_y, acc_body_hr_z,\
            rate_body_hr_x, rate_body_hr_y, rate_body_hr_z,\
            rate_body_x, rate_body_y, rate_body_z,\
            rate_hor_x, rate_hor_y, rate_hor_z,\
            acc_body_x, acc_body_y, acc_body_z,\
            acc_hor_x, acc_hor_y, acc_hor_z,\
            ext_vel_an_x, ext_vel_an_y,\
            ext_vel_dig_x, ext_vel_dig_y, ext_vel_dig_pulses_x, ext_vel_dig_pulses_y, ext_vel_x_corrected, ext_vel_y_corrected,\
            inv_path_radius, side_slip_angle, dist_trav,\
            trig_rising_1, trig_falling_1, trig_rising_2, trig_falling_2, trig_rising_3, trig_falling_3, trig_rising_4, trig_falling_4,\
            system_ta, system_temp, system_time_since_init, system_dsp_load,\
            gnss_lat_abs, gnss_long_abs, gnss_pos_rel_x, gnss_pos_rel_y,\
            gnss_stddev_lat, gnss_stddev_long, gnss_stddev_height,\
            gnss_vel_frame_x, gnss_vel_frame_y, gnss_vel_frame_z, gnss_vel_latency,\
            gnss_stddev_vel_x, gnss_stddev_vel_y, gnss_stddev_vel_z,\
            gnss_time_msec, gnss_time_week, gnss_trigger,\
            gnss_diffage, gnss_sats_used, gnss_sats_visible, gnss_sats_dualant_used, gnss_sats_dualant_visible,\
            gnss_log_delay, gnss_receiver_load, gnss_base_nr,\
            ins_roll, ins_pitch, ins_yaw, gnss_cog,\
            gnss_height, undulation, gnss_dualant_time_msec, gnss_dualant_time_week,\
            gnss_dualant_heading, gnss_dualant_pitch,\
            gnss_dualant_stddev_heading, gnss_dualant_stddev_pitch, gnss_dualant_stddev_heading_hr, gnss_dualant_stddev_pitch_hr,\
            ins_height, ins_time_msec, ins_time_week, leap_seconds,\
            ins_lat_abs, ins_long_abs, ins_pos_rel_x, ins_pos_rel_y,\
            ins_vel_hor_x, ins_vel_hor_y, ins_vel_hor_z,\
            ins_vel_frame_x, ins_vel_frame_y, ins_vel_frame_z,\
            ins_stddev_lat, ins_stddev_long, ins_stddev_height,\
            ins_stddev_vel_x, ins_stddev_vel_y, ins_stddev_vel_z,\
            ins_stddev_roll, ins_stddev_pitch, ins_stddev_yaw,\
            an1, an2, an3, an4,\
            gnss_receiver_error, gnss_receiver_status,\
            time_msec, time_nsec,\
            poi_1_acc_body_x, poi_1_acc_body_y, poi_1_acc_body_z,\
            poi_1_acc_hor_x, poi_1_acc_hor_y, poi_1_acc_hor_z,\
            poi_1_inv_path_radius, poi_1_side_slip_angle, poi_1_dist_trav, poi_1_ins_height,\
            poi_1_ins_lat_abs, poi_1_ins_long_abs, poi_1_ins_pos_rel_x, poi_1_ins_pos_rel_y,\
            poi_1_ins_vel_hor_x, poi_1_ins_vel_hor_y, poi_1_ins_vel_hor_z,\
            poi_2_acc_body_x, poi_2_acc_body_y, poi_2_acc_body_z,\
            poi_2_acc_hor_x, poi_2_acc_hor_y, poi_2_acc_hor_z,\
            poi_2_inv_path_radius, poi_2_side_slip_angle, poi_2_dist_trav, poi_2_ins_height,\
            poi_2_ins_lat_abs, poi_2_ins_long_abs, poi_2_ins_pos_rel_x, poi_2_ins_pos_rel_y,\
            poi_2_ins_vel_hor_x, poi_2_ins_vel_hor_y, poi_2_ins_vel_hor_z,\
            poi_3_acc_body_x, poi_3_acc_body_y, poi_3_acc_body_z,\
            poi_3_acc_hor_x, poi_3_acc_hor_y, poi_3_acc_hor_z,\
            poi_3_inv_path_radius, poi_3_side_slip_angle, poi_3_dist_trav, poi_3_ins_height,\
            poi_3_ins_lat_abs, poi_3_ins_long_abs, poi_3_ins_pos_rel_x, poi_3_ins_pos_rel_y,\
            poi_3_ins_vel_hor_x, poi_3_ins_vel_hor_y, poi_3_ins_vel_hor_z,\
            poi_4_acc_body_x, poi_4_acc_body_y, poi_4_acc_body_z,\
            poi_4_acc_hor_x, poi_4_acc_hor_y, poi_4_acc_hor_z,\
            poi_4_inv_path_radius, poi_4_side_slip_angle, poi_4_dist_trav, poi_4_ins_height,\
            poi_4_ins_lat_abs, poi_4_ins_long_abs, poi_4_ins_pos_rel_x, poi_4_ins_pos_rel_y,\
            poi_4_ins_vel_hor_x, poi_4_ins_vel_hor_y, poi_4_ins_vel_hor_z,\
            poi_5_acc_body_x, poi_5_acc_body_y, poi_5_acc_body_z,\
            poi_5_acc_hor_x, poi_5_acc_hor_y, poi_5_acc_hor_z,\
            poi_5_inv_path_radius, poi_5_side_slip_angle, poi_5_dist_trav, poi_5_ins_height,\
            poi_5_ins_lat_abs, poi_5_ins_long_abs, poi_5_ins_pos_rel_x, poi_5_ins_pos_rel_y,\
            poi_5_ins_vel_hor_x, poi_5_ins_vel_hor_y, poi_5_ins_vel_hor_z,\
            poi_6_acc_body_x, poi_6_acc_body_y, poi_6_acc_body_z,\
            poi_6_acc_hor_x, poi_6_acc_hor_y, poi_6_acc_hor_z,\
            poi_6_inv_path_radius, poi_6_side_slip_angle, poi_6_dist_trav, poi_6_ins_height,\
            poi_6_ins_lat_abs, poi_6_ins_long_abs, poi_6_ins_pos_rel_x, poi_6_ins_pos_rel_y,\
            poi_6_ins_vel_hor_x, poi_6_ins_vel_hor_y, poi_6_ins_vel_hor_z,\
            poi_7_acc_body_x, poi_7_acc_body_y, poi_7_acc_body_z,\
            poi_7_acc_hor_x, poi_7_acc_hor_y, poi_7_acc_hor_z,\
            poi_7_inv_path_radius, poi_7_side_slip_angle, poi_7_dist_trav, poi_7_ins_height,\
            poi_7_ins_lat_abs, poi_7_ins_long_abs, poi_7_ins_pos_rel_x, poi_7_ins_pos_rel_y,\
            poi_7_ins_vel_hor_x, poi_7_ins_vel_hor_y, poi_7_ins_vel_hor_z,\
            poi_8_acc_body_x, poi_8_acc_body_y, poi_8_acc_body_z,\
            poi_8_acc_hor_x, poi_8_acc_hor_y, poi_8_acc_hor_z,\
            poi_8_inv_path_radius, poi_8_side_slip_angle, poi_8_dist_trav, poi_8_ins_height,\
            poi_8_ins_lat_abs, poi_8_ins_long_abs, poi_8_ins_pos_rel_x, poi_8_ins_pos_rel_y,\
            poi_8_ins_vel_hor_x, poi_8_ins_vel_hor_y, poi_8_ins_vel_hor_z,\
            status_gnss_mode, status_standstill, status_skidding, status_external_vel_out,\
            status_trig_gnss, status_signal_in3, status_signal_in2, status_signal_in1,\
            status_alignment, status_ahrs_ins, status_dead_reckoning, status_synclock,\
            status_evk_activ, status_evk_estimates, status_heading_executed, status_config_changed,\
            status_tilt, status_pos, status_count,\
            status_kalmanfilter_settled, status_kf_lat_stimulated, status_kf_long_stimulated, status_kf_steady_state,\
            status_speed, status_robot,\
            error_gyro_hw, error_accel_hw, error_ext_speed_hw, error_gnss_hw,\
            error_data_bus_checksum, error_eeprom, error_xmit, error_cmd,\
            error_data_bus, error_can_bus, error_num,\
            error_temp_warning, error_reduced_accuracy, error_range_max,\
            warn_gnss_no_solution, warn_gnss_vel_ignored, warn_gnss_pos_ignored, warn_gnss_unable_to_cfg,\
            warn_speed_off, warn_gnss_dualant_ignored,\
            error_hw_sticky\
            ")                                                        

        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    l = Ros2CSVConverter()
    while rclpy.ok():
        rclpy.spin(l)


if __name__ == "__main__":
    main()


