#include "adma_ros2_driver/parser/adma2ros_parser.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>

#include "adma_core_lib/parser/parser_utils.hpp"

ADMA2ROSParser::ADMA2ROSParser(u_int16_t version)
: protocolVersion_(version){
  if(protocolVersion_ == 3200){
    parserV32_ = new ADMA2ROSParserV32();
  } else {
    mapping_ = new genesys::parser::Mapping(version);
  }
}

void ADMA2ROSParser::extractHeading(std_msgs::msg::Float64 &headingMsg, std::array<char, 856> & recv_data)
{
  headingMsg.data = mapping_->loadDataFromBuffer<uint16_t, double>("heading", recv_data);
}

void ADMA2ROSParser::extractAdmaStatus(adma_ros_driver_msgs::msg::AdmaStatus &statusMsg, std::array<char, 856> & recv_data)
{
  // first load bytes from buffer into ROS msg
  statusMsg.status_bytes.status_byte_0 = mapping_->loadDataFromBuffer<unsigned char, unsigned char>("status.status_bytes.status_byte_0", recv_data);
  statusMsg.status_bytes.status_byte_1 = mapping_->loadDataFromBuffer<unsigned char, unsigned char>("status.status_bytes.status_byte_1", recv_data);
  statusMsg.status_bytes.status_byte_2 = mapping_->loadDataFromBuffer<unsigned char, unsigned char>("status.status_bytes.status_byte_2", recv_data);
  statusMsg.status_bytes.status_count = mapping_->loadDataFromBuffer<unsigned char, unsigned char>("status.status_bytes.status_count", recv_data);
  statusMsg.status_bytes.status_byte_4 = mapping_->loadDataFromBuffer<unsigned char, unsigned char>("status.status_bytes.status_byte_4", recv_data);
  statusMsg.status_bytes.status_byte_5 = mapping_->loadDataFromBuffer<unsigned char, unsigned char>("status.status_bytes.status_byte_5", recv_data);
  statusMsg.error_warnings_bytes.error_1 = mapping_->loadDataFromBuffer<unsigned char, unsigned char>("status.error_warnings_bytes.error_byte_0", recv_data);
  statusMsg.error_warnings_bytes.error_2 = mapping_->loadDataFromBuffer<unsigned char, unsigned char>("status.error_warnings_bytes.error_byte_1", recv_data);
  statusMsg.error_warnings_bytes.warn_1 = mapping_->loadDataFromBuffer<unsigned char, unsigned char>("status.error_warnings_bytes.error_byte_2", recv_data);
  statusMsg.error_warnings_bytes.error_3 = mapping_->loadDataFromBuffer<unsigned char, unsigned char>("status.error_warnings_bytes.error_byte_3", recv_data);

  // then extract single bits
  statusMsg.error_warnings.error_gyro_hw = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.error_gyro_hw", recv_data);
  statusMsg.error_warnings.error_accel_hw = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.error_accel_hw", recv_data);
  statusMsg.error_warnings.error_ext_speed_hw = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.error_ext_speed_hw", recv_data);
  statusMsg.error_warnings.error_gnss_hw = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.error_gnss_hw", recv_data);
  statusMsg.error_warnings.error_data_bus_checksum = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.error_data_bus_checksum", recv_data);
  statusMsg.error_warnings.error_eeprom = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.error_eeprom", recv_data);
  statusMsg.error_warnings.error_cmd = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.error_cmd", recv_data);
  statusMsg.error_warnings.error_data_bus = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.error_data_bus", recv_data);
  statusMsg.error_warnings.error_can_bus = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.error_can_bus", recv_data);
  statusMsg.error_warnings.error_num = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.error_num", recv_data);
  statusMsg.error_warnings.error_temp_warning = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.error_temp_warning", recv_data);
  statusMsg.error_warnings.error_reduced_accuracy = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.error_reduced_accuracy", recv_data);
  statusMsg.error_warnings.error_range_max = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.error_range_max", recv_data);
  statusMsg.error_warnings.warn_gnss_no_solution = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.warn_gnss_no_solution", recv_data);
  statusMsg.error_warnings.warn_gnss_vel_ignored = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.warn_gnss_vel_ignored", recv_data);
  statusMsg.error_warnings.warn_gnss_pos_ignored = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.warn_gnss_pos_ignored", recv_data);
  statusMsg.error_warnings.warn_gnss_unable_to_cfg = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.warn_gnss_unable_to_cfg", recv_data);
  statusMsg.error_warnings.warn_speed_off = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.warn_speed_off", recv_data);
  statusMsg.error_warnings.warn_gnss_dualant_ignored = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.warn_gnss_dualant_ignored", recv_data);
  statusMsg.error_warnings.error_hw_sticky = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.error_warnings.error_hw_sticky", recv_data);

  // status_byte_0 (2100)
  statusMsg.status.status_gnss_mode = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_gnss_mode", recv_data);
  statusMsg.status.status_standstill = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_standstill", recv_data);
  statusMsg.status.status_skidding = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_skidding", recv_data);
  statusMsg.status.status_external_vel_out = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_external_vel_out", recv_data);
  // status_byte_1 (2101)
  statusMsg.status.status_trig_gnss = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_trig_gnss", recv_data);
  statusMsg.status.status_signal_in3 = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_signal_in3", recv_data);
  statusMsg.status.status_signal_in2 = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_signal_in2", recv_data);
  statusMsg.status.status_signal_in1 = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_signal_in1", recv_data);
  statusMsg.status.status_alignment = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_alignment", recv_data);
  statusMsg.status.status_ahrs_ins = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_ahrs_ins", recv_data);
  statusMsg.status.status_dead_reckoning = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_dead_reckoning", recv_data);
  statusMsg.status.status_synclock = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_synclock", recv_data);
  // status_byte_2 (2102)
  statusMsg.status.status_evk_activ = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_evk_activ", recv_data);
  statusMsg.status.status_evk_estimates = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_evk_estimates", recv_data);
  statusMsg.status.status_heading_executed = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_heading_executed", recv_data);
  statusMsg.status.status_config_changed = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_config_changed", recv_data);
  statusMsg.status.status_tilt = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_tilt", recv_data);
  statusMsg.status.status_pos = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_pos", recv_data);
  // status_byte_3 (2103)
  statusMsg.status.status_count = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_count", recv_data);
  // status_byte_4 (2104)
  statusMsg.status.status_kalmanfilter_settled = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_kalmanfilter_settled", recv_data);
  statusMsg.status.status_kf_lat_stimulated = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_kf_lat_stimulated", recv_data);
  statusMsg.status.status_kf_long_stimulated = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_kf_long_stimulated", recv_data);
  statusMsg.status.status_kf_steady_state = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_kf_steady_state", recv_data);
  statusMsg.status.status_speed = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_speed", recv_data);
  statusMsg.status.status_ips_mode = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_ips_mode", recv_data);
  // status_byte_5 (2105)
  statusMsg.status.status_robot = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_robot", recv_data);
  statusMsg.status.status_dualant_mode = mapping_->loadDataFromBuffer<unsigned char, uint8_t>("status.status.status_dualant_mode", recv_data);
}

void ADMA2ROSParser::extractAdmaDataScaled(adma_ros_driver_msgs::msg::AdmaDataScaled &admaScaledMsg, std::array<char, 856> & recv_data)
{
  mapping_->loadVector3FromBuffer("data_scaled.acc_body_hr", recv_data, admaScaledMsg.acc_body_hr);
  mapping_->loadVector3FromBuffer("data_scaled.rate_body_hr", recv_data, admaScaledMsg.rate_body_hr);
  mapping_->loadVector3FromBuffer("data_scaled.rate_body", recv_data, admaScaledMsg.rate_body);
  mapping_->loadVector3FromBuffer("data_scaled.rate_hor", recv_data, admaScaledMsg.rate_hor);
  mapping_->loadVector3FromBuffer("data_scaled.acc_body", recv_data, admaScaledMsg.acc_body);
  mapping_->loadVector3FromBuffer("data_scaled.acc_hor", recv_data, admaScaledMsg.acc_hor);

  admaScaledMsg.ext_vel_an_x = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.ext_vel_an_x", recv_data);
  admaScaledMsg.ext_vel_an_y = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.ext_vel_an_y", recv_data);
  admaScaledMsg.ext_vel_dig_x = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.ext_vel_dig_x", recv_data);
  admaScaledMsg.ext_vel_dig_y = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.ext_vel_dig_y", recv_data);
  admaScaledMsg.ext_vel_dig_pulses_x = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.ext_vel_dig_pulses_x", recv_data);
  admaScaledMsg.ext_vel_dig_pulses_y = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.ext_vel_dig_pulses_y", recv_data);
  admaScaledMsg.ext_vel_x_corrected = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.ext_vel_x_corrected", recv_data);
  admaScaledMsg.ext_vel_y_corrected = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.ext_vel_y_corrected", recv_data);
  admaScaledMsg.inv_path_radius = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.inv_path_radius", recv_data);
  admaScaledMsg.side_slip_angle = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.side_slip_angle", recv_data);
  admaScaledMsg.dist_trav = mapping_->loadDataFromBuffer<uint32_t, double>("data_scaled.dist_trav", recv_data);
  admaScaledMsg.trig_rising_1 = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.trig_rising_1", recv_data);
  admaScaledMsg.trig_falling_1 = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.trig_falling_1", recv_data);
  admaScaledMsg.trig_rising_2 = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.trig_rising_2", recv_data);
  admaScaledMsg.trig_falling_2 = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.trig_falling_2", recv_data);
  admaScaledMsg.trig_rising_3 = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.trig_rising_3", recv_data);
  admaScaledMsg.trig_falling_3 = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.trig_falling_3", recv_data);
  admaScaledMsg.trig_rising_4 = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.trig_rising_4", recv_data);
  admaScaledMsg.trig_falling_4 = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.trig_falling_4", recv_data);
  admaScaledMsg.system_ta = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.system_ta", recv_data);
  admaScaledMsg.system_temp = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.system_temp", recv_data);
  admaScaledMsg.system_time_since_init = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.system_time_since_init", recv_data);
  admaScaledMsg.system_dsp_load = mapping_->loadDataFromBuffer<uint16_t, double>("data_scaled.system_dsp_load", recv_data);
  admaScaledMsg.gnss_lat_abs = mapping_->loadDataFromBuffer<int32_t, double>("data_scaled.gnss_lat_abs", recv_data);
  admaScaledMsg.gnss_long_abs = mapping_->loadDataFromBuffer<int32_t, double>("data_scaled.gnss_long_abs", recv_data);
  admaScaledMsg.gnss_pos_rel_x = mapping_->loadDataFromBuffer<int32_t, double>("data_scaled.gnss_pos_rel_x", recv_data);
  admaScaledMsg.gnss_pos_rel_y = mapping_->loadDataFromBuffer<int32_t, double>("data_scaled.gnss_pos_rel_y", recv_data);
  admaScaledMsg.gnss_stddev_lat = mapping_->loadDataFromBuffer<uint16_t, double>("data_scaled.gnss_stddev_lat", recv_data);
  admaScaledMsg.gnss_stddev_long = mapping_->loadDataFromBuffer<uint16_t, double>("data_scaled.gnss_stddev_long", recv_data);
  admaScaledMsg.gnss_stddev_height = mapping_->loadDataFromBuffer<uint16_t, double>("data_scaled.gnss_stddev_height", recv_data);
  admaScaledMsg.gnss_stddev_cog = mapping_->loadDataFromBuffer<uint16_t, double>("data_scaled.gnss_stddev_cog", recv_data);

  mapping_->loadVector3FromBuffer("data_scaled.gnss_vel_frame", recv_data, admaScaledMsg.gnss_vel_frame);
  admaScaledMsg.gnss_vel_latency = mapping_->loadDataFromBuffer<uint16_t, double>("data_scaled.gnss_vel_latency", recv_data);
  mapping_->loadVector3FromBuffer("data_scaled.gnss_stddev_vel", recv_data, admaScaledMsg.gnss_stddev_vel);

  admaScaledMsg.gnss_time_msec = mapping_->loadDataFromBuffer<uint32_t, uint32_t>("data_scaled.gnss_time_msec", recv_data);
  admaScaledMsg.gnss_time_week = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.gnss_time_week", recv_data);
  admaScaledMsg.gnss_trigger = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.trigger_gnss", recv_data);
  admaScaledMsg.gnss_diffage = mapping_->loadDataFromBuffer<uint16_t, double>("data_scaled.gnss_diffage", recv_data);
  admaScaledMsg.gnss_sats_used = mapping_->loadDataFromBuffer<unsigned char, int8_t>("data_scaled.gnss_sats_used", recv_data);
  admaScaledMsg.gnss_sats_visible = mapping_->loadDataFromBuffer<unsigned char, int8_t>("data_scaled.gnss_sats_visible", recv_data);
  admaScaledMsg.gnss_sats_dualant_used = mapping_->loadDataFromBuffer<unsigned char, int8_t>("data_scaled.gnss_sats_dualant_used", recv_data);
  admaScaledMsg.gnss_sats_dualant_visible = mapping_->loadDataFromBuffer<unsigned char, int8_t>("data_scaled.gnss_sats_dualant_visible", recv_data);
  admaScaledMsg.gnss_sats_single_freq = mapping_->loadDataFromBuffer<unsigned char, int8_t>("data_scaled.gnss_sats_single_freq", recv_data);
  admaScaledMsg.gnss_sats_multi_freq = mapping_->loadDataFromBuffer<unsigned char, int8_t>("data_scaled.gnss_sats_multi_freq", recv_data);
  admaScaledMsg.gnss_log_delay = mapping_->loadDataFromBuffer<unsigned char, int8_t>("data_scaled.gnss_log_delay", recv_data);
  admaScaledMsg.gnss_receiver_load = mapping_->loadDataFromBuffer<unsigned char, double>("data_scaled.gnss_receiver_load", recv_data);
  // admaScaledMsg.gnss_base_nr = mapping_->loadDataFromBuffer<unsigned char[4], std::string>("data_scaled.gnss_base_nr", recv_data);
  admaScaledMsg.gnss_sats_dualant_multi_freq = mapping_->loadDataFromBuffer<unsigned char, int8_t>("data_scaled.gnss_sats_dualant_multi_freq", recv_data);
  admaScaledMsg.ins_roll = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.ins_roll", recv_data);
  admaScaledMsg.ins_pitch = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.ins_pitch", recv_data);
  admaScaledMsg.ins_yaw = mapping_->loadDataFromBuffer<uint16_t, double>("data_scaled.ins_yaw", recv_data);
  admaScaledMsg.gnss_cog = mapping_->loadDataFromBuffer<uint16_t, double>("data_scaled.gnss_cog", recv_data);
  admaScaledMsg.gnss_height = mapping_->loadDataFromBuffer<int32_t, double>("data_scaled.gnss_height", recv_data);
  admaScaledMsg.undulation = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.undulation", recv_data);
  admaScaledMsg.gnss_dualant_time_msec = mapping_->loadDataFromBuffer<uint32_t, uint32_t>("data_scaled.gnss_dualant_time_msec", recv_data);
  admaScaledMsg.gnss_dualant_time_week = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.gnss_dualant_time_week", recv_data);
  admaScaledMsg.gnss_dualant_heading = mapping_->loadDataFromBuffer<uint16_t, double>("data_scaled.gnss_dualant_heading", recv_data);
  admaScaledMsg.gnss_dualant_pitch = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.gnss_dualant_pitch", recv_data);
  admaScaledMsg.gnss_dualant_stddev_heading = mapping_->loadDataFromBuffer<unsigned char, double>("data_scaled.gnss_dualant_stddev_heading", recv_data);
  admaScaledMsg.gnss_dualant_stddev_pitch = mapping_->loadDataFromBuffer<unsigned char, double>("data_scaled.gnss_dualant_stddev_pitch", recv_data);
  admaScaledMsg.gnss_dualant_stddev_heading_hr = mapping_->loadDataFromBuffer<uint16_t, float>("data_scaled.gnss_dualant_stddev_heading_hr", recv_data);
  admaScaledMsg.gnss_dualant_stddev_pitch_hr = mapping_->loadDataFromBuffer<uint16_t, float>("data_scaled.gnss_dualant_stddev_pitch_hr", recv_data);
  admaScaledMsg.ins_height = mapping_->loadDataFromBuffer<int32_t, double>("data_scaled.ins_height", recv_data);
  admaScaledMsg.ins_yaw_rel = mapping_->loadDataFromBuffer<uint16_t, double>("data_scaled.ins_yaw_rel", recv_data);
  admaScaledMsg.ins_time_msec = mapping_->loadDataFromBuffer<uint32_t, uint32_t>("data_scaled.ins_time_msec", recv_data);
  admaScaledMsg.ins_time_week = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("data_scaled.ins_time_week", recv_data);
  admaScaledMsg.leap_seconds = mapping_->loadDataFromBuffer<int16_t, int16_t>("data_scaled.leap_seconds", recv_data);
  admaScaledMsg.ins_lat_abs = mapping_->loadDataFromBuffer<int32_t, double>("data_scaled.ins_lat_abs", recv_data);
  admaScaledMsg.ins_long_abs = mapping_->loadDataFromBuffer<int32_t, double>("data_scaled.ins_long_abs", recv_data);
  admaScaledMsg.ins_pos_rel_x = mapping_->loadDataFromBuffer<int32_t, double>("data_scaled.ins_pos_rel_x", recv_data);
  admaScaledMsg.ins_pos_rel_y = mapping_->loadDataFromBuffer<int32_t, double>("data_scaled.ins_pos_rel_y", recv_data);

  mapping_->loadVector3FromBuffer("data_scaled.ins_vel_hor", recv_data, admaScaledMsg.ins_vel_hor);
  admaScaledMsg.ins_stddev_lat = mapping_->loadDataFromBuffer<uint16_t, double>("data_scaled.ins_stddev_lat", recv_data);
  admaScaledMsg.ins_stddev_long = mapping_->loadDataFromBuffer<uint16_t, double>("data_scaled.ins_stddev_long", recv_data);
  admaScaledMsg.ins_stddev_height = mapping_->loadDataFromBuffer<uint16_t, double>("data_scaled.ins_stddev_height", recv_data);
  mapping_->loadVector3FromBuffer("data_scaled.ins_vel_frame", recv_data, admaScaledMsg.ins_vel_frame);
  mapping_->loadVector3FromBuffer("data_scaled.ins_stddev_vel", recv_data, admaScaledMsg.ins_stddev_vel);

  admaScaledMsg.ins_stddev_roll = mapping_->loadDataFromBuffer<int8_t, double>("data_scaled.ins_stddev_roll", recv_data);
  admaScaledMsg.ins_stddev_pitch = mapping_->loadDataFromBuffer<int8_t, double>("data_scaled.ins_stddev_pitch", recv_data);
  admaScaledMsg.ins_stddev_yaw = mapping_->loadDataFromBuffer<int8_t, double>("data_scaled.ins_stddev_yaw", recv_data);
  admaScaledMsg.an1 = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.an1", recv_data);
  admaScaledMsg.an2 = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.an2", recv_data);
  admaScaledMsg.an3 = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.an3", recv_data);
  admaScaledMsg.an4 = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled.an4", recv_data);
  admaScaledMsg.kf_lat_stimulated = mapping_->loadDataFromBuffer<uint8_t, uint8_t>("data_scaled.kf_lat_stimulated", recv_data);
  admaScaledMsg.kf_long_stimulated = mapping_->loadDataFromBuffer<uint8_t, uint8_t>("data_scaled.kf_long_stimulated", recv_data);
  admaScaledMsg.kf_steady_state = mapping_->loadDataFromBuffer<uint8_t, uint8_t>("data_scaled.kf_steady_state", recv_data);
  admaScaledMsg.gnss_receiver_error = mapping_->loadDataFromBuffer<uint32_t, uint32_t>("data_scaled.gnss_receiver_error", recv_data);
  admaScaledMsg.gnss_receiver_status = mapping_->loadDataFromBuffer<uint32_t, uint32_t>("data_scaled.gnss_receiver_status", recv_data);
}

void ADMA2ROSParser::extractPOIs(adma_ros_driver_msgs::msg::AdmaDataScaled &admaScaledMsg, std::array<char, 856> &recv_data)
{
  // first create a array for all available POI
  std::array<adma_ros_driver_msgs::msg::POI, 8> pois;
  // then fill the POIs with data
  for (size_t i = 0; i < 8; i++) {
    adma_ros_driver_msgs::msg::POI new_poi;
    std::string poiName = "poi_" + std::to_string(i+1);
    mapping_->loadVector3FromBuffer("data_scaled." + poiName + ".acc_body", recv_data, new_poi.acc_body);
    mapping_->loadVector3FromBuffer("data_scaled." + poiName + ".acc_hor", recv_data, new_poi.acc_hor);
    mapping_->loadVector3FromBuffer("data_scaled." + poiName + ".ins_vel_hor", recv_data, new_poi.ins_vel_hor);
    new_poi.inv_path_radius = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled." + poiName + ".inv_path_radius", recv_data);
    new_poi.side_slip_angle = mapping_->loadDataFromBuffer<int16_t, double>("data_scaled." + poiName + ".side_slip_angle", recv_data);
    new_poi.dist_trav = mapping_->loadDataFromBuffer<uint32_t, double>("data_scaled." + poiName + ".dist_trav", recv_data);
    new_poi.ins_height = mapping_->loadDataFromBuffer<int32_t, double>("data_scaled." + poiName + ".ins_height", recv_data);
    new_poi.ins_lat_abs = mapping_->loadDataFromBuffer<int32_t, double>("data_scaled." + poiName + ".ins_lat_abs", recv_data);
    new_poi.ins_long_abs = mapping_->loadDataFromBuffer<int32_t, double>("data_scaled." + poiName + ".ins_long_abs", recv_data);
    new_poi.ins_pos_rel_x = mapping_->loadDataFromBuffer<int32_t, double>("data_scaled." + poiName + ".ins_pos_rel_x", recv_data);
    new_poi.ins_pos_rel_y = mapping_->loadDataFromBuffer<int32_t, double>("data_scaled." + poiName + ".ins_pos_rel_y", recv_data);
    pois[i] = new_poi;
  }
  // finally set the filled POI data to the whole ROS msg
  admaScaledMsg.poi_1 = pois[0];
  admaScaledMsg.poi_2 = pois[1];
  admaScaledMsg.poi_3 = pois[2];
  admaScaledMsg.poi_4 = pois[3];
  admaScaledMsg.poi_5 = pois[4];
  admaScaledMsg.poi_6 = pois[5];
  admaScaledMsg.poi_7 = pois[6];
  admaScaledMsg.poi_8 = pois[7];
  
}

void ADMA2ROSParser::mapAdmaMessageToROS(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, std::array<char, 856> & recv_data)
{
  AdmaDataV32 adma_data;
  memcpy(&adma_data, &recv_data, sizeof(adma_data));
  getStatusGPS(ros_msg, adma_data.gpsStatus);
  getStatusTrigger(ros_msg, adma_data.gpsTriggerStatus);
  getEVKStatus(ros_msg, adma_data.evkStatus);
  unsigned char ew_bytes[] = {
    adma_data.dataError1, adma_data.dataError2, adma_data.dataWarn1, adma_data.dataErrorHW};
  getErrorandWarning(ros_msg, ew_bytes);
  parserV32_->mapAdmaMessageToROS(ros_msg, adma_data);
  parseScaledData(ros_msg);
}

/// \file
/// \brief  getstatusgps function - adma status information
/// \param  ros_msg ros message to fill with content
/// \param  gps_status byte with gps states
void ADMA2ROSParser::getStatusGPS(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char gps_status)
{
  bool status_external_vel = getbit(gps_status, 7);
  bool status_skidding = getbit(gps_status, 5);
  bool standstill_c = getbit(gps_status, 4);
  bool rtk_precise = getbit(gps_status, 3);
  bool rtk_coarse = getbit(gps_status, 2);
  bool gps_mode = getbit(gps_status, 1);
  bool gps_out = getbit(gps_status, 0);

  /* status gps mode */
  if (gps_out) {
    ros_msg.statusgpsmode = 1;
  } else if (gps_mode) {
    ros_msg.statusgpsmode = 2;
  } else if (rtk_coarse) {
    ros_msg.statusgpsmode = 4;
  } else if (rtk_precise) {
    ros_msg.statusgpsmode = 8;
  }
  /* status stand still */
  ros_msg.statusstandstill = standstill_c;
  /* status skidding */
  ros_msg.statusskidding = status_skidding;
  /* status external velocity slip */
  ros_msg.statusexternalvelout = status_external_vel;
}

/// \file
/// \brief  getstatustrigger function - adma gps trigger information
/// \param  ros_msg ros message to fill with content
/// \param  gps_trigger_status byte with gps trigger states
void ADMA2ROSParser::getStatusTrigger(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char gps_trigger_status)
{
  bool status_synclock = getbit(gps_trigger_status, 7);
  bool status_dead_reckoning = getbit(gps_trigger_status, 6);
  bool status_ahrs_ins = getbit(gps_trigger_status, 5);
  bool status_alignment = getbit(gps_trigger_status, 4);
  bool status_signal_in1 = getbit(gps_trigger_status, 3);
  bool status_signal_in2 = getbit(gps_trigger_status, 2);
  bool status_signal_in3 = getbit(gps_trigger_status, 1);
  bool status_trig_gps = getbit(gps_trigger_status, 0);
  /* status statustriggps */
  ros_msg.statustriggps = status_trig_gps;
  /* status statussignalin3 */
  ros_msg.statussignalin3 = status_signal_in3;
  /* status statussignalin2 */
  ros_msg.statussignalin2 = status_signal_in2;
  /* status statussignalin1 */
  ros_msg.statussignalin1 = status_signal_in1;
  /* status statusalignment */
  ros_msg.statusalignment = status_alignment;
  /* status statusahrsins */
  ros_msg.statusahrsins = status_ahrs_ins;
  /* status statusdeadreckoning */
  ros_msg.statusdeadreckoning = status_dead_reckoning;
  /* status statussynclock */
  ros_msg.statussynclock = status_synclock;
}

/// \file
/// \brief  getstatustrigger function - adma gps trigger information
/// \param  ros_msg ros message to fill with content
/// \param  evk_status byte with evk states
void ADMA2ROSParser::getEVKStatus(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char evk_status)
{
  bool status_pos_b2 = getbit(evk_status, 7);
  bool status_pos_b1 = getbit(evk_status, 6);
  bool status_tilt_b2 = getbit(evk_status, 5);
  bool status_tilt_b1 = getbit(evk_status, 4);
  bool status_configuration_changed = getbit(evk_status, 3);
  bool status_heading_executed = getbit(evk_status, 2);
  bool status_evk_estimates = getbit(evk_status, 1);
  bool status_evk_activ = getbit(evk_status, 0);
  /* status statustriggps */
  ros_msg.statusevkactiv = status_evk_activ;
  /* status status_evk_estimates */
  ros_msg.statusevkestimates = status_evk_estimates;
  /* status status_heading_executed */
  ros_msg.statusheadingexecuted = status_heading_executed;
  /* status status_configuration_changed */
  ros_msg.statusconfigurationchanged = status_configuration_changed;
  /* status tilt */
  if (status_tilt_b1 == 0 && status_tilt_b2 == 0) {
    ros_msg.statustilt = 0;
  } else if (status_tilt_b1 == 0 && status_tilt_b2 == 1) {
    ros_msg.statustilt = 1;
  } else if (status_tilt_b1 == 1 && status_tilt_b2 == 0) {
    ros_msg.statustilt = 2;
  }
  /* status pos */
  if (status_pos_b1 == 0 && status_pos_b2 == 0) {
    ros_msg.statuspos = 0;
  } else if (status_pos_b1 == 0 && status_pos_b2 == 1) {
    ros_msg.statuspos = 1;
  } else if (status_pos_b1 == 1 && status_pos_b2 == 0) {
    ros_msg.statuspos = 2;
  }
}

/// \file
/// \brief  geterrorandwarning function - adma error and warning
/// \param  ros_msg ros message to fill with content
/// \param  ew_bytes array of bytes with several error and warnings
void ADMA2ROSParser::getErrorandWarning(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char ew_bytes[4])
{
  std::bitset<8> bitdataerror1 = ew_bytes[0];
  std::bitset<8> bitdataerror2 = ew_bytes[1];
  std::bitset<8> bitdatawarn3 = ew_bytes[2];
  std::bitset<8> errorhw = ew_bytes[3];
  std::bitset<4> erhw1;
  std::bitset<4> ermisc1;
  std::bitset<4> ermisc2;
  std::bitset<4> ermisc3;
  std::bitset<4> warngps;
  std::bitset<4> warnmisc1;
  std::bitset<1> erhwsticky;

  for (size_t i = 0; i < 4; i++) {
    erhw1[i] = bitdataerror1[i];
    ermisc1[i] = bitdataerror1[i + 4];
    ermisc2[i] = bitdataerror2[i];
    ermisc3[i] = bitdataerror2[i + 4];
    warngps[i] = bitdatawarn3[i];
    warnmisc1[i] = bitdatawarn3[i + 4];
  }
  erhwsticky[0] = errorhw[1];
  ros_msg.errorhardware = erhw1.to_string();
  ros_msg.error_misc1 = ermisc1.to_string();
  ros_msg.error_misc2 = ermisc2.to_string();
  ros_msg.error_misc3 = ermisc3.to_string();
  ros_msg.warngps = warngps.to_string();
  ros_msg.warnmisc1 = warnmisc1.to_string();
  ros_msg.errorhwsticky = erhwsticky.to_string();
}

/// \file
/// \brief  pareScaledData function - fills scaled values with LSB factor
/// \param  ros_msg ros message to fill with content
void ADMA2ROSParser::parseScaledData(adma_ros_driver_msgs::msg::AdmaData & ros_msg)
{
  ros_msg.faccbodyhrx = getScaledValue(ros_msg.accbodyhrx, 0.0001);
  ros_msg.fratebodyhrx = getScaledValue(ros_msg.ratebodyhrx, 0.0001);
  ros_msg.faccbodyhry = getScaledValue(ros_msg.accbodyhry, 0.0001);
  ros_msg.fratebodyhry = getScaledValue(ros_msg.ratebodyhry, 0.0001);
  ros_msg.faccbodyhrz = getScaledValue(ros_msg.accbodyhrz, 0.0001);
  ros_msg.fratebodyhrz = getScaledValue(ros_msg.ratebodyhrz, 0.0001);

  ros_msg.fratebodyx = getScaledValue(ros_msg.ratebodyx, 0.01);
  ros_msg.fratebodyy = getScaledValue(ros_msg.ratebodyy, 0.01);
  ros_msg.fratebodyz = getScaledValue(ros_msg.ratebodyz, 0.01);
  ros_msg.fratehorx = getScaledValue(ros_msg.ratehorx, 0.01);
  ros_msg.fratehory = getScaledValue(ros_msg.ratehory, 0.01);
  ros_msg.fratehorz = getScaledValue(ros_msg.ratehorz, 0.01);

  ros_msg.faccbodyx = getScaledValue(ros_msg.accbodyx, 0.0004);
  ros_msg.faccbodyy = getScaledValue(ros_msg.accbodyy, 0.0004);
  ros_msg.faccbodyz = getScaledValue(ros_msg.accbodyz, 0.0004);
  ros_msg.facchorx = getScaledValue(ros_msg.acchorx, 0.0004);
  ros_msg.facchory = getScaledValue(ros_msg.acchory, 0.0004);
  ros_msg.facchorz = getScaledValue(ros_msg.acchorz, 0.0004);

  ros_msg.faccbodyx_1 = getScaledValue(ros_msg.accbodyx_1, 0.0004);
  ros_msg.faccbodyy_1 = getScaledValue(ros_msg.accbodyy_1, 0.0004);
  ros_msg.faccbodyz_1 = getScaledValue(ros_msg.accbodyz_1, 0.0004);
  ros_msg.faccbodyx_2 = getScaledValue(ros_msg.accbodyx_2, 0.0004);
  ros_msg.faccbodyy_2 = getScaledValue(ros_msg.accbodyy_2, 0.0004);
  ros_msg.faccbodyz_2 = getScaledValue(ros_msg.accbodyz_2, 0.0004);
  ros_msg.faccbodyx_3 = getScaledValue(ros_msg.accbodyx_3, 0.0004);
  ros_msg.faccbodyy_3 = getScaledValue(ros_msg.accbodyy_3, 0.0004);
  ros_msg.faccbodyz_3 = getScaledValue(ros_msg.accbodyz_3, 0.0004);
  ros_msg.faccbodyx_4 = getScaledValue(ros_msg.accbodyx_4, 0.0004);
  ros_msg.faccbodyy_4 = getScaledValue(ros_msg.accbodyy_4, 0.0004);
  ros_msg.faccbodyz_4 = getScaledValue(ros_msg.accbodyz_4, 0.0004);
  ros_msg.faccbodyx_5 = getScaledValue(ros_msg.accbodyx_5, 0.0004);
  ros_msg.faccbodyy_5 = getScaledValue(ros_msg.accbodyy_5, 0.0004);
  ros_msg.faccbodyz_5 = getScaledValue(ros_msg.accbodyz_5, 0.0004);
  ros_msg.faccbodyx_6 = getScaledValue(ros_msg.accbodyx_6, 0.0004);
  ros_msg.faccbodyy_6 = getScaledValue(ros_msg.accbodyy_6, 0.0004);
  ros_msg.faccbodyz_6 = getScaledValue(ros_msg.accbodyz_6, 0.0004);
  ros_msg.faccbodyx_7 = getScaledValue(ros_msg.accbodyx_7, 0.0004);
  ros_msg.faccbodyy_7 = getScaledValue(ros_msg.accbodyy_7, 0.0004);
  ros_msg.faccbodyz_7 = getScaledValue(ros_msg.accbodyz_7, 0.0004);

  ros_msg.facchorx_1 = getScaledValue(ros_msg.acchorx_1, 0.0004);
  ros_msg.facchory_1 = getScaledValue(ros_msg.acchory_1, 0.0004);
  ros_msg.facchorz_1 = getScaledValue(ros_msg.acchorz_1, 0.0004);
  ros_msg.facchorx_2 = getScaledValue(ros_msg.acchorx_2, 0.0004);
  ros_msg.facchory_2 = getScaledValue(ros_msg.acchory_2, 0.0004);
  ros_msg.facchorz_2 = getScaledValue(ros_msg.acchorz_2, 0.0004);
  ros_msg.facchorx_3 = getScaledValue(ros_msg.acchorx_3, 0.0004);
  ros_msg.facchory_3 = getScaledValue(ros_msg.acchory_3, 0.0004);
  ros_msg.facchorz_3 = getScaledValue(ros_msg.acchorz_3, 0.0004);
  ros_msg.facchorx_4 = getScaledValue(ros_msg.acchorx_4, 0.0004);
  ros_msg.facchory_4 = getScaledValue(ros_msg.acchory_4, 0.0004);
  ros_msg.facchorz_4 = getScaledValue(ros_msg.acchorz_4, 0.0004);
  ros_msg.facchorx_5 = getScaledValue(ros_msg.acchorx_5, 0.0004);
  ros_msg.facchory_5 = getScaledValue(ros_msg.acchory_5, 0.0004);
  ros_msg.facchorz_5 = getScaledValue(ros_msg.acchorz_5, 0.0004);
  ros_msg.facchorx_6 = getScaledValue(ros_msg.acchorx_6, 0.0004);
  ros_msg.facchory_6 = getScaledValue(ros_msg.acchory_6, 0.0004);
  ros_msg.facchorz_6 = getScaledValue(ros_msg.acchorz_6, 0.0004);
  ros_msg.facchorx_7 = getScaledValue(ros_msg.acchorx_7, 0.0004);
  ros_msg.facchory_7 = getScaledValue(ros_msg.acchory_7, 0.0004);
  ros_msg.facchorz_7 = getScaledValue(ros_msg.acchorz_7, 0.0004);

  ros_msg.fextvelanx = getScaledValue(ros_msg.extvelanx, 0.005);
  ros_msg.fextvelany = getScaledValue(ros_msg.extvelany, 0.005);
  ros_msg.fextveldigx = getScaledValue(ros_msg.extveldigx, 0.005);
  ros_msg.fextveldigy = getScaledValue(ros_msg.extveldigy, 0.005);
  ros_msg.fextvelxcorrected = getScaledValue(ros_msg.extvelxcorrected, 0.005);
  ros_msg.fextvelycorrected = getScaledValue(ros_msg.extvelycorrected, 0.005);

  ros_msg.fextbaropressure = getScaledValue(ros_msg.extbaropressure, 0.01);
  ros_msg.fextbaroheight = getScaledValue(ros_msg.extbaroheight, 0.01);
  ros_msg.fextbaroheightcorrected = getScaledValue(ros_msg.extbaroheightcorrected, 0.01);

  ros_msg.finvpathradius = getScaledValue(ros_msg.invpathradius, 0.0001);
  ros_msg.fsideslipangle = getScaledValue(ros_msg.sideslipangle, 0.01);
  ros_msg.fdisttrav = getScaledValue(ros_msg.disttrav, 0.01);

  ros_msg.finvpathradius_1 = getScaledValue(ros_msg.invpathradius_1, 0.0001);
  ros_msg.fsideslipangle_1 = getScaledValue(ros_msg.sideslipangle_1, 0.01);
  ros_msg.fdisttrav_1 = getScaledValue(ros_msg.disttrav_1, 0.01);
  ros_msg.finvpathradius_2 = getScaledValue(ros_msg.invpathradius_2, 0.0001);
  ros_msg.fsideslipangle_2 = getScaledValue(ros_msg.sideslipangle_2, 0.01);
  ros_msg.fdisttrav_2 = getScaledValue(ros_msg.disttrav_2, 0.01);
  ros_msg.finvpathradius_3 = getScaledValue(ros_msg.invpathradius_3, 0.0001);
  ros_msg.fsideslipangle_3 = getScaledValue(ros_msg.sideslipangle_3, 0.01);
  ros_msg.fdisttrav_3 = getScaledValue(ros_msg.disttrav_3, 0.01);
  ros_msg.finvpathradius_4 = getScaledValue(ros_msg.invpathradius_4, 0.0001);
  ros_msg.fsideslipangle_4 = getScaledValue(ros_msg.sideslipangle_4, 0.01);
  ros_msg.fdisttrav_4 = getScaledValue(ros_msg.disttrav_4, 0.01);
  ros_msg.finvpathradius_5 = getScaledValue(ros_msg.invpathradius_5, 0.0001);
  ros_msg.fsideslipangle_5 = getScaledValue(ros_msg.sideslipangle_5, 0.01);
  ros_msg.fdisttrav_5 = getScaledValue(ros_msg.disttrav_5, 0.01);
  ros_msg.finvpathradius_6 = getScaledValue(ros_msg.invpathradius_6, 0.0001);
  ros_msg.fsideslipangle_6 = getScaledValue(ros_msg.sideslipangle_6, 0.01);
  ros_msg.fdisttrav_6 = getScaledValue(ros_msg.disttrav_6, 0.01);
  ros_msg.finvpathradius_7 = getScaledValue(ros_msg.invpathradius_7, 0.0001);
  ros_msg.fsideslipangle_7 = getScaledValue(ros_msg.sideslipangle_7, 0.01);
  ros_msg.fdisttrav_7 = getScaledValue(ros_msg.disttrav_7, 0.01);

  ros_msg.fsystemtemp = getScaledValue(ros_msg.systemtemp, 0.1);
  ros_msg.fsystemdspload = getScaledValue(ros_msg.systemdspload, 0.1);

  ros_msg.fgpslatabs = getScaledValue(ros_msg.gpslatabs, 0.0000001);
  ros_msg.fgpslonabs = getScaledValue(ros_msg.gpslonabs, 0.0000001);
  ros_msg.fgpslatrel = getScaledValue(ros_msg.gpslatrel, 0.01);
  ros_msg.fgpslonrel = getScaledValue(ros_msg.gpslonrel, 0.01);

  ros_msg.fgpsstddevlat = getScaledValue(ros_msg.gpsstddevlat, 0.001);
  ros_msg.fgpsstddevlon = getScaledValue(ros_msg.gpsstddevlon, 0.001);
  ros_msg.fgpsstddevheight = getScaledValue(ros_msg.gpsstddevheight, 0.001);

  ros_msg.fgpsvelframex = getScaledValue(ros_msg.gpsvelframex, 0.005);
  ros_msg.fgpsvelframey = getScaledValue(ros_msg.gpsvelframey, 0.005);
  ros_msg.fgpsvelframez = getScaledValue(ros_msg.gpsvelframez, 0.005);
  ros_msg.fgpsvellatency = getScaledValue(ros_msg.gpsvellatency, 0.001);

  ros_msg.fgpsstddevvelx = getScaledValue(ros_msg.gpsstddevvelx, 0.001);
  ros_msg.fgpsstddevvely = getScaledValue(ros_msg.gpsstddevvely, 0.001);
  ros_msg.fgpsstddevvelz = getScaledValue(ros_msg.gpsstddevvelz, 0.001);

  ros_msg.fgpsdiffage = getScaledValue(ros_msg.gpsdiffage, 0.1);
  ros_msg.fgpsreceiverload = getScaledValue(ros_msg.gpsreceiverload, 0.5);

  ros_msg.finsroll = getScaledValue(ros_msg.insroll, 0.01);
  ros_msg.finspitch = getScaledValue(ros_msg.inspitch, 0.01);
  ros_msg.finsyaw = getScaledValue(ros_msg.insyaw, 0.01);
  ros_msg.fgpscog = getScaledValue(ros_msg.gpscog, 0.01);

  ros_msg.fgpsheight = getScaledValue(ros_msg.gpsheight, 0.01);
  ros_msg.fundulation = getScaledValue(ros_msg.undulation, 0.01);

  ros_msg.finsheight = getScaledValue(ros_msg.insheight, 0.01);
  ros_msg.finsheight_1 = getScaledValue(ros_msg.insheight_1, 0.01);
  ros_msg.finsheight_2 = getScaledValue(ros_msg.insheight_2, 0.01);
  ros_msg.finsheight_3 = getScaledValue(ros_msg.insheight_3, 0.01);
  ros_msg.finsheight_4 = getScaledValue(ros_msg.insheight_4, 0.01);
  ros_msg.finsheight_5 = getScaledValue(ros_msg.insheight_5, 0.01);
  ros_msg.finsheight_6 = getScaledValue(ros_msg.insheight_6, 0.01);
  ros_msg.finsheight_7 = getScaledValue(ros_msg.insheight_7, 0.01);

  ros_msg.finslatabs = getScaledValue(ros_msg.inslatabs, 0.0000001);
  ros_msg.finslonabs = getScaledValue(ros_msg.inslonabs, 0.0000001);
  ros_msg.finslatrel = getScaledValue(ros_msg.inslatrel, 0.01);
  ros_msg.finslonrel = getScaledValue(ros_msg.inslonrel, 0.01);
  ros_msg.finslatabs_1 = getScaledValue(ros_msg.inslatabs_1, 0.0000001);
  ros_msg.finslonabs_1 = getScaledValue(ros_msg.inslonabs_1, 0.0000001);
  ros_msg.finslatrel_1 = getScaledValue(ros_msg.inslatrel_1, 0.01);
  ros_msg.finslonrel_1 = getScaledValue(ros_msg.inslonrel_1, 0.01);
  ros_msg.finslatabs_2 = getScaledValue(ros_msg.inslatabs_2, 0.0000001);
  ros_msg.finslonabs_2 = getScaledValue(ros_msg.inslonabs_2, 0.0000001);
  ros_msg.finslatrel_2 = getScaledValue(ros_msg.inslatrel_2, 0.01);
  ros_msg.finslonrel_2 = getScaledValue(ros_msg.inslonrel_2, 0.01);
  ros_msg.finslatabs_3 = getScaledValue(ros_msg.inslatabs_3, 0.0000001);
  ros_msg.finslonabs_3 = getScaledValue(ros_msg.inslonabs_3, 0.0000001);
  ros_msg.finslatrel_3 = getScaledValue(ros_msg.inslatrel_3, 0.01);
  ros_msg.finslonrel_3 = getScaledValue(ros_msg.inslonrel_3, 0.01);
  ros_msg.finslatabs_4 = getScaledValue(ros_msg.inslatabs_4, 0.0000001);
  ros_msg.finslonabs_4 = getScaledValue(ros_msg.inslonabs_4, 0.0000001);
  ros_msg.finslatrel_4 = getScaledValue(ros_msg.inslatrel_4, 0.01);
  ros_msg.finslonrel_4 = getScaledValue(ros_msg.inslonrel_4, 0.01);
  ros_msg.finslatabs_5 = getScaledValue(ros_msg.inslatabs_5, 0.0000001);
  ros_msg.finslonabs_5 = getScaledValue(ros_msg.inslonabs_5, 0.0000001);
  ros_msg.finslatrel_5 = getScaledValue(ros_msg.inslatrel_5, 0.01);
  ros_msg.finslonrel_5 = getScaledValue(ros_msg.inslonrel_5, 0.01);
  ros_msg.finslatabs_6 = getScaledValue(ros_msg.inslatabs_6, 0.0000001);
  ros_msg.finslonabs_6 = getScaledValue(ros_msg.inslonabs_6, 0.0000001);
  ros_msg.finslatrel_6 = getScaledValue(ros_msg.inslatrel_6, 0.01);
  ros_msg.finslonrel_6 = getScaledValue(ros_msg.inslonrel_6, 0.01);
  ros_msg.finslatabs_7 = getScaledValue(ros_msg.inslatabs_7, 0.0000001);
  ros_msg.finslonabs_7 = getScaledValue(ros_msg.inslonabs_7, 0.0000001);
  ros_msg.finslatrel_7 = getScaledValue(ros_msg.inslatrel_7, 0.01);
  ros_msg.finslonrel_7 = getScaledValue(ros_msg.inslonrel_7, 0.01);

  ros_msg.finsvelhorx = getScaledValue(ros_msg.insvelhorx, 0.005);
  ros_msg.finsvelhory = getScaledValue(ros_msg.insvelhory, 0.005);
  ros_msg.finsvelhorz = getScaledValue(ros_msg.insvelhorz, 0.005);
  ros_msg.finsvelframex = getScaledValue(ros_msg.insvelframex, 0.005);
  ros_msg.finsvelframey = getScaledValue(ros_msg.insvelframey, 0.005);
  ros_msg.finsvelframez = getScaledValue(ros_msg.insvelframez, 0.005);

  ros_msg.finsvelhorx_1 = getScaledValue(ros_msg.insvelhorx_1, 0.005);
  ros_msg.finsvelhory_1 = getScaledValue(ros_msg.insvelhory_1, 0.005);
  ros_msg.finsvelhorz_1 = getScaledValue(ros_msg.insvelhorz_1, 0.005);
  ros_msg.finsvelhorx_2 = getScaledValue(ros_msg.insvelhorx_2, 0.005);
  ros_msg.finsvelhory_2 = getScaledValue(ros_msg.insvelhory_2, 0.005);
  ros_msg.finsvelhorz_2 = getScaledValue(ros_msg.insvelhorz_2, 0.005);
  ros_msg.finsvelhorx_3 = getScaledValue(ros_msg.insvelhorx_3, 0.005);
  ros_msg.finsvelhory_3 = getScaledValue(ros_msg.insvelhory_3, 0.005);
  ros_msg.finsvelhorz_3 = getScaledValue(ros_msg.insvelhorz_3, 0.005);
  ros_msg.finsvelhorx_4 = getScaledValue(ros_msg.insvelhorx_4, 0.005);
  ros_msg.finsvelhory_4 = getScaledValue(ros_msg.insvelhory_4, 0.005);
  ros_msg.finsvelhorz_4 = getScaledValue(ros_msg.insvelhorz_4, 0.005);
  ros_msg.finsvelhorx_5 = getScaledValue(ros_msg.insvelhorx_5, 0.005);
  ros_msg.finsvelhory_5 = getScaledValue(ros_msg.insvelhory_5, 0.005);
  ros_msg.finsvelhorz_5 = getScaledValue(ros_msg.insvelhorz_5, 0.005);
  ros_msg.finsvelhorx_6 = getScaledValue(ros_msg.insvelhorx_6, 0.005);
  ros_msg.finsvelhory_6 = getScaledValue(ros_msg.insvelhory_6, 0.005);
  ros_msg.finsvelhorz_6 = getScaledValue(ros_msg.insvelhorz_6, 0.005);
  ros_msg.finsvelhorx_7 = getScaledValue(ros_msg.insvelhorx_7, 0.005);
  ros_msg.finsvelhory_7 = getScaledValue(ros_msg.insvelhory_7, 0.005);
  ros_msg.finsvelhorz_7 = getScaledValue(ros_msg.insvelhorz_7, 0.005);

  ros_msg.finsstddevlat = getScaledValue(ros_msg.insstddevlat, 0.01);
  ros_msg.finsstddevlong = getScaledValue(ros_msg.insstddevlong, 0.01);
  ros_msg.finsstddevheight = getScaledValue(ros_msg.insstddevheight, 0.01);

  ros_msg.finsstddevvelx = getScaledValue(ros_msg.insstddevvelx, 0.01);
  ros_msg.finsstddevvely = getScaledValue(ros_msg.insstddevvely, 0.01);
  ros_msg.finsstddevvelz = getScaledValue(ros_msg.insstddevvelz, 0.01);
  ros_msg.finsstddevroll = getScaledValue(ros_msg.insstddevroll, 0.01);
  ros_msg.finsstddevpitch = getScaledValue(ros_msg.insstddevpitch, 0.01);
  ros_msg.finsstddevyaw = getScaledValue(ros_msg.insstddevyaw, 0.01);

  ros_msg.fan1 = getScaledValue(ros_msg.an1, 0.0005);
  ros_msg.fan2 = getScaledValue(ros_msg.an2, 0.0005);
  ros_msg.fan3 = getScaledValue(ros_msg.an3, 0.0005);
  ros_msg.fan4 = getScaledValue(ros_msg.an4, 0.0005);

  // only for >= v3.3.3
  ros_msg.fgpsdualantheading = getScaledValue(ros_msg.gpsdualantheading, 0.01);
  ros_msg.fgpsdualantpitch = getScaledValue(ros_msg.gpsdualantpitch, 0.01);
  ros_msg.fgpsdualantstddevheading = getScaledValue(ros_msg.gpsdualantstddevheading, 0.01);
  ros_msg.fgpsdualantstddevpitch = getScaledValue(ros_msg.gpsdualantstddevpitch, 0.01);
  ros_msg.fgpsdualantstddevheading_hr = getScaledValue(ros_msg.gpsdualantstddevheading_hr, 0.01);
  ros_msg.fgpsdualantstddevpitch_hr = getScaledValue(ros_msg.gpsdualantstddevpitch_hr, 0.01);
  ros_msg.faccbodyx_8 = getScaledValue(ros_msg.accbodyx_8, 0.0004);
  ros_msg.faccbodyy_8 = getScaledValue(ros_msg.accbodyy_8, 0.0004);
  ros_msg.faccbodyz_8 = getScaledValue(ros_msg.accbodyz_8, 0.0004);
  ros_msg.facchorx_8 = getScaledValue(ros_msg.acchorx_8, 0.0004);
  ros_msg.facchory_8 = getScaledValue(ros_msg.acchory_8, 0.0004);
  ros_msg.facchorz_8 = getScaledValue(ros_msg.acchorz_8, 0.0004);
  ros_msg.finvpathradius_8 = getScaledValue(ros_msg.invpathradius_8, 0.0001);
  ros_msg.fsideslipangle_8 = getScaledValue(ros_msg.sideslipangle_8, 0.01);
  ros_msg.fdisttrav_8 = getScaledValue(ros_msg.disttrav_8, 0.01);
  ros_msg.finsheight_8 = getScaledValue(ros_msg.insheight_8, 0.01);
  ros_msg.finslatabs_8 = getScaledValue(ros_msg.inslatabs_8, 0.0000001);
  ros_msg.finslonabs_8 = getScaledValue(ros_msg.inslonabs_8, 0.0000001);
  ros_msg.finslatrel_8 = getScaledValue(ros_msg.inslatrel_8, 0.01);
  ros_msg.finslonrel_8 = getScaledValue(ros_msg.inslonrel_8, 0.01);
  ros_msg.finsvelhorx_8 = getScaledValue(ros_msg.insvelhorx_8, 0.005);
  ros_msg.finsvelhory_8 = getScaledValue(ros_msg.insvelhory_8, 0.005);
  ros_msg.finsvelhorz_8 = getScaledValue(ros_msg.insvelhorz_8, 0.005);
}

void ADMA2ROSParser::extractNavSatFix(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, sensor_msgs::msg::NavSatFix & nav_ros_msg)
{
  // fil status
  switch (ros_msg.statusgpsmode) {
    case 1:
      // No GNSS Data
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      break;
    case 2:
      // single GNSS
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      break;
    case 4:
      // actually DGNSS Coarse Mode, but used to distinguish here
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      break;
    case 8:
      // DGNSS Precise Mode
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      break;
    default:
      break;
  }

  nav_ros_msg.altitude = ros_msg.finsheight + ros_msg.undulation;
  nav_ros_msg.latitude = ros_msg.finslatabs;
  nav_ros_msg.longitude = ros_msg.finslonabs;
  nav_ros_msg.position_covariance[0] = std::pow(ros_msg.finsstddevlat, 2);
  nav_ros_msg.position_covariance[4] = std::pow(ros_msg.finsstddevlong, 2);
  nav_ros_msg.position_covariance[8] = std::pow(ros_msg.finsstddevheight, 2);

  nav_ros_msg.position_covariance_type =
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
}

void ADMA2ROSParser::extractNavSatFix(
  adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, sensor_msgs::msg::NavSatFix & nav_ros_msg,
  std::array<adma_ros_driver_msgs::msg::POI, 8> &pois, uint8_t desiredSource)
{
  // fil status
  switch (ros_msg.status.status_gnss_mode) {
    case 1:
      // No GNSS Data
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      break;
    case 2:
      // single GNSS
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      break;
    case 4:
      // actually DGNSS Coarse Mode, but used to distinguish here
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      break;
    case 8:
      // DGNSS Precise Mode
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      break;
    default:
      break;
  }

  // read POI specific height for NavSatFix msg
  nav_ros_msg.altitude = desiredSource == 0 ? ros_msg.ins_height : pois[desiredSource - 1].ins_height;
  nav_ros_msg.latitude = desiredSource == 0 ? ros_msg.ins_lat_abs : pois[desiredSource - 1].ins_lat_abs;
  nav_ros_msg.longitude = desiredSource == 0 ? ros_msg.ins_long_abs : pois[desiredSource - 1].ins_long_abs;
  // add undulation to get WGS84 height for ROS standard
  nav_ros_msg.altitude += ros_msg.undulation;
  nav_ros_msg.position_covariance[0] = std::pow(ros_msg.ins_stddev_lat, 2);
  nav_ros_msg.position_covariance[4] = std::pow(ros_msg.ins_stddev_long, 2);
  nav_ros_msg.position_covariance[8] = std::pow(ros_msg.ins_stddev_height, 2);

  nav_ros_msg.position_covariance_type =
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
}

void ADMA2ROSParser::extractIMU(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, sensor_msgs::msg::Imu & imu_ros_msg)
{
  imu_ros_msg.linear_acceleration.x = ros_msg.faccbodyhrx * 9.81;
  imu_ros_msg.linear_acceleration.y = ros_msg.faccbodyhry * 9.81;
  imu_ros_msg.linear_acceleration.z = ros_msg.faccbodyhrz * 9.81;

  imu_ros_msg.angular_velocity.x = deg2Rad(ros_msg.fratebodyhrx);
  imu_ros_msg.angular_velocity.y = deg2Rad(ros_msg.fratebodyhry);
  imu_ros_msg.angular_velocity.z = deg2Rad(ros_msg.fratebodyhrz);

  tf2::Quaternion q;
  double roll_rad = deg2Rad(ros_msg.finsroll);
  double pitch_rad = deg2Rad(ros_msg.finspitch);
  double yaw_rad = deg2Rad(ros_msg.finsyaw);
  q.setRPY(roll_rad, pitch_rad, yaw_rad);
  imu_ros_msg.orientation = tf2::toMsg(q);

  imu_ros_msg.orientation_covariance[0] = std::pow(deg2Rad(ros_msg.finsstddevroll), 2);
  imu_ros_msg.orientation_covariance[4] = std::pow(deg2Rad(ros_msg.finsstddevpitch), 2);
  imu_ros_msg.orientation_covariance[8] = std::pow(deg2Rad(ros_msg.finsstddevyaw), 2);

  // ADMA does not provide covariance for linear acceleration and angular velocity.
  // These values need to be measured at standstill each ADMA model.
  imu_ros_msg.angular_velocity_covariance[0] = -1;
  imu_ros_msg.linear_acceleration_covariance[0] = -1;
}

void ADMA2ROSParser::extractIMU(
  adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, sensor_msgs::msg::Imu & imu_ros_msg,
  std::array<adma_ros_driver_msgs::msg::POI, 8> &pois, uint8_t desiredSource)
{
  // get POI specific IMU data
  imu_ros_msg.linear_acceleration.x = desiredSource == 0 ? ros_msg.acc_body_hr.x : pois[desiredSource - 1].acc_body.x;
  imu_ros_msg.linear_acceleration.y = desiredSource == 0 ? ros_msg.acc_body_hr.y : pois[desiredSource - 1].acc_body.y;
  imu_ros_msg.linear_acceleration.z = desiredSource == 0 ? ros_msg.acc_body_hr.z : pois[desiredSource - 1].acc_body.z;
  // convert to m/s²
  imu_ros_msg.linear_acceleration.x *= 9.81;
  imu_ros_msg.linear_acceleration.y *= 9.81;
  imu_ros_msg.linear_acceleration.z *= 9.81;

  imu_ros_msg.angular_velocity.x = deg2Rad(ros_msg.rate_body_hr.x);
  imu_ros_msg.angular_velocity.y = deg2Rad(ros_msg.rate_body_hr.y);
  imu_ros_msg.angular_velocity.z = deg2Rad(ros_msg.rate_body_hr.z);

  tf2::Quaternion q;
  double roll_rad = deg2Rad(ros_msg.ins_roll);
  double pitch_rad = deg2Rad(ros_msg.ins_pitch);
  double yaw_rad = deg2Rad(ros_msg.ins_yaw);
  q.setRPY(roll_rad, pitch_rad, yaw_rad);
  imu_ros_msg.orientation = tf2::toMsg(q);

  imu_ros_msg.orientation_covariance[0] = std::pow(deg2Rad(ros_msg.ins_stddev_roll), 2);
  imu_ros_msg.orientation_covariance[4] = std::pow(deg2Rad(ros_msg.ins_stddev_pitch), 2);
  imu_ros_msg.orientation_covariance[8] = std::pow(deg2Rad(ros_msg.ins_stddev_yaw), 2);

  // ADMA does not provide covariance for linear acceleration and angular velocity.
  // These values need to be measured at standstill each ADMA model.
  imu_ros_msg.angular_velocity_covariance[0] = -1;
  imu_ros_msg.linear_acceleration_covariance[0] = -1;
}

void ADMA2ROSParser::extractOdometry(
    adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, nav_msgs::msg::Odometry & odometry_msg,
    double yawOffset, std::array<adma_ros_driver_msgs::msg::POI, 8> &pois, uint8_t desiredSource)
{
  // extract POI specific odometry data
  odometry_msg.pose.pose.position.x = desiredSource == 0 ? ros_msg.ins_pos_rel_x : pois[desiredSource - 1].ins_pos_rel_x;
  odometry_msg.pose.pose.position.y = desiredSource == 0 ? ros_msg.ins_pos_rel_y : pois[desiredSource - 1].ins_pos_rel_y;
  odometry_msg.pose.pose.position.z = desiredSource == 0 ? ros_msg.ins_height : pois[desiredSource - 1].ins_height;

  double roll_rad = deg2Rad(ros_msg.ins_roll);
  double pitch_rad = deg2Rad(ros_msg.ins_pitch);
  double yaw_rad;

  if (protocolVersion_< 3350) {
    // relative yaw was introduced in admanet v3.3.5  
    double yaw_rad = deg2Rad((ros_msg.ins_yaw + yawOffset));
  }
  else {
      double yaw_rad = deg2Rad((ros_msg.ins_yaw_rel + yawOffset));
  }
  
  tf2::Quaternion q;
  q.setRPY(roll_rad, pitch_rad, yaw_rad);
  odometry_msg.pose.pose.orientation = tf2::toMsg(q);

  odometry_msg.pose.covariance[21] = std::pow(deg2Rad(ros_msg.ins_stddev_roll), 2);
  odometry_msg.pose.covariance[28] = std::pow(deg2Rad(ros_msg.ins_stddev_pitch), 2);
  odometry_msg.pose.covariance[35] = std::pow(deg2Rad(ros_msg.ins_stddev_yaw), 2);

  odometry_msg.twist.twist.linear.x = desiredSource == 0 ? ros_msg.ins_vel_hor.x : pois[desiredSource - 1].ins_vel_hor.x;
  odometry_msg.twist.twist.linear.y = desiredSource == 0 ? ros_msg.ins_vel_hor.y : pois[desiredSource - 1].ins_vel_hor.y;
  odometry_msg.twist.twist.linear.z = desiredSource == 0 ? ros_msg.ins_vel_hor.z : pois[desiredSource - 1].ins_vel_hor.z;
  odometry_msg.twist.twist.angular.x = deg2Rad(ros_msg.rate_body.x);
  odometry_msg.twist.twist.angular.y = deg2Rad(ros_msg.rate_body.y);
  odometry_msg.twist.twist.angular.z = deg2Rad(ros_msg.rate_body.z);
  odometry_msg.twist.covariance[0] = std::pow(ros_msg.ins_stddev_vel.x, 2);
  odometry_msg.twist.covariance[7] = std::pow(ros_msg.ins_stddev_vel.y, 2);
  odometry_msg.twist.covariance[14] = std::pow(ros_msg.ins_stddev_vel.z, 2);
  
}
