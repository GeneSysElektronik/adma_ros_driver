#include "adma_ros2_driver/parser/adma2ros_parser_v335.hpp"

#include "adma_ros2_driver/parser/parser_utils.hpp"

ADMA2ROSParserV335::ADMA2ROSParserV335() {}

void ADMA2ROSParserV335::mapAdmaMessageToROS(
  adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV335 & adma_data)
{
  mapAdmaHeader(ros_msg, adma_data);
  mapErrorWarningBytes(ros_msg.error_warning, adma_data);
  mapStatusBitfields(ros_msg.status, adma_data);
  mapUnscaledData(ros_msg, adma_data);
  mapScaledData(ros_msg, adma_data);
  mapPOI(ros_msg, adma_data);
}

void ADMA2ROSParserV335::mapStatusToROS(
  adma_ros_driver_msgs::msg::AdmaStatus & ros_msg, AdmaDataV335 & adma_data)
{
  mapStatusBytes(ros_msg.status_bytes, adma_data);
  mapErrorWarningBytes(ros_msg.error_warnings_bytes, adma_data);
  mapErrorWarningBitfields(ros_msg.error_warnings, adma_data);
}

void ADMA2ROSParserV335::mapAdmaHeader(
  adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV335 & adma_data)
{
  // fill static header information
  AdmaStaticHeader static_header = adma_data.staticHeader;
  ros_msg.genesys_id = static_header.genesysid;
  std::stringstream ss;
  ss << int(static_header.headerversion[0]) << int(static_header.headerversion[1])
     << int(static_header.headerversion[2]) << int(static_header.headerversion[3]);
  ros_msg.header_version = ss.str();
  ss.clear();
  ss.str("");
  ros_msg.format_id = static_header.formatid;
  //TODO: this value is parsed wrong?!
  ss << int(static_header.formatversion[0]) << int(static_header.formatversion[1])
     << int(static_header.formatversion[2]) << int(static_header.formatversion[3]);
  ros_msg.format_version = ss.str();
  ros_msg.serial_number = static_header.serialno;

  // fill dynamic header information
  AdmaDynamicHeader dynamic_header = adma_data.dynamicHeader;
  ros_msg.config_id = dynamic_header.configid;
  ros_msg.config_format = dynamic_header.configformat;
  ros_msg.config_version = dynamic_header.configversion;
  ros_msg.config_size = dynamic_header.configsize;
  ros_msg.byte_offset = dynamic_header.byteoffset;
  ros_msg.slice_size = dynamic_header.slicesize;
  ros_msg.slice_data = dynamic_header.slicedata;
}

void ADMA2ROSParserV335::mapStatusBytes(
  adma_ros_driver_msgs::msg::ByteStatus & ros_msg_byte_status, AdmaDataV335 & adma_data)
{
  ros_msg_byte_status.status_byte_0 = adma_data.gnssStatus;
  ros_msg_byte_status.status_byte_1 = adma_data.signalInStatus;
  ros_msg_byte_status.status_byte_2 = adma_data.miscStatus;
  ros_msg_byte_status.status_count = adma_data.statuscount;
  ros_msg_byte_status.status_byte_4 = adma_data.kfStatus;
  ros_msg_byte_status.status_byte_5 = adma_data.statusRobot;
}

void ADMA2ROSParserV335::mapStatusBitfields(
  adma_ros_driver_msgs::msg::Status & ros_msg_status, AdmaDataV335 & adma_data)
{
  mapStatusBit0(ros_msg_status, adma_data.gnssStatus);
  mapStatusBit1(ros_msg_status, adma_data.signalInStatus);
  mapStatusBit2(ros_msg_status, adma_data.miscStatus);
  ros_msg_status.status_count = adma_data.statuscount;
  mapStatusBit4(ros_msg_status, adma_data.kfStatus);
  mapStatusBit5(ros_msg_status, adma_data.statusRobot);
}

void ADMA2ROSParserV335::mapStatusBit0(adma_ros_driver_msgs::msg::Status & ros_msg_status, unsigned char status_byte)
{
  // status_byte_0
  unsigned char gnss_status = status_byte;
  /* status gnss mode */
  std::bitset<8> gnss_status_byte = status_byte;
  std::bitset<4> status_gnss_mode;
  status_gnss_mode[0] = gnss_status_byte[0];
  status_gnss_mode[1] = gnss_status_byte[1];
  status_gnss_mode[2] = gnss_status_byte[2];
  status_gnss_mode[3] = gnss_status_byte[3];
  ros_msg_status.status_gnss_mode = status_gnss_mode.to_ulong();
  bool standstill_c = getbit(gnss_status, 4);
  bool status_skidding = getbit(gnss_status, 5);
  bool status_external_vel = getbit(gnss_status, 7);

  /* status stand still */
  ros_msg_status.status_standstill = standstill_c;
  /* status skidding */
  ros_msg_status.status_skidding = status_skidding;
  /* status external velocity slip */
  ros_msg_status.status_external_vel_out = status_external_vel;
}

void ADMA2ROSParserV335::mapStatusBit1(adma_ros_driver_msgs::msg::Status & ros_msg_status, unsigned char status_byte)
{
  // status_byte_1
  unsigned char gnss_trigger_status = status_byte;
  bool status_trig_gnss = getbit(gnss_trigger_status, 0);
  bool status_signal_in3 = getbit(gnss_trigger_status, 1);
  bool status_signal_in2 = getbit(gnss_trigger_status, 2);
  bool status_signal_in1 = getbit(gnss_trigger_status, 3);
  bool status_alignment = getbit(gnss_trigger_status, 4);
  bool status_ahrs_ins = getbit(gnss_trigger_status, 5);
  bool status_dead_reckoning = getbit(gnss_trigger_status, 6);
  bool status_synclock = getbit(gnss_trigger_status, 7);
  /* status statustriggnss */
  ros_msg_status.status_trig_gnss = status_trig_gnss;
  /* status statussignalin3 */
  ros_msg_status.status_signal_in3 = status_signal_in3;
  /* status statussignalin2 */
  ros_msg_status.status_signal_in2 = status_signal_in2;
  /* status statussignalin1 */
  ros_msg_status.status_signal_in1 = status_signal_in1;
  /* status statusalignment */
  ros_msg_status.status_alignment = status_alignment;
  /* status statusahrsins */
  ros_msg_status.status_ahrs_ins = status_ahrs_ins;
  /* status statusdeadreckoning */
  ros_msg_status.status_dead_reckoning = status_dead_reckoning;
  /* status statussynclock */
  ros_msg_status.status_synclock = status_synclock;
}

void ADMA2ROSParserV335::mapStatusBit2(adma_ros_driver_msgs::msg::Status & ros_msg_status, unsigned char status_byte)
{
  // status_byte_2
  unsigned char evk_status = status_byte;
  bool status_evk_activ = getbit(evk_status, 0);
  bool status_evk_estimates = getbit(evk_status, 1);
  bool status_heading_executed = getbit(evk_status, 2);
  bool status_configuration_changed = getbit(evk_status, 3);
  /* status statustriggnss */
  ros_msg_status.status_evk_activ = status_evk_activ;
  /* status status_evk_estimates */
  ros_msg_status.status_evk_estimates = status_evk_estimates;
  /* status status_heading_executed */
  ros_msg_status.status_heading_executed = status_heading_executed;
  /* status status_configuration_changed */
  ros_msg_status.status_config_changed = status_configuration_changed;
  /* status tilt */
  std::bitset<8> evk_status_byte = status_byte;
  std::bitset<2> status_tilt;
  status_tilt[0] = evk_status_byte[4];
  status_tilt[1] = evk_status_byte[5];
  ros_msg_status.status_tilt = status_tilt.to_ulong();
  /* status pos */
  std::bitset<2> status_pos;
  status_pos[0] = evk_status_byte[6];
  status_pos[1] = evk_status_byte[7];
  ros_msg_status.status_pos = status_pos.to_ulong();
}

void ADMA2ROSParserV335::mapStatusBit4(adma_ros_driver_msgs::msg::Status & ros_msg_status, unsigned char status_byte)
{
  // status_byte_4
  unsigned char kf_status = status_byte;
  bool status_kalmanfilter_settled = getbit(kf_status, 0);
  bool status_kf_lat_stimulated = getbit(kf_status, 1);
  bool status_kf_long_stimulated = getbit(kf_status, 2);
  bool status_kf_steady_state = getbit(kf_status, 3);
  ros_msg_status.status_kalmanfilter_settled = status_kalmanfilter_settled;
  ros_msg_status.status_kf_lat_stimulated = status_kf_lat_stimulated;
  ros_msg_status.status_kf_long_stimulated = status_kf_long_stimulated;
  ros_msg_status.status_kf_steady_state = status_kf_steady_state;

  std::bitset<8> kf_status_byte = status_byte;
  std::bitset<2> status_speed;
  status_speed[0] = kf_status_byte[4];
  status_speed[1] = kf_status_byte[5];
  ros_msg_status.status_speed = status_speed.to_ulong();
}

void ADMA2ROSParserV335::mapStatusBit5(adma_ros_driver_msgs::msg::Status & ros_msg_status, unsigned char status_byte)
{
  // status_byte_5
  std::bitset<8> bit_status_robot = status_byte;
  std::bitset<4> status_robot;
  for (size_t i = 0; i < 4; i++) {
    status_robot[i] = bit_status_robot[i];
  }
  ros_msg_status.status_robot = status_robot.to_ulong();
}

void ADMA2ROSParserV335::mapErrorWarningBytes(
  adma_ros_driver_msgs::msg::ByteErrorWarning & ros_msg_byte_error_warning,
  AdmaDataV335 & adma_data)
{
  ros_msg_byte_error_warning.error_1 = adma_data.dataError1;
  ros_msg_byte_error_warning.error_2 = adma_data.dataError2;
  ros_msg_byte_error_warning.warn_1 = adma_data.dataWarn1;
  ros_msg_byte_error_warning.error_3 = adma_data.dataError3;
}

void ADMA2ROSParserV335::mapErrorWarningBitfields(
  adma_ros_driver_msgs::msg::ErrorWarning & ros_msg_error_warning, AdmaDataV335 & adma_data)
{
  mapErrorBit0(ros_msg_error_warning, adma_data.dataError1);
  mapErrorBit1(ros_msg_error_warning, adma_data.dataError2);
  mapWarningBit0(ros_msg_error_warning, adma_data.dataWarn1);
  mapErrorBit2(ros_msg_error_warning, adma_data.dataError3);
}

void ADMA2ROSParserV335::mapErrorBit0(adma_ros_driver_msgs::msg::ErrorWarning & ros_msg_error_warning, unsigned char error_byte)
{
  ros_msg_error_warning.error_gyro_hw = getbit(error_byte, 0);
  ros_msg_error_warning.error_accel_hw = getbit(error_byte, 1);
  ros_msg_error_warning.error_ext_speed_hw = getbit(error_byte, 2);
  ros_msg_error_warning.error_gnss_hw = getbit(error_byte, 3);
  ros_msg_error_warning.error_data_bus_checksum = getbit(error_byte, 4);
  ros_msg_error_warning.error_eeprom = getbit(error_byte, 5);
  ros_msg_error_warning.error_xmit = getbit(error_byte, 6);
  ros_msg_error_warning.error_cmd = getbit(error_byte, 7);
}

void ADMA2ROSParserV335::mapErrorBit1(adma_ros_driver_msgs::msg::ErrorWarning & ros_msg_error_warning, unsigned char error_byte)
{
  ros_msg_error_warning.error_data_bus = getbit(error_byte, 0);
  ros_msg_error_warning.error_can_bus = getbit(error_byte, 1);
  ros_msg_error_warning.error_num = getbit(error_byte, 3);
  ros_msg_error_warning.error_temp_warning = getbit(error_byte, 4);
  ros_msg_error_warning.error_reduced_accuracy = getbit(error_byte, 5);
  ros_msg_error_warning.error_range_max = getbit(error_byte, 6);
}

void ADMA2ROSParserV335::mapWarningBit0(adma_ros_driver_msgs::msg::ErrorWarning & ros_msg_error_warning, unsigned char warning_byte)
{
  ros_msg_error_warning.warn_gnss_no_solution = getbit(warning_byte, 0);
  ros_msg_error_warning.warn_gnss_vel_ignored = getbit(warning_byte, 1);
  ros_msg_error_warning.warn_gnss_pos_ignored = getbit(warning_byte, 2);
  ros_msg_error_warning.warn_gnss_unable_to_cfg = getbit(warning_byte, 3);
  ros_msg_error_warning.warn_speed_off = getbit(warning_byte, 4);
  ros_msg_error_warning.warn_gnss_dualant_ignored = getbit(warning_byte, 5);
}

void ADMA2ROSParserV335::mapErrorBit2(adma_ros_driver_msgs::msg::ErrorWarning & ros_msg_error_warning, unsigned char error_byte)
{
  ros_msg_error_warning.error_hw_sticky = getbit(error_byte, 0);
}

void ADMA2ROSParserV335::mapUnscaledData(
  adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV335 & adma_data)
{
  //fill external velocity
  ros_msg.ext_vel_dig_pulses_x = adma_data.extveldigpulsesx;
  ros_msg.ext_vel_dig_pulses_y = adma_data.extveldigpulsesy;
  // fill triggers
  ros_msg.trig_rising_1 = adma_data.trigrising1;
  ros_msg.trig_falling_1 = adma_data.trigfalling1;
  ros_msg.trig_rising_2 = adma_data.trigrising2;
  ros_msg.trig_falling_2 = adma_data.trigfalling2;
  ros_msg.trig_rising_3 = adma_data.trigrising3;
  ros_msg.trig_falling_3 = adma_data.trigfalling3;
  ros_msg.trig_rising_4 = adma_data.trigrising4;
  ros_msg.trig_falling_4 = adma_data.trigfalling4;
  //fill system data
  ros_msg.system_ta = adma_data.systemta;
  ros_msg.system_time_since_init = adma_data.systemtimesinceinit;
  //fill GNSS time
  ros_msg.gnss_time_msec = adma_data.gnsstimemsec;
  ros_msg.gnss_time_week = adma_data.gnsstimeweek;
  ros_msg.gnss_trigger = adma_data.gnsstrigger;
  //fill GPS AUX data
  ros_msg.gnss_sats_used = adma_data.gnsssatsused;
  ros_msg.gnss_sats_visible = adma_data.gnsssatsvisible;
  ros_msg.gnss_sats_dualant_used = adma_data.gnsssatsdualantused;
  ros_msg.gnss_sats_dualant_visible = adma_data.gnsssatsdualantvisible;
  ros_msg.gnss_log_delay = adma_data.gnsslogdelay;
  std::stringstream ss;
  ss << adma_data.gnssbasenr;
  ros_msg.gnss_base_nr = ss.str();
  // GNSS Dual ant information
  ros_msg.gnss_dualant_time_msec = adma_data.gnssDualAntTimeMsec;
  ros_msg.gnss_dualant_time_week = adma_data.gnssDualAntTimeWeek;
  //fill INS time UTC
  ros_msg.ins_time_msec = adma_data.instimemsec;
  ros_msg.ins_time_week = adma_data.instimeweek;
  ros_msg.leap_seconds = adma_data.leapseconds;
  // kalman filter status
  ros_msg.kf_lat_stimulated = adma_data.kflatstimulated;
  ros_msg.kf_long_stimulated = adma_data.kflongstimulated;
  ros_msg.kf_steady_state = adma_data.kfsteadystate;
  // gnss receiver status and error
  ros_msg.gnss_receiver_error = adma_data.gnssreceivererror;
  ros_msg.gnss_receiver_status = adma_data.gnssreceiverstatus;
}

void ADMA2ROSParserV335::mapScaledData(
  adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV335 & adma_data)
{
  ros_msg.acc_body_hr.x = getScaledValue(adma_data.sensorsBodyX.accHR, 0.0001);
  ros_msg.acc_body_hr.y = getScaledValue(adma_data.sensorsBodyY.accHR, 0.0001);
  ros_msg.acc_body_hr.z = getScaledValue(adma_data.sensorsBodyZ.accHR, 0.0001);
  ros_msg.rate_body_hr.x = getScaledValue(adma_data.sensorsBodyX.rateHR, 0.0001);
  ros_msg.rate_body_hr.y = getScaledValue(adma_data.sensorsBodyY.rateHR, 0.0001);
  ros_msg.rate_body_hr.z = getScaledValue(adma_data.sensorsBodyZ.rateHR, 0.0001);
  ros_msg.rate_body.x = getScaledValue(adma_data.ratesBody.x, 0.01);
  ros_msg.rate_body.y = getScaledValue(adma_data.ratesBody.y, 0.01);
  ros_msg.rate_body.z = getScaledValue(adma_data.ratesBody.z, 0.01);
  ros_msg.rate_hor.x = getScaledValue(adma_data.ratesHorizontal.x, 0.01);
  ros_msg.rate_hor.y = getScaledValue(adma_data.ratesHorizontal.y, 0.01);
  ros_msg.rate_hor.z = getScaledValue(adma_data.ratesHorizontal.z, 0.01);
  ros_msg.acc_body.x = getScaledValue(adma_data.accBody.x, 0.0004);
  ros_msg.acc_body.y = getScaledValue(adma_data.accBody.y, 0.0004);
  ros_msg.acc_body.z = getScaledValue(adma_data.accBody.z, 0.0004);
  ros_msg.acc_hor.x = getScaledValue(adma_data.accHorizontal.x, 0.0004);
  ros_msg.acc_hor.y = getScaledValue(adma_data.accHorizontal.y, 0.0004);
  ros_msg.acc_hor.z = getScaledValue(adma_data.accHorizontal.z, 0.0004);

  ros_msg.ext_vel_an_x = getScaledValue(adma_data.extVelAnalog.x, 0.005);
  ros_msg.ext_vel_an_y = getScaledValue(adma_data.extVelAnalog.y, 0.005);
  ros_msg.ext_vel_dig_x = getScaledValue(adma_data.extveldigx, 0.005);
  ros_msg.ext_vel_dig_y = getScaledValue(adma_data.extveldigy, 0.005);
  ros_msg.ext_vel_x_corrected = getScaledValue(adma_data.extVelCorrected.x, 0.005);
  ros_msg.ext_vel_y_corrected = getScaledValue(adma_data.extVelCorrected.y, 0.005);

  ros_msg.inv_path_radius = getScaledValue(adma_data.misc.invPathRadius, 0.0001);
  ros_msg.side_slip_angle = getScaledValue(adma_data.misc.sideSlipAngle, 0.01);
  ros_msg.dist_trav = getScaledValue(adma_data.misc.distanceTraveled, 0.01);

  ros_msg.system_temp = getScaledValue(adma_data.systemtemp, 0.1);
  ros_msg.system_dsp_load = getScaledValue(adma_data.systemdspload, 0.1);

  ros_msg.gnss_lat_abs = getScaledValue(adma_data.posAbs.latitude, 0.0000001);
  ros_msg.gnss_long_abs = getScaledValue(adma_data.posAbs.longitude, 0.0000001);
  ros_msg.gnss_pos_rel_x = getScaledValue(adma_data.posRel.latitude, 0.01);
  ros_msg.gnss_pos_rel_y = getScaledValue(adma_data.posRel.longitude, 0.01);

  ros_msg.gnss_stddev_lat = getScaledValue(adma_data.gnssstddevlat, 0.001);
  ros_msg.gnss_stddev_long = getScaledValue(adma_data.gnssstddevlon, 0.001);
  ros_msg.gnss_stddev_height = getScaledValue(adma_data.gnssstddevheight, 0.001);

  ros_msg.gnss_vel_frame.x = getScaledValue(adma_data.gnssvelframex, 0.005);
  ros_msg.gnss_vel_frame.y = getScaledValue(adma_data.gnssvelframey, 0.005);
  ros_msg.gnss_vel_frame.z = getScaledValue(adma_data.gnssvelframez, 0.005);
  ros_msg.gnss_vel_latency = getScaledValue(adma_data.gnssvellatency, 0.001);

  ros_msg.gnss_stddev_vel.x = getScaledValue(adma_data.gnssStdDevVel.x, 0.001);
  ros_msg.gnss_stddev_vel.y = getScaledValue(adma_data.gnssStdDevVel.y, 0.001);
  ros_msg.gnss_stddev_vel.z = getScaledValue(adma_data.gnssStdDevVel.z, 0.001);

  ros_msg.gnss_diffage = getScaledValue(adma_data.gnssdiffage, 0.1);
  ros_msg.gnss_receiver_load = getScaledValue(adma_data.gnssreceiverload, 0.5);

  ros_msg.ins_roll = getScaledValue(adma_data.insroll, 0.01);
  ros_msg.ins_pitch = getScaledValue(adma_data.inspitch, 0.01);
  ros_msg.ins_yaw = getScaledValue(adma_data.insyaw, 0.01);
  ros_msg.gnss_cog = getScaledValue(adma_data.gnsscog, 0.01);

  ros_msg.gnss_height = getScaledValue(adma_data.gnssheight, 0.01);
  ros_msg.undulation = getScaledValue(adma_data.undulation, 0.01);

  ros_msg.ins_height = getScaledValue(adma_data.insHeight, 0.01);

  ros_msg.ins_lat_abs = getScaledValue(adma_data.insPos.pos_abs.latitude, 0.0000001);
  ros_msg.ins_long_abs = getScaledValue(adma_data.insPos.pos_abs.longitude, 0.0000001);
  ros_msg.ins_pos_rel_x = getScaledValue(adma_data.insPos.pos_rel_x, 0.01);
  ros_msg.ins_pos_rel_y = getScaledValue(adma_data.insPos.pos_rel_y, 0.01);

  ros_msg.ins_vel_hor.x = getScaledValue(adma_data.insVelHor.x, 0.005);
  ros_msg.ins_vel_hor.y = getScaledValue(adma_data.insVelHor.y, 0.005);
  ros_msg.ins_vel_hor.z = getScaledValue(adma_data.insVelHor.z, 0.005);
  ros_msg.ins_vel_frame.x = getScaledValue(adma_data.insVelFrame.x, 0.005);
  ros_msg.ins_vel_frame.y = getScaledValue(adma_data.insVelFrame.y, 0.005);
  ros_msg.ins_vel_frame.z = getScaledValue(adma_data.insVelFrame.z, 0.005);

  ros_msg.ins_stddev_lat = getScaledValue(adma_data.insstddevlat, 0.01);
  ros_msg.ins_stddev_long = getScaledValue(adma_data.insstddevlong, 0.01);
  ros_msg.ins_stddev_height = getScaledValue(adma_data.insstddevheight, 0.01);

  ros_msg.ins_stddev_vel.x = getScaledValue(adma_data.insstddevvelx, 0.01);
  ros_msg.ins_stddev_vel.y = getScaledValue(adma_data.insstddevvely, 0.01);
  ros_msg.ins_stddev_vel.z = getScaledValue(adma_data.insstddevvelz, 0.01);
  ros_msg.ins_stddev_roll = getScaledValue(adma_data.insstddevroll, 0.01);
  ros_msg.ins_stddev_pitch = getScaledValue(adma_data.insstddevpitch, 0.01);
  ros_msg.ins_stddev_yaw = getScaledValue(adma_data.insstddevyaw, 0.01);

  ros_msg.an1 = getScaledValue(adma_data.an1, 0.0005);
  ros_msg.an2 = getScaledValue(adma_data.an2, 0.0005);
  ros_msg.an3 = getScaledValue(adma_data.an3, 0.0005);
  ros_msg.an4 = getScaledValue(adma_data.an4, 0.0005);

  ros_msg.gnss_dualant_heading = getScaledValue(adma_data.gnssDualAntHeading, 0.01);
  ros_msg.gnss_dualant_pitch = getScaledValue(adma_data.gnssDualAntPitch, 0.01);
  ros_msg.gnss_dualant_stddev_heading = getScaledValue(adma_data.gnssdualantstdevheading, 0.01);
  ros_msg.gnss_dualant_stddev_pitch = getScaledValue(adma_data.gnssdualantstddevpitch, 0.01);
  ros_msg.gnss_dualant_stddev_heading_hr =
    getScaledValue(adma_data.gnssdualantstdevheadinghr, 0.01);
  ros_msg.gnss_dualant_stddev_pitch_hr = getScaledValue(adma_data.gnssdualantstddevpitchhr, 0.01);
}

void ADMA2ROSParserV335::mapPOI(
  adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV335 & adma_data)
{
  // first create a array for all available POI
  std::array<adma_ros_driver_msgs::msg::POI, 8> pois;
  // then fill the POIs with data
  for (size_t i = 0; i < 8; i++) {
    adma_ros_driver_msgs::msg::POI new_poi;

    Vector3 cur_acc_body = adma_data.accBodyPOI[i];
    Vector3 cur_acc_horizontal = adma_data.accHorizontalPOI[i];
    Miscellaneous cur_misc = adma_data.miscPOI[i];
    int32_t cur_ins_height = adma_data.insHeightPOI[i];
    INSPosition cur_ins_position = adma_data.insPosPOI[i];
    Vector3 cur_ins_vel_hor = adma_data.insVelHorPOI[i];

    new_poi.acc_body.x = getScaledValue(cur_acc_body.x, 0.0004);
    new_poi.acc_body.y = getScaledValue(cur_acc_body.y, 0.0004);
    new_poi.acc_body.z = getScaledValue(cur_acc_body.z, 0.0004);
    new_poi.acc_hor.x = getScaledValue(cur_acc_horizontal.x, 0.0004);
    new_poi.acc_hor.y = getScaledValue(cur_acc_horizontal.y, 0.0004);
    new_poi.acc_hor.z = getScaledValue(cur_acc_horizontal.z, 0.0004);
    new_poi.inv_path_radius = getScaledValue(cur_misc.invPathRadius, 0.0001);
    new_poi.side_slip_angle = getScaledValue(cur_misc.sideSlipAngle, 0.01);
    new_poi.dist_trav = getScaledValue(cur_misc.distanceTraveled, 0.01);
    new_poi.ins_height = getScaledValue(cur_ins_height, 0.01);
    new_poi.ins_lat_abs = getScaledValue(cur_ins_position.pos_abs.latitude, 0.0000001);
    new_poi.ins_lon_abs = getScaledValue(cur_ins_position.pos_abs.longitude, 0.0000001);
    new_poi.ins_pos_rel_x = getScaledValue(cur_ins_position.pos_rel_x, 0.01);
    new_poi.ins_pos_rel_y = getScaledValue(cur_ins_position.pos_rel_y, 0.01);
    new_poi.ins_vel_hor.x = getScaledValue(cur_ins_vel_hor.x, 0.005);
    new_poi.ins_vel_hor.y = getScaledValue(cur_ins_vel_hor.y, 0.005);
    new_poi.ins_vel_hor.z = getScaledValue(cur_ins_vel_hor.z, 0.005);
    pois[i] = new_poi;
  }
  // finally set the filled POI data to the whole ROS msg
  ros_msg.poi_1 = pois[0];
  ros_msg.poi_2 = pois[1];
  ros_msg.poi_3 = pois[2];
  ros_msg.poi_4 = pois[3];
  ros_msg.poi_5 = pois[4];
  ros_msg.poi_6 = pois[5];
  ros_msg.poi_7 = pois[6];
  ros_msg.poi_8 = pois[7];
}
