#include "adma_ros_driver/parser/adma2ros_parser_v334.hpp"
#include "adma_ros_driver/parser/parser_utils.hpp"

ADMA2ROSParserV334::ADMA2ROSParserV334()
{
}

void ADMA2ROSParserV334::mapAdmaMessageToROS(adma_msgs::AdmaDataScaled& rosMsg, AdmaDataV334& admaData)
{
        mapAdmaHeader(rosMsg, admaData);
        mapErrorWarningBytes(rosMsg.error_warning, admaData);
        mapStatusBitfields(rosMsg.status, admaData);
        mapUnscaledData(rosMsg, admaData);
        mapScaledData(rosMsg, admaData);
        mapPOI(rosMsg, admaData);
}

void ADMA2ROSParserV334::mapStatusToROS(adma_msgs::AdmaStatus& rosMsg, AdmaDataV334& admaData)
{
        mapStatusBytes(rosMsg.status_bytes, admaData);
        mapErrorWarningBytes(rosMsg.error_warnings_bytes, admaData);
        mapErrorWarningBitfields(rosMsg.error_warnings, admaData);
}

void ADMA2ROSParserV334::mapAdmaHeader(adma_msgs::AdmaDataScaled& rosMsg, AdmaDataV334& admaData)
{
        // fill static header information
        AdmaStaticHeader staticHeader = admaData.staticHeader;
        rosMsg.genesys_id = staticHeader.genesysid;
        std::stringstream ss;
        ss <<  int(staticHeader.headerversion[0]) << int(staticHeader.headerversion[1]) << int(staticHeader.headerversion[2]) << int(staticHeader.headerversion[3]);
        rosMsg.header_version = ss.str();
        ss.clear();
        ss.str("");
        rosMsg.format_id = staticHeader.formatid;
        //TODO: this value is parsed wrong?!        
        ss <<  int(staticHeader.formatversion[0]) << int(staticHeader.formatversion[1]) << int(staticHeader.formatversion[2]) << int(staticHeader.formatversion[3]);
        rosMsg.format_version = ss.str();
        rosMsg.serial_number = staticHeader.serialno;

        // fill dynamic header information
        AdmaDynamicHeader dynamicHeader = admaData.dynamicHeader;
        rosMsg.config_id = dynamicHeader.configid;
        rosMsg.config_format = dynamicHeader.configformat;
        rosMsg.config_version = dynamicHeader.configversion;
        rosMsg.config_size = dynamicHeader.configsize;
        rosMsg.byte_offset = dynamicHeader.byteoffset;
        rosMsg.slice_size = dynamicHeader.slicesize;
        rosMsg.slice_data = dynamicHeader.slicedata;
}

void ADMA2ROSParserV334::mapStatusBytes(adma_msgs::ByteStatus& rosMsgByteStatus, AdmaDataV334& admaData)
{
        rosMsgByteStatus.status_byte_0 = admaData.gnssStatus;
        rosMsgByteStatus.status_byte_1 = admaData.signalInStatus;
        rosMsgByteStatus.status_byte_2 = admaData.miscStatus;
        rosMsgByteStatus.status_count = admaData.statuscount;
        rosMsgByteStatus.status_byte_4 = admaData.kfStatus;
        rosMsgByteStatus.status_byte_5 = admaData.statusRobot;
}

void ADMA2ROSParserV334::mapStatusBitfields(adma_msgs::Status& rosMsgStatus, AdmaDataV334& admaData)
{
        // status_byte_0
        unsigned char gnssStatus = admaData.gnssStatus;
        /* status gnss mode */
        std::bitset<8> gnssStatusByte = admaData.gnssStatus;
        std::bitset<4> status_gnss_mode;
        status_gnss_mode[0] = gnssStatusByte[0];
        status_gnss_mode[1] = gnssStatusByte[1];
        status_gnss_mode[2] = gnssStatusByte[2];
        status_gnss_mode[3] = gnssStatusByte[3];
        rosMsgStatus.status_gnss_mode = status_gnss_mode.to_ulong();
        bool standstill_c = getbit(gnssStatus,4);
        bool status_skidding = getbit(gnssStatus,5);
        bool status_external_vel = getbit(gnssStatus,7);

        /* status stand still */
        rosMsgStatus.status_standstill = standstill_c;
        /* status skidding */
        rosMsgStatus.status_skidding = status_skidding;
        /* status external velocity slip */
        rosMsgStatus.status_external_vel_out = status_external_vel;

        // status_byte_1
        unsigned char gnssTriggerStatus = admaData.signalInStatus;
        bool status_trig_gnss = getbit(gnssTriggerStatus,0);
        bool status_signal_in3 = getbit(gnssTriggerStatus,1);
        bool status_signal_in2 = getbit(gnssTriggerStatus,2);
        bool status_signal_in1 = getbit(gnssTriggerStatus,3);
        bool status_alignment = getbit(gnssTriggerStatus,4);
        bool status_ahrs_ins = getbit(gnssTriggerStatus,5);
        bool status_dead_reckoning = getbit(gnssTriggerStatus,6);
        bool status_synclock = getbit(gnssTriggerStatus,7);
        /* status statustriggnss */
        rosMsgStatus.status_trig_gnss = status_trig_gnss;
        /* status statussignalin3 */
        rosMsgStatus.status_signal_in3 = status_signal_in3;
        /* status statussignalin2 */
        rosMsgStatus.status_signal_in2 = status_signal_in2;
        /* status statussignalin1 */
        rosMsgStatus.status_signal_in1 = status_signal_in1;
        /* status statusalignment */
        rosMsgStatus.status_alignment = status_alignment;
        /* status statusahrsins */
        rosMsgStatus.status_ahrs_ins = status_ahrs_ins;
        /* status statusdeadreckoning */
        rosMsgStatus.status_dead_reckoning = status_dead_reckoning;
        /* status statussynclock */
        rosMsgStatus.status_synclock = status_synclock;

        // status_byte_2
        unsigned char evkStatus = admaData.miscStatus;
        bool status_evk_activ = getbit(evkStatus,0);
        bool status_evk_estimates = getbit(evkStatus,1);
        bool status_heading_executed = getbit(evkStatus,2);
        bool status_configuration_changed = getbit(evkStatus,3);
        /* status statustriggnss */
        rosMsgStatus.status_evk_activ = status_evk_activ;
        /* status status_evk_estimates */
        rosMsgStatus.status_evk_estimates = status_evk_estimates;
        /* status status_heading_executed */
        rosMsgStatus.status_heading_executed = status_heading_executed;
        /* status status_configuration_changed */
        rosMsgStatus.status_config_changed = status_configuration_changed;
        /* status tilt */
        std::bitset<8> evkStatusByte = admaData.miscStatus;
        std::bitset<2> status_tilt;
        status_tilt[0] = evkStatusByte[4];
        status_tilt[1] = evkStatusByte[5];
        rosMsgStatus.status_tilt = status_tilt.to_ulong();
        /* status pos */
        std::bitset<2> status_pos;
        status_pos[0] = evkStatusByte[6];
        status_pos[1] = evkStatusByte[7];
        rosMsgStatus.status_pos = status_pos.to_ulong();

        rosMsgStatus.status_count = admaData.statuscount;

        // status_byte_4
        unsigned char kfStatus = admaData.kfStatus;
        bool status_kalmanfilter_settled = getbit(kfStatus,0);
        bool status_kf_lat_stimulated = getbit(kfStatus,1);
        bool status_kf_long_stimulated = getbit(kfStatus,2);
        bool status_kf_steady_state = getbit(kfStatus,3);
        rosMsgStatus.status_kalmanfilter_settled = status_kalmanfilter_settled;
        rosMsgStatus.status_kf_lat_stimulated = status_kf_lat_stimulated;
        rosMsgStatus.status_kf_long_stimulated = status_kf_long_stimulated;
        rosMsgStatus.status_kf_steady_state = status_kf_steady_state;

        std::bitset<8> kfStatusByte = admaData.kfStatus;
        std::bitset<2> status_speed;
        status_speed[0] = kfStatusByte[4];
        status_speed[1] = kfStatusByte[5];
        rosMsgStatus.status_speed = status_speed.to_ulong();

        // status_byte_5
        std::bitset<8> bitStatusRobot = admaData.statusRobot;
        std::bitset<4> statusRobot;
        for(size_t i=0;i<4;i++)
        {
                statusRobot[i] = bitStatusRobot[i];
        }
        rosMsgStatus.status_robot = statusRobot.to_ulong();

}

void ADMA2ROSParserV334::mapErrorWarningBytes(adma_msgs::ByteErrorWarning& rosMsgByteErrorWarning, AdmaDataV334& admaData)
{
        rosMsgByteErrorWarning.error_1 = admaData.dataError1;
        rosMsgByteErrorWarning.error_2 = admaData.dataError2;
        rosMsgByteErrorWarning.warn_1 = admaData.dataWarn1;
        rosMsgByteErrorWarning.error_3 = admaData.dataError3;
}

void ADMA2ROSParserV334::mapErrorWarningBitfields(adma_msgs::ErrorWarning& rosMsgErrorWarning, AdmaDataV334& admaData)
{
        unsigned char error_byte_0 = admaData.dataError1;
        rosMsgErrorWarning.error_gyro_hw = getbit(error_byte_0,0);
        rosMsgErrorWarning.error_accel_hw = getbit(error_byte_0,1);
        rosMsgErrorWarning.error_ext_speed_hw = getbit(error_byte_0,2);
        rosMsgErrorWarning.error_gnss_hw = getbit(error_byte_0,3);
        rosMsgErrorWarning.error_data_bus_checksum = getbit(error_byte_0,4);
        rosMsgErrorWarning.error_eeprom = getbit(error_byte_0,5);
        rosMsgErrorWarning.error_xmit = getbit(error_byte_0,6);
        rosMsgErrorWarning.error_cmd = getbit(error_byte_0,7);

        unsigned char error_byte_1 = admaData.dataError2;
        rosMsgErrorWarning.error_data_bus = getbit(error_byte_1,0);
        rosMsgErrorWarning.error_can_bus = getbit(error_byte_1,1);
        rosMsgErrorWarning.error_num = getbit(error_byte_1,3);
        rosMsgErrorWarning.error_temp_warning = getbit(error_byte_1,4);
        rosMsgErrorWarning.error_reduced_accuracy = getbit(error_byte_1,5);
        rosMsgErrorWarning.error_range_max = getbit(error_byte_1,6);

        unsigned char warn_byte_1 = admaData.dataWarn1;
        rosMsgErrorWarning.warn_gnss_no_solution = getbit(warn_byte_1,0);
        rosMsgErrorWarning.warn_gnss_vel_ignored = getbit(warn_byte_1,1);
        rosMsgErrorWarning.warn_gnss_pos_ignored = getbit(warn_byte_1,2);
        rosMsgErrorWarning.warn_gnss_unable_to_cfg = getbit(warn_byte_1,3);
        rosMsgErrorWarning.warn_speed_off = getbit(warn_byte_1,4);
        rosMsgErrorWarning.warn_gnss_dualant_ignored = getbit(warn_byte_1,5);

        unsigned char error_byte_2 = admaData.dataError3;
        rosMsgErrorWarning.error_hw_sticky = getbit(error_byte_2,0);
}

void ADMA2ROSParserV334::mapUnscaledData(adma_msgs::AdmaDataScaled& rosMsg, AdmaDataV334& admaData)
{
        //fill external velocity
        rosMsg.ext_vel_dig_pulses_x = admaData.extveldigpulsesx;
        rosMsg.ext_vel_dig_pulses_y = admaData.extveldigpulsesy;
        // fill triggers
        rosMsg.trig_rising_1 = admaData.trigrising1;
        rosMsg.trig_falling_1 = admaData.trigfalling1;
        rosMsg.trig_rising_2 = admaData.trigrising2;
        rosMsg.trig_falling_2 = admaData.trigfalling2;
        rosMsg.trig_rising_3 = admaData.trigrising3;
        rosMsg.trig_falling_3 = admaData.trigfalling3;
        rosMsg.trig_rising_4 = admaData.trigrising4;
        rosMsg.trig_falling_4 = admaData.trigfalling4;
        //fill system data
        rosMsg.system_ta = admaData.systemta;
        rosMsg.system_time_since_init = admaData.systemtimesinceinit;
        //fill GNSS time
        rosMsg.gnss_time_msec = admaData.gnsstimemsec;
        rosMsg.gnss_time_week = admaData.gnsstimeweek;
        rosMsg.gnss_trigger = admaData.gnsstrigger;
        //fill GPS AUX data
        rosMsg.gnss_sats_used = admaData.gnsssatsused;
        rosMsg.gnss_sats_visible = admaData.gnsssatsvisible;
        rosMsg.gnss_sats_dualant_used = admaData.gnsssatsdualantused;
        rosMsg.gnss_sats_dualant_visible = admaData.gnsssatsdualantvisible;
        rosMsg.gnss_log_delay = admaData.gnsslogdelay;
        std::stringstream ss;
        ss <<  admaData.gnssbasenr;
        rosMsg.gnss_base_nr = ss.str();
        // GNSS Dual ant information 
        rosMsg.gnss_dualant_time_msec = admaData.gnssDualAntTimeMsec;
        rosMsg.gnss_dualant_time_week = admaData.gnssDualAntTimeWeek;
        //fill INS time UTC
        rosMsg.ins_time_msec = admaData.instimemsec;
        rosMsg.ins_time_week = admaData.instimeweek;
        rosMsg.leap_seconds = admaData.leapseconds;
        // kalman filter status
        rosMsg.kf_lat_stimulated = admaData.kflatstimulated;
        rosMsg.kf_long_stimulated = admaData.kflongstimulated;
        rosMsg.kf_steady_state = admaData.kfsteadystate;
        // gnss receiver status and error
        rosMsg.gnss_receiver_error = admaData.gnssreceivererror;
        rosMsg.gnss_receiver_status = admaData.gnssreceiverstatus;
}

void ADMA2ROSParserV334::mapScaledData(adma_msgs::AdmaDataScaled& rosMsg, AdmaDataV334& admaData)
{
        rosMsg.acc_body_hr.x = getScaledValue(admaData.sensorsBodyX.accHR, 0.0001);
        rosMsg.acc_body_hr.y = getScaledValue(admaData.sensorsBodyY.accHR, 0.0001);
        rosMsg.acc_body_hr.z = getScaledValue(admaData.sensorsBodyZ.accHR, 0.0001);
        rosMsg.rate_body_hr.x = getScaledValue(admaData.sensorsBodyX.rateHR, 0.0001);
        rosMsg.rate_body_hr.y = getScaledValue(admaData.sensorsBodyY.rateHR, 0.0001);
        rosMsg.rate_body_hr.z = getScaledValue(admaData.sensorsBodyZ.rateHR, 0.0001);
        rosMsg.rate_body.x = getScaledValue(admaData.ratesBody.x, 0.01);
        rosMsg.rate_body.y = getScaledValue(admaData.ratesBody.y, 0.01);
        rosMsg.rate_body.z = getScaledValue(admaData.ratesBody.z, 0.01);
        rosMsg.rate_hor.x = getScaledValue(admaData.ratesHorizontal.x, 0.01);
        rosMsg.rate_hor.y = getScaledValue(admaData.ratesHorizontal.y, 0.01);
        rosMsg.rate_hor.z = getScaledValue(admaData.ratesHorizontal.z, 0.01);
        rosMsg.acc_body.x = getScaledValue(admaData.accBody.x, 0.0004);
        rosMsg.acc_body.y = getScaledValue(admaData.accBody.y, 0.0004);
        rosMsg.acc_body.z = getScaledValue(admaData.accBody.z, 0.0004);
        rosMsg.acc_hor.x = getScaledValue(admaData.accHorizontal.x, 0.0004);
        rosMsg.acc_hor.y = getScaledValue(admaData.accHorizontal.y, 0.0004);
        rosMsg.acc_hor.z = getScaledValue(admaData.accHorizontal.z, 0.0004);

        rosMsg.ext_vel_an_x = getScaledValue(admaData.extVelAnalog.x, 0.005);
        rosMsg.ext_vel_an_y = getScaledValue(admaData.extVelAnalog.y, 0.005);
        rosMsg.ext_vel_dig_x = getScaledValue(admaData.extveldigx, 0.005);
        rosMsg.ext_vel_dig_y = getScaledValue(admaData.extveldigy, 0.005);
        rosMsg.ext_vel_x_corrected = getScaledValue(admaData.extVelCorrected.x, 0.005);
        rosMsg.ext_vel_y_corrected = getScaledValue(admaData.extVelCorrected.y, 0.005);

        rosMsg.inv_path_radius = getScaledValue(admaData.misc.invPathRadius, 0.0001);
        rosMsg.side_slip_angle = getScaledValue(admaData.misc.sideSlipAngle, 0.01);
        rosMsg.dist_trav = getScaledValue(admaData.misc.distanceTraveled, 0.01);

        rosMsg.system_temp = getScaledValue(admaData.systemtemp, 0.1);
        rosMsg.system_dsp_load = getScaledValue(admaData.systemdspload, 0.1);

        rosMsg.gnss_lat_abs = getScaledValue(admaData.posAbs.latitude, 0.0000001);
        rosMsg.gnss_long_abs = getScaledValue(admaData.posAbs.longitude, 0.0000001);
        rosMsg.gnss_pos_rel_x = getScaledValue(admaData.posRel.latitude, 0.01);
        rosMsg.gnss_pos_rel_y = getScaledValue(admaData.posRel.longitude, 0.01);

        rosMsg.gnss_stddev_lat = getScaledValue(admaData.gnssstddevlat, 0.001);
        rosMsg.gnss_stddev_long = getScaledValue(admaData.gnssstddevlon, 0.001);
        rosMsg.gnss_stddev_height = getScaledValue(admaData.gnssstddevheight, 0.001);

        rosMsg.gnss_vel_frame.x = getScaledValue(admaData.gnssvelframex, 0.005);
        rosMsg.gnss_vel_frame.y = getScaledValue(admaData.gnssvelframey, 0.005);
        rosMsg.gnss_vel_frame.z = getScaledValue(admaData.gnssvelframez, 0.005);
        rosMsg.gnss_vel_latency = getScaledValue(admaData.gnssvellatency, 0.001);

        rosMsg.gnss_stddev_vel.x = getScaledValue(admaData.gnssStdDevVel.x, 0.001);
        rosMsg.gnss_stddev_vel.y = getScaledValue(admaData.gnssStdDevVel.y, 0.001);
        rosMsg.gnss_stddev_vel.z = getScaledValue(admaData.gnssStdDevVel.z, 0.001);

        rosMsg.gnss_diffage = getScaledValue(admaData.gnssdiffage, 0.1);
        rosMsg.gnss_receiver_load = getScaledValue(admaData.gnssreceiverload, 0.5);

        rosMsg.ins_roll = getScaledValue(admaData.insroll, 0.01);
        rosMsg.ins_pitch = getScaledValue(admaData.inspitch, 0.01);
        rosMsg.ins_yaw = getScaledValue(admaData.insyaw, 0.01);
        rosMsg.gnss_cog = getScaledValue(admaData.gnsscog, 0.01);

        rosMsg.gnss_height = getScaledValue(admaData.gnssheight, 0.01);
        rosMsg.undulation = getScaledValue(admaData.undulation, 0.01);

        rosMsg.ins_height = getScaledValue(admaData.insHeight, 0.01);

        rosMsg.ins_lat_abs = getScaledValue(admaData.insPos.pos_abs.latitude, 0.0000001);
        rosMsg.ins_long_abs = getScaledValue(admaData.insPos.pos_abs.longitude, 0.0000001);
        rosMsg.ins_pos_rel_x = getScaledValue(admaData.insPos.pos_rel_x, 0.01);
        rosMsg.ins_pos_rel_y = getScaledValue(admaData.insPos.pos_rel_y, 0.01);
        
        rosMsg.ins_vel_hor.x = getScaledValue(admaData.insVelHor.x, 0.005);
        rosMsg.ins_vel_hor.y = getScaledValue(admaData.insVelHor.y, 0.005);
        rosMsg.ins_vel_hor.z = getScaledValue(admaData.insVelHor.z, 0.005);
        rosMsg.ins_vel_frame.x = getScaledValue(admaData.insVelFrame.x, 0.005);
        rosMsg.ins_vel_frame.y = getScaledValue(admaData.insVelFrame.y, 0.005);
        rosMsg.ins_vel_frame.z = getScaledValue(admaData.insVelFrame.z, 0.005);

        rosMsg.ins_stddev_lat = getScaledValue(admaData.insstddevlat, 0.01);
        rosMsg.ins_stddev_long = getScaledValue(admaData.insstddevlong, 0.01);
        rosMsg.ins_stddev_height = getScaledValue(admaData.insstddevheight, 0.01);

        rosMsg.ins_stddev_vel.x = getScaledValue(admaData.insstddevvelx, 0.01);
        rosMsg.ins_stddev_vel.y = getScaledValue(admaData.insstddevvely, 0.01);
        rosMsg.ins_stddev_vel.z = getScaledValue(admaData.insstddevvelz, 0.01);
        rosMsg.ins_stddev_roll = getScaledValue(admaData.insstddevroll, 0.01);
        rosMsg.ins_stddev_pitch = getScaledValue(admaData.insstddevpitch, 0.01);
        rosMsg.ins_stddev_yaw = getScaledValue(admaData.insstddevyaw, 0.01);

        rosMsg.an1 = getScaledValue(admaData.an1, 0.0005);
        rosMsg.an2 = getScaledValue(admaData.an2, 0.0005);
        rosMsg.an3 = getScaledValue(admaData.an3, 0.0005);
        rosMsg.an4 = getScaledValue(admaData.an4, 0.0005);

        rosMsg.gnss_dualant_heading = getScaledValue(admaData.gnssDualAntHeading, 0.01);
        rosMsg.gnss_dualant_pitch = getScaledValue(admaData.gnssDualAntPitch, 0.01);
        rosMsg.gnss_dualant_stddev_heading = getScaledValue(admaData.gnssdualantstdevheading, 0.01);
        rosMsg.gnss_dualant_stddev_pitch = getScaledValue(admaData.gnssdualantstddevpitch, 0.01);
        rosMsg.gnss_dualant_stddev_heading_hr = getScaledValue(admaData.gnssdualantstdevheadinghr, 0.01);
        rosMsg.gnss_dualant_stddev_pitch_hr = getScaledValue(admaData.gnssdualantstddevpitchhr, 0.01);
}

void ADMA2ROSParserV334::mapPOI(adma_msgs::AdmaDataScaled& rosMsg, AdmaDataV334& admaData)
{
        std::array<adma_msgs::POI, 8> pois;
        for (size_t i = 0; i < 8; i++)
        {
                adma_msgs::POI newPOI;

                Vector3 curAccBody = admaData.accBodyPOI[i];
                Vector3 curAccHorizontal = admaData.accHorizontalPOI[i];
                Miscellaneous curMisc = admaData.miscPOI[i];
                int32_t curINSHeight = admaData.insHeightPOI[i];
                INSPosition curINSPosition = admaData.insPosPOI[i];
                Vector3 curINSVelHor = admaData.insVelHorPOI[i];

                newPOI.acc_body.x = getScaledValue(curAccBody.x, 0.0004);
                newPOI.acc_body.y = getScaledValue(curAccBody.y, 0.0004);
                newPOI.acc_body.z = getScaledValue(curAccBody.z, 0.0004);
                newPOI.acc_hor.x = getScaledValue(curAccHorizontal.x, 0.0004);
                newPOI.acc_hor.y = getScaledValue(curAccHorizontal.y, 0.0004);
                newPOI.acc_hor.z = getScaledValue(curAccHorizontal.z, 0.0004);
                newPOI.inv_path_radius = getScaledValue(curMisc.invPathRadius, 0.0001);
                newPOI.side_slip_angle = getScaledValue(curMisc.sideSlipAngle, 0.01);
                newPOI.dist_trav = getScaledValue(curMisc.distanceTraveled, 0.01);
                newPOI.ins_height = getScaledValue(curINSHeight, 0.01);
                newPOI.ins_lat_abs = getScaledValue(curINSPosition.pos_abs.latitude, 0.0000001);
                newPOI.ins_lon_abs = getScaledValue(curINSPosition.pos_abs.longitude, 0.0000001);
                newPOI.ins_pos_rel_x = getScaledValue(curINSPosition.pos_rel_x, 0.01);
                newPOI.ins_pos_rel_y = getScaledValue(curINSPosition.pos_rel_y, 0.01);
                newPOI.ins_vel_hor.x = getScaledValue(curINSVelHor.x, 0.005);
                newPOI.ins_vel_hor.y = getScaledValue(curINSVelHor.y, 0.005);
                newPOI.ins_vel_hor.z = getScaledValue(curINSVelHor.z, 0.005);
                pois[i] = newPOI;
        }

        rosMsg.poi_1 = pois[0];
        rosMsg.poi_2 = pois[1];
        rosMsg.poi_3 = pois[2];
        rosMsg.poi_4 = pois[3];
        rosMsg.poi_5 = pois[4];
        rosMsg.poi_6 = pois[5];
        rosMsg.poi_7 = pois[6];
        rosMsg.poi_8 = pois[7];
}