#include "adma_ros2_driver/parser/adma2ros_parser_v334.hpp"
#include "adma_ros2_driver/parser/parser_utils.hpp"

ADMA2ROSParserV334::ADMA2ROSParserV334()
{
}

void ADMA2ROSParserV334::mapAdmaMessageToROS(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV333& admaData)
{
        mapAdmaHeader(rosMsg, admaData);
        mapBitfields(rosMsg, admaData);
        mapUnscaledData(rosMsg, admaData);
        mapScaledData(rosMsg, admaData);
}

void ADMA2ROSParserV334::mapAdmaHeader(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV333& admaData)
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

void ADMA2ROSParserV334::mapBitfields(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV333& admaData)
{
        unsigned char gnssStatus = admaData.gnssStatus;
        bool status_external_vel = getbit(gnssStatus,7);
        bool status_skidding = getbit(gnssStatus,5);
        bool standstill_c = getbit(gnssStatus,4);
        bool rtk_precise = getbit(gnssStatus,3);
        bool rtk_coarse = getbit(gnssStatus,2);
        bool gnss_mode = getbit(gnssStatus,1);
        bool gnss_out = getbit(gnssStatus,0);

        /* status gnss mode */
        if(gnss_out)
        {
                rosMsg.status_gnss_mode = 1;
        }
        else if (gnss_mode) 
        {
                rosMsg.status_gnss_mode = 2;
        }
        else if (rtk_coarse) 
        {
                rosMsg.status_gnss_mode = 4;
        }
        else if (rtk_precise) 
        {
                rosMsg.status_gnss_mode = 8;
        }
        /* status stand still */
        rosMsg.status_stand_still = standstill_c;
        /* status skidding */
        rosMsg.status_skidding = status_skidding;
        /* status external velocity slip */
        rosMsg.status_external_vel_out = status_external_vel;

        
        unsigned char gnssTriggerStatus = admaData.signalInStatus;
        bool status_synclock = getbit(gnssTriggerStatus,7);
        bool status_dead_reckoning = getbit(gnssTriggerStatus,6);
        bool status_ahrs_ins = getbit(gnssTriggerStatus,5);
        bool status_alignment = getbit(gnssTriggerStatus,4);
        bool status_signal_in1 = getbit(gnssTriggerStatus,3);
        bool status_signal_in2 = getbit(gnssTriggerStatus,2);
        bool status_signal_in3 = getbit(gnssTriggerStatus,1);
        bool status_trig_gnss = getbit(gnssTriggerStatus,0);
        /* status statustriggnss */
        rosMsg.status_trig_gnss = status_trig_gnss;
        /* status statussignalin3 */
        rosMsg.status_signal_in3 = status_signal_in3;
        /* status statussignalin2 */
        rosMsg.status_signal_in2 = status_signal_in2;
        /* status statussignalin1 */
        rosMsg.status_signal_in1 = status_signal_in1;
        /* status statusalignment */
        rosMsg.status_alignment = status_alignment;
        /* status statusahrsins */
        rosMsg.status_ahrs_ins = status_ahrs_ins;
        /* status statusdeadreckoning */
        rosMsg.status_dead_reckoning = status_dead_reckoning;
        /* status statussynclock */
        rosMsg.status_sync_lock = status_synclock;


        unsigned char evkStatus = admaData.miscStatus;
        bool status_pos_b2 = getbit(evkStatus,7);
        bool status_pos_b1 = getbit(evkStatus,6);
        bool status_tilt_b2 = getbit(evkStatus,5);
        bool status_tilt_b1 = getbit(evkStatus,4);
        bool status_configuration_changed = getbit(evkStatus,3);
        bool status_heading_executed = getbit(evkStatus,2);
        bool status_evk_estimates = getbit(evkStatus,1);
        bool status_evk_activ = getbit(evkStatus,0);
        /* status statustriggnss */
        rosMsg.status_evk_activ = status_evk_activ;
        /* status status_evk_estimates */
        rosMsg.status_evk_estimates = status_evk_estimates;
        /* status status_heading_executed */
        rosMsg.status_heading_executed = status_heading_executed;
        /* status status_configuration_changed */
        rosMsg.status_config_changed = status_configuration_changed;
        /* status tilt */
        if(status_tilt_b1==0 && status_tilt_b2==0)
        {
                rosMsg.status_tilt = 0;
        }
        else if(status_tilt_b1==0 && status_tilt_b2==1)
        {
                rosMsg.status_tilt = 1;
        }
        else if(status_tilt_b1==1 && status_tilt_b2==0)
        {
                rosMsg.status_tilt = 2;
        }
        /* status pos */
        if(status_pos_b1==0 && status_pos_b2==0)
        {
                rosMsg.status_pos = 0;
        }
        else if(status_pos_b1==0 && status_pos_b2==1)
        {
                rosMsg.status_pos = 1;
        }
        else if(status_pos_b1==1 && status_pos_b2==0)
        {
                rosMsg.status_pos = 2;
        }


        unsigned char kfStatus = admaData.kfStatus;
        bool status_speed_b2 = getbit(kfStatus,5);
        bool status_speed_b1 = getbit(kfStatus,4);
        bool status_kf_steady_state = getbit(kfStatus,3);
        bool status_kf_long_stimulated = getbit(kfStatus,2);
        bool status_kf_lat_stimulated = getbit(kfStatus,1);
        bool status_kalmanfilter_settled = getbit(kfStatus,0);
        rosMsg.status_kalmanfilter_settled = status_kalmanfilter_settled;
        rosMsg.status_kf_lat_stimulated = status_kf_lat_stimulated;
        rosMsg.status_kf_long_stimulated = status_kf_long_stimulated;
        rosMsg.status_kf_steady_state = status_kf_steady_state;
        if(status_speed_b1==0 && status_speed_b2==0)
        {
                rosMsg.status_speed = 0;
        }
        else if(status_speed_b1==0 && status_speed_b2==1)
        {
                rosMsg.status_speed = 1;
        }
        else if(status_speed_b1==1 && status_speed_b2==0)
        {
                rosMsg.status_speed = 2;
        }


        unsigned char ewBytes[] = {admaData.dataError1, admaData.dataError2, admaData.dataWarn1, admaData.dataError3};
        std::bitset<8> bitdataerror1 = ewBytes[0];
        std::bitset<8> bitdataerror2 = ewBytes[1];
        std::bitset<8> bitdatawarn3 = ewBytes[2];
        std::bitset<8> errorhw = ewBytes[3];
        std::bitset<4> erhw1;
        std::bitset<4> ermisc1;
        std::bitset<4> ermisc2;
        std::bitset<4> ermisc3;
        std::bitset<4> warngnss;
        std::bitset<4> warnmisc1;
        std::bitset<1> erhwsticky;

        for(size_t i=0;i<4;i++)
        {
                erhw1[i]    = bitdataerror1[i];
                ermisc1[i]  = bitdataerror1[i+4];
                ermisc2[i]  = bitdataerror2[i];
                ermisc3[i]  = bitdataerror2[i+4];
                warngnss[i]  = bitdatawarn3[i];
                warnmisc1[i]  = bitdatawarn3[i+4];
        }
        erhwsticky[0] = errorhw[1];
        rosMsg.error_hardware = erhw1.to_string();
        rosMsg.error_misc1 = ermisc1.to_string();
        rosMsg.error_misc2 = ermisc2.to_string();
        rosMsg.error_misc3 = ermisc3.to_string();
        rosMsg.warn_gnss = warngnss.to_string();
        rosMsg.warn_misc1 = warnmisc1.to_string();
        rosMsg.error_hw_sticky = erhwsticky.to_string();
}

void ADMA2ROSParserV334::mapUnscaledData(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV333& admaData)
{
        rosMsg.status_count = admaData.statuscount;
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

void ADMA2ROSParserV334::mapScaledData(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV333& admaData)
{
        rosMsg.acc_body_hr_x = getScaledValue(admaData.sensorsBodyX.accHR, 0.0001);
        rosMsg.acc_body_hr_y = getScaledValue(admaData.sensorsBodyY.accHR, 0.0001);
        rosMsg.acc_body_hr_z = getScaledValue(admaData.sensorsBodyZ.accHR, 0.0001);
        rosMsg.rate_body_hr_x = getScaledValue(admaData.sensorsBodyX.rateHR, 0.0001);
        rosMsg.rate_body_hr_y = getScaledValue(admaData.sensorsBodyY.rateHR, 0.0001);
        rosMsg.rate_body_hr_z = getScaledValue(admaData.sensorsBodyZ.rateHR, 0.0001);
        rosMsg.rate_body_x = getScaledValue(admaData.ratesBody.x, 0.01);
        rosMsg.rate_body_y = getScaledValue(admaData.ratesBody.y, 0.01);
        rosMsg.rate_body_z = getScaledValue(admaData.ratesBody.z, 0.01);
        rosMsg.rate_hor_x = getScaledValue(admaData.ratesHorizontal.x, 0.01);
        rosMsg.rate_hor_y = getScaledValue(admaData.ratesHorizontal.y, 0.01);
        rosMsg.rate_hor_z = getScaledValue(admaData.ratesHorizontal.z, 0.01);
        rosMsg.acc_body_x = getScaledValue(admaData.accBody.x, 0.0004);
        rosMsg.acc_body_y = getScaledValue(admaData.accBody.y, 0.0004);
        rosMsg.acc_body_z = getScaledValue(admaData.accBody.z, 0.0004);
        rosMsg.acc_hor_x = getScaledValue(admaData.accHorizontal.x, 0.0004);
        rosMsg.acc_hor_y = getScaledValue(admaData.accHorizontal.y, 0.0004);
        rosMsg.acc_hor_z = getScaledValue(admaData.accHorizontal.z, 0.0004);
        rosMsg.acc_body_x_1 = getScaledValue(admaData.accBodyPOI1.x, 0.0004);
        rosMsg.acc_body_y_1 = getScaledValue(admaData.accBodyPOI1.y, 0.0004);
        rosMsg.acc_body_z_1 = getScaledValue(admaData.accBodyPOI1.z, 0.0004);
        rosMsg.acc_body_x_2 = getScaledValue(admaData.accBodyPOI2.x, 0.0004);
        rosMsg.acc_body_y_2 = getScaledValue(admaData.accBodyPOI2.y, 0.0004);
        rosMsg.acc_body_z_2 = getScaledValue(admaData.accBodyPOI2.z, 0.0004);
        rosMsg.acc_body_x_3 = getScaledValue(admaData.accBodyPOI3.x, 0.0004);
        rosMsg.acc_body_y_3 = getScaledValue(admaData.accBodyPOI3.y, 0.0004);
        rosMsg.acc_body_z_3 = getScaledValue(admaData.accBodyPOI3.z, 0.0004);
        rosMsg.acc_body_x_4 = getScaledValue(admaData.accBodyPOI4.x, 0.0004);
        rosMsg.acc_body_y_4 = getScaledValue(admaData.accBodyPOI4.y, 0.0004);
        rosMsg.acc_body_z_4 = getScaledValue(admaData.accBodyPOI4.z, 0.0004);
        rosMsg.acc_body_x_5 = getScaledValue(admaData.accBodyPOI5.x, 0.0004);
        rosMsg.acc_body_y_5 = getScaledValue(admaData.accBodyPOI5.y, 0.0004);
        rosMsg.acc_body_z_5 = getScaledValue(admaData.accBodyPOI5.z, 0.0004);
        rosMsg.acc_body_x_6 = getScaledValue(admaData.accBodyPOI6.x, 0.0004);
        rosMsg.acc_body_y_6 = getScaledValue(admaData.accBodyPOI6.y, 0.0004);
        rosMsg.acc_body_z_6 = getScaledValue(admaData.accBodyPOI6.z, 0.0004);
        rosMsg.acc_body_x_7 = getScaledValue(admaData.accBodyPOI7.x, 0.0004);
        rosMsg.acc_body_y_7 = getScaledValue(admaData.accBodyPOI7.y, 0.0004);
        rosMsg.acc_body_z_7 = getScaledValue(admaData.accBodyPOI7.z, 0.0004);
        rosMsg.acc_body_x_8 = getScaledValue(admaData.accBodyPOI8.x, 0.0004);
        rosMsg.acc_body_y_8 = getScaledValue(admaData.accBodyPOI8.y, 0.0004);
        rosMsg.acc_body_z_8 = getScaledValue(admaData.accBodyPOI8.z, 0.0004);

        rosMsg.acc_hor_x_1 = getScaledValue(admaData.accHorizontalPOI1.x, 0.0004);
        rosMsg.acc_hor_y_1 = getScaledValue(admaData.accHorizontalPOI1.y, 0.0004);
        rosMsg.acc_hor_z_1 = getScaledValue(admaData.accHorizontalPOI1.z, 0.0004);
        rosMsg.acc_hor_x_2 = getScaledValue(admaData.accHorizontalPOI2.x, 0.0004);
        rosMsg.acc_hor_y_2 = getScaledValue(admaData.accHorizontalPOI2.y, 0.0004);
        rosMsg.acc_hor_z_2 = getScaledValue(admaData.accHorizontalPOI2.z, 0.0004);
        rosMsg.acc_hor_x_3 = getScaledValue(admaData.accHorizontalPOI3.x, 0.0004);
        rosMsg.acc_hor_y_3 = getScaledValue(admaData.accHorizontalPOI3.y, 0.0004);
        rosMsg.acc_hor_z_3 = getScaledValue(admaData.accHorizontalPOI3.z, 0.0004);
        rosMsg.acc_hor_x_4 = getScaledValue(admaData.accHorizontalPOI4.x, 0.0004);
        rosMsg.acc_hor_y_4 = getScaledValue(admaData.accHorizontalPOI4.y, 0.0004);
        rosMsg.acc_hor_z_4 = getScaledValue(admaData.accHorizontalPOI4.z, 0.0004);
        rosMsg.acc_hor_x_5 = getScaledValue(admaData.accHorizontalPOI5.x, 0.0004);
        rosMsg.acc_hor_y_5 = getScaledValue(admaData.accHorizontalPOI5.y, 0.0004);
        rosMsg.acc_hor_z_5 = getScaledValue(admaData.accHorizontalPOI5.z, 0.0004);
        rosMsg.acc_hor_x_6 = getScaledValue(admaData.accHorizontalPOI6.x, 0.0004);
        rosMsg.acc_hor_y_6 = getScaledValue(admaData.accHorizontalPOI6.y, 0.0004);
        rosMsg.acc_hor_z_6 = getScaledValue(admaData.accHorizontalPOI6.z, 0.0004);
        rosMsg.acc_hor_x_7 = getScaledValue(admaData.accHorizontalPOI7.x, 0.0004);
        rosMsg.acc_hor_y_7 = getScaledValue(admaData.accHorizontalPOI7.y, 0.0004);
        rosMsg.acc_hor_z_7 = getScaledValue(admaData.accHorizontalPOI7.z, 0.0004);
        rosMsg.acc_hor_x_8 = getScaledValue(admaData.accHorizontalPOI8.x, 0.0004);
        rosMsg.acc_hor_y_8 = getScaledValue(admaData.accHorizontalPOI8.y, 0.0004);
        rosMsg.acc_hor_z_8 = getScaledValue(admaData.accHorizontalPOI8.z, 0.0004);

        rosMsg.ext_vel_an_x = getScaledValue(admaData.extVelAnalog.x, 0.005);
        rosMsg.ext_vel_an_y = getScaledValue(admaData.extVelAnalog.y, 0.005);
        rosMsg.ext_vel_dig_x = getScaledValue(admaData.extveldigx, 0.005);
        rosMsg.ext_vel_dig_y = getScaledValue(admaData.extveldigy, 0.005);
        rosMsg.ext_vel_x_corrected = getScaledValue(admaData.extVelCorrected.x, 0.005);
        rosMsg.ext_vel_y_corrected = getScaledValue(admaData.extVelCorrected.y, 0.005);

        rosMsg.inv_path_radius = getScaledValue(admaData.misc.invPathRadius, 0.0001);
        rosMsg.side_slip_angle = getScaledValue(admaData.misc.sideSlipAngle, 0.01);
        rosMsg.dist_trav = getScaledValue(admaData.misc.distanceTraveled, 0.01);

        rosMsg.inv_path_radius_1 = getScaledValue(admaData.miscPOI1.invPathRadius, 0.0001);
        rosMsg.side_slip_angle_1 = getScaledValue(admaData.miscPOI1.sideSlipAngle, 0.01);
        rosMsg.dist_trav_1 = getScaledValue(admaData.miscPOI1.distanceTraveled, 0.01);
        rosMsg.inv_path_radius_2 = getScaledValue(admaData.miscPOI2.invPathRadius, 0.0001);
        rosMsg.side_slip_angle_2 = getScaledValue(admaData.miscPOI2.sideSlipAngle, 0.01);
        rosMsg.dist_trav_2 = getScaledValue(admaData.miscPOI2.distanceTraveled, 0.01);
        rosMsg.inv_path_radius_3 = getScaledValue(admaData.miscPOI3.invPathRadius, 0.0001);
        rosMsg.side_slip_angle_3 = getScaledValue(admaData.miscPOI3.sideSlipAngle, 0.01);
        rosMsg.dist_trav_3 = getScaledValue(admaData.miscPOI3.distanceTraveled, 0.01);
        rosMsg.inv_path_radius_4 = getScaledValue(admaData.miscPOI4.invPathRadius, 0.0001);
        rosMsg.side_slip_angle_4 = getScaledValue(admaData.miscPOI4.sideSlipAngle, 0.01);
        rosMsg.dist_trav_4 = getScaledValue(admaData.miscPOI4.distanceTraveled, 0.01);
        rosMsg.inv_path_radius_5 = getScaledValue(admaData.miscPOI5.invPathRadius, 0.0001);
        rosMsg.side_slip_angle_5 = getScaledValue(admaData.miscPOI5.sideSlipAngle, 0.01);
        rosMsg.dist_trav_5 = getScaledValue(admaData.miscPOI5.distanceTraveled, 0.01);
        rosMsg.inv_path_radius_6 = getScaledValue(admaData.miscPOI6.invPathRadius, 0.0001);
        rosMsg.side_slip_angle_6 = getScaledValue(admaData.miscPOI6.sideSlipAngle, 0.01);
        rosMsg.dist_trav_6 = getScaledValue(admaData.miscPOI6.distanceTraveled, 0.01);
        rosMsg.inv_path_radius_7 = getScaledValue(admaData.miscPOI7.invPathRadius, 0.0001);
        rosMsg.side_slip_angle_7 = getScaledValue(admaData.miscPOI7.sideSlipAngle, 0.01);
        rosMsg.dist_trav_7 = getScaledValue(admaData.miscPOI7.distanceTraveled, 0.01);
        rosMsg.inv_path_radius_8 = getScaledValue(admaData.miscPOI8.invPathRadius, 0.0001);
        rosMsg.side_slip_angle_8 = getScaledValue(admaData.miscPOI8.sideSlipAngle, 0.01);
        rosMsg.dist_trav_8 = getScaledValue(admaData.miscPOI8.distanceTraveled, 0.01);

        rosMsg.system_temp = getScaledValue(admaData.systemtemp, 0.1);
        rosMsg.system_dsp_load = getScaledValue(admaData.systemdspload, 0.1);

        rosMsg.gnss_lat_abs = getScaledValue(admaData.posAbs.latitude, 0.0000001);
        rosMsg.gnss_lon_abs = getScaledValue(admaData.posAbs.longitude, 0.0000001);
        rosMsg.gnss_pos_rel_x = getScaledValue(admaData.posRel.longitude, 0.01);
        rosMsg.gnss_pos_rel_y = getScaledValue(admaData.posRel.latitude, 0.01);

        rosMsg.gnss_stddev_lat = getScaledValue(admaData.gnssstddevlat, 0.001);
        rosMsg.gnss_stddev_lon = getScaledValue(admaData.gnssstddevlon, 0.001);
        rosMsg.gnss_stddev_height = getScaledValue(admaData.gnssstddevheight, 0.001);

        rosMsg.gnss_vel_frame_x = getScaledValue(admaData.gnssvelframex, 0.005);
        rosMsg.gnss_vel_frame_y = getScaledValue(admaData.gnssvelframey, 0.005);
        rosMsg.gnss_vel_frame_z = getScaledValue(admaData.gnssvelframez, 0.005);
        rosMsg.gnss_vel_latency = getScaledValue(admaData.gnssvellatency, 0.001);

        rosMsg.gnss_stddev_vel_x = getScaledValue(admaData.gnssStdDevVel.x, 0.001);
        rosMsg.gnss_stddev_vel_y = getScaledValue(admaData.gnssStdDevVel.y, 0.001);
        rosMsg.gnss_stddev_vel_z = getScaledValue(admaData.gnssStdDevVel.z, 0.001);

        rosMsg.gnss_diffage = getScaledValue(admaData.gnssdiffage, 0.1);
        rosMsg.gnss_receiver_load = getScaledValue(admaData.gnssreceiverload, 0.5);

        rosMsg.ins_roll = getScaledValue(admaData.insroll, 0.01);
        rosMsg.ins_pitch = getScaledValue(admaData.inspitch, 0.01);
        rosMsg.ins_yaw = getScaledValue(admaData.insyaw, 0.01);
        rosMsg.gnss_cog = getScaledValue(admaData.gnsscog, 0.01);

        rosMsg.gnss_height = getScaledValue(admaData.gnssheight, 0.01);
        rosMsg.undulation = getScaledValue(admaData.undulation, 0.01);

        rosMsg.ins_height = getScaledValue(admaData.insHeight, 0.01);
        rosMsg.ins_height_1 = getScaledValue(admaData.insHeightPOI1, 0.01);
        rosMsg.ins_height_2 = getScaledValue(admaData.insHeightPOI2, 0.01);
        rosMsg.ins_height_3 = getScaledValue(admaData.insHeightPOI3, 0.01);
        rosMsg.ins_height_4 = getScaledValue(admaData.insHeightPOI4, 0.01);
        rosMsg.ins_height_5 = getScaledValue(admaData.insHeightPOI5, 0.01);
        rosMsg.ins_height_6 = getScaledValue(admaData.insHeightPOI6, 0.01);
        rosMsg.ins_height_7 = getScaledValue(admaData.insHeightPOI7, 0.01);
        rosMsg.ins_height_8 = getScaledValue(admaData.insHeightPOI8, 0.01);

        rosMsg.ins_lat_abs = getScaledValue(admaData.insPosAbs.latitude, 0.0000001);
        rosMsg.ins_lon_abs = getScaledValue(admaData.insPosAbs.longitude, 0.0000001);
        rosMsg.ins_pos_rel_x = getScaledValue(admaData.insPosRel.longitude, 0.01);
        rosMsg.ins_pos_rel_y = getScaledValue(admaData.insPosRel.latitude, 0.01);
        rosMsg.ins_lat_abs_1 = getScaledValue(admaData.insPosAbsPOI1.latitude, 0.0000001);
        rosMsg.ins_lon_abs_1 = getScaledValue(admaData.insPosAbsPOI1.longitude, 0.0000001);
        rosMsg.ins_pos_rel_x_1 = getScaledValue(admaData.insPosRelPOI1.longitude, 0.01);
        rosMsg.ins_pos_rel_y_1 = getScaledValue(admaData.insPosRelPOI1.latitude, 0.01);
        rosMsg.ins_lat_abs_2 = getScaledValue(admaData.insPosAbsPOI2.latitude, 0.0000001);
        rosMsg.ins_lon_abs_2 = getScaledValue(admaData.insPosAbsPOI2.longitude, 0.0000001);
        rosMsg.ins_pos_rel_x_2 = getScaledValue(admaData.insPosRelPOI2.longitude, 0.01);
        rosMsg.ins_pos_rel_y_2 = getScaledValue(admaData.insPosRelPOI2.latitude, 0.01);
        rosMsg.ins_lat_abs_3 = getScaledValue(admaData.insPosAbsPOI3.latitude, 0.0000001);
        rosMsg.ins_lon_abs_3 = getScaledValue(admaData.insPosAbsPOI3.longitude, 0.0000001);
        rosMsg.ins_pos_rel_x_3 = getScaledValue(admaData.insPosRelPOI3.longitude, 0.01);
        rosMsg.ins_pos_rel_y_3 = getScaledValue(admaData.insPosRelPOI3.latitude, 0.01);
        rosMsg.ins_lat_abs_4 = getScaledValue(admaData.insPosAbsPOI4.latitude, 0.0000001);
        rosMsg.ins_lon_abs_4 = getScaledValue(admaData.insPosAbsPOI4.longitude, 0.0000001);
        rosMsg.ins_pos_rel_x_4 = getScaledValue(admaData.insPosRelPOI4.longitude, 0.01);
        rosMsg.ins_pos_rel_y_4 = getScaledValue(admaData.insPosRelPOI4.latitude, 0.01);
        rosMsg.ins_lat_abs_5 = getScaledValue(admaData.insPosAbsPOI5.latitude, 0.0000001);
        rosMsg.ins_lon_abs_5 = getScaledValue(admaData.insPosAbsPOI5.longitude, 0.0000001);
        rosMsg.ins_pos_rel_x_5 = getScaledValue(admaData.insPosRelPOI5.longitude, 0.01);
        rosMsg.ins_pos_rel_y_5 = getScaledValue(admaData.insPosRelPOI5.latitude, 0.01);
        rosMsg.ins_lat_abs_6 = getScaledValue(admaData.insPosAbsPOI6.latitude, 0.0000001);
        rosMsg.ins_lon_abs_6 = getScaledValue(admaData.insPosAbsPOI6.longitude, 0.0000001);
        rosMsg.ins_pos_rel_x_6 = getScaledValue(admaData.insPosRelPOI6.longitude, 0.01);
        rosMsg.ins_pos_rel_y_6 = getScaledValue(admaData.insPosRelPOI6.latitude, 0.01);
        rosMsg.ins_lat_abs_7 = getScaledValue(admaData.insPosAbsPOI7.latitude, 0.0000001);
        rosMsg.ins_lon_abs_7 = getScaledValue(admaData.insPosAbsPOI7.longitude, 0.0000001);
        rosMsg.ins_pos_rel_x_7 = getScaledValue(admaData.insPosRelPOI7.longitude, 0.01);
        rosMsg.ins_pos_rel_y_7 = getScaledValue(admaData.insPosRelPOI7.latitude, 0.01);
        rosMsg.ins_lat_abs_8 = getScaledValue(admaData.insPosAbsPOI8.latitude, 0.0000001);
        rosMsg.ins_lon_abs_8 = getScaledValue(admaData.insPosAbsPOI8.longitude, 0.0000001);
        rosMsg.ins_pos_rel_x_8 = getScaledValue(admaData.insPosRelPOI8.longitude, 0.01);
        rosMsg.ins_pos_rel_y_8 = getScaledValue(admaData.insPosRelPOI8.latitude, 0.01);
        
        rosMsg.ins_vel_hor_x = getScaledValue(admaData.insVelHor.x, 0.005);
        rosMsg.ins_vel_hor_y = getScaledValue(admaData.insVelHor.y, 0.005);
        rosMsg.ins_vel_hor_z = getScaledValue(admaData.insVelHor.z, 0.005);
        rosMsg.ins_vel_frame_x = getScaledValue(admaData.insVelFrame.x, 0.005);
        rosMsg.ins_vel_frame_y = getScaledValue(admaData.insVelFrame.y, 0.005);
        rosMsg.ins_vel_frame_z = getScaledValue(admaData.insVelFrame.z, 0.005);

        rosMsg.ins_vel_hor_x_1 = getScaledValue(admaData.insVelHorPOI1.x, 0.005);
        rosMsg.ins_vel_hor_y_1 = getScaledValue(admaData.insVelHorPOI1.y, 0.005);
        rosMsg.ins_vel_hor_z_1 = getScaledValue(admaData.insVelHorPOI1.z, 0.005);
        rosMsg.ins_vel_hor_x_2 = getScaledValue(admaData.insVelHorPOI2.x, 0.005);
        rosMsg.ins_vel_hor_y_2 = getScaledValue(admaData.insVelHorPOI2.y, 0.005);
        rosMsg.ins_vel_hor_z_2 = getScaledValue(admaData.insVelHorPOI2.z, 0.005);
        rosMsg.ins_vel_hor_x_3 = getScaledValue(admaData.insVelHorPOI3.x, 0.005);
        rosMsg.ins_vel_hor_y_3 = getScaledValue(admaData.insVelHorPOI3.y, 0.005);
        rosMsg.ins_vel_hor_z_3 = getScaledValue(admaData.insVelHorPOI3.z, 0.005);
        rosMsg.ins_vel_hor_x_4 = getScaledValue(admaData.insVelHorPOI4.x, 0.005);
        rosMsg.ins_vel_hor_y_4 = getScaledValue(admaData.insVelHorPOI4.y, 0.005);
        rosMsg.ins_vel_hor_z_4 = getScaledValue(admaData.insVelHorPOI4.z, 0.005);
        rosMsg.ins_vel_hor_x_5 = getScaledValue(admaData.insVelHorPOI5.x, 0.005);
        rosMsg.ins_vel_hor_y_5 = getScaledValue(admaData.insVelHorPOI5.y, 0.005);
        rosMsg.ins_vel_hor_z_5 = getScaledValue(admaData.insVelHorPOI5.z, 0.005);
        rosMsg.ins_vel_hor_x_6 = getScaledValue(admaData.insVelHorPOI6.x, 0.005);
        rosMsg.ins_vel_hor_y_6 = getScaledValue(admaData.insVelHorPOI6.y, 0.005);
        rosMsg.ins_vel_hor_z_6 = getScaledValue(admaData.insVelHorPOI6.z, 0.005);
        rosMsg.ins_vel_hor_x_7 = getScaledValue(admaData.insVelHorPOI7.x, 0.005);
        rosMsg.ins_vel_hor_y_7 = getScaledValue(admaData.insVelHorPOI7.y, 0.005);
        rosMsg.ins_vel_hor_z_7 = getScaledValue(admaData.insVelHorPOI7.z, 0.005);
        rosMsg.ins_vel_hor_x_8 = getScaledValue(admaData.insVelHorPOI8.x, 0.005);
        rosMsg.ins_vel_hor_y_8 = getScaledValue(admaData.insVelHorPOI8.y, 0.005);
        rosMsg.ins_vel_hor_z_8 = getScaledValue(admaData.insVelHorPOI8.z, 0.005);

        rosMsg.ins_stddev_lat = getScaledValue(admaData.insstddevlat, 0.01);
        rosMsg.ins_stddev_long = getScaledValue(admaData.insstddevlong, 0.01);
        rosMsg.ins_stddev_height = getScaledValue(admaData.insstddevheight, 0.01);

        rosMsg.ins_stddev_vel_x = getScaledValue(admaData.insstddevvelx, 0.01);
        rosMsg.ins_stddev_vel_y = getScaledValue(admaData.insstddevvely, 0.01);
        rosMsg.ins_stddev_vel_z = getScaledValue(admaData.insstddevvelz, 0.01);
        rosMsg.ins_stddev_roll = getScaledValue(admaData.insstddevroll, 0.01);
        rosMsg.ins_stddev_pitch = getScaledValue(admaData.insstddevpitch, 0.01);
        rosMsg.ins_stddev_yaw = getScaledValue(admaData.insstddevyaw, 0.01);

        rosMsg.an1 = getScaledValue(admaData.an1, 0.0005);
        rosMsg.an2 = getScaledValue(admaData.an2, 0.0005);
        rosMsg.an3 = getScaledValue(admaData.an3, 0.0005);
        rosMsg.an4 = getScaledValue(admaData.an4, 0.0005);

        // only for >= v3.3.3
        rosMsg.gnss_dualant_heading = getScaledValue(admaData.gnssDualAntHeading, 0.01);
        rosMsg.gnss_dualant_pitch = getScaledValue(admaData.gnssDualAntPitch, 0.01);
        rosMsg.gnss_dualant_stddev_heading = getScaledValue(admaData.gnssdualantstdevheading, 0.01);
        rosMsg.gnss_dualant_stddev_pitch = getScaledValue(admaData.gnssdualantstddevpitch, 0.01);
        rosMsg.gnss_dualant_stddev_heading_hr = getScaledValue(admaData.gnssdualantstdevheadinghr, 0.01);
        rosMsg.gnss_dualant_stddev_pitch_hr = getScaledValue(admaData.gnssdualantstddevpitchhr, 0.01);
}