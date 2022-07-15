#include "adma_ros2_driver/adma_parse.hpp"
#include <cstring>
#include <iostream>
#include <sstream>
#include <math.h>
using namespace std;

#define PI 3.1415926535897932384626433832795028841971f

void getparseddata(const std::string& local_data, adma_msgs::msg::AdmaData& message, sensor_msgs::msg::NavSatFix& msg_fix, std_msgs::msg::Float64& msg_heading, std_msgs::msg::Float64& msg_velocity)
{
    getadmastaticheader(local_data,message);
    getadmadynamicheader(local_data,message);
    getstatusgps(local_data,message);
    getstatustrigger(local_data,message);
    getstatuscount(local_data,message); 
    getevkstatus(local_data,message);
    geterrorandwarning(local_data,message);
    getsensorbodyxyz(local_data,message);
    getratesbodyxyz(local_data,message);
    getgpsabs(local_data,message);
    getrateshorizontalxyz(local_data,message);
    getaccelerationbody(local_data, message);
    getaccelerationhor(local_data, message);
    getPOI(local_data, message.poi);
    getexternalvelocityanalog(local_data,message);
    getexternalvecovitydigpulses(local_data,message);
    getexternalvelocitycorrected(local_data,message);
    getbarometerpressure(local_data,message);
    getbarometerheight(local_data,message);
    getmiscellaneuos(local_data,message);
    gettriggers(local_data,message);
    getsystemdata(local_data,message);
    getgpsposrel(local_data,message);
    getgpsepe(local_data,message);
    getgpsvelframe(local_data,message,msg_velocity);
    getgpsveleve(local_data,message);
    getgpstimeutc(local_data,message);
    getgpsauxdata1(local_data,message);
    getgpsauxdata2(local_data,message);
    getinsanglegpscog(local_data,message, msg_heading);
    getgpsheight(local_data,message);
    getgpsdualanttimeutc(local_data,message);
    getgpsdualantangle(local_data,message);
    getgpsdualantangleete(local_data,message);
    getinspositionheight(local_data,message);
    getinstimeutc(local_data,message);
    getinspositionabs(local_data,message,msg_fix);
    getinsposrel(local_data,message);
    getinsvelhorxyz(local_data,message);
    getinsvelframexyz(local_data,message);
    getinsepe(local_data,message);
    getinseveandete(local_data,message);
    getanalog(local_data,message);
    getkalmanfilter(local_data,message);
    getgnssreceiver(local_data,message);

}

/// \file
/// \brief  getadmastaticheader function - adma static header information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getadmastaticheader(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    char genesysid[] = {local_data[0],local_data[1],local_data[2],local_data[3]};
    char formatid[] = {local_data[8],local_data[9],local_data[10],local_data[11]};
    memcpy(&message.formatid , &formatid, sizeof(message.formatid));
    char serialno[] = {local_data[32],local_data[33],local_data[34],local_data[35]};
    memcpy(&message.serialno , &serialno, sizeof(message.serialno));
    message.genesysid =  genesysid;
    std::stringstream ss_hv;
    ss_hv <<  int(local_data[4]) << int(local_data[5]) << int(local_data[6]) << int(local_data[7]);
    message.headerversion = ss_hv.str();
    std::stringstream ss_fv;
    ss_fv <<  int(local_data[12]) << int(local_data[13]) << int(local_data[14]) << int(local_data[15]);
    message.formatversion = ss_fv.str();

}

/// \file
/// \brief  getadmastaticheader function - adma static header information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getadmadynamicheader(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    // ! adma dynamic header
    char configid[] = {local_data[68],local_data[69],local_data[70],local_data[71]};
    memcpy(&message.configid , &configid, sizeof(message.configid));
    char configformat[] = {local_data[72],local_data[73],local_data[74],local_data[75]};
    memcpy(&message.configformat , &configformat, sizeof(message.configformat));
    char configversion[] = {local_data[76],local_data[77],local_data[78],local_data[79]};
    memcpy(&message.configversion , &configversion, sizeof(message.configversion));
    char configsize[] = {local_data[80],local_data[81],local_data[82],local_data[83]};
    memcpy(&message.configsize , &configsize, sizeof(message.configsize));
    char byteoffset[] = {local_data[84],local_data[85],local_data[86],local_data[87]};
    memcpy(&message.byteoffset , &byteoffset, sizeof(message.byteoffset));
    char slicesize[] = {local_data[88],local_data[89],local_data[90],local_data[91]};
    memcpy(&message.slicesize , &slicesize, sizeof(message.slicesize));
    char slicedata[] = {local_data[92],local_data[93],local_data[94],local_data[95]};

}

/// \file
/// \brief  getstatusgps function - adma status information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getstatusgps(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    unsigned char statusgps;
    char status_gps[] = {local_data[96]};
    memcpy(&statusgps, &status_gps, sizeof(statusgps));
    bool status_external_vel = getbit(statusgps,7);
    bool status_skidding = getbit(statusgps,5);
    bool standstill_c = getbit(statusgps,4);
    bool rtk_precise = getbit(statusgps,3);
    bool rtk_coarse = getbit(statusgps,2);
    bool gps_mode = getbit(statusgps,1);
    bool gps_out = getbit(statusgps,0);

    /* status gps mode */
    if(gps_out)
    {
        message.statusgpsmode = 1;
    }
    else if (gps_mode) 
    {
        message.statusgpsmode = 2;
    }
    else if (rtk_coarse) 
    {
        message.statusgpsmode = 3;
    }
    else if (rtk_precise) 
    {
        message.statusgpsmode = 4;
    }
    /* status stand still */
    message.statusstandstill = standstill_c;
    /* status skidding */
    message.statusskidding = status_skidding;
    /* status external velocity slip */
    message.statusexternalvelout = status_external_vel;
}


/// \file
/// \brief  getstatustrigger function - adma gps trigger information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getstatustrigger(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    unsigned char statustriggergps;
    char statustrigger[] = {local_data[97]};
    memcpy(&statustriggergps, &statustrigger, sizeof(statustriggergps));
    bool status_synclock = getbit(statustriggergps,7);
    bool status_dead_reckoning = getbit(statustriggergps,6);
    bool status_ahrs_ins = getbit(statustriggergps,5);
    bool status_alignment = getbit(statustriggergps,4);
    bool status_signal_in1 = getbit(statustriggergps,3);
    bool status_signal_in2 = getbit(statustriggergps,2);
    bool status_signal_in3 = getbit(statustriggergps,1);
    bool status_trig_gps = getbit(statustriggergps,0);
    /* status statustriggps */
    message.statustriggps = status_trig_gps;
    /* status statussignalin3 */
    message.statussignalin3 = status_signal_in3;
    /* status statussignalin2 */
    message.statussignalin2 = status_signal_in2;
    /* status statussignalin1 */
    message.statussignalin1 = status_signal_in1;
    /* status statusalignment */
    message.statusalignment = status_alignment;
    /* status statusahrsins */
    message.statusahrsins = status_ahrs_ins;
    /* status statusdeadreckoning */
    message.statusdeadreckoning = status_dead_reckoning;
    /* status statussynclock */
    message.statussynclock = status_synclock;
}

/// \file
/// \brief  getstatustrigger function - adma gps trigger information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getevkstatus(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    unsigned char statusevk;
    char statusevkdata[] = {local_data[98]};
    memcpy(&statusevk, &statusevkdata, sizeof(statusevk));
    bool status_pos_b2 = getbit(statusevk,7);
    bool status_pos_b1 = getbit(statusevk,6);
    bool status_tilt_b2 = getbit(statusevk,5);
    bool status_tilt_b1 = getbit(statusevk,4);
    bool status_configuration_changed = getbit(statusevk,3);
    bool status_heading_executed = getbit(statusevk,2);
    bool status_evk_estimates = getbit(statusevk,1);
    bool status_evk_activ = getbit(statusevk,0);
    /* status statustriggps */
    message.statusevkactiv = status_evk_activ;
    /* status status_evk_estimates */
    message.statusevkestimates = status_evk_estimates;
    /* status status_heading_executed */
    message.statusheadingexecuted = status_heading_executed;
    /* status status_configuration_changed */
    message.statusconfigurationchanged = status_configuration_changed;
    /* status tilt */
    if(status_tilt_b1==0 && status_tilt_b2==0)
    {
        message.statustilt = 0;
    }
    else if(status_tilt_b1==0 && status_tilt_b2==1)
    {
        message.statustilt = 1;
    }
    else if(status_tilt_b1==1 && status_tilt_b2==0)
    {
        message.statustilt = 2;
    }
    /* status pos */
    if(status_pos_b1==0 && status_pos_b2==0)
    {
        message.statuspos = 0;
    }
    else if(status_pos_b1==0 && status_pos_b2==1)
    {
        message.statuspos = 1;
    }
    else if(status_pos_b1==1 && status_pos_b2==0)
    {
        message.statuspos = 2;
    }
}

/// \file
/// \brief  getstatuscount function - adma status count
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getstatuscount(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! status count
    char status_count[] = {local_data[99]};
    message.statuscount = int(local_data[99]);
    unsigned char statuskf;
    char status_kf[] = {local_data[100]};
    memcpy(&statuskf, &status_kf, sizeof(statuskf));
    bool status_speed_b2 = getbit(statuskf,5);
    bool status_speed_b1 = getbit(statuskf,4);
    bool status_kf_steady_state = getbit(statuskf,3);
    bool status_kf_long_stimulated = getbit(statuskf,2);
    bool status_kf_lat_stimulated = getbit(statuskf,1);
    bool status_kalmanfilter_settled = getbit(statuskf,0);
    message.statuskalmanfiltersetteled = status_kalmanfilter_settled;
    message.statuskflatstimulated = status_kf_lat_stimulated;
    message.statuskflongstimulated = status_kf_long_stimulated;
    message.statuskfsteadystate = status_kf_steady_state;
    if(status_speed_b1==0 && status_speed_b2==0)
    {
        message.statusspeed = 0;
    }
    else if(status_speed_b1==0 && status_speed_b2==1)
    {
        message.statusspeed = 1;
    }
    else if(status_speed_b1==1 && status_speed_b2==0)
    {
        message.statusspeed = 2;
    }
}

/// \file
/// \brief  geterrorandwarning function - adma error and warning
/// \param  local_data adma string
/// \param  message adma message to be loaded
void geterrorandwarning(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    std::bitset<8> bitdataerror1 = local_data[104];
    std::bitset<8> bitdataerror2 = local_data[105];
    std::bitset<8> bitdatawarn3 = local_data[106];
    std::bitset<8> errorhw = local_data[107];
    std::bitset<4> erhw1;
    std::bitset<4> ermisc1;
    std::bitset<4> ermisc2;
    std::bitset<4> ermisc3;
    std::bitset<4> warngps;
    std::bitset<4> warnmisc1;
    std::bitset<1> erhwsticky;

    for(size_t i=0;i<4;i++)
    {
        erhw1[i]    = bitdataerror1[i];
        ermisc1[i]  = bitdataerror1[i+4];
        ermisc2[i]  = bitdataerror2[i];
        ermisc3[i]  = bitdataerror2[i+4];
        warngps[i]  = bitdatawarn3[i];
        warnmisc1[i]  = bitdatawarn3[i+4];
    }
    erhwsticky[0] = errorhw[1];
    message.errorhardware = erhw1.to_string();
    message.error_misc1 = ermisc1.to_string();
    message.error_misc2 = ermisc2.to_string();
    message.error_misc3 = ermisc3.to_string();
    message.warngps = warngps.to_string();
    message.warnmisc1 = warnmisc1.to_string();
    message.errorhwsticky = erhwsticky.to_string();
}
/// \file
/// \brief  getsensorbodyxyz function - adma sensor body x y z acc and rate information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getsensorbodyxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! sensor body x
    char acc_body_hr_x[] = {local_data[112],local_data[113],local_data[114],local_data[115]};
    memcpy(&message.accbodyhrx , &acc_body_hr_x, sizeof(message.accbodyhrx));
    message.faccbodyhrx = message.accbodyhrx * 0.0001;
    
    char rate_body_hr_x[] = {local_data[116],local_data[117],local_data[118],local_data[119]};
    memcpy(&message.ratebodyhrx , &rate_body_hr_x, sizeof(message.ratebodyhrx));
    message.fratebodyhrx = message.ratebodyhrx * 0.0001;

    //! sensor body y
    char acc_body_hr_y[] = {local_data[120],local_data[121],local_data[122],local_data[123]};
    memcpy(&message.accbodyhry , &acc_body_hr_y, sizeof(message.accbodyhry));
    message.faccbodyhry = message.accbodyhry * 0.0001;   
    
    char rate_body_hr_y[] = {local_data[124],local_data[125],local_data[126],local_data[127]};
    memcpy(&message.ratebodyhry , &rate_body_hr_y, sizeof(message.ratebodyhry));
    message.fratebodyhry = message.ratebodyhry * 0.0001;

    //! sensor body z
    char acc_body_hr_z[] = {local_data[128],local_data[129],local_data[130],local_data[131]};
    memcpy(&message.accbodyhrz , &acc_body_hr_z, sizeof(message.accbodyhrz));
    message.faccbodyhrz = message.accbodyhrz * 0.0001;

    char rate_body_hr_z[] = {local_data[132],local_data[133],local_data[134],local_data[135]};
    memcpy(&message.ratebodyhrz , &rate_body_hr_z, sizeof(message.ratebodyhrz));
    message.fratebodyhrz = message.ratebodyhrz * 0.0001;

}

/// \file
/// \brief  getratesbodyxyz function - adma rate body information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getratesbodyxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! rates body
    char rate_body_x[] = {local_data[136],local_data[137]};
    memcpy(&message.ratebodyx , &rate_body_x, sizeof(message.ratebodyx));
    message.fratebodyx = message.ratebodyx * 0.01;

    char rate_body_y[] = {local_data[138],local_data[139]};
    memcpy(&message.ratebodyy , &rate_body_y, sizeof(message.ratebodyy));
    message.fratebodyy = message.ratebodyy * 0.01;

    char rate_body_z[] = {local_data[140],local_data[141]};
    memcpy(&message.ratebodyz , &rate_body_z, sizeof(message.ratebodyz));
    message.fratebodyz = message.ratebodyz * 0.01;
}

/// \file
/// \brief  getrateshorizontalxyz function - adma rates horizontal
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getrateshorizontalxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! rates horizontal
    char rate_hor_x[] = {local_data[144],local_data[145]};
    memcpy(&message.ratehorx , &rate_hor_x, sizeof(message.ratehorx));
    message.fratehorx = message.ratehorx * 0.01;

    char rate_hor_y[] = {local_data[146],local_data[147]};
    memcpy(&message.ratehory , &rate_hor_y, sizeof(message.ratehory));
    message.fratehory = message.ratehory * 0.01;

    char rate_hor_z[] = {local_data[148],local_data[149]};
    memcpy(&message.ratehorz , &rate_hor_z, sizeof(message.ratehorz));
    message.fratehorz = message.ratehorz * 0.01;
}

/// \file
/// \brief  getaccelerationbody function - adma acc body
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationbody(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{

    //! acceleration body
    char acceleration_body_x[] = {local_data[152],local_data[153]};
    memcpy(&message.accbodyx , &acceleration_body_x, sizeof(message.accbodyx));
    message.faccbodyx = message.accbodyx * 0.0004;    

    char acceleration_body_y[] = {local_data[154],local_data[155]};
    memcpy(&message.accbodyy , &acceleration_body_y, sizeof(message.accbodyy));
    message.faccbodyy = message.accbodyy * 0.0004;

    char acceleration_body_z[] = {local_data[156],local_data[157]};
    memcpy(&message.accbodyz , &acceleration_body_z, sizeof(message.accbodyz));
    message.faccbodyz = message.accbodyz * 0.0004;
}

/// \file
/// \brief  getaccelerationhor function - adma acc horizontal
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationhor(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! acceleration horizontal
    char acceleration_hor_x[] = {local_data[160],local_data[161]};
    memcpy(&message.acchorx , &acceleration_hor_x, sizeof(message.acchorx));
    message.facchorx = message.acchorx * 0.0004;

    char acceleration_hor_y[] = {local_data[162],local_data[163]};
    memcpy(&message.acchory , &acceleration_hor_y, sizeof(message.acchory));
    message.facchory = message.acchory * 0.0004;

    char acceleration_hor_z[] = {local_data[164],local_data[165]};
    memcpy(&message.acchorz , &acceleration_hor_z, sizeof(message.acchorz));
    message.facchorz = message.acchorz * 0.0004;

}

/// \file
/// \brief  extract all information of the POI's
/// \param  local_data adma string
/// \param  poiList list off all POI's
void getPOI(const std::string& local_data, std::vector<adma_msgs::msg::POI>& poiList){

    for (size_t i = 0; i < nrOfPOI; i++)
    {
        adma_msgs::msg::POI poi;

        // 1. read acceleration body
        uint8_t accBodyIndex = startIndexAccBodyPOI + (i * 8);
        char acceleration_body_x[] = {local_data[accBodyIndex],local_data[accBodyIndex + 1]};
        memcpy(&poi.acc_body_x, &acceleration_body_x, sizeof(poi.acc_body_x));
        poi.f_acc_body_x = poi.acc_body_x * 0.0004;
        char acceleration_body_y[] = {local_data[accBodyIndex + 2],local_data[accBodyIndex + 3]};
        memcpy(&poi.acc_body_y, &acceleration_body_y, sizeof(poi.acc_body_y));
        poi.f_acc_body_y = poi.acc_body_y * 0.0004;
        char acceleration_body_z[] = {local_data[accBodyIndex + 4],local_data[accBodyIndex  +5]};
        memcpy(&poi.acc_body_z, &acceleration_body_z, sizeof(poi.acc_body_z));
        poi.f_acc_body_z = poi.acc_body_z * 0.0004;

        // 2. read acceleration horizontal
        uint8_t accHorIndex = startIndexAccHorPOI + (i * 8);
        char acceleration_hor_x[] = {local_data[accHorIndex],local_data[accHorIndex + 1]};
        memcpy(&poi.acc_hor_x, &acceleration_hor_x, sizeof(poi.acc_hor_x));
        poi.f_acc_hor_x = poi.acc_hor_x * 0.0004;
        char acceleration_hor_y[] = {local_data[accBodyIndex + 2],local_data[accHorIndex + 3]};
        memcpy(&poi.acc_hor_y, &acceleration_hor_y, sizeof(poi.acc_hor_y));
        poi.f_acc_hor_y = poi.acc_hor_y * 0.0004;
        char acceleration_hor_z[] = {local_data[accHorIndex + 4],local_data[accHorIndex + 5]};
        memcpy(&poi.acc_hor_z, &acceleration_hor_z, sizeof(poi.acc_hor_z));
        poi.f_acc_hor_z = poi.acc_hor_z * 0.0004;

        // 3. read miscellaneous data of POI
        uint8_t miscIndex = startIndexMiscPOI + (i * 8);
        char inv_path_radius[] = {local_data[miscIndex],local_data[miscIndex + 1]};
        memcpy(&poi.inv_path_radius, &inv_path_radius, sizeof(poi.inv_path_radius));
        poi.f_inv_path_radius = poi.inv_path_radius * 0.0001;
        char side_slip_angle[] = {local_data[miscIndex + 2],local_data[miscIndex + 3]};
        memcpy(&poi.side_slip_angle, &side_slip_angle, sizeof(poi.side_slip_angle));
        poi.f_side_slip_angle = poi.side_slip_angle * 0.01;
        char dist_trav[] = {local_data[miscIndex + 4],local_data[miscIndex + 5],local_data[miscIndex + 6],local_data[miscIndex + 7]};
        memcpy(&poi.dist_trav, &dist_trav, sizeof(poi.dist_trav));
        poi.f_dist_trav = poi.dist_trav * 0.01;

        // 4. read ins position height
        uint8_t heightIndex = startIndexInsHeightPOI + (i * 4);
        char ins_height[] = {local_data[heightIndex],local_data[heightIndex + 1],local_data[heightIndex + 2],local_data[heightIndex + 3]};
        memcpy(&poi.ins_height, &ins_height, sizeof(poi.ins_height));
        poi.f_ins_height = poi.ins_height * 0.01;

        // 5. read ins position (absolute & relative)
        uint8_t insPosIndex = startIndexInsPositionPOI + (i * 16);
        char ins_lat_abs[] = {local_data[insPosIndex],local_data[insPosIndex + 1],local_data[insPosIndex + 2],local_data[insPosIndex + 3]};
        memcpy(&poi.ins_lat_abs, &ins_lat_abs, sizeof(poi.ins_lat_abs));
        poi.f_ins_lat_abs = poi.ins_lat_abs * 0.0000001;
        char ins_lon_abs[] = {local_data[insPosIndex + 4],local_data[insPosIndex + 5],local_data[insPosIndex + 6],local_data[insPosIndex + 7]};
        memcpy(&poi.ins_lon_abs, &ins_lon_abs, sizeof(poi.ins_lon_abs));
        poi.f_ins_lon_abs = poi.ins_lon_abs * 0.0000001;
        char ins_lat_rel[] = {local_data[insPosIndex + 8],local_data[insPosIndex + 9],local_data[insPosIndex + 10],local_data[insPosIndex + 11]};
        memcpy(&poi.ins_lat_rel, &ins_lat_rel, sizeof(poi.ins_lat_rel));
        poi.f_ins_lat_rel = poi.ins_lat_rel * 0.01;
        char ins_lon_rel[] = {local_data[insPosIndex + 12],local_data[insPosIndex + 13],local_data[insPosIndex + 14],local_data[insPosIndex + 15]};
        memcpy(&poi.ins_lon_rel, &ins_lon_rel, sizeof(poi.ins_lon_rel));
        poi.f_ins_lon_rel = poi.ins_lon_rel * 0.01;

        // 6. read ins velocity
        uint8_t velIndex = startIndexVelPOI + (i * 8);
        char ins_vel_hor_x[] = {local_data[velIndex],local_data[velIndex + 1]};
        memcpy(&poi.ins_vel_hor_x, &ins_vel_hor_x, sizeof(poi.ins_vel_hor_x));
        poi.f_ins_vel_hor_x = poi.ins_vel_hor_x * 0.005;
        char ins_vel_hor_y[] = {local_data[velIndex + 2],local_data[velIndex + 3]};
        memcpy(&poi.ins_vel_hor_y, &ins_vel_hor_y, sizeof(poi.ins_vel_hor_y));
        poi.f_ins_vel_hor_y = poi.ins_vel_hor_y * 0.005;
        char ins_vel_hor_z[] = {local_data[velIndex + 4],local_data[velIndex + 5]};
        memcpy(&poi.ins_vel_hor_z, &ins_vel_hor_z, sizeof(poi.ins_vel_hor_z));
        poi.f_ins_vel_hor_z = poi.ins_vel_hor_z * 0.005;

        poiList.push_back(poi);
    }
}

/// \file
/// \brief  getexternalvelocityanalog function - adma ext vel analog
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getexternalvelocityanalog(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! external velocity analalog
    char ext_vel_an_x[] = {local_data[296],local_data[297]};
    memcpy(&message.extvelanx , &ext_vel_an_x, sizeof(message.extvelanx));
    message.fextvelanx = message.extvelanx * 0.005;

    char ext_vel_an_y[] = {local_data[298],local_data[299]};
    memcpy(&message.extvelany , &ext_vel_an_y, sizeof(message.extvelany));
    message.fextvelany = message.extvelany * 0.005;
}

/// \file
/// \brief  getexternalvecovitydigpulses function - adma ext vel dig pulses
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getexternalvecovitydigpulses(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! external velocity digital pulses
    char ext_vel_dig_x[] = {local_data[304],local_data[305]};
    memcpy(&message.extveldigx , &ext_vel_dig_x, sizeof(message.extveldigx));
    message.fextveldigx = message.extveldigx * 0.005;

    char ext_vel_dig_y[] = {local_data[306],local_data[307]};
    memcpy(&message.extveldigy , &ext_vel_dig_y, sizeof(message.extveldigy));
    message.fextveldigy = message.extveldigy * 0.005;

    char ext_vel_dig_pulses_x[] = {local_data[308],local_data[309]};
    memcpy(&message.extveldigpulsesx , &ext_vel_dig_pulses_x, sizeof(message.extveldigpulsesx));

    char ext_vel_dig_pulses_y[] = {local_data[310],local_data[311]};
    memcpy(&message.extveldigpulsesy , &ext_vel_dig_pulses_y, sizeof(message.extveldigpulsesy));
}

/// \file
/// \brief  getexternalvelocitycorrected function - adma ext vel corrected
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getexternalvelocitycorrected(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! external velocity corrected
    char ext_vel_x_corrected[] = {local_data[312],local_data[313]};
    memcpy(&message.extvelxcorrected , &ext_vel_x_corrected, sizeof(message.extvelxcorrected));
    message.fextvelxcorrected = message.extvelxcorrected * 0.005;

    char ext_vel_y_corrected[] = {local_data[314],local_data[315]};
    memcpy(&message.extvelycorrected , &ext_vel_y_corrected, sizeof(message.extvelycorrected));
    message.fextvelycorrected = message.extvelycorrected * 0.005;
}

/// \file
/// \brief  getbarometerpressure function - adma barometer pressure
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getbarometerpressure(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! barometer pressure
    char ext_baro_pressue[] = {local_data[320],local_data[321],local_data[322],local_data[323]};
    memcpy(&message.extbaropressure , &ext_baro_pressue, sizeof(message.extbaropressure));
    message.fextbaropressure = message.extbaropressure * 0.01;
}

/// \file
/// \brief  getbarometerheight function - adma barometer height
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getbarometerheight(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! barometer height
    char ext_baro_height[] = {local_data[328],local_data[329],local_data[330],local_data[331]};
    memcpy(&message.extbaroheight , &ext_baro_height, sizeof(message.extbaroheight));
    message.fextbaroheight = message.extbaroheight * 0.01;

    char ext_baro_height_corrected[] = {local_data[332],local_data[333],
                                        local_data[334],local_data[335]};
    memcpy(&message.extbaroheightcorrected , &ext_baro_height_corrected, sizeof(message.extbaroheightcorrected));
    message.fextbaroheightcorrected = message.extbaroheightcorrected * 0.01;
}

/// \file
/// \brief  getmiscellaneuos function - adma misc.
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getmiscellaneuos(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! miscellaneous
    char inv_path_radius[] = {local_data[344],local_data[345]};
    memcpy(&message.invpathradius , &inv_path_radius, sizeof(message.invpathradius));
    message.finvpathradius = message.invpathradius * 0.0001;

    char side_slip_angle[] = {local_data[346],local_data[347]};
    memcpy(&message.sideslipangle , &side_slip_angle, sizeof(message.sideslipangle));
    message.fsideslipangle = message.sideslipangle * 0.01;

    char dist_trav[] = {local_data[348],local_data[349],local_data[350],local_data[351]};
    memcpy(&message.disttrav , &dist_trav, sizeof(message.disttrav));
    message.fdisttrav = message.disttrav * 0.01;
}

/// \file
/// \brief  gettriggers function - adma triggers 1,2,3 and 4
/// \param  local_data adma string
/// \param  message adma message to be loaded
void gettriggers(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! triggers 1 and 2
    char trigger_raising_1[] = {local_data[416],local_data[417]};
    memcpy(&message.trigrising1 , &trigger_raising_1, sizeof(message.trigrising1));
    char trigger_falling_1[] = {local_data[418],local_data[419]};
    memcpy(&message.trigfalling1 , &trigger_falling_1, sizeof(message.trigfalling1));
    char trigger_raising_2[] = {local_data[420],local_data[421]};
    memcpy(&message.trigrising2 , &trigger_raising_2, sizeof(message.trigrising2));
    char trigger_falling_2[] = {local_data[422],local_data[423]};
    memcpy(&message.trigfalling2 , &trigger_falling_2, sizeof(message.trigfalling2));
    //! triggers 3 and 4
    char trigger_raising_3[] = {local_data[424],local_data[425]};
    memcpy(&message.trigrising3 , &trigger_raising_3, sizeof(message.trigrising3));
    char trigger_falling_3[] = {local_data[426],local_data[427]};
    memcpy(&message.trigfalling3 , &trigger_falling_3, sizeof(message.trigfalling3));
    char trigger_raising_4[] = {local_data[428],local_data[429]};
    memcpy(&message.trigrising4 , &trigger_raising_4, sizeof(message.trigrising4));
    char trigger_falling_4[] = {local_data[430],local_data[431]};
    memcpy(&message.trigfalling4 , &trigger_falling_4, sizeof(message.trigfalling4));
}

/// \file
/// \brief  getsystemdata function - adma system data
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getsystemdata(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! system data
    char system_ta[] = {local_data[432],local_data[433]};
    memcpy(&message.systemta , &system_ta, sizeof(message.systemta));
    char system_temp[] = {local_data[434],local_data[435]};
    memcpy(&message.systemtemp , &system_temp, sizeof(message.systemtemp));
    message.fsystemtemp = message.systemtemp * 0.1;
    char system_timesinceinit[] = {local_data[436],local_data[437]};
    memcpy(&message.systemtimesinceinit , &system_timesinceinit, sizeof(message.systemtimesinceinit));
    char system_dsp_load[] = {local_data[438],local_data[439]};
    memcpy(&message.systemdspload , &system_dsp_load, sizeof(message.systemdspload));
    message.fsystemdspload = message.systemdspload * 0.1;

}

/// \file
/// \brief  getgpsabs function - adma absolute gps information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsabs(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps position absolute
    char gps_lat_abs[] = {local_data[440],local_data[441],local_data[442],local_data[443]};
    memcpy(&message.gpslatabs , &gps_lat_abs, sizeof(message.gpslatabs));
    message.fgpslatabs = message.gpslatabs * 0.0000001; 
    char gps_lon_abs[] = {local_data[444],local_data[445],local_data[446],local_data[447]};
    memcpy(&message.gpslonabs , &gps_lon_abs, sizeof(message.gpslonabs));
    message.fgpslonabs = message.gpslonabs * 0.0000001;
    
}

/// \file
/// \brief  getgpsposrel function - adma relative gps information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsposrel(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps position relative
    char gps_lat_rel[] = {local_data[448],local_data[449],local_data[450],local_data[451]};
    memcpy(&message.gpslatrel , &gps_lat_rel, sizeof(message.gpslatrel));
    message.fgpslatrel = message.gpslatrel * 0.01;

    char gps_lon_rel[] = {local_data[452],local_data[453],local_data[454],local_data[455]};
    memcpy(&message.gpslonrel , &gps_lon_rel, sizeof(message.gpslonrel));
    message.fgpslonrel = message.gpslonrel * 0.01;
}

/// \file
/// \brief  getgpsepe function - adma position error
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsepe(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
   //! gps position error
    char gps_stddev_lat[] = {local_data[456],local_data[457]};
    memcpy(&message.gpsstddevlat , &gps_stddev_lat, sizeof(message.gpsstddevlat));
    message.fgpsstddevlat = message.gpsstddevlat * 0.001;
    char gps_stddev_lon[] = {local_data[458],local_data[459]};
    memcpy(&message.gpsstddevlon , &gps_stddev_lon, sizeof(message.gpsstddevlon));
    message.fgpsstddevlon = message.gpsstddevlon * 0.001;
    char gps_stddev_height[] = {local_data[460],local_data[461]};
    memcpy(&message.gpsstddevheight , &gps_stddev_height, sizeof(message.gpsstddevheight));
    message.fgpsstddevheight = message.gpsstddevheight * 0.001;
}

/// \file
/// \brief  getgpsabs function - adma velocity frmae x y z
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsvelframe(const std::string& local_data, adma_msgs::msg::AdmaData& message, std_msgs::msg::Float64& msg_velocity)
{
    //! gps velocity frame
    char gps_vel_frame_x[] = {local_data[464],local_data[465]};
    memcpy(&message.gpsvelframex , &gps_vel_frame_x, sizeof(message.gpsvelframex));
    message.fgpsvelframex = message.gpsvelframex * 0.005;
    char gps_vel_frame_y[] = {local_data[466],local_data[467]};
    memcpy(&message.gpsvelframey , &gps_vel_frame_y, sizeof(message.gpsvelframey));
    message.fgpsvelframey = message.gpsvelframey * 0.005;
    msg_velocity.data = std::sqrt(message.fgpsvelframex * message.fgpsvelframex + message.fgpsvelframey * message.fgpsvelframey) * 3.6;
    char gps_vel_frame_z[] = {local_data[468],local_data[469]};
    memcpy(&message.gpsvelframez , &gps_vel_frame_z, sizeof(message.gpsvelframez));
    message.fgpsvelframez = message.gpsvelframez * 0.005;
    char gps_vel_latency[] = {local_data[470],local_data[471]};
    memcpy(&message.gpsvellatency , &gps_vel_latency, sizeof(message.gpsvellatency));
    message.fgpsvellatency = message.gpsvellatency * 0.001;    
}

/// \file
/// \brief  getgpsabs function - adma expected velocity error
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsveleve(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps velocity error
    char gps_stddev_vel_x[] = {local_data[472],local_data[473]};
    memcpy(&message.gpsstddevvelx , &gps_stddev_vel_x, sizeof(message.gpsstddevvelx));
    message.fgpsstddevvelx = message.gpsstddevvelx * 0.001;  
    char gps_stddev_vel_y[] = {local_data[474],local_data[475]};
    memcpy(&message.gpsstddevvely , &gps_stddev_vel_y, sizeof(message.gpsstddevvely));
    message.fgpsstddevvely = message.gpsstddevvely * 0.001;  
    char gps_stddev_vel_z[] = {local_data[476],local_data[477]};
    memcpy(&message.gpsstddevvelz , &gps_stddev_vel_z, sizeof(message.gpsstddevvelz));
    message.fgpsstddevvelz = message.gpsstddevvelz * 0.001;  
}

/// \file
/// \brief  getgpsabs function - adma expected velocity error
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpstimeutc(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps time utc
    char gps_time_msec[] = {local_data[480],local_data[481],local_data[482],local_data[483]};
    memcpy(&message.gpstimemsec , &gps_time_msec, sizeof(message.gpstimemsec));
    char gps_time_weel[] = {local_data[484], local_data[485]};
    memcpy(&message.gpstimeweek , &gps_time_weel, sizeof(message.gpstimeweek));
    char trigger_gps[] = {local_data[486],local_data[487]};
    memcpy(&message.gpstrigger , &trigger_gps, sizeof(message.gpstrigger));
}
/// \file
/// \brief  getgpsabs function - adma expected velocity error
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsauxdata1(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps aux data 1
    char gps_diff_age[] = {local_data[488],local_data[489]};
    memcpy(&message.gpsdiffage , &gps_diff_age, sizeof(message.gpsdiffage));
    message.fgpsdiffage = message.gpsdiffage * 0.1;  
    char gps_stats_used[] = {local_data[490]};
    memcpy(&message.gpsstatsused , &gps_stats_used, sizeof(message.gpsstatsused));
    char gps_stats_visible[] = {local_data[491]};
    memcpy(&message.gpsstatsvisible , &gps_stats_visible, sizeof(message.gpsstatsvisible));

}
/// \file
/// \brief  getgpsabs function - adma expected velocity error
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsauxdata2(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps aux data 2
    char gps_log_delay[] = {local_data[496]};
    memcpy(&message.gpslogdelay , &gps_log_delay, sizeof(message.gpslogdelay));
    char gps_receiver_load[] = {local_data[497]};
    memcpy(&message.gpsreceiverload , &gps_receiver_load, sizeof(message.gpsreceiverload));
    message.fgpsreceiverload = message.gpsreceiverload * 0.5;  
    char gps_basenr[] = {local_data[498],local_data[499],local_data[500],local_data[501]};
    memcpy(&message.gpsbasenr , &gps_basenr, sizeof(message.gpsbasenr));  
}
/// \file
/// \brief  getgpsabs function - adma expected velocity error
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsanglegpscog(const std::string& local_data, adma_msgs::msg::AdmaData& message, std_msgs::msg::Float64& msg_heading)
{
    //! ins angle and gps cog
    char ins_roll[] = {local_data[504],local_data[505]};
    memcpy(&message.insroll , &ins_roll, sizeof(message.insroll));
    message.finsroll = message.insroll * 0.01;  
    char ins_pitch[] = {local_data[506],local_data[507]};
    memcpy(&message.inspitch , &ins_pitch, sizeof(message.inspitch));
    message.finspitch = message.inspitch * 0.01;  
    char ins_yaw[] = {local_data[508],local_data[509]};
    memcpy(&message.insyaw , &ins_yaw, sizeof(message.insyaw));
    message.finsyaw = message.insyaw * 0.01;  
    msg_heading.data = message.finsyaw;
    char gps_cog[] = {local_data[510],local_data[511]};
    memcpy(&message.gpscog , &gps_cog, sizeof(message.gpscog));
    message.fgpscog = message.gpscog * 0.01;  
}

/// \file
/// \brief  getgpsheight function - adma gps height
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsheight(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps height (msl)
    char gps_height[] = {local_data[512],local_data[513],local_data[514],local_data[515]};
    memcpy(&message.gpsheight , &gps_height, sizeof(message.gpsheight));
    message.gpsheight = message.gpsheight * 0.01;  
    char undulation[] = {local_data[516],local_data[517]};
    memcpy(&message.undulation , &undulation, sizeof(message.undulation));
    message.fundulation = message.undulation * 0.01;  
}

/// \file
/// \brief  getgpsdualanttimeutc function - adma gps dual ant time utc
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsdualanttimeutc(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps dualant time utc
    char gps_dualant_time_msec[] = {local_data[520],local_data[521],local_data[522],local_data[523]};
    memcpy(&message.gpsdualanttimemsec , &gps_dualant_time_msec, sizeof(message.gpsdualanttimemsec));
    char gps_dualant_time_week[] = {local_data[524],local_data[525]};
    memcpy(&message.gpsdualanttimeweek , &gps_dualant_time_week, sizeof(message.gpsdualanttimeweek));
}

/// \file
/// \brief  getgpsdualantangle function - adma gps dual ant angle
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsdualantangle(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps dualant angle
    char gps_dualant_heading[] = {local_data[528],local_data[529]};
    memcpy(&message.gpsdualantheading , &gps_dualant_heading, sizeof(message.gpsdualantheading));
    message.fgpsdualantheading = message.gpsdualantheading * 0.01;  
    char gps_dualant_pitch[] = {local_data[530],local_data[531]};
    memcpy(&message.gpsdualantpitch , &gps_dualant_pitch, sizeof(message.gpsdualantpitch));
    message.fgpsdualantpitch = message.gpsdualantpitch * 0.01;  
}

/// \file
/// \brief  getgpsdualantangleete function - adma gps dual ant angle ete
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsdualantangleete(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps dualant angle ete
    char gps_dualant_stddev_heading[] =  {local_data[536]};
    memcpy(&message.gpsdualantstddevheading , &gps_dualant_stddev_heading, sizeof(message.gpsdualantstddevheading));
    message.fgpsdualantstddevheading = message.gpsdualantstddevheading * 0.01;  
    char gps_dualant_stddev_pitch[] = {local_data[537]};
    memcpy(&message.gpsdualantstddevpitch , &gps_dualant_stddev_pitch, sizeof(message.gpsdualantstddevpitch));
    message.fgpsdualantstddevpitch = message.gpsdualantstddevpitch * 0.01;  
}


/// \file
/// \brief  getgpsdualantangleete function - adma gps dual ant angle ete
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinspositionheight(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    char ins_height[] = {local_data[544],local_data[545],local_data[546],local_data[547]};
    memcpy(&message.insheight , &ins_height, sizeof(message.insheight));
    message.finsheight = message.insheight * 0.01;
}

/// \file
/// \brief  getinstimeutc function -adma ins time utc
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinstimeutc(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins time utc
    char ins_time_msec[] = {local_data[584],local_data[585],local_data[586],local_data[587]};
    memcpy(&message.instimemsec , &ins_time_msec, sizeof(message.instimemsec));
    char ins_time_week[] = {local_data[588],local_data[589]};
    memcpy(&message.instimeweek , &ins_time_week, sizeof(message.instimeweek));
    char leap_seconds[] = {local_data[590],local_data[591]};
    memcpy(&message.leapseconds , &leap_seconds, sizeof(message.leapseconds));
}

/// \file
/// \brief  getinspositionabs function -  adma ins pos abs
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinspositionabs(const std::string& local_data, adma_msgs::msg::AdmaData& message, sensor_msgs::msg::NavSatFix& msg_fix)
{
    //! ins position abs
    char ins_lat_abs[] = {local_data[592],local_data[593],local_data[594],local_data[595]};
    memcpy(&message.inslatabs , &ins_lat_abs, sizeof(message.inslatabs));
    message.finslatabs = message.inslatabs * 0.0000001;
    msg_fix.latitude = message.fgpslatabs;
    char ins_lon_abs[] = {local_data[596],local_data[597],local_data[598],local_data[599]};
    memcpy(&message.inslonabs , &ins_lon_abs, sizeof(message.inslonabs));
    message.finslonabs = message.inslonabs * 0.0000001;
    msg_fix.longitude = message.finslonabs; 
}

/// \file
/// \brief  getinsposrel function -  adma ins pos rel 
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsposrel(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins position rel 
    char ins_lat_rel[] = {local_data[600],local_data[601],local_data[602],local_data[603]};
    memcpy(&message.inslatrel , &ins_lat_rel, sizeof(message.inslatrel));
    message.finslatrel = message.inslatrel * 0.01;
    char ins_lon_rel[] = {local_data[604],local_data[605],local_data[606],local_data[607]};
    memcpy(&message.inslonrel , &ins_lon_rel, sizeof(message.inslonrel));
    message.finslonrel = message.inslonrel * 0.01;
}

/// \file
/// \brief  getinsvelhorxyz function - adma ins vel hor x y z
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsvelhorxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins velocity horizontal
    char ins_vel_hor_x[] = {local_data[736],local_data[737]};
    memcpy(&message.insvelhorx , &ins_vel_hor_x, sizeof(message.insvelhorx));
    message.finsvelhorx = message.insvelhorx * 0.005;
    char ins_vel_hor_y[] = {local_data[738],local_data[739]};
    memcpy(&message.insvelhory , &ins_vel_hor_y, sizeof(message.insvelhory));
    message.finsvelhory = message.insvelhory * 0.005;
    char ins_vel_hor_z[] = {local_data[740],local_data[741]};
    memcpy(&message.insvelhorz , &ins_vel_hor_z, sizeof(message.insvelhorz));
    message.finsvelhorz = message.insvelhorz * 0.005;  
} 

/// \file
/// \brief  getinsvelframexyz function - adma ins vel frame x y z
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsvelframexyz(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins velocity frame
    char ins_vel_frame_x[] = {local_data[744],local_data[745]};
    memcpy(&message.insvelframex , &ins_vel_frame_x, sizeof(message.insvelframex));
    message.finsvelframex = message.insvelframex * 0.005;
    char ins_vel_frame_y[] = {local_data[746],local_data[747]};
    memcpy(&message.insvelframey , &ins_vel_frame_y, sizeof(message.insvelframey));
    message.finsvelframey = message.insvelframey * 0.005;
    char ins_vel_frame_z[] = {local_data[748],local_data[749]};
    memcpy(&message.insvelframez , &ins_vel_frame_z, sizeof(message.insvelframez));
    message.finsvelframez = message.insvelframez * 0.005;
}

/// \file
/// \brief  getinsepe function - adma ins sepe
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsepe(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins epe
    char ins_stddev_lat[] = {local_data[816],local_data[817]};
    memcpy(&message.insstddevlat , &ins_stddev_lat, sizeof(message.insstddevlat));
    message.finsstddevlat = message.insstddevlat * 0.01;
    char ins_stddev_lon[] = {local_data[818],local_data[819]};
    memcpy(&message.insstddevlong , &ins_stddev_lon, sizeof(message.insstddevlong));
    message.finsstddevlong = message.insstddevlong * 0.01;
    char ins_stddev_height[] = {local_data[820],local_data[821]};
    memcpy(&message.insstddevheight , &ins_stddev_height, sizeof(message.insstddevheight));
    message.finsstddevheight = message.insstddevheight * 0.01;
}

/// \file
/// \brief  getinseveandete function - adma ins eve and ete
/// \param  message adma message to be loaded
void getinseveandete(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins eve and ins ete
    char ins_stddev_vel_x[] = {local_data[824]};
    memcpy(&message.insstddevvelx , &ins_stddev_vel_x, sizeof(message.insstddevvelx));
    message.finsstddevvelx = message.insstddevvelx * 0.01;  
    char ins_stddev_vel_y[] = {local_data[825]};
    memcpy(&message.insstddevvely , &ins_stddev_vel_y, sizeof(message.insstddevvely));
    message.finsstddevvely = message.insstddevvely * 0.01;
    char ins_stddev_vel_z[] = {local_data[826]};
    memcpy(&message.insstddevvelz , &ins_stddev_vel_z, sizeof(message.insstddevvelz));
    message.finsstddevvelz = message.insstddevvelz * 0.01;
    char ins_stddev_roll[] = {local_data[827]};
    memcpy(&message.insstddevroll , &ins_stddev_roll, sizeof(message.insstddevroll));
    message.finsstddevroll = message.insstddevroll * 0.01;
    char ins_stddev_pitch[] = {local_data[828]};
    memcpy(&message.insstddevpitch , &ins_stddev_pitch, sizeof(message.insstddevpitch));
    message.finsstddevpitch = message.insstddevpitch * 0.01;
    char ins_stddev_yaw[] = {local_data[829]};
    memcpy(&message.insstddevyaw , &ins_stddev_yaw, sizeof(message.insstddevyaw));
    message.finsstddevyaw = message.insstddevyaw * 0.01;
}

/// \file
/// \brief  getanalog function - adma analog
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getanalog(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! analog in 1
    char an1[] = {local_data[832],local_data[833]};
    memcpy(&message.an1 , &an1, sizeof(message.an1));
    message.fan1 = message.an1 * 0.0005;
    char an2[] = {local_data[834],local_data[835]};
    memcpy(&message.an2 , &an2, sizeof(message.an2));
    message.fan2 = message.an2 * 0.0005;
    char an3[] = {local_data[836],local_data[837]};
    memcpy(&message.an3 , &an3, sizeof(message.an3));
    message.fan3 = message.an3 * 0.0005;
    char an4[] = {local_data[838],local_data[839]};
    memcpy(&message.an4 , &an4, sizeof(message.an4));
    message.fan4 = message.an4 * 0.0005;
}

/// \file
/// \brief  getkalmanfilter function - adma kalman filter
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getkalmanfilter(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! kalman filer status
    char kf_lat_stimulated[] = {local_data[840]};
    memcpy(&message.kflatstimulated , &kf_lat_stimulated, sizeof(message.kflatstimulated));
    char kf_lon_stimulated[] = {local_data[841]};
    memcpy(&message.kflongstimulated , &kf_lon_stimulated, sizeof(message.kflongstimulated));
    char kf_steady_state[] = {local_data[842]};
    memcpy(&message.kfsteadystate , &kf_steady_state, sizeof(message.kfsteadystate));
}

/// \file
/// \brief  getgnssreceiver function - adma gps error status
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgnssreceiver(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps receiver error
    char gps_receiver_error[] = {local_data[848],local_data[849],local_data[850],local_data[851]};
    memcpy(&message.gpsreceivererror , &gps_receiver_error, sizeof(message.gpsreceivererror));
    char gps_receiver_status[] = {local_data[852],local_data[853],local_data[854],local_data[855]};
    memcpy(&message.gpsreceiverstatus , &gps_receiver_status, sizeof(message.gpsreceiverstatus));
}

/// \file
/// \brief  bit shift function
/// \param  byte byte information
/// \param  position message
/// \return an integer 0 upon exit success
bool getbit(unsigned char byte, int position) // position in range 0-7
{
    return (byte >> position) & 0x1;
}

