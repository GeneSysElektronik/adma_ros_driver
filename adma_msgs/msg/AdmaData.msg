# adma static header
string genesysid
string headerversion
uint32 formatid
string formatversion
uint32 serialno
uint32 alias

# adma dynamic header
uint32 configid
uint32 configformat
uint32 configversion
uint32 configsize
uint32 byteoffset
uint32 slicesize
#byte slicedata

# status
int32 statusgpsmode
int32 statusstandstill
int32 statusskidding
int32 statusexternalvelout
int32 statustriggps
int32 statussignalin3
int32 statussignalin2
int32 statussignalin1
int32 statusalignment
int32 statusahrsins
int32 statusdeadreckoning
int32 statussynclock
int32 statusevkactiv
int32 statusevkestimates
int32 statusheadingexecuted
int32 statusconfigurationchanged
int32 statustilt
int32 statuspos
int8 statuscount
int32 statuskalmanfiltersetteled
int32 statuskflatstimulated
int32 statuskflongstimulated
int32 statuskfsteadystate
int32 statusspeed

# error and warning
string errorhardware
string error_misc1
string error_misc2
string error_misc3
string warngps
string warnmisc1
string errorhwsticky

# sensors body x
# acc in g
int32 accbodyhrx
float64 faccbodyhrx
# rate in deg/s
int32 ratebodyhrx
float64 fratebodyhrx

# sensors body y
# acc in g
int32 accbodyhry
float64 faccbodyhry

# rate in deg/s
int32 ratebodyhry
float64 fratebodyhry


# sensors body z
# acc in g
int32 accbodyhrz
float64 faccbodyhrz

# rate in deg/s
int32 ratebodyhrz
float64 fratebodyhrz


# rates body
# rates in deg/s
int16 ratebodyx
int16 ratebodyy
int16 ratebodyz
float64 fratebodyx
float64 fratebodyy
float64 fratebodyz


# rates horz
# rates in deg/s
int16 ratehorx
int16 ratehory
int16 ratehorz
float64 fratehorx
float64 fratehory
float64 fratehorz

# accelerations body
# acc in g
int16 accbodyx
int16 accbodyy
int16 accbodyz
float64 faccbodyx
float64 faccbodyy
float64 faccbodyz

# acceleration horizontal
# acc in g
int16 acchorx
int16 acchory
int16 acchorz
float64 facchorx
float64 facchory
float64 facchorz

# external velocity analog
#  vel in m/s
int16 extvelanx
int16 extvelany
float64 fextvelanx
float64 fextvelany

# external velocity digital pulses
#  vel in m/s
int16 extveldigx
int16 extveldigy
int16 extveldigpulsesx
int16 extveldigpulsesy
float64 fextveldigx
float64 fextveldigy


# externaal velocity corrected
#  vel in m/s
int16 extvelxcorrected
int16 extvelycorrected
float64 fextvelxcorrected
float64 fextvelycorrected

# barometer pressure
# pressure in mbar
uint32 extbaropressure
float64 fextbaropressure

# barometer height
# height in m
int32 extbaroheight
int32 extbaroheightcorrected
float64 fextbaroheight
float64 fextbaroheightcorrected

# miscellaneous
# invpath radius in 1/m
int16 invpathradius
float64 finvpathradius
# side slip in degrees
int16 sideslipangle
float64 fsideslipangle
# dis_trav in m
uint32 disttrav
float64 fdisttrav

# triggers 1 and 2
# triggers in mu.s
uint16 trigrising1
uint16 trigfalling1
uint16 trigrising2
uint16 trigfalling2


# triggers 3 and 4
# triggers in mu.s
uint16 trigrising3
uint16 trigfalling3
uint16 trigrising4
uint16 trigfalling4

# system data
# systemta in mu.s
uint16 systemta
# systemtemp in deg. c
int16 systemtemp
float64 fsystemtemp
# systemtimesinceinit in s
uint16 systemtimesinceinit
# system dsp load in %
uint16 systemdspload
float64 fsystemdspload


# gps position absolute
# position in deg
int32 gpslatabs
int32 gpslonabs
float64 fgpslatabs
float64 fgpslonabs

# gps position relative
# position in m
int32 gpslatrel
int32 gpslonrel
float64 fgpslatrel
float64 fgpslonrel

# gps epe
#  error in m
uint16 gpsstddevlat
uint16 gpsstddevlon
uint16 gpsstddevheight
float64 fgpsstddevlat
float64 fgpsstddevlon
float64 fgpsstddevheight


# gps velocity frame
# vel in m/s
int16 gpsvelframex
int16 gpsvelframey
int16 gpsvelframez
float64 fgpsvelframex
float64 fgpsvelframey
float64 fgpsvelframez
# latency in s
uint16 gpsvellatency
float64 fgpsvellatency


# gps eve
# error in m/s
uint16 gpsstddevvelx
uint16 gpsstddevvely
uint16 gpsstddevvelz
float64 fgpsstddevvelx
float64 fgpsstddevvely
float64 fgpsstddevvelz

# gps time utc
# gpstimemsec in ms
uint32 gpstimemsec
# gpstimeweek in week
uint16 gpstimeweek
# gpstrigger in mu.s
uint16 gpstrigger

# gps auxdata 1
# gpsdiffage in s
uint16 gpsdiffage
float64 fgpsdiffage
int8 gpsstatsused
int8 gpsstatsvisible

# gps auxdata 2
# gpslogdelay in ta
int8 gpslogdelay
# gpsreceiverload in %
int8 gpsreceiverload
float64 fgpsreceiverload

int32 gpsbasenr

# ins angle and gps cog
# all units in deg
int16 insroll
int16 inspitch
uint16 insyaw
uint16 gpscog
float64 finsroll
float64 finspitch
float64 finsyaw
float64 fgpscog

# gps height(msl)
int32 gpsheight
int16 undulation
float64 fgpsheight
float64 fundulation

# gps dualant time utc
# gpsdualanttimemsec in ms
uint32 gpsdualanttimemsec
# gpsdualanttimeweek in week
uint16 gpsdualanttimeweek

# gps dualant angle
# all units in deg
uint16 gpsdualantheading
int16 gpsdualantpitch
float64 fgpsdualantheading
float64 fgpsdualantpitch

# gps dualant angle ete
# all units in deg
int8 gpsdualantstddevheading
int8 gpsdualantstddevpitch
float64 fgpsdualantstddevheading
float64 fgpsdualantstddevpitch

# ins position height (msl)
int32 insheight
float64 finsheight

# ins time utc
# instimemsec in ms
uint32 instimemsec
# instimeweek in week
uint16 instimeweek
# leapseconds in s
int16 leapseconds

# ins position absolute
# position in deg
int32 inslatabs
int32 inslonabs
float64 finslatabs
float64 finslonabs

# ins position relative
# position in m
int32 inslatrel
int32 inslonrel
float64 finslatrel
float64 finslonrel

# ins velocity horizontal
# velocity in m/s
int16 insvelhorx
int16 insvelhory
int16 insvelhorz
float64 finsvelhorx
float64 finsvelhory
float64 finsvelhorz

# ins velocity frame
# velocity in m/s
int16 insvelframex
int16 insvelframey
int16 insvelframez
float64 finsvelframex
float64 finsvelframey
float64 finsvelframez

# ins epe
# error in m
uint16 insstddevlat
uint16 insstddevlong
uint16 insstddevheight
float64 finsstddevlat
float64 finsstddevlong
float64 finsstddevheight

# ins eve and ins ete
#  std vel in m/s
int8 insstddevvelx
int8 insstddevvely
int8 insstddevvelz
float64 finsstddevvelx
float64 finsstddevvely
float64 finsstddevvelz
# std ete in deg
int8 insstddevroll
int8 insstddevpitch
int8 insstddevyaw
float64 finsstddevroll
float64 finsstddevpitch
float64 finsstddevyaw

# analog in1
# analog in v
int16 an1
int16 an2
int16 an3
int16 an4
float64 fan1
float64 fan2
float64 fan3
float64 fan4

# kalman filter status
# all uints in %
uint8 kflatstimulated
uint8 kflongstimulated
uint8 kfsteadystate

# gnss receiver status
uint32 gpsreceivererror
uint32 gpsreceiverstatus


# mobileeye time comparison
float64 timemsec
float64 timensec

POI[] poi