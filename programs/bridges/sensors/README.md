BRIDGES SENSORS
===============

Turner Designs
--------------
12 V / 9600 8N1 / auto-shutdown

 $CYTUR,BB,CCCC,DDD
  type , time in seconds, reading (0-1024), gain (1, 10, 100) 



Contros Hydroflash
------------------
6-30V / 115200 8N1

 $CODT1,0,0,D,2017-04-28,17:52:25,48297,1319891,21.69044,273899,-302731,20075,187.76056,0.00000,107.46900\r\n



SeaBird 37-SI CTD
-----------------
8.5-24V / 9600 8N1, changed to 115200 8N1
sampling between 6-21600

  8.5796, 0.15269, 531.316, 527.021, 1.1348,1451.478, 3.2486, 20 Oct 2012, 09:01:44
  (tempe, conduct, pressur, depth, salinity, sound v, densit, date, time)



OCTOPUS Camera
--------------
12V / 19200 8N1 / auto-shutdown
(use \r or \n, never both..)

 $start:111111111;
 $pt:102.35,20161122,161850;
 OCT001,LIG001,OCTOPUS123456789,1210,4530.15,20150101,120258,247,1023, 4500,270,3,0,0,0;



NOC lab-on-a-chip
-----------------
12V / 9600 8N1 / auto-shutdown
(\r only)

 clock-set=D,M,Y,h,m,s
 start=dive_number
 status=<a=dive,b=climb>,depth,p1,p2,p3
 sample
 stop

readings:
 t,R,T,Q,S
 1494595519,12,1,0,2
 1494595679,12,1,0,2
 timestamp,reading,temperature(*10^-3),quality(0=good,1=bad),std_dev(*1000)

[conversion factor]
ammonia=(tbc)*10^-9
nitrate=M*10^-9
phosphate=M*10^-9
silicate=(tbc)*10^-3

CMRE SBP
--------

(NMEA 0183)
$PSUBC,hhmmss.ss,CMDNM,CMDPARAM*hh
 1) time (UTC)
 2) Command name
 3) Command parameter
 4) checksum


FLUIDION SAMPLER
----------------

12V / 38400 8N1 / auto-shutdown
