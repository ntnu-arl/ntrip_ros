# ntrip_ros

Use the supplied launch file to launch:
```bash
roslaunch ntrip_ros ntrip_ros.launch
```

## Functional overview:

Receives RTCM corrections from an NTRIP Caster and publishes them on /rtcm as rtcm_msgs.
Subscribes to NavSatFixed msgs from gps topic to update the NTRIP caster about the current location by NMEA GGA sentences.
(Needed for NTRIP to work, see https://www.use-snip.com/kb/knowledge-base/nmea-gga-strings-in-ntrip-clients/ for more information)


## Credits
Original work: https://github.com/tilk/ntrip_ros

Inspired by ntrip_client_ros2 to make it run with python3: https://github.com/SGroe/ntrip_client_ros2

https://github.com/mantelt/ntrip_ros

