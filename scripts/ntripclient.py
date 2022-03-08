#!/usr/bin/python3

import rospy

from rtcm_msgs.msg import Message
from sensor_msgs.msg import NavSatFix

import datetime
from http.client import HTTPConnection
from base64 import b64encode
from threading import Thread

class ntripconnect(Thread):
    def __init__(self, ntc):
        super(ntripconnect, self).__init__()
        self.ntc = ntc
        self.stop = False

    def run(self):
        authenticationScheme = 'Basic '
        userCredentials = str(self.ntc.ntrip_user) + ':' + str(self.ntc.ntrip_pass)
        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros',
            'Connection': 'close',
            'Authorization': authenticationScheme.encode() + b64encode(userCredentials.encode())
        }
        connection = HTTPConnection(self.ntc.ntrip_server + ':' + str(self.ntc.ntrip_port))
        now = datetime.datetime.utcnow()
        connection.request('GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga_string, headers)
        
        response = connection.getresponse()
        if response.status != 200: raise Exception("Connection error. Got HTTP response status " + str(response.status))
        buf = ""
        rmsg = Message()
        while not self.stop:
            data = response.read(1)
            if data!=chr(211).encode('latin-1'):
                continue
            l1 = ord(response.read(1))
            l2 = ord(response.read(1))
            pkt_len = ((l1&0x3)<<8)+l2

            pkt = response.read(pkt_len)
            parity = response.read(3)
            if len(pkt) != pkt_len:
                rospy.logerr("Length error: {} {}".format(len(pkt), pkt_len))
                continue

            rmsg.header.seq += 1
            rmsg.header.stamp = rospy.get_rostime()
            rmsg.message = data + chr(l1).encode("latin-1") + chr(l2).encode("latin-1") + pkt + parity
            self.ntc.pub.publish(rmsg)

        connection.close()


class ntripclient:
    def __init__(self):
        rospy.init_node('ntripclient', anonymous=True)

        self.rtcm_topic = rospy.get_param('~rtcm_topic', '/rtcm')

        self.ntrip_server = rospy.get_param('~ntrip_server', 'no.nrtk.eu')
        self.ntrip_port = rospy.get_param('~ntrip_port', 9301)
        self.ntrip_user = rospy.get_param('~ntrip_user', 'USER')
        self.ntrip_pass = rospy.get_param('~ntrip_pass', 'PASSWORD')
        self.ntrip_stream = rospy.get_param('~ntrip_stream', 'MSM_NEAR')
        self.gps_topic = None
        if rospy.has_param('~gps_topic'):
            self.gps_topic = rospy.get_param('~gps_topic', '/ublox/fix')
        self.sub = None

        if self.gps_topic is not None and self.gps_topic != '':
            rospy.loginfo('Using coordinates supplied in ' + self.gps_topic)
            rospy.loginfo('Will start streaming corrections once 3D fix is acquired')
            self.sub = rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_fix_cb)
            self.latitude = None
            self.longitude = None
        else:
            rospy.loginfo('Using coordinates from params')
            self.latitude = rospy.get_param('~latitude',63.416108)
            self.longitude = rospy.get_param('~longitude', 10.401436)
            self.nmea_gga_string = self.generate_gga_string()
            self.connection = ntripconnect(self)
            self.connection.start()

        
        self.pub = rospy.Publisher(self.rtcm_topic, Message, queue_size=10)

        self.connection = None


    def generate_gga_string(self):
        """
        Generates NMEA GGA message to send to the NTRIP caster
        """

        now = datetime.datetime.utcnow()
        latitude = int(self.latitude) * 100 + \
            (self.latitude - int(self.latitude)) * 60
        longitude = int(self.longitude) * 100 + \
            (self.longitude - int(self.longitude)) * 60
        latitude_sign = 'N' if self.latitude >= 0 else 'S'
        longitude_sign = 'E' if self.longitude >= 0 else 'W'

        nmeadata = 'GPGGA,%02d%02d%04.1f,%09.4f,%s,%010.4f,%s,1,12,1.0,0.0,M,0.0,M,,' % (now.hour, now.minute, now.second, latitude, latitude_sign, longitude, longitude_sign)
        
        csum = 0
        for c in nmeadata:
               # XOR'ing value of csum against the next char in line
               # and storing the new XOR value in csum
               if ord(c)!=',':
                  csum ^= ord(c)

        #convert hex characters to upper case
        csum = hex(csum).upper() 

        #add 0x0 if checksum value is less than 0x10
        if len(csum)==3:	
            csum='0'+csum[2]
        else:
            csum=csum[2:4]

        nmeastring = '$'+nmeadata+'*'+csum
                
        return(nmeastring)

    def gps_fix_cb(self, msg):
        """
        Callback for mavros raw gps msg to read in current position
        """
        if msg.status.status >= 0:
            self.latitude = msg.latitude
            self.longitude = msg.longitude
            self.nmea_gga_string = self.generate_gga_string()
            rospy.loginfo('Got position information with 3D fix, starting relaying RTCM data')
            self.connection = ntripconnect(self)
            self.connection.start()
            self.sub.unregister()
            



    def run(self):
        rospy.spin()
        if self.connection is not None:
            self.connection.stop = True

if __name__ == '__main__':
    c = ntripclient()
    c.run()

