#!/usr/bin/python

import rospy

from mavros_msgs.msg import RTCM

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
        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros',
            'Connection': 'close',
            'Authorization': b'Basic ' + b64encode((self.ntc.ntrip_user + ':' + self.ntc.ntrip_pass).encode())
        }
        connection = HTTPConnection(self.ntc.ntrip_server)
        now = datetime.datetime.utcnow()
        connection.request('GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga_string, headers)
        
        response = connection.getresponse()
        if response.status != 200: raise Exception("Connection error. Got HTTP response status " + response.status)
        buf = ""
        rmsg = RTCM()
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
            rmsg.data = data + chr(l1).encode("latin-1") + chr(l2).encode("latin-1") + pkt + parity
            self.ntc.pub.publish(rmsg)

        connection.close()


class ntripclient:
    def __init__(self):
        rospy.init_node('ntripclient', anonymous=True)

        self.rtcm_topic = rospy.get_param('~rtcm_topic')

        self.ntrip_server = rospy.get_param('~ntrip_server')
        self.ntrip_user = rospy.get_param('~ntrip_user')
        self.ntrip_pass = rospy.get_param('~ntrip_pass')
        self.ntrip_stream = rospy.get_param('~ntrip_stream')
        self.latitude = rospy.get_param('~latitude')
        self.longitude = rospy.get_param('~longitude')
        self.nmea_gga_string = self.generate_gga_string()

        self.pub = rospy.Publisher(self.rtcm_topic, RTCM, queue_size=10)

        self.connection = None
        self.connection = ntripconnect(self)
        self.connection.start()

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

    def run(self):
        rospy.spin()
        if self.connection is not None:
            self.connection.stop = True

if __name__ == '__main__':
    c = ntripclient()
    c.run()

