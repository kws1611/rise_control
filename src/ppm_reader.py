#!/usr/bin/env python

import time
import math
import pigpio # http://abyz.co.uk/rpi/pigpio/python.html
import rospy
import numpy as np
from rise_control.msg import ppm_msg
from std_msgs.msg import String


class PWM_read:
    def __init__(self, pi, gpio):
        self.pi = pi
        self.gpio = gpio

        self._high_tick = None
        self._p = None
        self._hp = None
        self.correcting_channel = 0
        self.correcting_switch = False

        self.count = 0
        self.ch = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.ch_final = [0,0,0,0,0,0,0,0,0]
        self._cb = pi.callback(gpio, pigpio.EITHER_EDGE, self._cbf)
        self.input_pub = rospy.Publisher("/input_ppm",ppm_msg,queue_size=1)

    def _cbf(self, gpio, level, tick):
        input_channel = ppm_msg()
        if level == 1:
            if self._high_tick is not None:
                self._p = pigpio.tickDiff(self._high_tick, tick)
            self._high_tick = tick
        elif level == 0:
            if self._high_tick is not None:
                    self._hp = pigpio.tickDiff(self._high_tick, tick)
        if (self._p is not None) and (self._hp is not None):
            #print("g={} f={:.1f} dc={:.1f}".
            #	format(gpio, 1000000.0/self._p, 100.0 * self._hp/self._p))
            self.ch[self.count] = self._hp
            self.count += 1
            if self.count == 18:
                #print(self.ch)
                self.count = 0
                self.ch_final = [self.ch[0],self.ch[2],self.ch[4],self.ch[6],self.ch[8],self.ch[10],self.ch[12],self.ch[14],self.ch[16]]
                
                self.ch = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                if(self.ch_final[8] > 7000):
                    del self.ch_final[8]
                    print(self.ch_final)
                    input_channel.channel = self.ch_final
                    self.input_pub.publish(input_channel)
                else :
                    for i in range(0,8):
                        if (self.ch_final[i] > 7000):
                            self.correcting_channel = i
                            self.correcting_switch = True
                
            if self.correcting_switch:
                self.count -= (self.correcting_channel*2+2)
                self.correcting_switch = False

    def cancel(self):
        self._cb.cancel()

if __name__ == "__main__":
    rospy.init_node("ppm_reading", anonymous=True)
    rospy.loginfo("ppm_reading start")
    pi = pigpio.pi()    
    try:
        while not rospy.is_shutdown():
            p1 = PWM_read(pi, 4)
        p1.cancel()
    except rospy.ROSInterruptException:
        print "ROS terminated"
        pass
    pi.stop()
