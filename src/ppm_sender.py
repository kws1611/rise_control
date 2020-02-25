#!/usr/bin/env python

import time
import pigpio 
import rospy
import rospy
import numpy as np
from rise_control.msg import ppm_msg
from std_msgs.msg import String

class X:
    
    WAVES=1
    def __init__(self, pi, gpio, channels=8, frame_ms=20):
        self.pi = pi
        self.gpio = gpio
        self.rate = rospy.Rate(150)
        self.GAP=300

        if frame_ms < 5:
            frame_ms = 5
            channels = 2
        elif frame_ms > 100:
            frame_ms = 100

        self.frame_ms = frame_ms
        self._frame_us = int(frame_ms * 1000)
        self._frame_secs = frame_ms / 1000.0
        self.check_count = 0

        self.ch1 = 1500
        self.ch2 = 1000
        self.ch3 = 1000
        self.ch4 = 1000
        self.ch5 = 1000
        self.ch6 = 1500
        self.ch7 = 1000
        self.ch8 = 1000

        if channels < 1:
            channels = 1
        elif channels > (frame_ms // 2):
            channels = int(frame_ms // 2)

        self.channels = channels

        self._widths = [1000] * channels # set each channel to minimum pulse width
        print(self._widths)

        self._wid = [None]*self.WAVES
        self._next_wid = 0

        pi.write(gpio, pigpio.LOW)

        self._update_time = time.time()
        rospy.Subscriber("/input_ppm",ppm_msg,self.ppm_cb)
        self.ppm_output_pub = rospy.Publisher("/output_ppm",ppm_msg,queue_size=1)

    def ppm_cb(self,msg):
        self.ppm_input_msg = msg
        self.ch1 = int(self.ppm_input_msg.channel_1)
        self.ch2 = int(self.ppm_input_msg.channel_2)
        self.ch3 = int(self.ppm_input_msg.channel_3)
        self.ch4 = int(self.ppm_input_msg.channel_4)
        self.ch5 = int(self.ppm_input_msg.channel_5)
        self.ch6 = int(self.ppm_input_msg.channel_6)
        self.ch7 = int(self.ppm_input_msg.channel_7)
        self.ch8 = int(self.ppm_input_msg.channel_8)

    def _update(self):
        wf =[]
        micros = 0
        for i in self._widths:
            wf.append(pigpio.pulse(1<<self.gpio, 0, self.GAP))
            wf.append(pigpio.pulse(0, 1<<self.gpio, i-self.GAP))
            micros += i
        # off for the remaining frame period
        wf.append(pigpio.pulse(1<<self.gpio, 0, self.GAP))
        micros += self.GAP
        wf.append(pigpio.pulse(0, 1<<self.gpio, self._frame_us-micros))

        self.pi.wave_add_generic(wf)

        wid = self.pi.wave_create()
        #print(wid)
        self.pi.wave_send_using_mode(wid, pigpio.WAVE_MODE_ONE_SHOT)
        self._wid[self._next_wid] = wid

        self._next_wid += 1
        if self._next_wid >= self.WAVES:
            self._next_wid = 0


        remaining = self._update_time + self._frame_secs - time.time()
        #if remaining > 0:
        #    time.sleep(remaining)
        self._update_time = time.time()

        wid = self._wid[self._next_wid]
        if wid is not None:
            self.pi.wave_delete(wid)
            self._wid[self._next_wid] = None

    def update_channel(self, channel, width):
        self._widths[channel] = width
        #self._update()

    def update_channels(self, widths):
        self._widths[0:len(widths)] = widths[0:self.channels]
        self._update()

    def cancel(self):
        self.pi.wave_tx_stop()
        for i in self._wid:
            if i is not None:
                self.pi.wave_delete(i)
    def channel_value(self):
        channel_value = [self.ch1,self.ch2,self.ch3,self.ch4,self.ch5,self.ch6,self.ch7,self.ch8]        
        return channel_value

    def sending_process(self):
        rospy.Subscriber("/input_ppm",ppm_msg,self.ppm_cb)
        self.sending_topic = ppm_msg()
        chan_1 = self.ch1
        chan_2 = self.ch2
        chan_3 = self.ch3
        chan_4 = self.ch4
        chan_5 = self.ch5
        chan_6 = self.ch6
        chan_7 = self.ch7
        chan_8 = self.ch8
        self.check_count += 1
        """
        self.update_channel(0, chan_1)
        self.update_channel(1, chan_2)
        self.update_channel(2, chan_3)
        self.update_channel(3, chan_4)
        self.update_channel(4, chan_5)
        self.update_channel(5, chan_6)
        self.update_channel(6, chan_7)
        self.update_channel(7, chan_8)
        
        self._widths[0] = self.ch1
        self._widths[1] = self.ch2
        self._widths[2] = self.ch3
        self._widths[3] = self.ch4
        self._widths[4] = self.ch5
        self._widths[5] = self.ch6
        self._widths[6] = self.ch7
        self._widths[7] = self.ch8
        """
        pw = self.check_count*5 + 700
        if pw > 1800:
            self.check_count = 0
        #self.update_channels([chan_1,chan_2,chan_3,chan_4,chan_5,chan_6,chan_7,chan_8])
        #self._width = [chan_8,chan_1,chan_2,chan_3,chan_4,chan_5,chan_6,chan_7]
        #self._update()
        #self.update_channels([chan_8,chan_1,chan_2,chan_3,chan_4,chan_5,chan_6,chan_7])
        self._widths[0] = self.ch1
        self._widths[1] = self.ch2
        self._widths[2] = self.ch3
        self._widths[3] = self.ch4
        
        self._update()
        """
        for pw in range(500, 2000, 100):
            self.update_channel(0, pw)
            self.update_channel(1, pw)
            self.update_channel(2, pw)
            self.update_channel(3, pw)
            self.update_channel(4, pw)
            self.update_channel(5, pw)
            self.update_channel(6, pw)
            self.update_channel(7, pw)
        
        """
        print(chan_1,chan_2,chan_3,chan_4,chan_5,chan_6,chan_7,chan_8)
        """
        self.sending_topic.channel_1 = self.ch1
        self.sending_topic.channel_2 = self.ch2
        self.sending_topic.channel_3 = self.ch3
        self.sending_topic.channel_4 = self.ch4
        self.sending_topic.channel_5 = self.ch5
        self.sending_topic.channel_6 = self.ch6
        self.sending_topic.channel_7 = self.ch7
        self.sending_topic.channel_8 = self.ch8
        self.ppm_output_pub.publish(self.sending_topic)
        """
        self.rate.sleep()
        


if __name__ == "__main__":
    rospy.init_node("ppm_sending", anonymous=True)
    rospy.loginfo("ppm_generating start")
    pi = pigpio.pi()

    try:
        if not pi.connected:
            exit(0)
        pi.wave_tx_stop() # Start with a clean slate.
        ppm = X(pi, 17, frame_ms=20)
        time.sleep(2)

        
        while not rospy.is_shutdown():
            #ppm.update_channels([1000, 2000, 1000, 2000, 1000, 2000, 1000, 2000])    
            #ppm.update_channels(ppm.channel_value())
            #print(ppm.channel_value())
            #ppm._update()  
            ppm.sending_process()          
            #start = time.time()
            """
            for chan in range(8):
                for pw in range(500, 2000, 5):
                    ppm.update_channel(chan, pw)
                    time.sleep(0.1)                    
                    #ppm._update()
            """
        
        
        
        
        ppm.cancel()

        pi.stop()
    except rospy.ROSInterruptException:
        print "ROS terminated"
        pass

    #ppm.update_channels([1000, 2000, 1000, 2000, 1000, 2000, 1000, 2000])

    #time.sleep(2)




