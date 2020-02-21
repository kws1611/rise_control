#!/usr/bin/env python

import time
import pigpio 
import rospy
import rospy
import numpy as np
from rise_control.msg import ppm_msg
from std_msgs.msg import String

class X:
    GAP=300
    WAVES=3
    def __init__(self, pi, gpio, channels=8, frame_ms=27):
        self.pi = pi
        self.gpio = gpio

        if frame_ms < 5:
            frame_ms = 5
            channels = 2
        elif frame_ms > 100:
            frame_ms = 100

        self.frame_ms = frame_ms
        self._frame_us = int(frame_ms * 1000)
        self._frame_secs = frame_ms / 1000.0

        self.ch1 = 0.0
        self.ch2 = 0.0
        self.ch3 = 0.0
        self.ch4 = 0.0
        self.ch5 = 0.0
        self.ch6 = 0.0
        self.ch7 = 0.0
        self.ch8 = 0.0

        if channels < 1:
            channels = 1
        elif channels > (frame_ms // 2):
            channels = int(frame_ms // 2)

        self.channels = channels

        self._widths = [1000] * channels # set each channel to minimum pulse width

        self._wid = [None]*self.WAVES
        self._next_wid = 0

        pi.write(gpio, pigpio.LOW)

        self._update_time = time.time()
        rospy.Subscriber("/input_ppm",ppm_msg,self.ppm_cb)
        self.ppm_output_pub = rospy.Publisher("/output_ppm",ppm_msg,queue_size=1)

    def ppm_cb(self,msg):
        self.ppm_input_msg = msg
        self.ch1 = self.ppm_input_msg.channel[0]
        self.ch2 = self.ppm_input_msg.channel[1]
        self.ch3 = self.ppm_input_msg.channel[2]
        self.ch4 = self.ppm_input_msg.channel[3]
        self.ch5 = self.ppm_input_msg.channel[4]
        self.ch6 = self.ppm_input_msg.channel[5]
        self.ch7 = self.ppm_input_msg.channel[6]
        self.ch8 = self.ppm_input_msg.channel[7]

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
        self.pi.wave_send_using_mode(wid, pigpio.WAVE_MODE_REPEAT_SYNC)
        self._wid[self._next_wid] = wid

        self._next_wid += 1
        if self._next_wid >= self.WAVES:
            self._next_wid = 0


        remaining = self._update_time + self._frame_secs - time.time()
        if remaining > 0:
            time.sleep(remaining)
        self._update_time = time.time()

        wid = self._wid[self._next_wid]
        if wid is not None:
            self.pi.wave_delete(wid)
            self._wid[self._next_wid] = None

    def update_channel(self, channel, width):
        self._widths[channel] = width
        self._update()

    def update_channels(self, widths):
        self._widths[0:len(widths)] = widths[0:self.channels]
        self._update()

    def cancel(self):
        self.pi.wave_tx_stop()
        for i in self._wid:
            if i is not None:
                self.pi.wave_delete(i)

    def sending_process(self):
        self.update_channel(1, self.ch1)
        self.update_channel(2, self.ch2)
        self.update_channel(3, self.ch3)
        self.update_channel(4, self.ch4)
        self.update_channel(5, self.ch5)
        self.update_channel(6, self.ch6)
        self.update_channel(7, self.ch7)
        self.update_channel(8, self.ch8)
        self._update()



if __name__ == "__main__":
    rospy.init_node("ppm_sending", anonymous=True)
    rospy.loginfo("ppm_generating start")
    pi = pigpio.pi()

    try:
        if not pi.connected:
            exit(0)
        pi.wave_tx_stop() # Start with a clean slate.
        while not rospy.is_shutdown():
            ppm = X(pi, 14, frame_ms=20)
            #start = time.time()
            """
            for chan in range(8):
                for pw in range(500, 2000, 5):
                    ppm.update_channel(chan, pw)
            """


    except rospy.ROSInterruptException:
        print "ROS terminated"
        pass

    ppm.update_channels([1000, 2000, 1000, 2000, 1000, 2000, 1000, 2000])

    time.sleep(2)

    ppm.cancel()

    pi.stop()


