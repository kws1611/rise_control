#!/usr/bin/env python

import time
import pigpio
import rospy
import rospy
import numpy as np
from rise_control.msg import ppm_msg
from std_msgs.msg import String

class X:
    WAVES = 1
    def __init__(self, pi, gpio, channels=8, frame_ms=33):
        self.pi = pi
        self.gpio = gpio
        self.rate = rospy.Rate(150)
        self.GAP = 150

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
        self._widths = [1000] * channels  # set each channel to minimum pulse width
        self._wid = [None] * self.WAVES
        self._next_wid = 0
        pi.write(gpio, pigpio.LOW)
        self._update_time = time.time()
        rospy.Subscriber("/input_ppm", ppm_msg, self.ppm_cb)
        self.ppm_output_pub = rospy.Publisher("/output_ppm", ppm_msg, queue_size=1)

    def ppm_cb(self, msg):
        self.ppm_input_msg = msg
        self.ch1 = int(self.ppm_input_msg.channel_1)
        self.ch2 = int(self.ppm_input_msg.channel_2)
        self.ch3 = int(self.ppm_input_msg.channel_3)
        self.ch4 = int(self.ppm_input_msg.channel_4)
        self.ch5 = int(self.ppm_input_msg.channel_5)

    def _update(self):
        wf = []
        micros = 0
        for i in self._widths:
            wf.append(pigpio.pulse(1 << self.gpio, 0, self.GAP))
            wf.append(pigpio.pulse(0, 1 << self.gpio, i - self.GAP))
            micros += i
        # off for the remaining frame period
        wf.append(pigpio.pulse(1 << self.gpio, 0, self.GAP))
        micros += self.GAP
        wf.append(pigpio.pulse(0, 1 << self.gpio, self._frame_us - micros))
        self.pi.wave_add_generic(wf)
        wid = self.pi.wave_create()
        self.pi.wave_send_using_mode(wid, pigpio.WAVE_MODE_ONE_SHOT_SYNC)
        self._wid[self._next_wid] = wid

        remaining = self._update_time + self._frame_secs - time.time()
        if remaining > 0:
            time.sleep(remaining)
        self._update_time = time.time()

        self.pi.wave_delete(wid)

    def cancel(self):
        self.pi.wave_tx_stop()
        for i in self._wid:
            if i is not None:
                self.pi.wave_delete(i)
    def update_channels(self, widths):
        self._widths[0:len(widths)] = widths[0:self.channels]
        self._update()

    def sending_process(self):
        rospy.Subscriber("/input_ppm", ppm_msg, self.ppm_cb)
        self.sending_topic = ppm_msg()
        
        if self.ch5 > 1700 :
            chan_5 = 2000
        elif self.ch5 < 700:
            chan_5 = 500
        else :
            chan_5 = 1500

        self.update_channels([self.ch1,self.ch2,self.ch3,self.ch4,self.ch5,1000,1000,1000])

        self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("ppm_sending", anonymous=True)
    rospy.loginfo("ppm_generating start")
    pi = pigpio.pi()

    try:
        if not pi.connected:
            exit(0)
        pi.wave_tx_stop()
        ppm = X(pi, 17, frame_ms=33)
        time.sleep(2)

        while not rospy.is_shutdown():

            ppm.sending_process()
        ppm.cancel()

        pi.stop()
    except rospy.ROSInterruptException:
        print
        "ROS terminated"
        pass





