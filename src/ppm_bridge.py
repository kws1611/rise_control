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

        self.channel_number = [0,0,0,0,0,0,0,0]
        self.count = 0
        self.basic_ppm_signal_count = 0
        self.ch = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.ch_final = [0,0,0,0,0,0,0,0,0]
        
        self.first_switch = True

        self._cb = pi.callback(gpio, pigpio.EITHER_EDGE, self._cbf)
        
    def _cbf(self, gpio, level, tick):
        #print("_cb")
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
            #print(self.ch)
            if self.count == 18:
                #print(self.ch)
                self.count = 0



    def reading_process(self):
        #print("reading_proce")
        read_message = ppm_msg()
        if self.first_switch:
            if self.ch[self.count] > 7000:
                self.basic_ppm_signal_count = self.count
                self.first_switch = False

            for i in range(0,7):
                if self.basic_ppm_signal_count + 2 + 2*i > 7:
                    self.channel_number[i] = self.basic_ppm_signal_count + 2 + 2*i - 17
                else :
                    self.channel_number[i] = self.basic_ppm_signal_count + 2 + 2*i
                #print(self.ch)
        
        self.ch_final = [self.ch[self.channel_number[0]], self.ch[self.channel_number[1]], self.ch[self.channel_number[2]], self.ch[self.channel_number[3]], self.ch[self.channel_number[4]], self.ch[self.channel_number[5]], self.ch[self.channel_number[6]],self.ch[self.channel_number[7]]]

    def cancel(self):
        self._cb.cancel()

    def to_global(self):
        print(self.ch_final)
        return self.ch_final


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
        
        self.ppm_output_pub = rospy.Publisher("/output_ppm",ppm_msg,queue_size=1)

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
        #if remaining > 0:
            #time.sleep(remaining)
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

    def sending_process(self, chan):
        self.sending_topic = ppm_msg()
        chan_1 = chan[0]
        chan_2 = chan[1]
        chan_3 = chan[2]
        chan_4 = chan[3]
        chan_5 = chan[4]
        chan_6 = chan[5]
        chan_7 = chan[6]
        chan_8 = chan[7]
        """
        self.update_channel(0, chan_1)
        self.update_channel(1, chan_2)
        self.update_channel(2, chan_3)
        self.update_channel(3, chan_4)
        self.update_channel(4, chan_5)
        self.update_channel(5, chan_6)
        self.update_channel(6, chan_7)
        self.update_channel(7, chan_8)
        """
        print("processing")

        #self.update_channels([self.ch1,self.ch2,self.ch3,self.ch4,self.ch5,self.ch6,self.ch7,self.ch8])
        #self._update()
        


if __name__ == "__main__":
    rospy.init_node("ppm_reading", anonymous=True)
    rospy.loginfo("ppm_reading start")
    pi = pigpio.pi()    

    try:
        p1 = PWM_read(pi, 4)
        time.sleep(1)
        pi.wave_tx_stop() # Start with a clean slate.
        ppm = X(pi, 17, frame_ms=20)
        
        while not rospy.is_shutdown():
            p1.reading_process()
            ppm.sending_process(p1.to_global())
        
    except rospy.ROSInterruptException:
        print "ROS terminated"
        pass
    pi.stop()
