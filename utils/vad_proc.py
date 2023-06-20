import yaml
import rospy
import os
import re
import threading
from signal import signal
from signal import SIGINT
import logging
import sys
from datetime import datetime
import time

import dynamic_reconfigure.client
import sensor_msgs.msg
import std_msgs.msg
import hr_msgs.msg
import grace_attn_msgs.msg
import grace_attn_msgs.srv
import hr_msgs.msg
import hr_msgs.cfg
import hr_msgs.srv
import std_msgs




class VADProc:
    def __init__(self, freq, vad_topic_name, logger):
        self.__vad_sub = rospy.Subscriber(vad_topic_name, std_msgs.msg.String, self.__vadMsgCallback, queue_size=10)
        self.__logger = logger.getChild(self.__class__.__name__)
        
        self.__vad_freq = freq
        self.__vad_reset_interval = 1 / self.__vad_freq
        self.vad_flag = False
        self.__vad_flag_cnt = 0


    def __vadMsgCallback(self,msg):
        #Receiving a vad message indicates that there is some human voice in the past 1 / freq second
        self.vad_flag = True

        #Increment the flag cnt
        self.__vad_flag_cnt = self.__vad_flag_cnt + 1
        
        #Initiate a thread which will reset the flag if no new vad flag is received
        #which will be shown in the vad flag counter
        reset_thread = threading.Thread(target=self.__reset_vad_thread)
        reset_thread.start()

    def __reset_vad_thread(self):
        #Keep the old counter
        cnt_old = self.__vad_flag_cnt

        #Sleep for the normial interval computed by vad frequency
        time.sleep(self.__vad_reset_interval)

        #Reset vad flag if necessary
        if(self.__vad_flag_cnt == cnt_old):
            #No new flag is received
            self.vad_flag = False
            # self.__logger.info("Reset VAD flag")
