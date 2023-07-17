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
import queue

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
    def __init__(self, config_data, logger):
        #Miscellaneous
        self.__config_data = config_data
        self.__logger = logger.getChild(self.__class__.__name__)
        
        #VAD message handling
        self.__vad_sub = rospy.Subscriber(
                            self.__config_data['Custom']['Sensors']['topic_silero_vad_name'], 
                            std_msgs.msg.String, 
                            self.__vadMsgCallback, 
                            queue_size=self.__config_data['Custom']['Ros']['queue_size'])
        self.__speech_flag = False


    def __vadMsgCallback(self,msg):
        self.__speech_flag = (msg.data == self.__config_data['Sensors']['SileroVAD']['speech_string'])
        self.__logger.debug("Speech flag %s." % self.__speech_flag)


    def readSpeechFlag(self):
        return self.__speech_flag
