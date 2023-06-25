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


class TTSEventProc:
    def __init__(self, tts_event_topic_name, logger):
        self.__tts_event_sub = rospy.Subscriber(tts_event_topic_name, std_msgs.msg.String, self.__ttsEventCallback, queue_size=100)
        self.__logger = logger.getChild(self.__class__.__name__)
        self.tts_playing = False

    def __ttsEventCallback(self,msg):
        if(msg.data == 'start'):
            self.tts_playing = True
        elif(msg.data == 'stop'):
            self.tts_playing = False
        else:
            pass
        self.__logger.debug("New TTS event: %s, current flag is %s." % (msg.data, self.tts_playing) )



