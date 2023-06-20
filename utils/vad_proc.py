import yaml
import rospy
import os
import re
import threading
from signal import signal
from signal import SIGINT
import logging
import sys

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
        self.vad_sub = rospy.Subscriber(vad_topic_name, std_msgs.msg.String, self.__vadMsgCallback, queue_size=10)
        self.logger = logger.getChild(self.__class__.__name__)

    def __vadMsgCallback(self,msg):
        self.logger.info(msg.data)
        pass

