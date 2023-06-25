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


class BehavEventProc:
    def __init__(self, behav_event_topic_name, logger):
        self.__behav_event_sub = rospy.Subscriber(behav_event_topic_name, std_msgs.msg.String, self.__behavEventCallback, queue_size=100)
        self.__logger = logger.getChild(self.__class__.__name__)
        self.behav_playing = ''

    def __behavEventCallback(self,msg):
        self.behav_playing = msg.data
        self.__logger.debug("New behav event: %s." % self.behav_playing )



