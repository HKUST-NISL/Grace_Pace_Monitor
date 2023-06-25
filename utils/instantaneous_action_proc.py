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



class InstantaneousActionProc:

    def __init__(self, instantaneous_action_topic_name, logger):
        self.__instantaneous_action_sub = rospy.Subscriber(instantaneous_action_topic_name, std_msgs.msg.String, self.__instantaneousActionCallback, queue_size=100)
        self.__logger = logger.getChild(self.__class__.__name__)
        self.current_action = ''


    def __instantaneousActionCallback(self, msg):
        self.current_action = msg.data
        self.__logger.debug("New instantaneous action: %s." % self.current_action )

