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



class TurnActionProc:

    def __init__(self, turn_action_topic_name, logger):
        self.__turn_action_sub = rospy.Subscriber(turn_action_topic_name, std_msgs.msg.String, self.__turnActionCallback, queue_size=100)
        self.__logger = logger.getChild(self.__class__.__name__)
        self.__latest_action = ''


    def __turnActionCallback(self, msg):
        self.__latest_action = msg.data
        self.__logger.debug("New turn action: %s." % self.__latest_action )

    def readLatestAction(self):
        latest_action = self.__latest_action
        self.__latest_action = ''
        return latest_action