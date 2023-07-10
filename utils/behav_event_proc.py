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
    def __init__(self, config_data, logger):
        self.__config_data = config_data
        self.__logger = logger.getChild(self.__class__.__name__)

        #Speaking event
        self.__speak_event_sub = rospy.Subscriber(
                                    self.__config_data['Ros']['speak_event_topic'], 
                                    std_msgs.msg.String, 
                                    self.__speakEventCallback, 
                                    queue_size=self.__config_data['Ros']['queue_size'])
        self.__latest_speak_event = ''

        #Humming event
        self.__hum_event_sub = rospy.Subscriber(
                                    self.__config_data['Ros']['hum_event_topic'], 
                                    std_msgs.msg.String, 
                                    self.__humEventCallback, 
                                    queue_size=self.__config_data['Ros']['queue_size']) 
        self.__latest_hum_event = ''

        #Nodding event
        self.__nod_event_sub = rospy.Subscriber(      
                                    self.__config_data['Ros']['nod_event_topic'], 
                                    std_msgs.msg.String,
                                    self.__nodEventCallback, 
                                    queue_size=self.__config_data['Ros']['queue_size'])
        self.__latest_nod_event = ''


        #Gaze event
        self.__gaze_event_sub = rospy.Subscriber(
                                    self.__config_data['Ros']['gaze_event_topic'], 
                                    std_msgs.msg.String, 
                                    self.__gazeEventCallback, 
                                    queue_size=self.__config_data['Ros']['queue_size'])
        self.__latest_gaze_event = ''


    def __speakEventCallback(self,msg):
        self.__latest_speak_event = msg.data
        self.__logger.debug("New speak event: %s." % self.__latest_speak_event )

    def readSpeakEvent(self):
        latest_speak_event = self.__latest_speak_event
        self.__latest_speak_event = ''
        return latest_speak_event


    def __humEventCallback(self,msg):
        self.__latest_hum_event = msg.data
        self.__logger.debug("New humming event: %s." % self.__latest_hum_event )

    def readHumEvent(self):
        latest_hum_event = self.__latest_hum_event
        self.__latest_hum_event = ''
        return latest_hum_event

    def __nodEventCallback(self,msg):
        self.__latest_nod_event = msg.data
        self.__logger.debug("New nod event: %s." % self.__latest_nod_event )

    def readNodEvent(self):
        latest_nod_event = self.__latest_nod_event
        self.__latest_nod_event = ''
        return latest_nod_event

    def __gazeEventCallback(self,msg):
        self.__latest_gaze_event = msg.data
        self.__logger.debug("New gaze event: %s." % self.__latest_gaze_event )

    def readGazeEvent(self):
        latest_gaze_event = self.__latest_gaze_event
        self.__latest_gaze_event = ''
        return latest_gaze_event