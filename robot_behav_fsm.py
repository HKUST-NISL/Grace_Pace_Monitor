#general
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
from inspect import getsourcefile
from os.path import abspath

#ros
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

#specific
import utils.behav_event_proc
import robot_nod_fsm
import robot_gaze_fsm
import robot_speaking_fsm
import robot_humming_fsm

class RobotBehavFSM:

    def __init__(self, config_data, logger):
        #Logging
        self.__logger = logger.getChild(self.__class__.__name__)

        #Configs
        self.__config_data = config_data

        #Sub-fsm
        self.__robot_speaking_fsm = robot_speaking_fsm.RobotSpeakingFSM(
                            self.__config_data,
                            self.__logger)
        self.__robot_humming_fsm = robot_humming_fsm.RobotHummingFSM(
                            self.__config_data,
                            self.__logger)
        self.__robot_nodding_fsm = robot_nod_fsm.RobotNoddingFSM(
                            self.__config_data,
                            self.__logger)
        self.__robot_gaze_fsm = robot_gaze_fsm.RobotGazeFSM(
                            self.__config_data,
                            self.__logger)



    def procEvent(self, behav_event_proc_handle):
        self.__robot_speaking_fsm.procEvent(behav_event_proc_handle.readSpeakEvent())
        self.__robot_humming_fsm.procEvent(behav_event_proc_handle.readHumEvent())
        self.__robot_nodding_fsm.procEvent(behav_event_proc_handle.readNodEvent())
        self.__robot_gaze_fsm.procEvent(behav_event_proc_handle.readGazeEvent())

    def getState(self):
        return {
                'robot_speaking': {
                    'val': self.__robot_speaking_fsm.current_state.id,
                    'stamp': self.__robot_speaking_fsm.stamp_upon_entering,
                    'transition': self.__robot_speaking_fsm.is_transition
                    },
                'robot_humming': {
                    'val': self.__robot_humming_fsm.current_state.id,
                    'stamp': self.__robot_humming_fsm.stamp_upon_entering,
                    'transition': self.__robot_humming_fsm.is_transition
                    },
                'robot_nodding': {
                    'val': self.__robot_nodding_fsm.current_state.id,
                    'stamp': self.__robot_nodding_fsm.stamp_upon_entering,
                    'transition': self.__robot_nodding_fsm.is_transition
                    },
                'robot_gaze': {
                    'val': self.__robot_gaze_fsm.current_state.id,
                    'stamp': self.__robot_gaze_fsm.stamp_upon_entering,
                    'transition': self.__robot_gaze_fsm.is_transition
                    }
                }



    def initializeState(self):
        self.__robot_speaking_fsm.stamp_upon_entering = time.time()
        self.__robot_humming_fsm.stamp_upon_entering = time.time()
        self.__robot_nodding_fsm.stamp_upon_entering = time.time()
        self.__robot_gaze_fsm.stamp_upon_entering = time.time()
