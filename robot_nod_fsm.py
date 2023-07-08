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
from statemachine import StateMachine, State
import utils.vad_proc
import utils.asr_proc

class RobotNoddingFSM(StateMachine):

    #Time stamp of entering each state
    stamp_upon_entering = time.time()

    #States
    nodding = State()
    not_nodding = State(initial=True)

    #Events
    is_nodding = (
        not_nodding.to(nodding, on = 'on_nodding')
        |
        nodding.to(nodding, on = 'on_nodding')
    )
    is_not_nodding = (
        not_nodding.to(not_nodding, on = 'on_not_nodding')
        |
        nodding.to(not_nodding, on = 'on_not_nodding')
    )

    def __init__(self, config_data, logger):
        #FSM base class
        super(self.__class__, self).__init__(rtc=True)

        #Logging
        self.__logger = logger.getChild(self.__class__.__name__)
        
        #Configs
        self.__config_data = config_data


    '''
        Transition actions named after to state machine convention
    '''

    #General transition action
    def on_transition(self, event: str, source: State, target: State):
        if(source != target):
            self.stamp_upon_entering = time.time()
        self.__logger.debug(f"on '{event}' from '{source.id}' to '{target.id}' @ %f" % (self.stamp_upon_entering) )        
        return "on_transition"

    def on_nodding(self):
        self.__logger.info("Robot nodding.")

    def on_not_nodding(self):
        self.__logger.info("Robot not nodding.")

    '''
        Wrapper
    '''
    def procEvent(self, event_code):
        if(event_code == self.__config_data['General']['start_nodding_event_name']):
            self.is_nodding()
        elif(event_code == self.__config_data['General']['stop_nodding_event_name']):
            self.is_not_nodding()
        else:
            #Irrelevant event
            pass

