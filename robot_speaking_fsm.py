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

class RobotSpeakingFSM(StateMachine):

    #States
    speaking = State()#speaking
    not_speaking = State(initial=True)#not speaking

    #Events
    is_speaking = (
        not_speaking.to(speaking, on="on_silence_broken")
        |
        speaking.to(speaking, on="on_continues_speaking")
    )
    is_not_speaking = (
        not_speaking.to(not_speaking, on="on_silence_persists")
        |
        speaking.to(not_speaking, on="on_stopped_speaking")
    )

    def __init__(self, config_data, logger):
        #FSM base class
        super(self.__class__, self).__init__(rtc=True)

        #Logging
        self.__logger = logger.getChild(self.__class__.__name__)
        
        #Code name for start speaking event
        self.__start_speaking_event_name = config_data['Ros']['start_speaking_event_name']


    '''
        Transition actions named after to state machine convention
    '''

    #General transition action
    def on_transition(self, event: str, source: State, target: State):
        self.__logger.debug(f"on '{event}' from '{source.id}' to '{target.id}'")        
        return "on_transition"

    #Specific transition actions
    def on_silence_broken(self):
        self.__logger.info("Robot starts speaking.")

    def on_silence_persists(self):
        self.__logger.debug("Still not speaking.")

    def on_continues_speaking(self):
        self.__logger.debug("Still speaking.")

    def on_stopped_speaking(self):
        self.__logger.info("Robot stops speaking.")






    '''
        Wrapper
    '''

    def procBehavEvent(self, behav_event_flag):
        if(behav_event_flag == self.__start_speaking_event_name):
            self.is_speaking()
        else:
            self.is_not_speaking()


