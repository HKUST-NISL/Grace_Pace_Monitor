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

class RobotHummingFSM(StateMachine):

    #Time stamp of entering each state
    stamp_upon_entering = None
    is_transition = True

    #States
    humming = State()#humming
    not_humming = State(initial=True)#not humming

    #Events
    is_humming = (
        not_humming.to(humming, on="on_silence_broken")
        |
        humming.to(humming, on="on_continues_humming")
    )
    is_not_humming = (
        not_humming.to(not_humming, on="on_silence_persists")
        |
        humming.to(not_humming, on="on_stopped_humming")
    )

    is_staying = (
        not_humming.to(not_humming,on="on_silence_persists")
        |
        humming.to(humming,on="on_continues_humming")
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
            self.is_transition = True
        else:
            self.is_transition = False        
        self.__logger.debug(f"on '{event}' from '{source.id}' to '{target.id}' @ %f" % (self.stamp_upon_entering) )        
        return "on_transition"

    #Specific transition actions
    def on_silence_broken(self):
        self.__logger.info("Robot starts humming.")

    def on_silence_persists(self):
        self.__logger.debug("Still not humming.")

    def on_continues_humming(self):
        self.__logger.debug("Still humming.")

    def on_stopped_humming(self):
        self.__logger.info("Robot stops humming.")






    '''
        Wrapper
    '''

    def procEvent(self, event_code):
        if(event_code == self.__config_data['BehavExec']['BehavEvent']['start_humming_event_name']):
            self.is_humming()
        elif(event_code == self.__config_data['BehavExec']['BehavEvent']['stop_humming_event_name']):
            self.is_not_humming()
        elif(event_code == self.__config_data['BehavExec']['BehavEvent']['empty_event_code']):
            self.is_staying()
        else:
            #Irrelevant event
            pass

