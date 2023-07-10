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

class RobotGazeFSM(StateMachine):

    #Time stamp of entering each state
    stamp_upon_entering = None
    is_transition = True

    #States
    following = State(initial=True)
    averting = State()

    #Events
    is_following = (
        following.to(following,on="on_gaze_following")
        |
        averting.to(following,on="on_gaze_following")
    )
    is_averting = (
        averting.to(averting,on="on_gaze_averting")
        |
        following.to(averting,on="on_gaze_averting")
    )


    is_staying = (
        averting.to(averting,on="on_gaze_averting")
        |
        following.to(following,on="on_gaze_following")
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

    def on_gaze_following(self):
        self.__logger.debug("Robot gaze following.")

    def on_gaze_averting(self):
        self.__logger.debug("Robot gaze averting.")


    '''
        Wrapper
    '''
    def procEvent(self, event_code):
        if(event_code == self.__config_data['General']['start_following_event_name']):
            self.is_following()
        elif(event_code == self.__config_data['General']['start_aversion_event_name']):
            self.is_averting()
        elif(event_code == self.__config_data['General']['empty_event_code']):
            self.is_staying()
        else:
            #Neutral is not handled
            pass

