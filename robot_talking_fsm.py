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

class RobotTalkingFSM(StateMachine):

    #States
    not_talking = State(initial=True)#not talking
    talking = State()#talking

    #Events
    tts_is_playing = (
        not_talking.to(talking, on="on_silence_broken")
        |
        talking.to(talking, on="on_continues_talking")
    )
    tts_not_playing = (
        not_talking.to(not_talking, on="on_silence_persists")
        |
        talking.to(not_talking, on="on_stopped_talking")
    )

    def __init__(self, logger):
        #FSM base class
        super(self.__class__, self).__init__(rtc=True)
        #Logging
        self.__logger = logger.getChild(self.__class__.__name__)


    '''
        Transition actions named after to state machine convention
    '''

    #General transition action
    def on_transition(self, event: str, source: State, target: State):
        self.__logger.debug(f"on '{event}' from '{source.id}' to '{target.id}'")        
        return "on_transition"

    #Specific transition actions
    def on_silence_broken(self):
        self.__logger.info("TTS starts playing.")

    def on_silence_persists(self):
        self.__logger.debug("Still no tts playing.")

    def on_continues_talking(self):
        self.__logger.debug("Still playing tts.")

    def on_stopped_talking(self):
        self.__logger.info("TTS stops playing.")






    '''
        Wrapper
    '''

    def procTTSPlayingFlag(self, tts_playing_flag):
        if(tts_playing_flag):
            self.tts_is_playing()
        else:
            self.tts_not_playing()


