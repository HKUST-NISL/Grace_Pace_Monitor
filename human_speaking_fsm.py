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




class HumanSpeakingFSM(StateMachine):

    #States
    not_speaking = State(initial=True)#not speaking
    indefinite = State()#either speaking or backchanneling
    speaking = State()#speaking

    #Transitions and Events
    heard_voice = (
        not_speaking.to(indefinite, on="on_silence_broken")
        |
        indefinite.to(indefinite, on="on_keep_hearing", unless="true_speaking")
        |
        indefinite.to(speaking, on="on_should_be_speaking", cond="true_speaking")
        |
        speaking.to(speaking, on="on_continues_speaking")
    )
    not_hearing_voice = (
        not_speaking.to(not_speaking, on="on_silence_persists")
        |
        indefinite.to(not_speaking, on="on_should_not_be_speaking", cond="true_silence")
        |
        indefinite.to(indefinite, on="on_keep_hearing", unless="true_silence")
        |
        speaking.to(not_speaking, on="on_stopped_speaking", cond="true_silence")
        |
        speaking.to(speaking, on="on_continues_speaking", unless="true_silence")
    )


    def __init__(self, min_voice_cnt, min_silence_cnt, logger):
        #FSM base class
        super(self.__class__, self).__init__(rtc=True)
        #Extra parameters
        self.__min_voice_cnt = min_voice_cnt
        self.__voice_cnt = 0
        self.__min_silence_cnt = min_silence_cnt
        self.__silence_cnt = 0
        self.__logger = logger.getChild(self.__class__.__name__)





    '''
        Transition actions named after to state machine convention
    '''
        
    #General transition action
    def on_transition(self, event: str, source: State, target: State):
        self.__logger.debug(f"on '{event}' from '{source.id}' to '{target.id}'")        
        return "on_transition"

    #Specific transition actions
    def on_silence_persists(self):
        self.__logger.debug("Still not hearing anything.")
        
    def on_silence_broken(self):
        self.__voice_cnt = 0
        self.__logger.info("Heard human voices." )

    def on_keep_hearing(self):
        self.__logger.debug("Keep hearing voices." )

    def on_should_be_speaking(self):
        self.__logger.info("The guy should be speaking.")

    def on_should_not_be_speaking(self):
        self.__logger.info("That's just bc / noise.")

    def on_continues_speaking(self):
        self.__logger.debug("The guy is still speaking.")

    def on_stopped_speaking(self):
        self.__logger.info("The guy finished speaking.")

    def true_silence(self, event_data):
        #Make the pace state less sensitive to pauses
        if(self.__silence_cnt >= self.__min_silence_cnt):
            return True
        else:
            return False

    def true_speaking(self, event_data):
        #Make the pace state less sensitive to noise
        if( self.__voice_cnt >= self.__min_voice_cnt ):
            return True
        else:
            return False

    def on_heard_voice(self):
        self.__silence_cnt = 0
        self.__voice_cnt = self.__voice_cnt + 1
        self.__logger.debug("Silence cnt %d, voice cnt %d." % (self.__silence_cnt, self.__voice_cnt) )

    def on_not_hearing_voice(self):
        self.__silence_cnt = self.__silence_cnt + 1
        self.__voice_cnt = 0
        self.__logger.debug("Silence cnt %d, voice cnt %d." % (self.__silence_cnt, self.__voice_cnt) )





    '''
        Wrapper
    '''

    def procVadFlag(self, vad_flag):
        if(vad_flag):
            self.heard_voice()
        else:
            self.not_hearing_voice()


