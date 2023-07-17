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

    #Time stamp of entering each state
    stamp_upon_entering = None
    is_transition = True

    #States
    not_speaking = State(initial=True)#not speaking
    indefinite = State()#either speaking or backchanneling
    speaking = State()#speaking

    #Transitions and Events
    heard_speech = (
        not_speaking.to(indefinite, on="on_silence_broken")
        |
        indefinite.to(indefinite, on="on_keep_hearing", unless="true_speaking")
        |
        indefinite.to(speaking, on="on_should_be_speaking", cond="true_speaking")
        |
        speaking.to(speaking, on="on_continues_speaking")
    )
    not_heard_speech = (
        not_speaking.to(not_speaking, on="on_silence_persists")
        |
        indefinite.to(not_speaking, on="on_should_not_be_speaking", cond="true_not_speaking")
        |
        indefinite.to(indefinite, on="on_keep_hearing", unless="true_not_speaking")
        |
        speaking.to(not_speaking, on="on_stopped_speaking", cond="true_not_speaking")
        |
        speaking.to(speaking, on="on_continues_speaking", unless="true_not_speaking")
    )


    def __init__(self, main_freq, config_data, logger):
        #FSM base class
        super(self.__class__, self).__init__(rtc=True)

        #Export state diagram
        self._graph().write_png( os.path.join(
            config_data['Custom']['IO']['image_path'],
            self.__class__.__name__ + '.jpg'
            ))

        #Extra parameters
        self.__true_speaking_cnt_threshold = main_freq * config_data['InstState']['Main']['true_speaking_threshold_sec']

        self.__true_silence_cnt_threshold = main_freq * config_data['InstState']['Main']['true_silence_threshold_sec']
        
        self.__logger = logger.getChild(self.__class__.__name__)

        self.initializeState()



    def initializeState(self):
        self.__speech_cnt = 0
        self.__not_speech_cnt = 0
        self.current_state = self.not_speaking
        self.stamp_upon_entering = time.time()
        self.is_transition = True


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
    def on_silence_persists(self):
        self.__logger.debug("Still not speaking.")
        
    def on_silence_broken(self):
        # self.__speech_cnt = 0
        self.__logger.info("Heard human speaking." )

    def on_keep_hearing(self):
        self.__logger.info("Keep hearing speech." )

    def on_should_be_speaking(self):
        self.__logger.info("The guy should be truly speaking.")

    def on_should_not_be_speaking(self):
        self.__logger.info("That's just bc / noise / silence.")

    def on_continues_speaking(self):
        self.__logger.info("The guy is still speaking.")

    def on_stopped_speaking(self):
        self.__logger.info("The guy finished speaking.")




    '''
        Transition guards
        They are called BEFORE transition (and on-transition actions)
    '''

    def true_not_speaking(self, event_data):
        #Make the pace state less sensitive to pauses
        if(self.__not_speech_cnt >= self.__true_silence_cnt_threshold):
            return True
        else:
            return False

    def true_speaking(self, event_data):
        #Make the pace state less sensitive to noise
        if( self.__speech_cnt >= self.__true_speaking_cnt_threshold ):
            return True
        else:
            return False




    '''
        Event actions named after to state machine convention
        They are called BEFORE transition guards
    '''
    def on_heard_speech(self):
        self.__not_speech_cnt = 0
        self.__speech_cnt = self.__speech_cnt + 1
        self.__logger.info("Non-speech cnt %d, speech cnt %d." % (self.__not_speech_cnt, self.__speech_cnt) )

    def on_not_heard_speech(self):
        self.__not_speech_cnt = self.__not_speech_cnt + 1
        self.__speech_cnt = 0
        self.__logger.debug("Non-speech cnt %d, speech cnt %d." % (self.__not_speech_cnt, self.__speech_cnt) )



    '''
        Wrapper
    '''

    def procVadEvent(self, speech_flag):
        if(speech_flag):
            #Has speech flag
            self.heard_speech()
        else:
            #No speech flag
            self.not_heard_speech()



