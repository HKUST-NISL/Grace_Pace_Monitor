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

#Load configs
def loadConfig(path):
    #Load configs
    with open(path, "r") as config_file:
        config_data = yaml.load(config_file, Loader=yaml.FullLoader)
        # print("Config file loaded")
    return config_data

#Created Logger
def setupLogger(file_log_level, terminal_log_level, logger_name, log_file_name):
    log_formatter = logging.Formatter('%(asctime)s %(msecs)03d %(name)s %(levelname)s %(funcName)s(%(lineno)d) %(message)s', 
                                  datefmt='%d/%m/%Y %H:%M:%S')

    f = open(log_file_name, "a")
    f.close()
    file_handler = logging.FileHandler(log_file_name)
    file_handler.setFormatter(log_formatter)
    file_handler.setLevel(file_log_level)

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(log_formatter)
    stream_handler.setLevel(terminal_log_level)

    logger = logging.getLogger(logger_name)
    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)
    logger.setLevel( min(file_log_level,terminal_log_level) )#set to lowest

    return logger

#Respond to exit signal
def handle_sigint(signalnum, frame):
    # terminate
    print('Main interrupted! Exiting.')
    sys.exit()

class PaceStateMachine(StateMachine):

    #States
    not_talking = State(initial=True)#not talking
    indefinite = State()#either talking or backchanneling
    talking = State()#talking

    #Transitions and Events
    heard_voice = (
        not_talking.to(indefinite, on="on_silence_broken")
        |
        indefinite.to(indefinite, on="on_keep_hearing", unless="true_talking")
        |
        indefinite.to(talking, on="on_should_be_talking", cond="true_talking")
        |
        talking.to(talking, on="on_continues_talking")
    )
    not_hearing_voice = (
        not_talking.to(not_talking, on="on_silence_persists")
        |
        indefinite.to(not_talking, on="on_should_not_be_talking", cond="true_silence")
        |
        indefinite.to(indefinite, on="on_keep_hearing", unless="true_silence")
        |
        talking.to(not_talking, on="on_stopped_talking", cond="true_silence")
        |
        talking.to(talking, on="on_continues_talking", unless="true_silence")
    )


    def __init__(self, min_talking_cnt, min_silence_cnt, logger):
        #FSM base class
        super(self.__class__, self).__init__(rtc=True)
        #Extra parameters
        self.__min_talking_cnt = min_talking_cnt
        self.__talking_cnt = 0
        self.__min_silence_cnt = min_silence_cnt
        self.__silence_cnt = 0
        self.__logger = logger.getChild(self.__class__.__name__)


        
    #General transition action
    def on_transition(self, event: str, source: State, target: State):
        self.__logger.debug(f"on '{event}' from '{source.id}' to '{target.id}'")        
        return "on_transition"

    #Specific transition actions
    def on_silence_persists(self):
        self.__logger.debug("Still not hearing anything.")
        
    def on_silence_broken(self):
        self.__talking_cnt = 0
        self.__logger.info("Heard human voices." )

    def on_keep_hearing(self):
        self.__logger.debug("Keep hearing voices." )

    def on_should_be_talking(self):
        self.__logger.info("The guy should be talking.")

    def on_should_not_be_talking(self):
        self.__logger.info("That's just bc / noise.")

    def on_continues_talking(self):
        self.__logger.debug("The guy is still talking.")

    def on_stopped_talking(self):
        self.__logger.info("The guy finished talking.")

    def true_silence(self, event_data):
        #Make the pace state less sensitive to pauses
        if(self.__silence_cnt >= self.__min_silence_cnt):
            return True
        else:
            return False

    def true_talking(self, event_data):
        #Make the pace state less sensitive to noise
        if( self.__talking_cnt >= self.__min_talking_cnt ):
            return True
        else:
            return False

    def on_heard_voice(self):
        self.__silence_cnt = 0
        self.__talking_cnt = self.__talking_cnt + 1
        self.__logger.debug("Silence cnt %d, talking cnt %d." % (self.__silence_cnt, self.__talking_cnt) )

    def on_not_hearing_voice(self):
        self.__silence_cnt = self.__silence_cnt + 1
        self.__talking_cnt = 0
        self.__logger.debug("Silence cnt %d, talking cnt %d." % (self.__silence_cnt, self.__talking_cnt) )

    def proc_vad_flag(self, vad_flag):
        if(vad_flag):
            self.heard_voice()
        else:
            self.not_hearing_voice()




class PaceMonitor:


    def __init__(self):
        #miscellaneous
        signal(SIGINT, handle_sigint)
        self.__logger = setupLogger(
                    logging.DEBUG, 
                    logging.INFO, 
                    self.__class__.__name__,
                    "./logs/log_" + datetime.now().strftime("%a_%d_%b_%Y_%I_%M_%S_%p"))

        path = "./config/config.yaml"
        self.__config_data = loadConfig(path)

        #ros, sensors
        self.__nh = rospy.init_node(self.__config_data['Ros']['node_name'])
        self.__vad_proc = utils.vad_proc.VADProc(
                                        self.__config_data['Main']['vad_freq'],
                                        self.__config_data['Ros']['vad_topic'],
                                        self.__logger)
        self.__asr_proc = utils.asr_proc.ASRProc(
                                        self.__config_data['Ros']['asr_interim_speech_topic'],
                                        self.__logger)

        #core pace state machine
        self.__it_freq = self.__config_data['Main']['pace_monitor_freq']

        self.__min_talking_dur = self.__config_data['Main']['min_talking_time']
        self.__min_talking_iteration = self.__it_freq * self.__min_talking_dur

        self.__min_silence_dur = self.__config_data['Main']['min_silence_time']
        self.__min_silence_iteration = self.__it_freq * self.__min_silence_dur
        self.__pace_fsm = PaceStateMachine(
                                self.__min_talking_iteration,
                                self.__min_silence_iteration,
                                self.__logger)

    def mainLoop(self):
        rate = rospy.Rate(self.__it_freq)

        while True:
            #Update state machine by vad flag
            self.__pace_fsm.proc_vad_flag(self.__vad_proc.vad_flag)
            rate.sleep()


if __name__ == '__main__':
    pace_monitor = PaceMonitor()
    pace_monitor.mainLoop()








