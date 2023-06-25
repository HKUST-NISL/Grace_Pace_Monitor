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
import utils.tts_event_proc

#Load configs
def loadConfig(path):
    #Load configs
    with open(path, "r") as config_file:
        config_data = yaml.load(config_file, Loader=yaml.FullLoader)
        # print("Config file loaded")
    return config_data

#Create Logger
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
        self.__tts_event_proc = utils.tts_event_proc.TTSEventProc(
                                        self.__config_data['Ros']['tts_event_topic'],
                                        self.__logger)

        #core pace state fsm
        self.__pace_it_freq = self.__config_data['Main']['pace_monitor_freq']


        '''
            Talking state of the robot
        '''





        '''
            Talking state of the human interlocutor
        '''
        self.__human__min_talking_dur = self.__config_data['Main']['human_min_talking_time']
        self.__human_min_talking_iteration = self.__pace_it_freq * self.__human__min_talking_dur

        self.__human_min_silence_dur = self.__config_data['Main']['human_min_silence_time']
        self.__human_min_silence_iteration = self.__pace_it_freq * self.__human_min_silence_dur
        
        self.__human_talking_fsm = HumanTalkingFSM(
                                self.__human_min_talking_iteration,
                                self.__human_min_silence_iteration,
                                self.__logger)

        '''
            Turn ownership
        '''


    def mainLoop(self):
        rate = rospy.Rate(self.__pace_it_freq)

        while True:
            #Update robot talking state

            #Update human talking state
            self.__human_talking_fsm.proc_vad_flag(self.__vad_proc.vad_flag)
            
            #Update turn ownership


            #Sleep by rate
            rate.sleep()


if __name__ == '__main__':
    pace_monitor = PaceMonitor()
    pace_monitor.mainLoop()








