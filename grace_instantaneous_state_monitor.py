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
from statemachine import StateMachine, State
import utils.vad_proc
import utils.asr_proc
import utils.behav_event_proc
import utils.turn_action_proc
import robot_behav_fsm
import human_speaking_fsm
import turn_owner_fsm


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




class InstantaneousStateMonitor:

    def __init__(self, nh_in = None):
        #miscellaneous
        signal(SIGINT, handle_sigint)
        self.__logger = setupLogger(
                    logging.DEBUG, 
                    logging.INFO, 
                    self.__class__.__name__,
                    "./logs/log_" + datetime.now().strftime("%a_%d_%b_%Y_%I_%M_%S_%p"))

        path = os.path.dirname(os.path.realpath(getsourcefile(lambda:0))) + "/config/config.yaml"
        self.__config_data = loadConfig(path)

        #ros, sensors
        if(nh_in == None):
            self.__nh = rospy.init_node(self.__config_data['Ros']['node_name'])
        else:
            self.__nh = nh_in
            
        self.__vad_proc = utils.vad_proc.VADProc(
                                        self.__config_data['Main']['vad_freq'],
                                        self.__config_data['Ros']['vad_topic'],
                                        self.__logger)
        self.__asr_proc = utils.asr_proc.ASRProc(
                                        self.__config_data['Ros']['asr_interim_speech_topic'],
                                        self.__logger)
        self.__behav_event_proc = utils.behav_event_proc.BehavEventProc(
                                        self.__config_data,
                                        self.__logger)
        self.__turn_action_proc = utils.turn_action_proc.TurnActionProc(
                                        self.__config_data['Ros']['turn_action_topic'],
                                        self.__logger)


        #core pace state fsm
        self.__pace_it_freq = self.__config_data['Main']['pace_monitor_freq']
        

        '''
            Behavioral state of the robot
        ''' 
        self.__robot_behav_fsm = robot_behav_fsm.RobotBehavFSM(
                                self.__config_data,
                                self.__logger)



        '''
            Speaking state of the human interlocutor
        '''
        self.__human_speaking_fsm = human_speaking_fsm.HumanSpeakingFSM(
                                self.__pace_it_freq,
                                self.__config_data,
                                self.__logger)

        '''
            Turn ownership
        '''
        self.__turn_owner_fsm = turn_owner_fsm.TurnOwnerFSM(
                                    self.__pace_it_freq,
                                    self.__config_data,
                                    self.__human_speaking_fsm,
                                    self.__logger)

    def getState(self):
        inst_state = self.__robot_behav_fsm.getState() | {
            'human_speaking': {
                'val': self.__human_speaking_fsm.current_state.id,
                'stamp': self.__human_speaking_fsm.stamp_upon_entering
            },
            'turn_ownership': {
                'val': self.__turn_owner_fsm.current_state.id,
                'stamp': self.__turn_owner_fsm.stamp_upon_entering
            }
        }
        return inst_state

    def updateState(self):
        #Update robot behavioral state
        self.__robot_behav_fsm.procEvent(self.__behav_event_proc)

        #Update human speaking state
        self.__human_speaking_fsm.procVadFlag(self.__vad_proc.vad_flag)
        
        #Update turn ownership
        self.__turn_owner_fsm.procTurnAction(self.__turn_action_proc.current_action)

    def mainLoop(self):
        rate = rospy.Rate(self.__pace_it_freq)

        while True:
            #Update pace state
            self.updateState()

            #Sleep by rate
            rate.sleep()


if __name__ == '__main__':
    inst_state_monitor = InstantaneousStateMonitor()
    inst_state_monitor.mainLoop()








