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

file_path = os.path.dirname(os.path.realpath(getsourcefile(lambda:0)))
sys.path.append(file_path)

import utils.vad_proc
import utils.asr_proc
import utils.behav_event_proc
import utils.turn_action_proc
import robot_behav_fsm
import human_speaking_fsm
import robot_speaking_fsm
import robot_humming_fsm
import turn_owner_fsm

#Misc
sys.path.append(os.path.join(file_path, '..'))
from CommonConfigs.grace_cfg_loader import *
from CommonConfigs.logging import setupLogger

#Respond to exit signal
def handle_sigint(signalnum, frame):
    # terminate
    print('Main interrupted! Exiting.')
    sys.exit()

    
class InstantaneousStateMonitor:

    def __init__(self, config_data, logger = None, nh_in = None):
        #miscellaneous
        signal(SIGINT, handle_sigint)

        #Config
        self.__config_data = config_data

        #Monitor uses its own logger (output to its own dir) or the input logger
        if(logger == None):
            self.__logger = setupLogger(
                        logging.DEBUG, 
                        logging.INFO, 
                        self.__class__.__name__,
                        os.path.join(file_path,"./logs/log_") + datetime.now().strftime(self.__config_data['Custom']['Logging']['time_format']))
        else:
            self.__logger = logger.getChild(self.__class__.__name__)


        #ros, sensors
        if(nh_in == None):
            self.__nh = rospy.init_node(self.__config_data['InstState']['Ros']['node_name'])
        else:
            self.__nh = nh_in
            
        self.__vad_proc = utils.vad_proc.VADProc(
                                        self.__config_data,
                                        self.__logger)
        self.__asr_proc = utils.asr_proc.ASRProc(
                                        self.__config_data['HR']['ASRVAD']['asr_interim_speech_topic'],
                                        self.__logger)
        self.__behav_event_proc = utils.behav_event_proc.BehavEventProc(
                                        self.__config_data,
                                        self.__logger)
        self.__turn_action_proc = utils.turn_action_proc.TurnActionProc(
                                        self.__config_data,
                                        self.__logger)


        #core pace state fsm
        self.__pace_it_freq = self.__config_data['InstState']['Main']['test_freq']
        

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


        '''
            VAD config
        '''
        self.__vad_model_pub = rospy.Publisher(
                                    self.__config_data['Custom']['Sensors']['topic_vad_model'],
                                    std_msgs.msg.String,
                                    queue_size=self.__config_data['Custom']['Ros']['queue_size']
                                    )
        self.__vad_thresh_pub = rospy.Publisher(
                                    self.__config_data['Custom']['Sensors']['topic_vad_conf_thresh'],
                                    std_msgs.msg.Float32,
                                    queue_size=self.__config_data['Custom']['Ros']['queue_size']
                                    )

    def initVAD(self):
        self.__vad_model_pub.publish(std_msgs.msg.String(self.__config_data['Sensors']['PyannoteVAD']['ls_model_code']))
        self.__vad_thresh_pub.publish(self.__config_data['Sensors']['VAD']['conf_threshold'])


    def getState(self):
        inst_state = {
            **self.__robot_behav_fsm.getState() ,
            **{
                'human_speaking': {
                    'val': self.__human_speaking_fsm.current_state.id,
                    'stamp': self.__human_speaking_fsm.stamp_upon_entering,
                    'transition': self.__human_speaking_fsm.is_transition
                },
                'turn_ownership': {
                    'val': self.__turn_owner_fsm.current_state.id,
                    'stamp': self.__turn_owner_fsm.stamp_upon_entering,
                    'transition': self.__turn_owner_fsm.is_transition,
                    'from': self.__turn_owner_fsm.from_state
                }
            }
        }
        return inst_state

    def updateState(self):
        #Update robot behavioral state
        self.__robot_behav_fsm.procEvent(self.__behav_event_proc)

        #Update vad model (ugly)
        if( self.__robot_behav_fsm._robot_humming_fsm.current_state == robot_humming_fsm.RobotHummingFSM.humming
           or
            self.__robot_behav_fsm._robot_speaking_fsm.current_state == robot_speaking_fsm.RobotSpeakingFSM.speaking
           or
            self.__turn_owner_fsm.current_state == turn_owner_fsm.TurnOwnerFSM.robot_turn):
            #Use less sensitive vad model
            self.__vad_model_pub.publish(std_msgs.msg.String(self.__config_data['Sensors']['PyannoteVAD']['ls_model_code']))
        else:
            #Use more sensitive vad model
            self.__vad_model_pub.publish(std_msgs.msg.String(self.__config_data['Sensors']['PyannoteVAD']['ms_model_code']))



        #Update human speaking state
        self.__human_speaking_fsm.procVadEvent(self.__vad_proc.readSpeechFlag())

        #Update vad config 
        if(self.__human_speaking_fsm.is_transition):
            if(
                self.__human_speaking_fsm.current_state == human_speaking_fsm.HumanSpeakingFSM.indefinite
                or
                self.__human_speaking_fsm.current_state == human_speaking_fsm.HumanSpeakingFSM.speaking
            ):
                #Make it easier to go into and stay within speaking state
                self.__vad_thresh_pub.publish(self.__config_data['Sensors']['VAD']['conf_threshold_alt'])
            elif(self.__human_speaking_fsm.current_state == human_speaking_fsm.HumanSpeakingFSM.not_speaking):
                #Make it hard to trigger initially
                self.__vad_thresh_pub.publish(self.__config_data['Sensors']['VAD']['conf_threshold'])
            else:
                self.__logger.error('Unexpected speaking state.')


        #Update turn ownership
        self.__turn_owner_fsm.procTurnAction(self.__turn_action_proc.readLatestAction())

    def initializeState(self):
        #Force reset the starting time of the initial states
        self.__robot_behav_fsm.initializeState()
        self.__human_speaking_fsm.initializeState()
        self.__turn_owner_fsm.initializeState()


    def mainLoop(self):#For debugging
        rate = rospy.Rate(self.__pace_it_freq)
        self.initializeState()
        while True:
            #Update pace state
            self.updateState()

            #Sleep by rate
            rate.sleep()


if __name__ == '__main__':

    grace_config = loadGraceConfigs()
    inst_state_monitor = InstantaneousStateMonitor(grace_config)
    inst_state_monitor.mainLoop()








