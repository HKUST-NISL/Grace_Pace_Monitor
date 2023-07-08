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



class TurnOwnerFSM(StateMachine):

    #Time stamp of entering each state
    stamp_upon_entering = time.time()

    #States
    not_owned = State(initial=True)#no one owns the turn initially
    robot_turn = State()#The turn is owned by robot
    human_turn = State()#The turn is owned by human


    #Transitions and Events
    update_turn_ownership = (
        #Into and outof human turn (put on top so it has higer priority)
        not_owned.to(human_turn, on="on_human_taking_turn", cond="human_taking_turn")
        |
        human_turn.to(human_turn, on="on_human_holding_turn", unless="human_yielding_turn")
        |
        human_turn.to(not_owned, on="on_human_yielding_turn", cond="human_yielding_turn")
        |
        #Into and outof robot turn
        not_owned.to(robot_turn, on="on_robot_taking_turn", cond="robot_taking_turn")
        |
        robot_turn.to(robot_turn, on="on_robot_holding_turn", unless="robot_yielding_turn")
        |
        robot_turn.to(not_owned, on="on_robot_yielding_turn", cond="robot_yielding_turn")
        | 
        #No one owns the turn
        not_owned.to(not_owned, on="on_remain_not_ownered", unless=["robot_taking_turn","human_taking_turn"])
    )



    def __init__(
        self, 
        main_freq, 
        config_data, 
        human_speaking_fsm, 
        logger):
        #FSM base class
        super(self.__class__, self).__init__(rtc=True)

        #Extra parameters
        #Action names
        self.__robot_take_turn_action_name = config_data['Ros']['robot_take_turn_action_name']
        self.__robot_yield_turn_action_name = config_data['Ros']['robot_yield_turn_action_name']
        #Fsm handles
        self.__human_speaking_fsm_handle = human_speaking_fsm
        #Human not speaking counting
        self.__human_turn_max_not_speaking_cnt = main_freq * config_data['Main']['human_turn_max_not_speaking_time']
        self.__human_turn_not_speaking_cnt = 0


        self.__logger = logger.getChild(self.__class__.__name__)





    '''
        Transition actions named after to state machine convention
    '''
    #General transition action
    def on_transition(self, event: str, source: State, target: State):
        if(source != target):
            self.stamp_upon_entering = time.time()
        self.__logger.debug(f"on '{event}' from '{source.id}' to '{target.id}' @ %f" % (self.stamp_upon_entering) )        
        return "on_transition"
    
    #Specific transition actions
    def on_robot_taking_turn(self):
        self.__logger.info("Robot takes the turn.")
        
    def on_robot_holding_turn(self):
        self.__logger.debug("Robot is holding the turn.")

    def on_robot_yielding_turn(self):
        self.__logger.info("Robot yields the turn.")

    def on_human_taking_turn(self):
        self.__logger.info("Human takes the turn.")
        
    def on_human_holding_turn(self):
        self.__logger.debug("Human is holding the turn.")

    def on_human_yielding_turn(self):
        self.__logger.info("Human yields the turn.")

    def on_remain_not_ownered(self):
        self.__logger.debug("Turn is not owned.")



    '''
        Transition guards
        They are called BEFORE transition (and on-transition actions)
    '''
    def human_taking_turn(self):
        #If the human starts speaking when nobody owns the turn, then s/he takes the turn
        return self.__is_human_speaking()

    def human_yielding_turn(self):
        #If the human stops speaking for long enough in his / her turn, then s/he has yielded the turn
        return (self.__human_turn_not_speaking_cnt >= self.__human_turn_max_not_speaking_cnt)

    def robot_taking_turn(self):
        #We know exactly what the robot is doing
        return ( self.__current_turn_action == self.__robot_take_turn_action_name )

    def robot_yielding_turn(self):
        #We know exactly what the robot is doing
        return ( self.__current_turn_action == self.__robot_yield_turn_action_name )


    '''
        Event actions named after to state machine convention
        They are called BEFORE transition guards
    '''
    def on_update_turn_ownership(self):
        if(self.__is_human_speaking()):
            self.__human_turn_not_speaking_cnt = 0
        else:
            self.__human_turn_not_speaking_cnt = self.__human_turn_not_speaking_cnt + 1









    '''
        Wrapper
    '''
    def procTurnAction(self, current_turn_action):
        self.__current_turn_action = current_turn_action
        self.update_turn_ownership()

    def __is_human_speaking(self):
        return (self.__human_speaking_fsm_handle.current_state == self.__human_speaking_fsm_handle.speaking)
