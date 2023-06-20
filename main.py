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


import utils.vad_proc
import utils.asr_proc

#Load configs
def loadConfig(path):
    #Load configs
    with open(path, "r") as config_file:
        config_data = yaml.load(config_file, Loader=yaml.FullLoader)
        print("Config file loaded")
    return config_data

#Created Logger
def setupLogger(log_level, logger_name, log_file_name):
    log_formatter = logging.Formatter('%(asctime)s %(name)s %(levelname)s %(funcName)s(%(lineno)d) %(message)s', 
                                  datefmt='%d/%m/%Y %H:%M:%S')

    f = open(log_file_name, "a")
    f.close()
    file_handler = logging.FileHandler(log_file_name)
    file_handler.setFormatter(log_formatter)
    file_handler.setLevel(log_level)

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(log_formatter)
    stream_handler.setLevel(log_level)

    logger = logging.getLogger(logger_name)
    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)
    logger.setLevel(log_level)

    return logger

#Respond to exit signal
def handle_sigint(signalnum, frame):
    # terminate
    print('Main interrupted! Exiting.')
    sys.exit()


class PaceMonitor:
    def __init__(self):
        signal(SIGINT, handle_sigint)

        self.logger = setupLogger(
                    logging.NOTSET, 
                    self.__class__.__name__,
                    "./logs/log_" + datetime.now().strftime("%a_%d_%b_%Y_%I_%M_%p"))

        path = "./config/config.yaml"
        self.config_data = loadConfig(path)

        self.nh = rospy.init_node(self.config_data['Ros']['node_name'])


        self.vad_proc = utils.vad_proc.VADProc(
                                        self.config_data['Main']['pace_monitor_freq'],
                                        self.config_data['Ros']['vad_topic'],
                                        self.logger)
        self.asr_proc = utils.asr_proc.ASRProc(
                                        self.config_data['Ros']['asr_interim_speech_topic'],
                                        self.logger)



        self.logger.info("test")
        rospy.spin()


if __name__ == '__main__':
    pace_monitor = PaceMonitor()









