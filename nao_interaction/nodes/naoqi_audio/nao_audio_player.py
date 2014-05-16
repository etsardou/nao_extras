#!/usr/bin/env python

#
# ROS node to serve NAO's ALAudioPlayer's functionalities
# This code is currently compatible to NaoQI version 1.6
#
# Copyright 2014 Manos Tsardoulias, CERTH/ITI
# http://www.ros.org/wiki/nao
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the CERTH/ITI nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import rospy
import naoqi

from nao_driver import NaoNode

from std_msgs.msg import String, Int32

#
# Notes:
# - A port number > 0 for the module must be specified.
#   If port 0 is used, a free port will be assigned automatically,
#   but naoqi is unable to pick up the assigned port number, leaving
#   the module unable to communicate with naoqi (1.10.52).
# 
# - Callback functions _must_ have a docstring, otherwise they won't get bound.
# 
# - Shutting down the broker manually will result in a deadlock,
#   not shutting down the broker will sometimes result in an exception 
#   when the script ends (1.10.52).
#

class Constants:
    NODE_NAME = "nao_audio_player"

class NaoAudioPlayerDetection(NaoNode):
    def __init__(self):
        # ROS initialization
        NaoNode.__init__(self)
        rospy.init_node( Constants.NODE_NAME )
        
        self.connectNaoQi()
        
        #~ ROS initializations
        #Subscribe to speech topic
        self.sub = rospy.Subscriber("nao_audio/play_file", String, self.playFile )
        
        rospy.loginfo(Constants.NODE_NAME + " initialized")

    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.audioPlayerProxy = self.getProxy("ALAudioPlayer")
        if self.audioPlayerProxy is None:
            exit(1)

    def playFile(self, req):
        self.audioPlayerProxy.playFile("/home/nao/" + req.data)
          

if __name__ == '__main__':
  
    n = NaoAudioPlayerDetection()
    rospy.loginfo("Starting " + Constants.NODE_NAME + " ...")
    rospy.spin()
    rospy.loginfo("Stopping " + Constants.NODE_NAME + " ...")
    exit(0)
