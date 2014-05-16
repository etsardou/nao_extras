#!/usr/bin/env python

#
# ROS node to serve NAO's ALAudioRecorder's functionalities
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
from nao_interaction_msgs.srv import AudioRecorder
from std_srvs.srv import EmptyResponse

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
    NODE_NAME = "nao_audio_recorder"

class NaoAudioRecorderDetection(NaoNode):
    def __init__(self):
        # ROS initialization
        NaoNode.__init__(self)
        rospy.init_node( Constants.NODE_NAME )
        
        self.connectNaoQi()
        
        #~ ROS initializations
        self.enableRecordSrv = rospy.Service("nao_audio/record", AudioRecorder, self.handleRecorderSrv)
        
        rospy.loginfo(Constants.NODE_NAME + " initialized")

    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.audioRecorderProxy = self.getProxy("ALAudioRecorder")
        if self.audioRecorderProxy is None:
            exit(1)

    def handleRecorderSrv(self, req):
        
        file_path = req.file_path.data
        secs = req.secs.data
        channels = []
        
        #~ Channel setup
        if req.left_channel.data == True:
            channels.append(1)
        else:
            channels.append(0)
            
        if req.right_channel.data == True:
            channels.append(1)
        else:
            channels.append(0)
            
        if req.front_channel.data == True:
            channels.append(1)
        else:
            channels.append(0)
            
        if req.rear_channel.data == True:
            channels.append(1)
        else:
            channels.append(0)
        
        #~ Set audio type
        audio_type = ""
        if req.audio_type.data == 0:
            audio_type = "wav"
        elif req.audio_type.data == 1:
            audio_type = "ogg"
        else:
            return EmptyResponse
        
        #~ Set samplerate    
        samplerate = req.samplerate.data
        if samplerate <= 0:
            return EmptyResponse
        
        #~ Set recording type 
        if (secs <= 0.0):
            return EmptyResponse

        self.audioRecorderProxy.startMicrophonesRecording("/home/nao/" + file_path, audio_type, samplerate, channels)
        rospy.sleep(secs)        
        self.audioRecorderProxy.stopMicrophonesRecording()
        
        return EmptyResponse

if __name__ == '__main__':
  
    n = NaoAudioRecorderDetection()
    rospy.loginfo("Starting " + Constants.NODE_NAME + " ...")
    rospy.spin()
    rospy.loginfo("Stopping " + Constants.NODE_NAME + " ...")
    exit(0)
