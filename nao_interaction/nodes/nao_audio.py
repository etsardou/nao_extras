#!/usr/bin/env python

#
# ROS node to serve NAOqi audio functionalities
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

from naoqi import ( 
    ALModule, 
    ALBroker, 
    ALProxy )

from nao_driver import (
    NaoNode)

from std_msgs.msg import (
    String, 
    Int32)

from std_srvs.srv import (
    EmptyResponse,
    Empty)

from nao_interaction_msgs.msg import (
    AudioSourceLocalization)

from nao_interaction_msgs.srv import (
    AudioMasterVolume,
    AudioRecorder)


class Constants:
    NODE_NAME = "nao_audio_interface"
    MODULE_NAME = "ROSNaoAudioModule"

class NaoAudioInterface(ALModule, NaoNode):
    def __init__(self, moduleName):
        # ROS initialization
        NaoNode.__init__(self)
        rospy.init_node( Constants.NODE_NAME )
        
        # NAOQi initialization
        self.ip = ""
        self.port = 10602
        self.moduleName = moduleName
        self.init_almodule()
        
        #~ ROS initializations
        self.playFileSubscriber = rospy.Subscriber("nao_audio/play_file", String, self.playFile )        
        self.masterVolumeSrv = rospy.Service("nao_audio/master_volume", AudioMasterVolume, self.handleAudioMasterVolumeSrv)
        self.enableRecordSrv = rospy.Service("nao_audio/record", AudioRecorder, self.handleRecorderSrv)
        #~ self.audioSourceLocalizationPub = rospy.Publisher("nao_audio/audio_source_localization", AudioSourceLocalization)
        
        self.subscribe()
        
        rospy.loginfo(Constants.NODE_NAME + " initialized")
  
    def init_almodule(self):
        # before we can instantiate an ALModule, an ALBroker has to be created
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        try:
            self.broker = ALBroker("%sBroker" % self.moduleName, self.ip, self.port, self.pip, self.pport)
        except RuntimeError,e:
            print("Could not connect to NaoQi's main broker")
            exit(1)
        ALModule.__init__(self, self.moduleName)
        
        self.audioPlayerProxy = ALProxy("ALAudioPlayer",self.pip,self.pport)
        if self.audioPlayerProxy is None:
            rospy.logerror("Could not get a proxy to ALAudioPlayer on %s:%d", self.pip, self.pport)
            exit(1)
            
        self.audioRecorderProxy = ALProxy("ALAudioRecorder",self.pip,self.pport)
        if self.audioRecorderProxy is None:
            rospy.logerror("Could not get a proxy to ALAudioRecorder on %s:%d", self.pip, self.pport)
            exit(1)
        
        self.audioDeviceProxy = ALProxy("ALAudioDevice",self.pip,self.pport)
        if self.audioDeviceProxy is None:
            rospy.logerror("Could not get a proxy to ALAudioDevice on %s:%d", self.pip, self.pport)
            exit(1)
            
        #~ self.audioSourceLocalizationProxy = ALProxy("ALAudioSourceLocalization",self.pip,self.pport)
        #~ if self.audioSourceLocalizationProxy is None:
            #~ rospy.logerror("Could not get a proxy to ALAudioSourceLocalization on %s:%d", self.pip, self.pport)
            #~ exit(1)
            #~ 
        #~ self.audioSourceLocalizationProxy.setParameter("EnergyComputation", 1)
        #~ self.audioSourceLocalizationProxy.setParameter("Sensibility", 0.5)

    def playFile(self, req):
        self.audioPlayerProxy.playFile("/home/nao/" + req.data)
        
    def handleAudioMasterVolumeSrv(self, req):
        if (req.master_volume.data < 0) or (req.master_volume.data > 100):
            return EmptyResponse
        
        self.audioDeviceProxy.setOutputVolume(req.master_volume.data)
        return EmptyResponse
    
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
    
    def shutdown(self): 
        self.unsubscribe()

    def subscribe(self):
        #~ self.memProxy.subscribeToEvent("ALAudioSourceLocalization/SoundLocated", self.moduleName, "onSoundLocated")
        pass

    def unsubscribe(self):
        #~ self.memProxy.unsubscribeToEvent("ALAudioSourceLocalization/SoundLocated", self.moduleName)
        pass
        
    def onSoundLocated(self, strVarName, value, strMessage):
        print value

if __name__ == '__main__':
  
    ROSNaoAudioModule = NaoAudioInterface(Constants.MODULE_NAME)
    rospy.spin()
    rospy.loginfo("Stopping ROSNaoAudioModule ...")
    ROSNaoAudioModule.shutdown();        
    rospy.loginfo("ROSNaoAudioModule stopped.")
    exit(0)
