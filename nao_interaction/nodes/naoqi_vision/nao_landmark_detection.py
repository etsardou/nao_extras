#!/usr/bin/env python

#
# ROS node to serve NAO's ALLandmarkDetectionProxy's functionalities
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

from naoqi import ( ALModule, ALBroker, ALProxy )
from nao_driver import NaoNode

from std_msgs.msg import String, Float32, Int32
from nao_interaction_msgs.msg import LandmarkDetected
from geometry_msgs.msg import Point

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

class NaoLandmarkDetection(ALModule, NaoNode):
    "Sends callbacks for NAO's landmark detection to ROS"
    def __init__(self, moduleName):
        # ROS initialization
        NaoNode.__init__(self)
        rospy.init_node( "nao_landmark_detection" )
        
        # NAOQi initialization
        self.ip = ""
        self.port = 10601
        self.moduleName = moduleName
        self.init_almodule()
             
        #~ initialize the publisher
        self.landmarkPub = rospy.Publisher("landmark_detected", LandmarkDetected)
        
        self.subscribe()
        
        rospy.loginfo("nao_landmark_detection initialized")

    def init_almodule(self):
        # before we can instantiate an ALModule, an ALBroker has to be created
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        try:
            self.broker = ALBroker("%sBroker" % self.moduleName, self.ip, self.port, self.pip, self.pport)
        except RuntimeError,e:
            print("Could not connect to NaoQi's main broker")
            exit(1)
        ALModule.__init__(self, self.moduleName)
        
        self.memProxy = ALProxy("ALMemory",self.pip,self.pport)
        if self.memProxy is None:
            rospy.logerror("Could not get a proxy to ALMemory on %s:%d", self.pip, self.pport)
            exit(1)
        self.landmarkDetectionProxy = ALProxy("ALLandMarkDetection",self.pip,self.pport)
        if self.landmarkDetectionProxy is None:
            rospy.logerror("Could not get a proxy to ALLandMarkDetection on %s:%d", self.pip, self.pport)
            exit(1)


    def shutdown(self): 
        self.unsubscribe()
        # Shutting down broker seems to be not necessary any more
        # try:
        #     self.broker.shutdown()
        # except RuntimeError,e:
        #     rospy.logwarn("Could not shut down Python Broker: %s", e)

    def subscribe(self):
        # Subscription to the FaceDetected event
        self.memProxy.subscribeToEvent("LandmarkDetected", self.moduleName, "onLandmarkDetected")

    def unsubscribe(self):
        self.memProxy.unsubscribeToEvent("LandmarkDetected", self.moduleName)

    def onLandmarkDetected(self, strVarName, value, strMessage):
        "Called when landmark was detected"
        
        if len(value) == 0:
            return
            
        # For the specific fields in the value variable check here:
        # https://community.aldebaran-robotics.com/doc/1-14/naoqi/vision/allandmarkdetection-api.html#landmarkdetected-value-structure
        
        msg = LandmarkDetected()  
        
        for i in range (0, len(value[1])):
            msg.shape_alpha.append(Float32(value[1][i][0][1]))
            msg.shape_beta.append(Float32(value[1][i][0][2]))
            msg.shape_sizex.append(Float32(value[1][i][0][3]))
            msg.shape_sizey.append(Float32(value[1][i][0][4]))
            msg.mark_ids.append(Int32(value[1][i][1][0]))
            
        msg.camera_local_pose.position.x = value[2][0]
        msg.camera_local_pose.position.y = value[2][1]
        msg.camera_local_pose.position.z = value[2][2]
        msg.camera_local_pose.orientation.x = value[2][3]
        msg.camera_local_pose.orientation.y = value[2][4]
        msg.camera_local_pose.orientation.z = value[2][5]
        
        msg.camera_world_pose.position.x = value[3][0]
        msg.camera_world_pose.position.y = value[3][1]
        msg.camera_world_pose.position.z = value[3][2]
        msg.camera_world_pose.orientation.x = value[3][3]
        msg.camera_world_pose.orientation.y = value[3][4]
        msg.camera_world_pose.orientation.z = value[3][5]
        
        msg.camera_name = String(value[4])
        
        self.landmarkPub.publish(msg)

if __name__ == '__main__':
  
    ROSNaoLandmarkDetectionModule = NaoLandmarkDetection("ROSNaoLandmarkDetectionModule")

    rospy.spin()
    
    rospy.loginfo("Stopping nao_landmark_detection ...")
    ROSNaoLandmarkDetectionModule.shutdown();        
    rospy.loginfo("nao_landmark_detection stopped.")
    exit(0)
