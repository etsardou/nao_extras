#!/usr/bin/env python

#
# ROS node to serve NAO's ALMovementDetectionProxy's functionalities
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

from std_msgs.msg import String
from nao_interaction_msgs.msg import MovementDetected
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

class NaoMovementDetection(ALModule, NaoNode):
    "Sends callbacks for NAO's movement detection to ROS"
    def __init__(self, moduleName):
        # ROS initialization
        NaoNode.__init__(self)
        rospy.init_node( "nao_movement_detection" )
        
        # NAOQi initialization
        self.ip = ""
        self.port = 10600
        self.moduleName = moduleName
        self.init_almodule()
             
        # init. messages:
        self.movementPub = rospy.Publisher("movement_detected", MovementDetected)
        
        self.subscribe()
        
        rospy.loginfo("nao_movement_detection initialized")

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
        # TODO: check self.memProxy.version() for > 1.6
        if self.memProxy is None:
            rospy.logerror("Could not get a proxy to ALMemory on %s:%d", self.pip, self.pport)
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
        self.memProxy.subscribeToEvent("MovementDetection/MovementDetected", self.moduleName, "onMovementDetected")

    def unsubscribe(self):
        self.memProxy.unsubscribeToEvent("MovementDetection/MovementDetected", self.moduleName)

    def onMovementDetected(self, strVarName, value, strMessage):
        "Called when movement was detected"
        datavar = self.memProxy.getData("MovementDetection/MovementInfo")
        
        # For the specific fields in the datavar variable check here:
        # https://community.aldebaran-robotics.com/doc/1-14/naoqi/vision/almovementdetection.html
        
        movement = MovementDetected()  
        
        movement.gravity_center.x = datavar[0][0][0]
        movement.gravity_center.y = datavar[0][0][1]

        movement.mean_velocity.x = datavar[0][1][0]
        movement.mean_velocity.y = datavar[0][1][1]

        for i in range(0, len(datavar[0][2])):
          movement.points_poses.append(Point())
          movement.points_poses[i].x = datavar[0][2][i][0]
          movement.points_poses[i].y = datavar[0][2][i][1]
          
          movement.points_speeds.append(Point())
          movement.points_speeds[i].x = datavar[0][3][i][0]
          movement.points_speeds[i].y = datavar[0][3][i][1]
        
        self.movementPub.publish(movement)

if __name__ == '__main__':
  
    ROSNaoMovementDetectionModule = NaoMovementDetection("ROSNaoMovementDetectionModule")

    rospy.spin()
    
    rospy.loginfo("Stopping nao_movement_detection ...")
    ROSNaoMovementDetectionModule.shutdown();        
    rospy.loginfo("nao_movement_detection stopped.")
    exit(0)
