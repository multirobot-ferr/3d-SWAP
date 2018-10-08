#!/usr/bin/python
#
# Copyright (c) 2017, University of Duisburg-Essen, swap-ferr
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of swap-ferr nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#


# @file swap_visualization.py
# @author Eduardo Ferrera
# @version 0.1
# @date    8/10/18
#
# @short: Performs a representation in RVIZ of what is happeing in 3D-SWAP
#
# This node is designed to connect to all available UAVs, print all their cilyinders and related information.

import rospy 
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from uav_avoidance.msg      import Announcement

class RepresentRobot:

    def __init__(self):
        rospy.loginfo("Representing robot")

        # Publication
        self._markerPub = rospy.Publisher('z/uav_model', Marker, queue_size=10, latch=True)
        
        self._publish()

        # Definitions
        self._body = MarkerArray()
        self._prepare_body()
        return

    def _publish(self):
        # Publishes all visualization parts of the UAV
        self._publish_shape()

    def _publish_shape(self):
        # Publishes the shape of a UAV
        pass
        
    def _prepare_body(self):
        # Prepares the body of a UAV
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "uav_body_main"
        marker.id = 0

        marker.type = Marker.MESH_RESOURCE
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0 
        marker.color.a = 1.0
        marker.pose.position.x  =  0
        marker.pose.position.y  =  0
        marker.pose.position.z  =  0
        # marker.pose.orientation =  [0,0,0]
        # Normalizing the .stl
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.mesh_resource = "package://avoidance_visualization/rviz/drone_model.stl"
        
        self._markerPub.publish(marker)




class RepresentationsManager:
    # Manages the representation of the robots

    def __init__(self):
        # Subscriptions
        rospy.Subscriber("/3d_swap/uavs_announcements", Announcement, self._announcement_cb)
        return

    def _announcement_cb(self, announcement_msg):
        rospy.loginfo(announcement_msg.uav_id)
        robot = RepresentRobot()

if __name__ == '__main__':
    rospy.init_node('swap_visualization', anonymous=True)
    rep_manager = RepresentationsManager()
    rospy.spin()