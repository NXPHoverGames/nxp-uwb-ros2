#!/usr/bin/python3
# Copyright 2025 NXP
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rclpy
import numpy as np
from rclpy.node import Node
from uwb_msgs.msg import UltraWideBandRanging, TwoWayRanging
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_default

#ros run in terminal:
#ros2 run ros_gz_bridge parameter_bridge /uwb_twr_point@geometry_msgs/msg/Point@gz.msgs.Vector3d

class UWBDataGen(Node):

    def __init__(self):
        super().__init__('uwb_data_generator_node')
        
        self.uwb_point_topic = '/uwb_twr_point'
        self.uwb_pub_topic = '/sr1xx/twr_responder'
        self.UWBPub = self.create_publisher(UltraWideBandRanging,'{:s}'.format(self.uwb_pub_topic), 1)
        self.UWBSub = self.create_subscription(Point,'{:s}'.format(self.uwb_point_topic), self.callback, qos_profile_default)
        
        # Introduce std, multipath differs in real world. Smaller scale number makes bigger deviation.
        self.dist_std_scale=20.0
        self.az_std_scale=10.0
        self.el_std_scale=10.0

    def double_to_q9_7(self, val):
        val = int(val*(1<<7))
        if val < 0:
            val = (1<<16)+val
        return val

    def callback(self, msgPoint):
        msg = UltraWideBandRanging()
        dist= np.linalg.norm(np.array([msgPoint.x, msgPoint.y, msgPoint.z]))
        az_deg=np.rad2deg(np.arctan2(msgPoint.y, msgPoint.x))
        el_deg=np.rad2deg(np.arctan(msgPoint.z/np.linalg.norm(np.array([msgPoint.x, msgPoint.y]))))
        msgTWR = TwoWayRanging()
        msgTWR.aoa_azimuth=self.double_to_q9_7(az_deg-np.random.normal(0.0, np.abs(az_deg/self.az_std_scale), 1))
        msgTWR.aoa_elevation=self.double_to_q9_7(el_deg-np.random.normal(0.0, np.abs(el_deg/self.el_std_scale), 1))
        msgTWR.distance=int(np.abs(dist-np.random.normal(0.0, dist/self.dist_std_scale, 1))*100)
        msg.two_way_data.append(msgTWR)
        self.UWBPub.publish(msg)
        return

            
if __name__ == '__main__':
    rclpy.init()
    UWBDG = UWBDataGen()
    rclpy.spin(UWBDG)
    UWBDG.destroy_node()
    rclpy.shutdown()