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
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import math
from uwb_msgs.msg import UltraWideBandRanging

class TWRPointPublisher(Node):

    def __init__(self):
        super().__init__('uwb_to_point')
        self.subscription = self.create_subscription(
            UltraWideBandRanging,
            '/sr1xx/twr_responder',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(PointStamped, '/point_stamped', 10)

    def q9_7_to_double(self, val):
        '''
            Initially taken from: 
            https://github.com/microsoft/nearobject-framework/blob/3dc13b123391d177595863e67998ab565e06874a/lib/uwb/UwbPeer.cxx#L52
            with painful realization that it's wrong and they should have implemented a basic: 
            https://github.com/microsoft/nearobject-framework/blob/3dc13b123391d177595863e67998ab565e06874a/lib/uwb/UwbPeer.cxx#L46
        '''
        signMask = 0b1000000000000000
        unsignedIntegerMask = 0b0111111110000000
        fractionMask = ~(signMask | unsignedIntegerMask)
        sign = val & signMask
        unsignedIntegerPart = (val & unsignedIntegerMask) >> 7
        fractionPart = val & fractionMask
        if sign:
            unsignedIntegerPart = (~unsignedIntegerPart + 1) & 0x00FF
            unsignedNumber = unsignedIntegerPart - fractionPart * 2**(-7)
            return -unsignedNumber
        else:
            unsignedNumber = unsignedIntegerPart + fractionPart * 2**(-7)
            return unsignedNumber


    def spherical_to_cartesian(self, distance, azimuth_deg, elevation_deg):
        # Convert angles from degrees to radians
        azimuth = math.radians(azimuth_deg)
        elevation = math.radians(elevation_deg)

        x = distance / 100.0 * math.cos(elevation) * math.cos(azimuth)
        y = distance / 100.0 * math.cos(elevation) * math.sin(azimuth)
        z = distance / 100.0 * math.sin(elevation)

        return x, y, z

    def listener_callback(self, msg):
        if len(msg.two_way_data) == 1 and msg.two_way_data[0].distance != 65535:
            x, y, z = self.spherical_to_cartesian(msg.two_way_data[0].distance,
                                             self.q9_7_to_double(msg.two_way_data[0].aoa_azimuth),
                                             self.q9_7_to_double(msg.two_way_data[0].aoa_elevation))
                                             
            point = PointStamped()
            point.point.x = x
            point.point.y = y
            point.point.z = z
            point.header.frame_id = "twr_responder"
            
            self.publisher_.publish(point)

def main(args=None):
    rclpy.init(args=args)

    point_publisher = TWRPointPublisher()

    rclpy.spin(point_publisher)

    point_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()