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
import time
import gz.transport13 as gzt
from gz.msgs10.logical_camera_image_pb2 import LogicalCameraImage
from gz.msgs10.vector3d_pb2 import Vector3d

def main():
    node = gzt.Node()
    topic_sub = "/uwb_twr_source"
    topic_pub = "/uwb_twr_point"
    pub_options = gzt.AdvertiseMessageOptions()
    pub = node.advertise(topic_pub, Vector3d, pub_options)

    def callback(msg):
        anchor=msg.pose
        for model in msg.model:
            if model.name=="b3rb":
                twr=Vector3d()
                twr=model.pose.position
                pub.publish(twr)
        return

    node.subscribe(topic=topic_sub, callback=callback, msg_type=LogicalCameraImage)

    # wait for shutdown
    try:
      while True:
        time.sleep(0.001)
    except KeyboardInterrupt:
      pass

if __name__ == "__main__":
    main()