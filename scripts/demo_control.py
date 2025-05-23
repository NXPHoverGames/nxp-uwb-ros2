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
import time
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from synapse_msgs.msg import Status
from geometry_msgs.msg import PointStamped, PoseStamped, Pose, PoseWithCovarianceStamped
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from collections import deque
import statistics

class UWB_DEMO(Node):

    def __init__(self):
        super().__init__('uwb_demo_node')
        self.uwb_point=False
        self.point_direction_reverse=False
        self.loop_through_points=True
        self.intial_point_done=False
        self.goal_sent=False
        self.currentGoalIndex=0
        self.currentPose=PoseWithCovarianceStamped()
        self.currentGoalPose=Pose()
        self.previous_check_time=time.time()
        self.previousGoalPose=Pose()
        
        self.led_countdown = 0
        self.led_on=False

        self.msgJEmpty = Joy()
        self.msgJEmpty.axes = np.zeros(8, dtype=float)
        self.msgJEmpty.buttons = np.zeros(11, dtype=int)
        
        self.msgJOnLED = Joy()
        self.msgJOnLED.axes = np.zeros(8, dtype=float)
        self.msgJOnLED.buttons = np.zeros(11, dtype=int)
        self.msgJOnLED.axes[6]=-1.0
        
        self.msgJOffLED = Joy()
        self.msgJOffLED.axes = np.zeros(8, dtype=float)
        self.msgJOffLED.buttons = np.zeros(11, dtype=int)
        self.msgJOffLED.axes[6]=1.0
        
        self.msgJModeVel = Joy()
        self.msgJModeVel.axes = np.zeros(8, dtype=float)
        self.msgJModeVel.buttons = np.zeros(11, dtype=int)
        self.msgJModeVel.buttons[1]=1
        
        self.msgJTopicSrcEnet = Joy()
        self.msgJTopicSrcEnet.axes = np.zeros(8, dtype=float)
        self.msgJTopicSrcEnet.buttons = np.zeros(11, dtype=int)
        self.msgJTopicSrcEnet.buttons[5]=1
        
        self.current_status=Status()
        
        self.GoalAchievedThreshold=.5
        self.uwb_index = -1
        self.points=['5.0,-1.0', '7, 4.2', 'UWB_TWR', '0,0']
        
        self.nav2_goal_topic = '/goal_pose'
        self.nav2_pose_topic = '/pose'
        self.joy_topic = '/joy'
        self.stat_topic = '/cerebri/out/status'
        self.joyPub = self.create_publisher(Joy,'{:s}'.format(self.joy_topic), 10)
        self.statSub = self.create_subscription(Status,'{:s}'.format(self.stat_topic), self.callbackStat, qos_profile_default)
        self.goalPub = self.create_publisher(PoseStamped,'{:s}'.format(self.nav2_goal_topic), 1)
        self.poseSub = self.create_subscription(PoseWithCovarianceStamped,'{:s}'.format(self.nav2_pose_topic), self.callbackPose, qos_profile_default)
        self.timer = self.create_timer(0.1, self.input_routine)
        

        if 'UWB_TWR' in self.points:
            self.uwb_point=True
            self.uwb_index=self.points.index('UWB_TWR')
        if self.uwb_point:
            self.UWBPoseAvgX=deque()
            self.UWBPoseAvgY=deque()
            self.max_queue_size=20
            self.uwb_point_topic = '/point_stamped'
            self.UWBSub = self.create_subscription(PointStamped,'{:s}'.format(self.uwb_point_topic), self.callbackUWB, qos_profile_default)

    def input_routine(self):
        if self.led_countdown > 0:
            if self.led_on == True:
                self.joyPub.publish(self.msgJOffLED)
                self.led_on = False
            else:
                self.joyPub.publish(self.msgJOnLED)
                self.led_on = True
            self.led_countdown-=1
        elif self.current_status.mode != Status.MODE_VELOCITY:
            self.joyPub.publish(self.msgJModeVel)
        elif self.current_status.topic_source != Status.TOPIC_SOURCE_ETHERNET and self.current_status.mode == Status.MODE_VELOCITY:
            self.joyPub.publish(self.msgJTopicSrcEnet)
        else:
            self.joyPub.publish(self.msgJEmpty)
        return

    def callbackStat(self, msgStat):
        self.current_status=msgStat
        return

    def callbackPose(self, msgPose):
        self.currentPose=msgPose
        if not self.intial_point_done:
            self.currentGoalIndex=0
            goal_temp=np.fromstring(self.points[self.currentGoalIndex], dtype=float, sep=',')
            self.currentGoalPose.position.x=goal_temp[0]
            self.currentGoalPose.position.y=goal_temp[1]
        else:
            goal_prox = np.linalg.norm([(self.currentGoalPose.position.x-self.currentPose.pose.pose.position.x),
                                        (self.currentGoalPose.position.y-self.currentPose.pose.pose.position.y)])
            if goal_prox < self.GoalAchievedThreshold:
                self.led_countdown=10
                if self.currentGoalIndex < len(self.points)-1 and not self.point_direction_reverse:
                    self.currentGoalIndex+=1
                elif self.currentGoalIndex > 0 and self.point_direction_reverse and not self.loop_through_points:
                    self.currentGoalIndex-=1
                elif self.currentGoalIndex == 0 and self.point_direction_reverse and not self.loop_through_points:
                    self.currentGoalIndex+=1
                    self.point_direction_reverse=False
                elif self.currentGoalIndex == len(self.points)-1:
                    if not self.point_direction_reverse and not self.loop_through_points:
                        self.currentGoalIndex-=1
                        self.point_direction_reverse=True
                    elif self.loop_through_points:
                        self.currentGoalIndex=0
                        self.point_direction_reverse=False
                    else:
                        print("Failed loop logic")
                else:
                    print("Failed to iterate next goal")
                if self.currentGoalIndex != self.uwb_index:
                    goal_temp=np.fromstring(self.points[self.currentGoalIndex], dtype=float, sep=',')
                    self.currentGoalPose.position.x=goal_temp[0]
                    self.currentGoalPose.position.y=goal_temp[1]
                    self.previousGoalPose.position=msgPose.pose.pose.position
                    self.goal_sent=False
                elif self.uwb_point:
                    if len(self.UWBPoseAvgX) > 0 and len(self.UWBPoseAvgY) > 0:
                        self.currentGoalPose.position.x=statistics.mean(self.UWBPoseAvgX)
                        self.currentGoalPose.position.y=statistics.mean(self.UWBPoseAvgY)
                        self.previousGoalPose.position=msgPose.pose.pose.position
                        self.goal_sent=False
                    else:
                        print("No UWB Data yet, might be out of Range")
                else:
                    print("Unknown Goal state")
            
            prev_goal_progress = np.linalg.norm([(self.previousGoalPose.position.x-msgPose.pose.pose.position.x),
                                        (self.previousGoalPose.position.y-msgPose.pose.pose.position.y)])
            if (time.time()-self.previous_check_time) > 10.0:
                if prev_goal_progress < .75:
                    self.goal_sent=False
                    print("Failed to make progress sending goal again")
                self.previousGoalPose.position=msgPose.pose.pose.position
                self.previous_check_time=time.time()

        if not self.goal_sent:
            msgG = PoseStamped()
            msgG.header.frame_id='map'
            msgG.pose=self.currentGoalPose
            self.goalPub.publish(msgG)
            self.previous_command_time=time.time()
            self.intial_point_done=True
            self.goal_sent=True
        return
    
    def callbackUWB(self, msgUWB):
        self.UWBPoseAvgX.append(self.currentPose.pose.pose.position.x-msgUWB.point.y+.3)
        self.UWBPoseAvgY.append(self.currentPose.pose.pose.position.y+msgUWB.point.x-.3)
        if len(self.UWBPoseAvgX) > self.max_queue_size:
            self.UWBPoseAvgX.popleft()
        if len(self.UWBPoseAvgY) > self.max_queue_size:
            self.UWBPoseAvgY.popleft()
        return
            
if __name__ == '__main__':
    rclpy.init()
    UWBDN = UWB_DEMO()
    rclpy.spin(UWBDN)
    UWBDN.destroy_node()
    rclpy.shutdown()
