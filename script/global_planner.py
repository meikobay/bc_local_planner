#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped


import numpy as np
import math
import copy
import scipy.interpolate as spi

wps = np.loadtxt("/program/sim_ugv_ws/src/sim_ugv_master/4Path_Planning/bc_local_planner/script/waypoints.txt", dtype=np.float64, delimiter=',')
print(len(wps))


class Global_Planner:
    def __init__(self):
        rospy.init_node('global_planner', anonymous=True)
        self.pub_lg = rospy.Publisher('/cwp', PointStamped, queue_size=1)
        self.cwp_index = 0
        self.ugv_pose = np.zeros(2)
        self.gpath = Path()
        self.gpath.header.frame_id = '/map'
        self.listener = tf.TransformListener()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            self.gpath.poses = []
            self.ugv_pose[0] = trans[0]
            self.ugv_pose[1] = trans[1]
            if self.cwp_index < len(wps)-2:
                dist2c = self.cul_dist(self.ugv_pose, wps[self.cwp_index])
                dist2n = self.cul_dist(self.ugv_pose, wps[self.cwp_index+1])
                if dist2n < dist2c:
                    self.cwp_index += 1
                    print "current waypoint index: "+str(self.cwp_index)
            else:
                self.cwp_index = len(wps) - 2
                
            cwp = PointStamped()
            cwp.point.x = wps[self.cwp_index+1,0]
            cwp.point.y = wps[self.cwp_index+1,1]
            cwp.header.frame_id = "/map"
            cwp.header.stamp = rospy.Time.now()
            self.pub_lg.publish(cwp)      
            rate.sleep()        

    
    def cul_dist(self, p1, p2):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return np.sqrt(dx*dx + dy*dy)
    
    def get_lg(self):
#         dist_sum = 0.0
        lg_pose = PoseStamped()
        lg_pose.header.frame_id = '/map'
        lg_pose.header.stamp = rospy.Time.now()
        lg_index = 0
        head = 0.0
        for i in range(len(self.gpath.poses)-1):
            cg = self.gpath.poses[i]
            ng = self.gpath.poses[i+1]
            dx = ng.pose.position.x - cg.pose.position.x
            dy = ng.pose.position.y - cg.pose.position.y
#             dist = np.sqrt(dx*dx+dy*dy)
#             dist_sum += dist
            dist = self.cul_dist(self.ugv_pose, np.array([ng.pose.position.x, ng.pose.position.y]))
            lg_index = i+1
            if dist > 10.5:
                head = math.atan2(dy, dx)
                break
        lg_pose.pose.position = self.gpath.poses[lg_index].pose.position
        x, y, z, w = tf.transformations.quaternion_from_euler(0.0, 0.0, head)
        lg_pose.pose.orientation.x = x
        lg_pose.pose.orientation.y = y
        lg_pose.pose.orientation.z = z
        lg_pose.pose.orientation.w = w
        return lg_pose


if __name__ == '__main__':
    gp = Global_Planner()
    

