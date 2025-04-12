#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray

class VirtualMarkerGenerator:
    def __init__(self):
        self.pub = rospy.Publisher('/virtual_markers', PoseArray, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.update_markers)
        
        # 初始标记位置（base_link坐标系）
        self.markers = {
            100: [1.0, 0.5, 0],   # 左侧标记
            101: [1.0, -0.5, 0],  # 右侧标记
            102: [2.0, 0, 0]      # 中间标记
        }

    def update_markers(self, event):
        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        
        for marker_id, pos in self.markers.items():
            pose = PoseStamped().pose
            pose.position.x = pos[0]
            pose.position.y = pos[1]
            pose.position.z = pos[2]
            msg.poses.append(pose)
            
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('virtual_marker_generator')
    VirtualMarkerGenerator()
    rospy.spin()