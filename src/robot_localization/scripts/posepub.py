import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
import random
import time

def create_pose():
    # 模拟小幅度变化的位姿
    position_x = 0.5325837662888886 + random.uniform(-0.1, 0.1)
    position_y = 0.047537752292013194 + random.uniform(-0.1, 0.1)
    position_z = 0.5283333659172058 + random.uniform(-0.1, 0.1)
    
    orientation_x = -0.3173872545667158 + random.uniform(-0.1, 0.1)
    orientation_y = -0.5003836154187378 + random.uniform(-0.1, 0.1)
    orientation_z = 0.6295694207998966 + random.uniform(-0.1, 0.1)
    orientation_w = 0.5025175742725408 + random.uniform(-0.1, 0.1)

    pose = Pose()
    pose.position.x = position_x
    pose.position.y = position_y
    pose.position.z = position_z

    pose.orientation.x = orientation_x
    pose.orientation.y = orientation_y
    pose.orientation.z = orientation_z
    pose.orientation.w = orientation_w

    return pose

def publish_pose():
    rospy.init_node('pose_publisher')
    pub = rospy.Publisher('/camera/aruco_102/pose', PoseStamped, queue_size=10)
    pub_1 = rospy.Publisher('/camera/aruco_103/pose', PoseStamped, queue_size=10)
    pub_2 = rospy.Publisher('/camera/aruco_104/pose', PoseStamped, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        header = Header()
        header.stamp = rospy.Time.now()
        header.seq += 1
        header.frame_id = "camera_link"

        pose = create_pose()

        pose_stamped = PoseStamped()
        pose_stamped.header = header
        pose_stamped.pose = pose

        pub.publish(pose_stamped)
        rospy.loginfo(f"Published Pose: {pose_stamped}")
        pub_1.publish(pose_stamped)
        rospy.loginfo(f"Published Pose_1: {pose_stamped}")
        pub_2.publish(pose_stamped)
        rospy.loginfo(f"Published Pose_2: {pose_stamped}")
        # 休眠以保持发布频率    
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass
