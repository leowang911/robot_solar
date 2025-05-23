import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
import random
import time

def create_pose(pose_type):
    # 定义不同类型的位姿
    if pose_type == "center":
        position_x = 1.2325837662888886+ random.uniform(0, 0.2)
        position_y = 0.047537752292013194+ random.uniform(0, 0.1)
        position_z = 0.5283333659172058+ random.uniform(-0.1, 0.1)
        
        orientation_x = 0.979426071750607
        orientation_y = -0.025105682264710397
        orientation_z = 0.08636290509290838
        orientation_w = 0.3361425345354393

    elif pose_type == "right":
        position_x = 0.8325837662888886 + random.uniform(0, 0.2)
        position_y = 0.047537752292013194 + random.uniform(0, 0.1)
        position_z = 0.5283333659172058 + random.uniform(-0.1, 0.1)
        
        orientation_x = 0.979426071750607 + random.uniform(-0.1, 0.1)
        orientation_y = -0.025105682264710397 + random.uniform(-0.1, 0.1)
        orientation_z = 0.08636290509290838 + random.uniform(-0.1, 0.1)
        orientation_w = 0.3361425345354393 + random.uniform(-0.1, 0.1)

    elif pose_type == 'left': 
        position_x = 1.4325837662888886
        position_y = 0.047537752292013194
        position_z = 0.5283333659172058
        
        orientation_x = 0.979426071750607 
        orientation_y = -0.025105682264710397 
        orientation_z = 0.08636290509290838 
        orientation_w = 0.3361425345354393
    else:
        position_x = 1.5325837662888886 - random.uniform(0, 0.2)
        position_y = 0.047537752292013194 - random.uniform(0, 0.1)
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
    # side
    pub_3 = rospy.Publisher('/camera/aruco_100/pose', PoseStamped, queue_size=10)
    pub_4 = rospy.Publisher('/camera/aruco_101/pose', PoseStamped, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        header = Header()
        header.stamp = rospy.Time.now()
        header.seq += 1
        header.frame_id = "camera_link"

        pose = create_pose("left")
        pose_stamped = PoseStamped()
        pose_stamped.header = header
        pose_stamped.pose = pose

        pose_1 = create_pose("right")
        pose_stamped_1 = PoseStamped()
        pose_stamped_1.header = header
        pose_stamped_1.pose = pose_1

        pose_2 = create_pose("center")
        pose_stamped_2 = PoseStamped()
        pose_stamped_2.header = header
        pose_stamped_2.pose = pose_2

        pose_3 = create_pose("leftside")
        pose_stamped_3 = PoseStamped()
        pose_stamped_3.header = header
        pose_stamped_3.pose = pose_3

        pose_4 = create_pose("rightside")
        pose_stamped_4 = PoseStamped()
        pose_stamped_4.header = header
        pose_stamped_4.pose = pose_4

        # pub.publish(pose_stamped)
        # rospy.loginfo(f"Published Pose: {pose_stamped}")
        # pub_1.publish(pose_stamped_1)
        # rospy.loginfo(f"Published Pose_1: {pose_stamped_1}")
        # pub_2.publish(pose_stamped_2)
        # rospy.loginfo(f"Published Pose_2: {pose_stamped_2}")
        pub_3.publish(pose_stamped_3)
        rospy.loginfo(f"Published Pose_3: {pose_stamped_3}")
        # pub_4.publish(pose_stamped_4)
        # rospy.loginfo(f"Published Pose_4: {pose_stamped_4}")
        # 休眠以保持发布频率    
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass
