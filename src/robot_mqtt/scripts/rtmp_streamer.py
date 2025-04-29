#!/usr/bin/env python3
import rospy
import subprocess
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class RTMPStreamer:
    def __init__(self):
        self.bridge = CvBridge()
        self.ffmpeg_cmd = [
            'ffmpeg',
            '-f', 'rawvideo',      # 输入格式为原始视频
            '-pix_fmt', 'bgr24',   # OpenCV默认格式为BGR
            '-s', '640x480',       # 分辨率（需与图像一致）
            '-r', '30',            # 帧率
            '-i', '-',             # 从标准输入读取数据
            '-c:v', 'libx264',     # 编码器
            '-preset', 'fast',     # 编码预设
            '-f', 'flv',          # 输出格式为FLV
            'rtmp://tx.direct.huya.com/huyalive/1199574560753-1199574560753-7484892029064985869-2399149244962-10057-A-1742712467-1?seq=1745919923386&type=simple'  # RTMP服务器地址
        ]
        self.ffmpeg_proc = subprocess.Popen(self.ffmpeg_cmd, stdin=subprocess.PIPE)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.ffmpeg_proc.stdin.write(cv_image.tobytes())
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    rospy.init_node('rtmp_streamer')
    streamer = RTMPStreamer()
    rospy.Subscriber('/camera/color/image_raw', Image, streamer.image_callback)
    rospy.spin()