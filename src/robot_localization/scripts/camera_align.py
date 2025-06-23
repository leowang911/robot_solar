#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32

class WhiteLineDetector:
    def __init__(self):
        rospy.init_node('white_line_detector', anonymous=True)
        
        # 参数设置
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.angle_pub = rospy.Publisher('/selected_line_angle', Float32, queue_size=10)
        
        # 图像处理参数
        self.gaussian_kernel = (5, 5)
        self.canny_thresholds = (50, 150)
        self.hough_threshold = 50
        self.min_line_length = 100
        self.max_line_gap = 20
        
        # 白线检测阈值
        self.lower_white = np.array([200, 200, 200])
        self.upper_white = np.array([255, 255, 255])
        
        rospy.loginfo("White line detector initialized")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return

        # 图像处理流程
        processed_img = self.preprocess_image(cv_image)
        lines = self.detect_lines(processed_img)
        
        if lines is not None:
            angles = self.calculate_angles(lines, cv_image.shape)
            selected_angle = self.select_line(angles, lines, cv_image.shape)
            
            # 发布选择的线角度
            if selected_angle is not None:
                self.angle_pub.publish(selected_angle)
                rospy.loginfo(f"Selected line angle: {np.degrees(selected_angle):.2f} degrees")
        else:
            rospy.logwarn("No lines detected")

    def preprocess_image(self, img):
        # 颜色空间转换并提取白色区域
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img, self.lower_white, self.upper_white)
        white_only = cv2.bitwise_and(img, img, mask=mask)
        
        # 转换为灰度图并应用高斯模糊
        gray = cv2.cvtColor(white_only, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, self.gaussian_kernel, 0)
        
        # Canny边缘检测
        edges = cv2.Canny(blurred, *self.canny_thresholds)
        return edges

    def detect_lines(self, img):
        # 霍夫变换检测直线
        return cv2.HoughLinesP(
            img,
            rho=1,
            theta=np.pi/180,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )

    def calculate_angles(self, lines, img_shape):
        angles = []
        height, width = img_shape[:2]
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # 计算角度（相对于垂直方向）
            dx = x2 - x1
            dy = y2 - y1
            
            # 确保线段方向一致（从下往上）
            if dy > 0:
                dx = -dx
                dy = -dy
            
            # 计算与垂直方向的夹角
            angle = np.arctan2(dx, abs(dy))
            angles.append(angle)
            
        return angles

    def select_line(self, angles, lines, img_shape):
        if not angles:
            return None
        
        height, width = img_shape[:2]
        center_x = width // 2
        min_distance = float('inf')
        selected_angle = None
        
        # 选择最靠近图像中心的线
        for i, line in enumerate(lines):
            x1, y1, x2, y2 = line[0]
            mid_x = (x1 + x2) // 2
            
            # 只考虑图像下半部分的线
            if min(y1, y2) > height // 2:
                distance = abs(mid_x - center_x)
                
                if distance < min_distance:
                    min_distance = distance
                    selected_angle = angles[i]
        
        return selected_angle

if __name__ == '__main__':
    try:
        detector = WhiteLineDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass