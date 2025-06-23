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
        self.debug_pub = rospy.Publisher('/debug_image', Image, queue_size=1)
        self.edge_pub = rospy.Publisher('/edge_image', Image, queue_size=1)  # 新增边缘图像发布
        
        # 图像处理参数
        self.gaussian_kernel = (5, 5)
        self.canny_thresholds = (50, 150)
        self.hough_threshold = 50
        self.min_line_length = 100
        self.max_line_gap = 20
        
        # 白线检测阈值
        self.lower_white = np.array([100, 100, 100])
        self.upper_white = np.array([255, 255, 255])
        
        # 竖直白线检测参数
        self.max_angle = np.radians(45)  # 只考虑±30度内的线
        self.min_vertical_length = 0.3   # 最小垂直长度比例
        
        # 跟踪线参数
        self.selected_line_color = (0, 0, 255)  # 红色
        self.vertical_lines_color = (0, 255, 0)  # 绿色
        
        rospy.loginfo("Vertical white line detector initialized")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return

        # 创建调试图像
        debug_img = cv_image.copy()
        
        # 图像处理流程
        processed_img = self.preprocess_image(cv_image)
        lines = self.detect_lines(processed_img)
        
        # 发布边缘图像
        try:
            edge_msg = self.bridge.cv2_to_imgmsg(processed_img, "mono8")
            self.edge_pub.publish(edge_msg)
        except Exception as e:
            rospy.logerr(f"Error publishing edge image: {e}")
        
        if lines is not None:
            # 计算每条线的角度并过滤垂直方向的线
            vertical_lines = []
            vertical_angles = []
            
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = self.calculate_line_angle(x1, y1, x2, y2)
                
                # 只考虑接近垂直的线
                if abs(angle) < self.max_angle:
                    # 计算线段的垂直长度比例
                    line_length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
                    min_y = min(y1, y2)
                    max_y = max(y1, y2)
                    vertical_length = max_y - min_y
                    vertical_ratio = vertical_length / line_length
                    
                    # 只考虑垂直部分占比大的线
                    if vertical_ratio > self.min_vertical_length:
                        vertical_lines.append(line)
                        vertical_angles.append(angle)
            
            # 绘制所有垂直白线并显示角度
            for i, line in enumerate(vertical_lines):
                x1, y1, x2, y2 = line[0]
                cv2.line(debug_img, (x1, y1), (x2, y2), self.vertical_lines_color, 2)
                
                # 在线段中点显示角度
                mid_x = (x1 + x2) // 2
                mid_y = (y1 + y2) // 2
                angle_deg = np.degrees(vertical_angles[i])
                angle_text = f"{abs(angle_deg):.1f}°"
                
                # 根据角度方向调整文本位置
                text_offset = 30 if angle_deg > 0 else -80
                cv2.putText(debug_img, angle_text, 
                            (mid_x + text_offset, mid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                            (0, 255, 255), 2)
            
            # 选择并跟踪一条线
            if vertical_lines:
                selected_line, selected_angle = self.select_line(vertical_angles, vertical_lines, cv_image.shape)
                
                if selected_line is not None:
                    x1, y1, x2, y2 = selected_line
                    cv2.line(debug_img, (x1, y1), (x2, y2), self.selected_line_color, 4)
                    
                    # 绘制方向指示器
                    center_x = debug_img.shape[1] // 2
                    center_y = debug_img.shape[0] - 50
                    angle_deg = np.degrees(selected_angle)
                    
                    # 计算方向箭头
                    end_x = center_x + int(100 * np.sin(selected_angle))
                    end_y = center_y - int(100 * np.cos(selected_angle))
                    cv2.arrowedLine(debug_img, (center_x, center_y), (end_x, end_y), 
                                   (255, 0, 0), 3, tipLength=0.3)
                    
                    # 在顶部显示选定线的角度
                    angle_text = f"Tracking: {angle_deg:.1f}°"
                    cv2.putText(debug_img, angle_text, (20, 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                (0, 0, 255), 2)
                    
                    # 发布选择的线角度
                    self.angle_pub.publish(selected_angle)
            else:
                rospy.logwarn("No vertical lines detected")
                cv2.putText(debug_img, "No vertical lines", (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            rospy.logwarn("No lines detected")
            cv2.putText(debug_img, "No lines detected", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # 发布调试图像
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            rospy.logerr(f"Error publishing debug image: {e}")

    def preprocess_image(self, img):
        # 颜色空间转换并提取白色区域
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

    def calculate_line_angle(self, x1, y1, x2, y2):
        # 计算线段与垂直方向的夹角
        dx = x2 - x1
        dy = y2 - y1
        
        # 确保线段方向一致（从下往上）
        if dy > 0:
            dx = -dx
            dy = -dy
        
        # 计算与垂直方向的夹角（弧度）
        if abs(dy) > 1e-5:  # 避免除以零
            return np.arctan2(dx, abs(dy))
        return 0

    def select_line(self, angles, lines, img_shape):
        if not angles:
            return None, None
        
        height, width = img_shape[:2]
        center_x = width // 2
        min_distance = float('inf')
        selected_angle = None
        selected_line = None
        
        # 选择最靠近图像中心的线
        for i, line in enumerate(lines):
            x1, y1, x2, y2 = line[0]
            mid_x = (x1 + x2) // 2
            distance = abs(mid_x - center_x)
            
            if distance < min_distance:
                min_distance = distance
                selected_angle = angles[i]
                selected_line = [x1, y1, x2, y2]
        
        return selected_line, selected_angle

if __name__ == '__main__':
    try:
        detector = WhiteLineDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass