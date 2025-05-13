#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class LaneDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.lane_offset = 0  # Lệch so với giữa làn (-1: lệch trái, 0: giữa, 1: lệch phải)
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback, queue_size=1)

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_img.shape
            roi = cv_img[h//2:, :]  # Chỉ lấy nửa dưới ảnh
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0,0,180), (180,50,255))
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                self.lane_offset = (cx - w//2) / (w//2)
            else:
                self.lane_offset = 0
            # Debug: hiển thị ảnh
            # cv2.imshow('mask', mask)
            # cv2.waitKey(1)
        except Exception as e:
            rospy.logwarn(f"Lỗi xử lý ảnh camera: {e}")

class LaneAndObstacleAvoidance:
    def __init__(self):
        rospy.init_node('lane_obstacle_avoidance', anonymous=True)
        
        # Khởi tạo publisher và subscriber
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)

        # Khởi tạo biến điều khiển
        self.cmd = Twist()
        self.linear_speed = 0.5
        self.safe_distance = 2.7  # Khoảng cách an toàn để tránh vật cản
        self.lane_k = 0.3         # Hệ số giữ làn
        self.car_width = 0.6      # Chiều rộng xe (m)
        self.margin = 0.1         # Biên an toàn nhỏ
        
        # Biến để lưu dữ liệu laser mới nhất
        self.latest_scan = None
        self.scan_lock = False

        # Trạng thái tránh vật cản
        self.avoid_state = 0  # 0: bình thường, 1: quay 45, 2: đi thẳng, 3: quay lại 45, 4: kết thúc tránh
        self.avoid_start_time = None
        self.avoid_direction = 0  # -1: trái, 1: phải
        self.avoid_step_time = 0

        # Khởi tạo vận tốc ban đầu
        self.cmd.linear.x = self.linear_speed
        self.cmd.angular.z = 0
        
        # Tích hợp camera
        self.lane_detector = LaneDetector()
        
        rospy.loginfo("Lane & Obstacle Avoidance node started")
        
        # Chạy vòng lặp điều khiển chính
        self.control_loop()

    def control_loop(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            if self.latest_scan is not None:
                self.process_scan()
            self.cmd_pub.publish(self.cmd)  # Luôn publish lệnh điều khiển
            rate.sleep()

    def scan_callback(self, data):
        if not self.scan_lock:
            self.latest_scan = data

    def process_scan(self):
        try:
            if self.latest_scan is None:
                return

            self.scan_lock = True
            
            data = self.latest_scan
            ranges = np.array(data.ranges)
            n = len(ranges)
            # Chia vùng quét
            center_width = n // 6  # 1/3 giữa
            center_idx = n // 2
            center_ranges = ranges[center_idx - center_width:center_idx + center_width]
            left_ranges = ranges[:center_idx - center_width]
            right_ranges = ranges[center_idx + center_width:]
            # Lọc giá trị hợp lệ
            center_valid = [r for r in center_ranges if data.range_min < r < data.range_max]
            left_valid = [r for r in left_ranges if data.range_min < r < data.range_max]
            right_valid = [r for r in right_ranges if data.range_min < r < data.range_max]

            # Nếu đang trong quá trình tránh vật cản
            if self.avoid_state > 0:
                self.avoid_obstacle_step()
                self.scan_lock = False
                return

            # 1. Phát hiện vật cản ở giữa
            if center_valid and min(center_valid) < self.safe_distance:
                min_center = min(center_valid)
                rospy.loginfo(f"Vật cản chắn đường! Khoảng cách: {min_center:.2f}m")
                # Quyết định hướng tránh dựa vào khoảng trống hai bên
                left_space = np.mean(left_valid) if left_valid else float('inf')
                right_space = np.mean(right_valid) if right_valid else float('inf')
                min_space = self.car_width + self.margin
                if left_space > right_space:
                    self.avoid_direction = 1  # Lách phải
                else:
                    self.avoid_direction = -1  # Lách trái
                # Bắt đầu chuỗi tránh vật cản
                self.avoid_state = 1
                self.avoid_start_time = time.time()
                self.avoid_step_time = 0
                rospy.loginfo(f"Bắt đầu tránh vật cản: quay 45 độ {'phải' if self.avoid_direction==1 else 'trái'}")
                self.avoid_obstacle_step()
                self.scan_lock = False
                return
            # 2. Nếu không có vật cản chắn đường, giữ làn bằng camera
            lane_offset = self.lane_detector.lane_offset
            self.cmd.linear.x = self.linear_speed
            self.cmd.angular.z = max(min(self.lane_k * lane_offset, 0.3), -0.3)
            rospy.loginfo(f"Giữ làn bằng camera - offset: {lane_offset:.2f}, linear: {self.cmd.linear.x:.2f}, angular: {self.cmd.angular.z:.2f}")
        except Exception as e:
            rospy.logerr(f"Lỗi trong xử lý scan: {str(e)}")
        finally:
            self.scan_lock = False

    def avoid_obstacle_step(self):
        # Thông số động tác tránh
        turn_angle = np.pi/4  # 45 độ
        turn_speed = 0.5      # rad/s
        move_dist = 2.0       # mét
        move_speed = 0.3      # m/s
        now = time.time()
        if self.avoid_state == 1:
            # Quay 45 độ
            duration = abs(turn_angle / turn_speed)
            if self.avoid_step_time == 0:
                self.avoid_step_time = now
            if now - self.avoid_step_time < duration:
                self.cmd.linear.x = 0
                self.cmd.angular.z = self.avoid_direction * turn_speed
                rospy.loginfo("Đang quay 45 độ để tránh vật cản")
            else:
                self.avoid_state = 2
                self.avoid_step_time = now
                rospy.loginfo("Quay xong, bắt đầu đi thẳng 2m")
        elif self.avoid_state == 2:
            # Tiến thẳng 2m
            duration = move_dist / move_speed
            if now - self.avoid_step_time < duration:
                self.cmd.linear.x = move_speed
                self.cmd.angular.z = 0
                rospy.loginfo("Đang tiến thẳng để vượt vật cản")
            else:
                self.avoid_state = 3
                self.avoid_step_time = now
                rospy.loginfo("Tiến xong, quay lại 45 độ về hướng cũ")
        elif self.avoid_state == 3:
            # Quay lại 45 độ về hướng cũ
            duration = abs(turn_angle / turn_speed)
            if now - self.avoid_step_time < duration:
                self.cmd.linear.x = 0
                self.cmd.angular.z = -self.avoid_direction * turn_speed
                rospy.loginfo("Đang quay lại 45 độ về hướng cũ")
            else:
                self.avoid_state = 0
                self.avoid_step_time = 0
                rospy.loginfo("Kết thúc tránh, trở lại giữ làn")
        else:
            self.avoid_state = 0
            self.avoid_step_time = 0
            rospy.loginfo("Kết thúc tránh, trở lại giữ làn")

if __name__ == '__main__':
    try:
        node = LaneAndObstacleAvoidance()
    except rospy.ROSInterruptException:
        pass