#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class DiseaseDetector(Node):
    def __init__(self):
        super().__init__('disease_detector')
        
        # الاشتراك في كاميرا الروبوت
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # ناشر العلامات الخاصة بـ RViz
        self.marker_pub = self.create_publisher(MarkerArray, '/disease_markers', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info('Disease Detector Node has been started successfully.')

    def image_callback(self, msg):
        try:
            # تحويل صورة ROS إلى OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # معالجة بسيطة لاكتشاف الألوان (HSV Masking)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # نطاق اللون الأخضر (نباتات سليمة)
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        # عرض نافذة الكاميرا بشكل ثابت وبدون أخطاء في الحجم
        cv2.imshow('Camera Feed', cv_image)
        cv2.waitKey(1)

        # إنشاء علامات وهمية لعرضها في RViz كنموذج أولي للاختبار
        marker_array = MarkerArray()
        
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'diseases'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.2
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0  # أحمر (مريض)
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = DiseaseDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
