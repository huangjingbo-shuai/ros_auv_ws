#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Point  # 用于发布二维角度
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import time
from ultralytics import YOLO
import torch  # 用于检测 GPU 是否可用

class PersonAngleDetector:
    def __init__(self):
        rospy.init_node('person_angle_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # **发布器设置**
        self.angle_pub = rospy.Publisher("/person_angle", Float32, queue_size=1)  # 水平角度
        self.vertical_angle_pub = rospy.Publisher("/person_vertical_angle", Float32, queue_size=1)  # 垂直角度
        self.angles_pub = rospy.Publisher("/person_angles", Point, queue_size=1)  # 二维角度 (x=水平, y=垂直)
        self.image_pub = rospy.Publisher("/camera/image_detected", Image, queue_size=1, latch=True)

        # **选择使用 GPU 或 CPU**
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        rospy.loginfo(f"使用设备: {self.device}")

        # **加载 YOLO 模型**
        rospy.loginfo("加载 YOLO 模型中...")
        self.model = YOLO("/home/yunxia/ros_auv_ws/src/yolo/ultralytics/yolo11n.pt").to(self.device)
        rospy.loginfo("YOLO 模型加载完成！")

        # **订阅相机原始图像**
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
        rospy.loginfo("订阅 /camera/image_raw 话题")

        # **相机视场角设置**
        self.hfov = 90.0  # 水平视场角
        self.vfov = 60.0  # 垂直视场角（可根据实际相机参数调整）
        self.last_time = time.time()  # **初始化帧率计时器**

    def image_callback(self, data):
        """检测人体，计算水平和垂直角度，并发布带有目标框的图像，同时计算帧率"""
        start_time = time.time()  # **记录开始时间**

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # **ROS 转 OpenCV**
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # **YOLO 推理**
        results = self.model(cv_image)

        persons = []
        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])  # **类别索引**
                conf = float(box.conf[0])  # **置信度**
                if cls == 0 and conf > 0.5:  # **检测 "person"**
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    persons.append((x1, y1, x2, y2, conf))

        if not persons:
            rospy.loginfo("未检测到人")
            self.publish_image(cv_image)  # **发布原始图像**
            return

        # **选择置信度最高的人**
        person = max(persons, key=lambda x: x[4])
        x1, y1, x2, y2, conf = person
        
        # **计算目标框中心点**
        bbox_center_x = (x1 + x2) / 2.0
        bbox_center_y = (y1 + y2) / 2.0

        # **计算图像中心**
        height, width, _ = cv_image.shape
        image_center_x = width / 2.0
        image_center_y = height / 2.0

        # **计算水平偏离角度（左右）**
        focal_length_h = (width / 2.0) / math.tan(math.radians(self.hfov / 2.0))
        dx = bbox_center_x - image_center_x
        horizontal_angle_rad = math.atan(dx / focal_length_h)
        horizontal_angle_deg = math.degrees(horizontal_angle_rad)

        # **计算垂直偏离角度（上下）**
        focal_length_v = (height / 2.0) / math.tan(math.radians(self.vfov / 2.0))
        dy = image_center_y - bbox_center_y  # 上面为正，下面为负
        vertical_angle_rad = math.atan(dy / focal_length_v)
        vertical_angle_deg = math.degrees(vertical_angle_rad)

        # **打印和发布角度信息**
        rospy.loginfo("水平偏离角度：{:.2f}度，垂直偏离角度：{:.2f}度".format(horizontal_angle_deg, vertical_angle_deg))
        
        # 发布单独的角度
        self.angle_pub.publish(horizontal_angle_deg)  # 水平角度
        self.vertical_angle_pub.publish(vertical_angle_deg)  # 垂直角度
        
        # 发布组合角度
        angles_msg = Point()
        angles_msg.x = horizontal_angle_deg  # 水平角度
        angles_msg.y = vertical_angle_deg    # 垂直角度
        angles_msg.z = 0.0                   # 保留字段
        self.angles_pub.publish(angles_msg)

        # **绘制检测框和信息**
        # 绘制目标框
        cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        
        # 绘制中心点
        cv2.circle(cv_image, (int(bbox_center_x), int(bbox_center_y)), 5, (0, 0, 255), -1)
        
        # 绘制图像中心点（参考）
        cv2.circle(cv_image, (int(image_center_x), int(image_center_y)), 3, (255, 255, 0), -1)
        
        # 绘制中心线（参考）
        cv2.line(cv_image, (int(image_center_x), 0), (int(image_center_x), height), (255, 255, 0), 1)
        cv2.line(cv_image, (0, int(image_center_y)), (width, int(image_center_y)), (255, 255, 0), 1)
        
        # 显示角度信息
        cv2.putText(cv_image, "H-Angle: {:.2f} deg".format(horizontal_angle_deg), 
                    (int(x1), int(y1) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(cv_image, "V-Angle: {:.2f} deg".format(vertical_angle_deg), 
                    (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # **计算并显示 FPS**
        end_time = time.time()
        fps = 1.0 / (end_time - self.last_time)
        self.last_time = end_time  # **更新计时器**
        rospy.loginfo("FPS: {:.2f}".format(fps))

        cv2.putText(cv_image, "FPS: {:.2f}".format(fps), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        # **发布处理后的图像**
        self.publish_image(cv_image)

    def publish_image(self, cv_image):
        """将 OpenCV 图像转换为 ROS 图像并发布"""
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = PersonAngleDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass