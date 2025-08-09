#!/usr/bin/env python
# import rospy
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Point
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import math
# import time
# from ultralytics import YOLO
# import torch

# class PersonPixelDetector:
#     def __init__(self):
#         rospy.init_node('person_pixel_detector', anonymous=True)
#         self.bridge = CvBridge()
        
#         # **发布器设置**
#         self.pixel_pub = rospy.Publisher("/person_pixel", Point, queue_size=1)
#         self.image_pub = rospy.Publisher("/camera/image_detected", Image, queue_size=1, latch=True)

#         # **选择使用 GPU 或 CPU**
#         self.device = "cuda" if torch.cuda.is_available() else "cpu"
#         rospy.loginfo(f"使用设备: {self.device}")

#         # **加载 YOLO 模型**
#         rospy.loginfo("加载 YOLO 模型中...")
#         self.model = YOLO("/home/yunxia/ros_auv_ws/src/yolo/ultralytics/yolo11n.pt").to(self.device)
#         rospy.loginfo("YOLO 模型加载完成！")

#         # **订阅相机原始图像**
#         self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
#         rospy.loginfo("订阅 /camera/image_raw 话题")

#         # **相机标定参数**
#         self.camera_width = 640
#         self.camera_height = 480
#         self.camera_cx = 318.509118   # 相机光心X坐标
#         self.camera_cy = 247.737583   # 相机光心Y坐标
#         self.camera_fx = 410.971988   # X方向焦距
#         self.camera_fy = 412.393625   # Y方向焦距
        
#         self.last_time = time.time()
        
#         rospy.loginfo("人体像素检测器初始化完成")
#         rospy.loginfo(f"相机参数: 640x480, 光心({self.camera_cx:.1f},{self.camera_cy:.1f})")

#     def image_callback(self, data):
#         """检测人体，发布像素坐标"""
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr("CvBridge Error: {0}".format(e))
#             return

#         # **YOLO 推理**
#         results = self.model(cv_image)

#         persons = []
#         for result in results:
#             for box in result.boxes:
#                 cls = int(box.cls[0])
#                 conf = float(box.conf[0])
#                 if cls == 64 and conf > 0.2:  # 检测 "person"
#                     x1, y1, x2, y2 = box.xyxy[0].tolist()
#                     persons.append((x1, y1, x2, y2, conf))

#         if not persons:
#             self.publish_image(cv_image)
#             return

#         # **选择置信度最高的人**
#         person = max(persons, key=lambda x: x[4])
#         x1, y1, x2, y2, conf = person
        
#         # **计算目标框中心点像素坐标**
#         bbox_center_x = int((x1 + x2) / 2.0)
#         bbox_center_y = int((y1 + y2) / 2.0)

#         # **发布像素坐标**
#         pixel_msg = Point()
#         pixel_msg.x = float(bbox_center_x)
#         pixel_msg.y = float(bbox_center_y)
#         pixel_msg.z = conf
#         self.pixel_pub.publish(pixel_msg)

#         # **计算误差（用于显示）**
#         error_x = bbox_center_x - int(self.camera_cx)
#         error_y = bbox_center_y - int(self.camera_cy)

#         # **计算角度（用于显示）**
#         dx = bbox_center_x - self.camera_cx
#         horizontal_angle_deg = math.degrees(math.atan(dx / self.camera_fx))
#         dy = self.camera_cy - bbox_center_y
#         vertical_angle_deg = math.degrees(math.atan(dy / self.camera_fy))

#         # **打印信息**
#         rospy.loginfo("目标: ({}, {}), 误差: ({}, {}), 角度: ({:.1f}°, {:.1f}°)".format(
#             bbox_center_x, bbox_center_y, error_x, error_y, 
#             horizontal_angle_deg, vertical_angle_deg))

#         # **绘制检测结果**
#         # 绘制目标框
#         cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        
#         # 绘制目标中心点
#         cv2.circle(cv_image, (bbox_center_x, bbox_center_y), 8, (0, 0, 255), -1)
        
#         # 绘制相机光心点
#         cv2.circle(cv_image, (int(self.camera_cx), int(self.camera_cy)), 6, (255, 255, 0), -1)
        
#         # 绘制中心线
#         cv2.line(cv_image, (int(self.camera_cx), 0), (int(self.camera_cx), 480), (255, 255, 0), 1)
#         cv2.line(cv_image, (0, int(self.camera_cy)), (640, int(self.camera_cy)), (255, 255, 0), 1)
        
#         # 绘制连接线
#         cv2.line(cv_image, (int(self.camera_cx), int(self.camera_cy)), 
#                  (bbox_center_x, bbox_center_y), (255, 0, 255), 2)
        
#         # 显示信息
#         cv2.putText(cv_image, "Pixel: ({}, {})".format(bbox_center_x, bbox_center_y), 
#                     (int(x1), int(y1) - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
#         cv2.putText(cv_image, "Error: ({}, {})".format(error_x, error_y), 
#                     (int(x1), int(y1) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
#         cv2.putText(cv_image, "Angle: ({:.1f}, {:.1f})".format(horizontal_angle_deg, vertical_angle_deg), 
#                     (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

#         # **计算并显示 FPS**
#         end_time = time.time()
#         fps = 1.0 / (end_time - self.last_time)
#         self.last_time = end_time
#         cv2.putText(cv_image, "FPS: {:.1f}".format(fps), (10, 30),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

#         # **发布处理后的图像**
#         self.publish_image(cv_image)

#     def publish_image(self, cv_image):
#         """将 OpenCV 图像转换为 ROS 图像并发布"""
#         try:
#             ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
#             self.image_pub.publish(ros_image)
#         except CvBridgeError as e:
#             rospy.logerr("CvBridge Error: {0}".format(e))

#     def run(self):
#         rospy.loginfo("人体像素检测器开始运行...")
#         rospy.spin()

# if __name__ == '__main__':
#     try:
#         detector = PersonPixelDetector()
#         detector.run()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("人体像素检测器退出")
#         pass
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import time
import numpy as np

class ArucoPixelDetector:
    def __init__(self):
        rospy.init_node('aruco_pixel_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # **发布器设置**
        self.pixel_pub = rospy.Publisher("/aruco_pixel", Point, queue_size=1)
        self.image_pub = rospy.Publisher("/camera/image_detected", Image, queue_size=1, latch=True)

        # **ArUco检测器设置 - 5x5字典**
        rospy.loginfo("初始化 ArUco 检测器 (5x5字典)...")
        
        # 兼容不同版本的OpenCV
        try:
            # OpenCV 4.6.0 之后的新API
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            rospy.loginfo("使用新版本OpenCV ArUco API")
        except AttributeError:
            try:
                # OpenCV 4.6.0 之前的旧API
                self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
                self.aruco_params = cv2.aruco.DetectorParameters_create()
                rospy.loginfo("使用旧版本OpenCV ArUco API")
            except AttributeError:
                rospy.logerr("OpenCV ArUco模块不可用，请检查OpenCV安装")
                raise
        
        # 优化检测参数
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 53
        self.aruco_params.adaptiveThreshWinSizeStep = 4
        self.aruco_params.adaptiveThreshConstant = 7
        self.aruco_params.minMarkerPerimeterRate = 0.01
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        self.aruco_params.polygonalApproxAccuracyRate = 0.05
        self.aruco_params.minCornerDistanceRate = 0.01
        self.aruco_params.minDistanceToBorder = 1
        self.aruco_params.minMarkerDistanceRate = 0.01
        
        # 角点精化设置
        try:
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        except AttributeError:
            try:
                self.aruco_params.cornerRefinementMethod = cv2.aruco.CornerRefineMethod_SUBPIX
            except AttributeError:
                pass
        
        self.aruco_params.cornerRefinementWinSize = 5
        self.aruco_params.cornerRefinementMaxIterations = 30
        self.aruco_params.cornerRefinementMinAccuracy = 0.1
        
        rospy.loginfo("ArUco 检测器初始化完成！")

        # **订阅相机原始图像**
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
        rospy.loginfo("订阅 /camera/image_raw 话题")

        # **相机标定参数**
        self.camera_width = 640
        self.camera_height = 480
        self.camera_cx = 318.509118   # 相机光心X坐标
        self.camera_cy = 247.737583   # 相机光心Y坐标
        self.camera_fx = 410.971988   # X方向焦距
        self.camera_fy = 412.393625   # Y方向焦距
        
        # **目标ArUco ID = 0**
        self.target_aruco_id = 0
        
        self.last_time = time.time()
        
        rospy.loginfo("ArUco像素检测器初始化完成")
        rospy.loginfo(f"相机参数: 640x480, 光心({self.camera_cx:.1f},{self.camera_cy:.1f})")
        rospy.loginfo("使用ArUco字典: 5X5_50")
        rospy.loginfo(f"检测目标: ArUco ID = {self.target_aruco_id}")

    def image_callback(self, data):
        """检测ArUco码，发布像素坐标"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # **转换为灰度图像进行ArUco检测**
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # **对比度增强**
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced_gray = clahe.apply(gray)

        # **ArUco检测**
        try:
            # 尝试新版本API
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, rejected = detector.detectMarkers(enhanced_gray)
        except AttributeError:
            # 使用旧版本API
            corners, ids, rejected = cv2.aruco.detectMarkers(enhanced_gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None or len(ids) == 0:
            self.publish_image(cv_image)
            return

        # **查找目标ArUco码 ID=0**
        target_corners = None
        target_found = False
        
        for i, aruco_id in enumerate(ids):
            if aruco_id[0] == self.target_aruco_id:
                target_corners = corners[i]
                target_found = True
                break
        
        if not target_found:
            self.publish_image(cv_image)
            return

        # **计算ArUco码中心点像素坐标**
        corner_points = target_corners[0]
        center_x = np.mean(corner_points[:, 0])
        center_y = np.mean(corner_points[:, 1])
        
        bbox_center_x = int(center_x)
        bbox_center_y = int(center_y)

        # **发布像素坐标**
        pixel_msg = Point()
        pixel_msg.x = float(bbox_center_x)
        pixel_msg.y = float(bbox_center_y)
        pixel_msg.z = float(self.target_aruco_id)
        self.pixel_pub.publish(pixel_msg)

        # **计算误差（用于显示）**
        error_x = bbox_center_x - int(self.camera_cx)
        error_y = bbox_center_y - int(self.camera_cy)

        # **计算角度（用于显示）**
        dx = bbox_center_x - self.camera_cx
        horizontal_angle_deg = math.degrees(math.atan(dx / self.camera_fx))
        dy = self.camera_cy - bbox_center_y
        vertical_angle_deg = math.degrees(math.atan(dy / self.camera_fy))

        # **打印信息**
        rospy.loginfo("ArUco ID {}: ({}, {}), 误差: ({}, {}), 角度: ({:.1f}°, {:.1f}°)".format(
            self.target_aruco_id, bbox_center_x, bbox_center_y, error_x, error_y, 
            horizontal_angle_deg, vertical_angle_deg))

        # **绘制检测结果**
        # 绘制所有检测到的ArUco码
        if corners is not None and ids is not None:
            try:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            except:
                # 手动绘制
                for i, corner in enumerate(corners):
                    pts = corner[0].astype(np.int32)
                    cv2.polylines(cv_image, [pts], True, (255, 0, 0), 2)
                    center = np.mean(corner[0], axis=0).astype(int)
                    cv2.putText(cv_image, str(ids[i][0]), tuple(center), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        
        # **高亮显示目标ArUco码**
        if target_corners is not None:
            # 绘制目标ArUco码的绿色外框
            pts = target_corners[0].astype(np.int32)
            cv2.polylines(cv_image, [pts], True, (0, 255, 0), 4)
            
            # 绘制目标中心点
            cv2.circle(cv_image, (bbox_center_x, bbox_center_y), 10, (0, 0, 255), -1)
        
        # 绘制相机光心点
        cv2.circle(cv_image, (int(self.camera_cx), int(self.camera_cy)), 8, (255, 255, 0), -1)
        
        # 绘制中心线
        cv2.line(cv_image, (int(self.camera_cx), 0), (int(self.camera_cx), 480), (255, 255, 0), 2)
        cv2.line(cv_image, (0, int(self.camera_cy)), (640, int(self.camera_cy)), (255, 255, 0), 2)
        
        # 绘制连接线
        cv2.line(cv_image, (int(self.camera_cx), int(self.camera_cy)), 
                 (bbox_center_x, bbox_center_y), (255, 0, 255), 3)
        
        # **显示信息**
        cv2.putText(cv_image, "Target ArUco ID: {}".format(self.target_aruco_id), 
                    (bbox_center_x - 80, bbox_center_y - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(cv_image, "Pixel: ({}, {})".format(bbox_center_x, bbox_center_y), 
                    (bbox_center_x - 80, bbox_center_y - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(cv_image, "Error: ({}, {})".format(error_x, error_y), 
                    (bbox_center_x - 80, bbox_center_y - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(cv_image, "Angle: ({:.1f}, {:.1f})".format(horizontal_angle_deg, vertical_angle_deg), 
                    (bbox_center_x - 80, bbox_center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # **计算并显示 FPS**
        end_time = time.time()
        fps = 1.0 / (end_time - self.last_time)
        self.last_time = end_time
        cv2.putText(cv_image, "FPS: {:.1f}".format(fps), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        # **显示检测状态**
        cv2.putText(cv_image, "ArUco 5X5_50 - Target ID: 0", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(cv_image, "Status: DETECTED", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

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
        rospy.loginfo("ArUco像素检测器开始运行...")
        rospy.loginfo("正在搜索 5x5 ArUco码，ID=0...")
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = ArucoPixelDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ArUco像素检测器退出")
        pass