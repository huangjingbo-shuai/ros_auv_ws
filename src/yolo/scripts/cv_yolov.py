#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import time
from ultralytics import YOLO
import torch

class YoloPixelDetector:
    def __init__(self):
        rospy.init_node('yolo_pixel_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # **发布器设置**
        self.pixel_pub = rospy.Publisher("/yolo_pixel", Point, queue_size=1)
        self.image_pub = rospy.Publisher("/camera/image_detected", Image, queue_size=1, latch=True)

        # **选择使用 GPU 或 CPU**
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        rospy.loginfo(f"使用设备: {self.device}")

        # **YOLO模型配置 - 请根据您的需求修改以下参数**
        # 模型文件路径 - 请修改为您的YOLOv11模型路径
        self.model_path = "/home/yunxia/ros_auv_ws/src/yolo/ultralytics/best.pt"
        
        # 目标类别ID - 请修改为您训练模型中要检测的类别ID
        self.target_class_id = 0  # 例如：0表示第一个类别
        
        # 置信度阈值 - 请根据您的需求调整
        self.confidence_threshold = 0.5
        
        # **加载 YOLO 模型**
        rospy.loginfo("加载 YOLOv11 模型中...")
        try:
            self.model = YOLO(self.model_path).to(self.device)
            rospy.loginfo("YOLOv11 模型加载完成！")
        except Exception as e:
            rospy.logerr(f"YOLO模型加载失败: {e}")
            raise

        # **订阅相机压缩图像 - 修改为CompressedImage类型**
        self.image_sub = rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, self.image_callback, queue_size=1, buff_size=2**24)
        rospy.loginfo("订阅 /camera/image_raw/compressed 话题")

        # **相机标定参数**
        self.camera_width = 640
        self.camera_height = 480
        self.camera_cx = 318.509118   # 相机光心X坐标
        self.camera_cy = 247.737583   # 相机光心Y坐标
        self.camera_fx = 410.971988   # X方向焦距
        self.camera_fy = 412.393625   # Y方向焦距
        
        self.last_time = time.time()
        
        rospy.loginfo("YOLO像素检测器初始化完成")
        rospy.loginfo(f"相机参数: 640x480, 光心({self.camera_cx:.1f},{self.camera_cy:.1f})")
        rospy.loginfo(f"模型路径: {self.model_path}")
        rospy.loginfo(f"检测目标类别ID: {self.target_class_id}")
        rospy.loginfo(f"置信度阈值: {self.confidence_threshold}")

    def image_callback(self, data):
        """使用YOLO检测目标，发布像素坐标"""
        try:
            # **修改：使用compressed_imgmsg_to_cv2处理压缩图像**
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return
        except Exception as e:
            rospy.logerr("图像解压缩错误: {0}".format(e))
            return

        # **YOLO 推理**
        results = self.model(cv_image)

        # **提取目标检测结果**
        targets = []
        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                if cls == self.target_class_id and conf > self.confidence_threshold:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    targets.append((x1, y1, x2, y2, conf))

        if not targets:
            # 没有检测到目标时，只显示原图像
            self.add_status_info(cv_image, "未检测到目标")
            self.publish_image(cv_image)
            return

        # **选择置信度最高的目标**
        target = max(targets, key=lambda x: x[4])
        x1, y1, x2, y2, conf = target
        
        # **计算目标框中心点像素坐标**
        bbox_center_x = int((x1 + x2) / 2.0)
        bbox_center_y = int((y1 + y2) / 2.0)

        # **发布像素坐标**
        pixel_msg = Point()
        pixel_msg.x = float(bbox_center_x)
        pixel_msg.y = float(bbox_center_y)
        pixel_msg.z = conf  # 将置信度作为z值发布
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
        rospy.loginfo("目标类别{}: ({}, {}), 置信度: {:.2f}, 误差: ({}, {}), 角度: ({:.1f}°, {:.1f}°)".format(
            self.target_class_id, bbox_center_x, bbox_center_y, conf, error_x, error_y, 
            horizontal_angle_deg, vertical_angle_deg))

        # **绘制检测结果**
        self.draw_detection_results(cv_image, targets, bbox_center_x, bbox_center_y, 
                                   error_x, error_y, horizontal_angle_deg, vertical_angle_deg, conf)

        # **发布处理后的图像**
        self.publish_image(cv_image)

    def draw_detection_results(self, cv_image, targets, bbox_center_x, bbox_center_y, 
                              error_x, error_y, horizontal_angle_deg, vertical_angle_deg, best_conf):
        """绘制检测结果和相关信息"""
        
        # **绘制所有检测到的目标框（灰色）**
        for x1, y1, x2, y2, conf in targets:
            cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (128, 128, 128), 1)
            cv2.putText(cv_image, f"{conf:.2f}", (int(x1), int(y1) - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
        
        # **高亮显示最佳目标框（绿色）**
        best_target = max(targets, key=lambda x: x[4])
        x1, y1, x2, y2, conf = best_target
        cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 3)
        
        # **绘制目标中心点（红色）**
        cv2.circle(cv_image, (bbox_center_x, bbox_center_y), 10, (0, 0, 255), -1)
        
        # **绘制相机光心点（黄色）**
        cv2.circle(cv_image, (int(self.camera_cx), int(self.camera_cy)), 8, (255, 255, 0), -1)
        
        # **绘制中心线**
        cv2.line(cv_image, (int(self.camera_cx), 0), (int(self.camera_cx), 480), (255, 255, 0), 2)
        cv2.line(cv_image, (0, int(self.camera_cy)), (640, int(self.camera_cy)), (255, 255, 0), 2)
        
        # **绘制连接线（紫色）**
        cv2.line(cv_image, (int(self.camera_cx), int(self.camera_cy)), 
                 (bbox_center_x, bbox_center_y), (255, 0, 255), 3)
        
        # **显示目标信息**
        info_x = bbox_center_x - 100
        info_y = bbox_center_y - 100
        
        # 确保文字不超出图像边界
        if info_x < 0:
            info_x = 10
        if info_y < 50:
            info_y = bbox_center_y + 50
            
        cv2.putText(cv_image, f"Class ID: {self.target_class_id}", 
                    (info_x, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(cv_image, f"Confidence: {best_conf:.3f}", 
                    (info_x, info_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(cv_image, f"Pixel: ({bbox_center_x}, {bbox_center_y})", 
                    (info_x, info_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(cv_image, f"Error: ({error_x}, {error_y})", 
                    (info_x, info_y + 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(cv_image, f"Angle: ({horizontal_angle_deg:.1f}, {vertical_angle_deg:.1f})", 
                    (info_x, info_y + 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # **计算并显示 FPS**
        end_time = time.time()
        fps = 1.0 / (end_time - self.last_time)
        self.last_time = end_time
        cv2.putText(cv_image, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        # **显示检测状态**
        cv2.putText(cv_image, f"YOLOv11 - Class: {self.target_class_id}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(cv_image, "Status: DETECTED", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(cv_image, f"Targets: {len(targets)}", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    def add_status_info(self, cv_image, status="搜索中"):
        """在未检测到目标时添加状态信息"""
        # **计算并显示 FPS**
        end_time = time.time()
        fps = 1.0 / (end_time - self.last_time)
        self.last_time = end_time
        cv2.putText(cv_image, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        # **绘制相机光心点和中心线**
        cv2.circle(cv_image, (int(self.camera_cx), int(self.camera_cy)), 8, (255, 255, 0), -1)
        cv2.line(cv_image, (int(self.camera_cx), 0), (int(self.camera_cx), 480), (255, 255, 0), 1)
        cv2.line(cv_image, (0, int(self.camera_cy)), (640, int(self.camera_cy)), (255, 255, 0), 1)

        # **显示检测状态**
        cv2.putText(cv_image, f"YOLOv11 - Class: {self.target_class_id}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(cv_image, f"Status: {status}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(cv_image, f"Threshold: {self.confidence_threshold}", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    def publish_image(self, cv_image):
        """将 OpenCV 图像转换为 ROS 图像并发布"""
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def run(self):
        rospy.loginfo("YOLO像素检测器开始运行...")
        rospy.loginfo(f"正在使用YOLOv11检测类别ID={self.target_class_id}的目标...")
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = YoloPixelDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("YOLO像素检测器退出")
        pass