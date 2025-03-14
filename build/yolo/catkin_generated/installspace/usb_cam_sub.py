#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class UsbCameraPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('usb_camera_publisher', anonymous=True)
        
        # 创建图像发布者，发布到 /camera/image_raw 话题
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        
        # 创建CvBridge，用于OpenCV图像和ROS图像消息之间的转换
        self.bridge = CvBridge()
        
        # 打开USB摄像头，参数0表示默认摄像头，可以更改为其他数字尝试其他摄像头
        self.camera_id = rospy.get_param('~camera_id', 0)
        self.cap = cv2.VideoCapture(self.camera_id)
        
        # 设置摄像头分辨率
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        
        # 检查摄像头是否成功打开
        if not self.cap.isOpened():
            rospy.logerr("无法打开USB摄像头，请检查设备连接")
            return
        
        # 设置发布频率
        self.rate = rospy.Rate(rospy.get_param('~rate', 30))  # 默认30Hz
        
        rospy.loginfo("USB摄像头发布节点已启动")
    
    def publish_frames(self):
        """读取摄像头帧并发布为ROS话题"""
        while not rospy.is_shutdown():
            # 从摄像头读取一帧
            ret, frame = self.cap.read()
            
            if not ret:
                rospy.logwarn("无法获取视频帧，尝试重新连接摄像头...")
                self.cap.release()
                self.cap = cv2.VideoCapture(self.camera_id)
                continue
            
            try:
                # 将OpenCV图像转换为ROS图像消息
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                
                # 添加时间戳
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = "camera_frame"
                
                # 发布图像
                self.image_pub.publish(ros_image)
                
            except CvBridgeError as e:
                rospy.logerr("CvBridge转换错误: %s", e)
            
            # 按照设置的频率发布
            self.rate.sleep()
    
    def shutdown(self):
        """关闭摄像头"""
        self.cap.release()
        rospy.loginfo("USB摄像头节点已关闭")

if __name__ == '__main__':
    try:
        camera_node = UsbCameraPublisher()
        
        # 设置关闭时的清理函数
        rospy.on_shutdown(camera_node.shutdown)
        
        # 开始发布图像
        camera_node.publish_frames()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("USB摄像头节点被中断")
    except Exception as e:
        rospy.logerr("发生错误: %s", str(e))