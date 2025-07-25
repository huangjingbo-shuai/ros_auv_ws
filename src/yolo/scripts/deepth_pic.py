#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

depth_data = deque(maxlen=100)
time_data = deque(maxlen=100)
start_time = None
lock = threading.Lock()

def depth_callback(msg):
    global start_time
    with lock:
        depth = -msg.pose.position.z  # ArduSub 向下为负
        now = rospy.Time.now().to_sec()
        if start_time is None:
            start_time = now
        depth_data.append(depth)
        time_data.append(now - start_time)

def ros_spin_thread():
    rospy.spin()  # 只让 spin() 在子线程运行

def update_plot(frame):
    with lock:
        plt.cla()
        plt.plot(time_data, depth_data, label="Depth (m)")
        plt.xlabel("Time (s)")
        plt.ylabel("Depth (m)")
        plt.title("Real-time Depth of Underwater Robot")
        plt.grid(True)
        plt.legend()

if __name__ == '__main__':
    rospy.init_node('depth_plot_listener', anonymous=True)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, depth_callback)
    rospy.loginfo("深度订阅器启动")

    # 启动 ROS spin 的线程（非主线程）
    t = threading.Thread(target=ros_spin_thread)
    t.daemon = True
    t.start()

    # 启动 matplotlib 动画
    fig = plt.figure()
    ani = animation.FuncAnimation(fig, update_plot, interval=500)
    plt.show()
