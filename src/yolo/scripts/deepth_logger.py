#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import FluidPressure
import csv
import os

class StaticPressureLogger:
    def __init__(self):
        self.start_time = None
        self.filename = "depth_from_pressure.csv"
        self.atm_pressure = 101325.0  # 大气压（Pa）
        self.water_density = 1000.0   # kg/m^3
        self.gravity = 9.80665        # m/s^2

        self.init_csv()
        rospy.init_node('static_pressure_logger', anonymous=True)
        rospy.Subscriber("/mavros/imu/static_pressure", FluidPressure, self.pressure_callback)
        rospy.loginfo("水深记录器启动，监听 /mavros/imu/static_pressure")
        rospy.spin()

    def init_csv(self):
        if not os.path.exists(self.filename):
            with open(self.filename, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Time (s)", "Pressure (Pa)", "Depth (m)"])

    def pressure_callback(self, msg):
        pressure = msg.fluid_pressure  # 原始压强（Pa）
        now = rospy.Time.now().to_sec()
        if self.start_time is None:
            self.start_time = now
        elapsed = now - self.start_time

        # 估算深度
        depth = max(0.0, (pressure - self.atm_pressure) / (self.water_density * self.gravity))

        # 打印
        rospy.loginfo("Time: %.2f s, Pressure: %.2f Pa, Estimated Depth: %.2f m", elapsed, pressure, depth)

        # 写入CSV
        with open(self.filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([f"{elapsed:.2f}", f"{pressure:.2f}", f"{depth:.2f}"])

if __name__ == '__main__':
    try:
        StaticPressureLogger()
    except rospy.ROSInterruptException:
        pass
