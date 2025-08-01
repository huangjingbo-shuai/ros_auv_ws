#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <iostream>

// 大气压力常数 (1013.25 hPa 或 101325 Pa)
const double ATMOSPHERIC_PRESSURE_PA = 64896;

// 水密度 (kg/m^3)，淡水约为1000，海水约为1025
const double WATER_DENSITY = 10.0;

// 重力加速度 (m/s^2)
const double GRAVITY = 9.80665;

// 压力传感器回调函数
void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
    // 获取当前压力值（Pa）
    double current_pressure = msg->fluid_pressure;
    
    // 计算压力差
    double pressure_difference = current_pressure - ATMOSPHERIC_PRESSURE_PA;
    
    // 使用压力公式计算深度：depth = pressure/(density*gravity)
    double depth = pressure_difference / (WATER_DENSITY * GRAVITY);
    
    // 打印当前时间、压力和计算出的深度值
    ros::Time now = ros::Time::now();
    ROS_INFO("Time: %.2f, Pressure: %.2f Pa, Depth: %.2f meters", 
             now.toSec(), current_pressure, depth);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "depth_calculator");
    
    // 创建节点句柄
    ros::NodeHandle nh;
    
    // 允许用户通过参数修改大气压力参考值（如果需要）
    double atmospheric_pressure = ATMOSPHERIC_PRESSURE_PA;
    nh.param("atmospheric_pressure", atmospheric_pressure, ATMOSPHERIC_PRESSURE_PA);
    
    // 允许用户通过参数修改水密度（如果需要）
    double water_density = WATER_DENSITY;
    nh.param("water_density", water_density, WATER_DENSITY);
    
    // 创建订阅者，订阅静态压力话题
    ros::Subscriber pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>
        ("/mavros/imu/static_pressure", 10, pressureCallback);
        
    ROS_INFO("深度计算节点已启动");
    ROS_INFO("使用大气压力参考值: %.2f Pa", atmospheric_pressure);
    ROS_INFO("使用水密度: %.2f kg/m^3", water_density);
    
    // 进入ROS主循环
    ros::spin();
    
    return 0;
}
