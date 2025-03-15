#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

class SimpleROVActuatorControl {
private:
    // ROS节点句柄
    ros::NodeHandle nh;
    
    // 目标深度（0.25米）
    double target_depth = 0.25;
    
    // PID控制器参数
    double Kp = 1.5;  // 比例系数
    double Ki = 0.1;  // 积分系数
    double Kd = 0.5;  // 微分系数
    
    // PID控制器状态
    double integral = 0.0;
    double previous_error = 0.0;
    double previous_time = 0.0;
    
    // 当前位置和状态变量
    double current_depth = 0.0;
    bool armed = false;
    std::string mode = "";
    bool guided_mode_active = false;
    
    // 控制参数
    double depth_error_threshold = 0.05;  // 深度误差阈值，±5厘米
    
    // 定时器
    ros::Timer control_timer;
    
    // 发布者和订阅者
    ros::Publisher actuator_control_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber state_sub;
    
    // 服务客户端
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

public:
    SimpleROVActuatorControl() {
        // 初始化发布者 - 使用actuator_control接口直接控制电机
        actuator_control_pub = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);
        
        // 初始化订阅者
        pose_sub = nh.subscribe("/mavros/local_position/pose", 10, &SimpleROVActuatorControl::poseCallback, this);
        state_sub = nh.subscribe("/mavros/state", 10, &SimpleROVActuatorControl::stateCallback, this);
        
        // 初始化服务客户端
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        
        // 创建定时器，控制频率为20Hz
        control_timer = nh.createTimer(ros::Duration(0.05), &SimpleROVActuatorControl::controlCallback, this);
        
        previous_time = ros::Time::now().toSec();
        
        ROS_INFO("SimpleROV Actuator控制节点已初始化，目标深度: %.2f 米", target_depth);
    }
    
    // 位姿回调函数
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 获取当前深度 (注意：在ArduSub中，Z轴向下为正)
        current_depth = -msg->pose.position.z;  // 如果Z轴向上为正，则取负值
    }
    
    // 状态回调函数
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        armed = msg->armed;
        mode = msg->mode;
        
        // 检查是否处于GUIDED模式
        if (mode == "GUIDED") {
            if (!guided_mode_active) {
                guided_mode_active = true;
                ROS_INFO("ROV已进入GUIDED模式，开始自主深度控制");
            }
        } else {
            if (guided_mode_active) {
                guided_mode_active = false;
                ROS_WARN("ROV已离开GUIDED模式，自主深度控制已停止");
            }
        }
    }
    
    // 控制回调函数
    void controlCallback(const ros::TimerEvent& event) {
        // 检查ROV状态
        if (!armed) {
            ROS_WARN_THROTTLE(5, "ROV未解锁，无法控制深度！");
            return;
        }
        
        if (mode != "GUIDED") {
            ROS_WARN_THROTTLE(5, "ROV不在GUIDED模式，当前模式: %s", mode.c_str());
            return;
        }
        
        // 获取当前时间
        double current_time = ros::Time::now().toSec();
        double dt = current_time - previous_time;
        previous_time = current_time;
        
        if (dt <= 0.0) dt = 0.05;  // 防止除以零
        
        // 计算深度误差
        double depth_error = target_depth - current_depth;
        
        // 计算PID各项
        double p_term = Kp * depth_error;
        
        // 积分项（带抗积分饱和）
        integral += depth_error * dt;
        if (integral > 1.0) integral = 1.0;
        if (integral < -1.0) integral = -1.0;
        double i_term = Ki * integral;
        
        // 微分项
        double derivative = (depth_error - previous_error) / dt;
        previous_error = depth_error;
        double d_term = Kd * derivative;
        
        // 计算总控制输出 - 值范围应为[-1, 1]
        double control_output = p_term + i_term + d_term;
        
        // 限制控制输出范围
        if (control_output > 1.0) control_output = 1.0;
        if (control_output < -1.0) control_output = -1.0;
        
        // 若深度误差小于阈值，则不需要调整
        if (fabs(depth_error) <= depth_error_threshold) {
            control_output = 0.0;
        }
        
        // 创建并发送actuator_control消息
        mavros_msgs::ActuatorControl actuator_msg;
        actuator_msg.header.stamp = ros::Time::now();
        actuator_msg.group_mix = 0;  // 使用主组混控
        
        // 初始化所有控制值为0
        for (int i = 0; i < 8; ++i) {
            actuator_msg.controls[i] = 0.0;
        }
        
        // 设置上升/下降控制 - 通常是throttle控制值
        // 根据ArduSub的SITL混控配置，控制索引可能为2（对应第三个控制通道）
        // 注意：可能需要根据实际ROV配置调整索引
        actuator_msg.controls[2] = control_output;  // 通常为Z轴控制（油门）
        
        // 发布actuator控制消息
        actuator_control_pub.publish(actuator_msg);
        
        // 记录深度信息
        if (fabs(depth_error) <= depth_error_threshold) {
            ROS_INFO_THROTTLE(5, "深度已稳定: %.2f 米（目标: %.2f 米，误差: %.3f 米）", 
                             current_depth, target_depth, depth_error);
        } else {
            ROS_INFO_THROTTLE(1, "调整深度中: %.2f 米（目标: %.2f 米，误差: %.3f 米，控制输出: %.3f）", 
                             current_depth, target_depth, depth_error, control_output);
        }
    }
    
    // 设置GUIDED模式
    bool setGuidedMode() {
        mavros_msgs::SetMode mode_cmd;
        mode_cmd.request.custom_mode = "GUIDED";
        
        if (set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent) {
            ROS_INFO("GUIDED模式设置成功");
            return true;
        } else {
            ROS_ERROR("GUIDED模式设置失败!");
            return false;
        }
    }
    
    // 解锁/上锁
    bool setArming(bool arm) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = arm;
        
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO(arm ? "解锁成功!" : "上锁成功!");
            return true;
        } else {
            ROS_ERROR(arm ? "解锁失败!" : "上锁失败!");
            return false;
        }
    }
    
    // 设置目标深度
    void setTargetDepth(double depth) {
        target_depth = depth;
        ROS_INFO("设置新的目标深度: %.2f 米", target_depth);
    }
    
    // 停止所有控制输出
    void stopControl() {
        mavros_msgs::ActuatorControl actuator_msg;
        actuator_msg.header.stamp = ros::Time::now();
        actuator_msg.group_mix = 0;
        
        for (int i = 0; i < 8; ++i) {
            actuator_msg.controls[i] = 0.0;
        }
        
        actuator_control_pub.publish(actuator_msg);
        ROS_INFO("已停止所有控制输出");
    }
};

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");

    ros::init(argc, argv, "simplerov_actuator_control");
    
    SimpleROVActuatorControl actuator_controller;
    
    // 等待连接建立
    ros::Duration(2.0).sleep();
    
    // 尝试切换到GUIDED模式并解锁
    actuator_controller.setGuidedMode();
    actuator_controller.setArming(true);
    
    ros::spin();
    
    // 停止所有控制输出
    actuator_controller.stopControl();
    
    return 0;
}