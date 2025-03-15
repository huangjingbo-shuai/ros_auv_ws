#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>

class SimpleROVDepthControl {
private:
    // ROS节点句柄
    ros::NodeHandle nh;
    
    // 目标深度（0.25米）
    double target_depth = 0.25;
    
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
    ros::Publisher setpoint_raw_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber state_sub;
    
    // 服务客户端
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

public:
    SimpleROVDepthControl() {
        // 初始化发布者 - 使用setpoint_raw接口直接发送位置指令
        setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        
        // 初始化订阅者
        pose_sub = nh.subscribe("/mavros/local_position/pose", 10, &SimpleROVDepthControl::poseCallback, this);
        state_sub = nh.subscribe("/mavros/state", 10, &SimpleROVDepthControl::stateCallback, this);
        
        // 初始化服务客户端
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        
        // 创建定时器，控制频率为10Hz
        control_timer = nh.createTimer(ros::Duration(0.1), &SimpleROVDepthControl::controlCallback, this);
        
        ROS_INFO("SimpleROV 深度控制节点已初始化，目标深度: %.2f 米", target_depth);
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
        
        // 发送深度设定点
        mavros_msgs::PositionTarget pos_target;
        pos_target.header.stamp = ros::Time::now();
        pos_target.header.frame_id = "base_link";
        
        // 设置消息类型标志 - 只设置Z位置，其他维度忽略
        // 使用正确的mavros_msgs::PositionTarget常量
        // 参考: https://github.com/mavlink/mavros/blob/master/mavros_msgs/msg/PositionTarget.msg
        pos_target.type_mask = 
            mavros_msgs::PositionTarget::IGNORE_PX |     // 忽略X位置
            mavros_msgs::PositionTarget::IGNORE_PY |     // 忽略Y位置
            mavros_msgs::PositionTarget::IGNORE_VX |     // 忽略X速度
            mavros_msgs::PositionTarget::IGNORE_VY |     // 忽略Y速度
            mavros_msgs::PositionTarget::IGNORE_VZ |     // 忽略Z速度
            mavros_msgs::PositionTarget::IGNORE_AFX |    // 忽略X加速度
            mavros_msgs::PositionTarget::IGNORE_AFY |    // 忽略Y加速度
            mavros_msgs::PositionTarget::IGNORE_AFZ |    // 忽略Z加速度
            mavros_msgs::PositionTarget::IGNORE_YAW |    // 忽略偏航角
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;// 忽略偏航角速率
        
        // 我们只想控制Z位置，所以取消IGNORE_PZ标志
        pos_target.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
        
        // 设置深度目标 (注意z轴的正负)
        pos_target.position.x = 0;
        pos_target.position.y = 0;
        pos_target.position.z = -target_depth;  // Z轴向下为正，所以目标深度需要取负值
        
        // 发布位置目标
        setpoint_raw_pub.publish(pos_target);
        
        // 记录深度信息
        double depth_error = target_depth - current_depth;
        if (fabs(depth_error) <= depth_error_threshold) {
            ROS_INFO_THROTTLE(5, "深度已稳定: %.2f 米（目标: %.2f 米，误差: %.3f 米）", 
                             current_depth, target_depth, depth_error);
        } else {
            ROS_INFO_THROTTLE(1, "调整深度中: %.2f 米（目标: %.2f 米，误差: %.3f 米）", 
                             current_depth, target_depth, depth_error);
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
};

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "simplerov_depth_control");
    
    SimpleROVDepthControl depth_controller;
    
    // 等待连接建立
    ros::Duration(2.0).sleep();
    
    // 尝试切换到GUIDED模式并解锁
    depth_controller.setGuidedMode();
    depth_controller.setArming(true);
    
    ros::spin();
    
    return 0;
}