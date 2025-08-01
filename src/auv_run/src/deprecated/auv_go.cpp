#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMavFrame.h>
#include <signal.h>

// 全局变量用于信号处理
bool should_exit = false;

// 信号处理函数（用于捕获Ctrl+C）
void signalHandler(int signum) {
    ROS_INFO("捕获到终止信号，准备停止ROV...");
    should_exit = true;
}

class ROVController {
private:
    ros::NodeHandle nh_;
    ros::Publisher velocity_pub_;
    ros::ServiceClient arm_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient set_mav_frame_client_;
    ros::Subscriber state_sub_;
    
    mavros_msgs::State current_state_;

public:
    ROVController() {
        // 订阅状态信息
        state_sub_ = nh_.subscribe<mavros_msgs::State>
                     ("/mavros/state", 10, &ROVController::stateCallback, this);
        
        // 创建发布者用于发布速度命令
        velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>
                       ("/mavros/setpoint_velocity/cmd_vel", 10);
        
        // 创建服务客户端用于解锁电机
        arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
                     ("/mavros/cmd/arming");
        
        // 创建服务客户端用于设置飞行模式
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
                          ("/mavros/set_mode");
        
        // 创建服务客户端用于设置MAV坐标系
        set_mav_frame_client_ = nh_.serviceClient<mavros_msgs::SetMavFrame>
                               ("/mavros/setpoint_velocity/mav_frame");
        
        // 等待MAVROS服务准备就绪
        ROS_INFO("等待MAVROS服务和连接...");
        ros::Duration(1.0).sleep();
    }
    
    // 状态回调函数
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
        ROS_INFO_THROTTLE(5.0, "当前模式: %s, 已连接: %s, 已解锁: %s", 
                        current_state_.mode.c_str(), 
                        current_state_.connected ? "是" : "否",
                        current_state_.armed ? "是" : "否");
    }
    
    // 设置MAV坐标系
    bool setMAVFrame() {
        mavros_msgs::SetMavFrame frame_msg;
        frame_msg.request.mav_frame = mavros_msgs::SetMavFrame::Request::FRAME_BODY_NED;
        
        if (set_mav_frame_client_.call(frame_msg)) {
            ROS_INFO("MAV坐标系设置为BODY_NED");
            return frame_msg.response.success;
        } else {
            ROS_WARN("无法设置MAV坐标系");
            return false;
        }
    }
    
    // 切换到Guided模式
    bool setGuidedMode() {
        mavros_msgs::SetMode mode_cmd;
        mode_cmd.request.custom_mode = "GUIDED";
        
        // 等待连接建立
        int retry_count = 0;
        while (!current_state_.connected && retry_count < 10) {
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            retry_count++;
        }
        
        if (!current_state_.connected) {
            ROS_ERROR("未能连接到FCU");
            return false;
        }
        
        if (set_mode_client_.call(mode_cmd)) {
            if (mode_cmd.response.mode_sent) {
                ROS_INFO("成功切换到GUIDED模式");
                return true;
            } else {
                ROS_ERROR("无法切换到GUIDED模式");
                return false;
            }
        } else {
            ROS_ERROR("模式切换服务调用失败");
            return false;
        }
    }
    
    // 解锁电机
    bool armThrusters() {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        
        if (arm_client_.call(arm_cmd)) {
            if (arm_cmd.response.success) {
                ROS_INFO("电机解锁成功");
                return true;
            } else {
                ROS_ERROR("电机解锁失败");
                return false;
            }
        } else {
            ROS_ERROR("电机解锁服务调用失败");
            return false;
        }
    }
    
    // 锁定电机
    bool disarmThrusters() {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = false;
        
        if (arm_client_.call(arm_cmd)) {
            if (arm_cmd.response.success) {
                ROS_INFO("电机锁定成功");
                return true;
            } else {
                ROS_ERROR("电机锁定失败");
                return false;
            }
        } else {
            ROS_ERROR("电机锁定服务调用失败");
            return false;
        }
    }
    
    // 使用速度命令控制向前移动
    void setForwardVelocity(float speed) {
        geometry_msgs::TwistStamped twist_msg;
        
        // 设置时间戳
        twist_msg.header.stamp = ros::Time::now();
        twist_msg.header.frame_id = "base_link";
        
        // 在ArduSub中，X轴是向前的方向
        twist_msg.twist.linear.x = speed;  // 单位: m/s
        twist_msg.twist.linear.y = 0.0;
        twist_msg.twist.linear.z = 0.0;
        
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = 0.0;
        
        // 发布速度命令
        velocity_pub_.publish(twist_msg);
        ROS_INFO_THROTTLE(5.0, "发送速度: %.2f m/s", speed);
    }
    
    // 停止ROV
    void stop() {
        geometry_msgs::TwistStamped twist_msg;
        
        twist_msg.header.stamp = ros::Time::now();
        twist_msg.header.frame_id = "base_link";
        
        // 所有速度设为0
        twist_msg.twist.linear.x = 0.0;
        twist_msg.twist.linear.y = 0.0;
        twist_msg.twist.linear.z = 0.0;
        
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = 0.0;
        
        velocity_pub_.publish(twist_msg);
        ROS_INFO("ROV速度停止命令已发送");
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");

    // 注册信号处理器
    signal(SIGINT, signalHandler);
    
    ros::init(argc, argv, "rov_forward_controller", ros::init_options::NoSigintHandler);
    ROVController controller;
    
    // 给ROS节点一些时间来初始化
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    
    // 切换到Guided模式
    if (!controller.setGuidedMode()) {
        ROS_ERROR("无法设置GUIDED模式，退出程序");
        return 1;
    }
    
    // 确保模式切换生效
    ros::Duration(2.0).sleep();
    ros::spinOnce();
    
    // 设置MAV坐标系为机体坐标系
    controller.setMAVFrame();
    
    // 解锁电机 - 这是移动ROV的必要步骤
    if (!controller.armThrusters()) {
        ROS_ERROR("电机解锁失败，无法移动ROV");
        return 1;
    }
    
    // 等待电机解锁状态更新
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    
    float forward_speed = 0.8; // 速度: 0.8 m/s
    ROS_INFO("开始持续向前移动，速度为: %.2f m/s", forward_speed);
    ROS_INFO("按Ctrl+C停止程序并使ROV停止");
    
    // 设置更新频率
    ros::Rate rate(10); // 10Hz
    
    // 持续发送速度指令
    while (ros::ok() && !should_exit) {
        controller.setForwardVelocity(forward_speed);
        ros::spinOnce();
        rate.sleep();
    }
    
    // 程序结束时停止ROV并锁定电机
    ROS_INFO("程序终止，停止ROV...");
    controller.stop();
    
    // 等待停止命令生效
    ros::Duration(1.0).sleep();
    
    // 锁定电机
    controller.disarmThrusters();
    
    // 确保命令被发送
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    
    return 0;
}