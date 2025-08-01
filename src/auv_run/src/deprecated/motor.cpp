#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <signal.h>

// 全局变量用于信号处理
bool should_exit = false;

// 信号处理函数（用于捕获Ctrl+C）
void signalHandler(int signum) {
    ROS_INFO("捕获到终止信号，准备停止ROV...");
    should_exit = true;
}

class ROVActuatorController {
private:
    ros::NodeHandle nh_;
    ros::Publisher actuator_pub_;
    ros::ServiceClient arm_client_;
    ros::ServiceClient set_mode_client_;
    ros::Subscriber state_sub_;
    
    mavros_msgs::State current_state_;
    
    // 推进器组索引
    const int MAIN_THRUSTERS_GROUP = 0;  // 主推进器组

public:
    ROVActuatorController() {
        // 订阅状态信息
        state_sub_ = nh_.subscribe<mavros_msgs::State>
                     ("/mavros/state", 10, &ROVActuatorController::stateCallback, this);
        
        // 创建发布者用于发布执行器控制命令
        actuator_pub_ = nh_.advertise<mavros_msgs::ActuatorControl>
                       ("/mavros/actuator_control", 10);
        
        // 创建服务客户端用于解锁电机
        arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
                     ("/mavros/cmd/arming");
        
        // 创建服务客户端用于设置飞行模式
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
                          ("/mavros/set_mode");
        
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
    
    // 切换到模式
    bool setMode(const std::string& mode) {
        mavros_msgs::SetMode mode_cmd;
        mode_cmd.request.custom_mode = mode;
        
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
                ROS_INFO("成功切换到%s模式", mode.c_str());
                return true;
            } else {
                ROS_ERROR("无法切换到%s模式", mode.c_str());
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
    
    // 使用actuator_control控制推进器
    void setThrusters(float forward, float lateral, float vertical, float yaw) {
        mavros_msgs::ActuatorControl actuator_msg;
        
        // 设置组索引
        actuator_msg.group_mix = MAIN_THRUSTERS_GROUP;
        
        // 设置时间戳
        actuator_msg.header.stamp = ros::Time::now();
        
        // 在SimpleROV的四电机配置中:
        // controls[0] = 前进/后退 (X轴)
        // controls[1] = 左右横移 (Y轴)
        // controls[2] = 上升/下降 (Z轴)
        // controls[3] = 偏航 (绕Z轴旋转)
        // 数值范围为[-1, 1]
        actuator_msg.controls[0] = forward;   // 前进/后退
        actuator_msg.controls[1] = lateral;   // 左右横移
        actuator_msg.controls[2] = vertical;  // 上升/下降
        actuator_msg.controls[3] = yaw;       // 偏航控制
        
        // 其余通道设为0
        for (int i = 4; i < 8; i++) {
            actuator_msg.controls[i] = 0.0;
        }
        
        // 发布控制命令
        actuator_pub_.publish(actuator_msg);
        
        ROS_INFO_THROTTLE(5.0, "推进器控制: 前进=%.2f, 横移=%.2f, 垂直=%.2f, 偏航=%.2f", 
                         forward, lateral, vertical, yaw);
    }
    
    // 停止所有推进器
    void stopThrusters() {
        mavros_msgs::ActuatorControl actuator_msg;
        
        actuator_msg.group_mix = MAIN_THRUSTERS_GROUP;
        actuator_msg.header.stamp = ros::Time::now();
        
        // 所有通道设为0
        for (int i = 0; i < 8; i++) {
            actuator_msg.controls[i] = 0.0;
        }
        
        actuator_pub_.publish(actuator_msg);
        ROS_INFO("所有推进器已停止");
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");

    // 注册信号处理器
    signal(SIGINT, signalHandler);
    
    ros::init(argc, argv, "rov_actuator_controller", ros::init_options::NoSigintHandler);
    ROVActuatorController controller;
    
    // 给ROS节点一些时间来初始化
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    
    // 切换到MANUAL模式或STABILIZE模式，因为这些模式下actuator_control更有效
    // 对于直接控制执行器，通常MANUAL模式更合适
    if (!controller.setMode("MANUAL")) {
        ROS_WARN("无法设置MANUAL模式，尝试STABILIZE模式");
        if (!controller.setMode("STABILIZE")) {
            ROS_ERROR("无法设置合适的模式，退出程序");
            return 1;
        }
    }
    
    // 确保模式切换生效
    ros::Duration(2.0).sleep();
    ros::spinOnce();
    
    // 解锁电机
    if (!controller.armThrusters()) {
        ROS_ERROR("电机解锁失败，无法控制推进器");
        return 1;
    }
    
    // 等待电机解锁状态更新
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    
    float forward_thrust = 0.3; // 30% 前进推力
    ROS_INFO("开始持续向前移动，推力为: %.2f", forward_thrust);
    ROS_INFO("按Ctrl+C停止程序并使ROV停止");
    
    // 设置更新频率
    ros::Rate rate(10); // 10Hz
    
    // 持续发送推进器控制命令
    while (ros::ok() && !should_exit) {
        controller.setThrusters(forward_thrust, 0.0, 0.0, 0.0);
        ros::spinOnce();
        rate.sleep();
    }
    
    // 程序结束时停止推进器并锁定电机
    ROS_INFO("程序终止，停止ROV...");
    controller.stopThrusters();
    
    // 等待停止命令生效
    ros::Duration(1.0).sleep();
    
    // 锁定电机
    controller.disarmThrusters();
    
    // 确保命令被发送
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    
    return 0;
}