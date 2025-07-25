#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Float64.h>

// 全局变量
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::Publisher rc_override_pub;
ros::Publisher target_depth_pub;

mavros_msgs::State current_state;
double current_depth = 0.0;
double target_depth = 1.0; // 目标深度(米)，可以根据需要调整
bool depth_reached = false;

// RC通道定义
const int THROTTLE_CHANNEL = 2; // 在ArduSub中，通道3(索引2)控制垂直运动
const int MID_RC = 1500;        // RC中值
const int RC_RANGE = 400;       // RC值范围，通常用±400表示全速

// 回调函数：获取当前状态
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// 回调函数：获取当前深度
// 注意：该函数假设已经有一个节点将压力转换为深度并发布
void depth_cb(const std_msgs::Float64::ConstPtr& msg) {
    current_depth = msg->data;
}

// 压力传感器回调 - 如果你需要直接从压力计算深度
void pressure_cb(const sensor_msgs::FluidPressure::ConstPtr& msg) {
    // 标准大气压 (Pa)
    const double atmospheric_pressure = 101325.0;
    // 水密度 (kg/m^3)
    const double water_density = 1000.0;
    // 重力加速度 (m/s^2)
    const double gravity = 9.8;
    
    // 计算深度 (m)
    current_depth = (msg->fluid_pressure - atmospheric_pressure) / (water_density * gravity);
}

// 深度控制函数
void control_depth() {
    // 简单的P控制器
    double depth_error = target_depth - current_depth;
    
    // 如果已经达到目标深度附近，停止调整
    if (fabs(depth_error) < 0.1) {
        if (!depth_reached) {
            ROS_INFO("Target depth reached: %.2f m", current_depth);
            depth_reached = true;
        }
        // 发送中值（悬停）
        mavros_msgs::OverrideRCIn rc_msg;
        for (int i = 0; i < 8; i++) {
            rc_msg.channels[i] = 0; // 0表示不覆盖该通道
        }
        rc_msg.channels[THROTTLE_CHANNEL] = MID_RC; // 中值
        rc_override_pub.publish(rc_msg);
        return;
    }
    
    depth_reached = false;
    
    // 修正：确保使用相同类型进行比较
    double control_output = std::min(std::max(depth_error * 200, static_cast<double>(-RC_RANGE)), static_cast<double>(RC_RANGE));
    int throttle_value = MID_RC + static_cast<int>(control_output);
    
    // 创建并发送RC覆盖消息
    mavros_msgs::OverrideRCIn rc_msg;
    for (int i = 0; i < 8; i++) {
        rc_msg.channels[i] = 0; // 0表示不覆盖该通道
    }
    rc_msg.channels[THROTTLE_CHANNEL] = throttle_value;
    
    rc_override_pub.publish(rc_msg);
    
    ROS_INFO("Current depth: %.2f m, Target: %.2f m, Throttle: %d", 
             current_depth, target_depth, throttle_value);
}

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");

    ros::init(argc, argv, "ardusub_depth_control");
    ros::NodeHandle nh;
    
    // 从参数服务器获取目标深度参数（如果有）
    nh.param("target_depth", target_depth, 1.0);
    
    // 状态订阅者
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    
    // 深度订阅者（假设有节点将压力转换为深度）
    // 如果你需要自己计算深度，你需要订阅压力传感器并转换
    ros::Subscriber depth_sub = nh.subscribe<std_msgs::Float64>
            ("mavros/imu/water_depth", 10, depth_cb);
    
    // 可选：直接从压力传感器订阅数据计算深度
    // 如果你使用这个，请注释掉上面的depth_sub
    // ros::Subscriber pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>
    //         ("mavros/imu/atm_pressure", 10, pressure_cb);
    
    // 服务客户端
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    
    // 发布器
    rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>
            ("mavros/rc/override", 10);
    
    // 可选：发布目标深度，用于可视化或记录
    target_depth_pub = nh.advertise<std_msgs::Float64>
            ("ardusub/target_depth", 10);
    
    // 设置循环频率
    ros::Rate rate(20.0);
    
    // 等待连接
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("Connected to ArduSub FCU");
    
    // 解锁载具
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();
    bool armed = false;
    
    // 尝试解锁
    while (ros::ok() && !armed) {
        if (ros::Time::now() - last_request > ros::Duration(1.0)) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed successfully");
                armed = true;
            } else {
                ROS_WARN("Arming failed, retrying...");
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    // 设置ALT_HOLD模式
    mavros_msgs::SetMode set_mode_cmd;
    set_mode_cmd.request.custom_mode = "ALT_HOLD";
    
    last_request = ros::Time::now();
    bool mode_set = false;
    
    // 尝试设置模式 - 修正检查方式
    while (ros::ok() && !mode_set) {
        if (ros::Time::now() - last_request > ros::Duration(1.0)) {
            if (set_mode_client.call(set_mode_cmd) && set_mode_cmd.response.mode_sent) {
                ROS_INFO("Mode set to ALT_HOLD successfully");
                mode_set = true;
            } else {
                ROS_WARN("Setting mode failed, retrying...");
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    // 发布目标深度
    std_msgs::Float64 target_msg;
    target_msg.data = target_depth;
    target_depth_pub.publish(target_msg);
    
    ROS_INFO("Starting depth control to %.2f meters", target_depth);
    
    // 主循环，执行深度控制
    while (ros::ok()) {
        // 发布目标深度（用于记录或调试）
        target_msg.data = target_depth;
        target_depth_pub.publish(target_msg);
        
        // 执行深度控制
        if (current_state.mode == "ALT_HOLD" && current_state.armed) {
            control_depth();
        } else {
            ROS_WARN_THROTTLE(5.0, "Vehicle not in ALT_HOLD mode or not armed");
            
            // 如果掉出ALT_HOLD模式，尝试重新设置
            if (ros::Time::now() - last_request > ros::Duration(5.0)) {
                if (set_mode_client.call(set_mode_cmd)) {
                    ROS_INFO("Attempted to set ALT_HOLD mode again");
                }
                last_request = ros::Time::now();
            }
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    // 退出前释放RC覆盖
    mavros_msgs::OverrideRCIn rc_msg;
    for (int i = 0; i < 8; i++) {
        rc_msg.channels[i] = 0;
    }
    rc_override_pub.publish(rc_msg);
    
    return 0;
}
