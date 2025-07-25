#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>

// 存储当前状态的全局变量
mavros_msgs::State current_state;

// 状态回调函数
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");

    // 初始化ROS节点
    ros::init(argc, argv, "ardusub_arm_and_set_mode");
    ros::NodeHandle nh;
    
    // 创建订阅器获取当前状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    
    // 创建服务客户端，用于解锁和设置模式
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");
    
    // 创建RC覆盖发布器，用于发送控制信号
    ros::Publisher rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>
            ("/mavros/rc/override", 10);
    
    // 设置循环频率为20Hz
    ros::Rate rate(20.0);
    
    // 等待FCU连接
    ROS_INFO("等待FCU连接...");
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU已连接");
    
    // 创建RC覆盖消息
    mavros_msgs::OverrideRCIn rc_override_msg;
    // 设置所有通道为中间位置（通常是1500）
    for(int i = 0; i < 8; i++) {
        rc_override_msg.channels[i] = 1500;
    }
    
    // 以20Hz的频率发送RC消息一段时间（5秒）
    ROS_INFO("发送RC消息以建立连接...");
    ros::Time start_time = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - start_time) < ros::Duration(5.0)) {
        rc_override_pub.publish(rc_override_msg);
        ros::spinOnce();
        rate.sleep();
    }
    
    // 创建设置模式的请求
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = "GUIDED"; // 设置为GUIDED模式
    
    // 尝试设置GUIDED模式
    ROS_INFO("尝试设置GUIDED模式...");
    if(set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent) {
        ROS_INFO("成功将模式设置为GUIDED");
    } else {
        ROS_ERROR("设置GUIDED模式失败");
        return 1;
    }
    
    // 等待模式切换（最多3秒）
    start_time = ros::Time::now();
    while(ros::ok() && current_state.mode != "GUIDED" && 
          (ros::Time::now() - start_time) < ros::Duration(3.0)) {
        // 继续发送RC消息
        rc_override_pub.publish(rc_override_msg);
        ros::spinOnce();
        rate.sleep();
    }
    
    if(current_state.mode != "GUIDED") {
        ROS_ERROR("无法切换到GUIDED模式，当前模式: %s", current_state.mode.c_str());
        return 1;
    }
    
    // 创建解锁请求
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true; // true代表解锁
    
    // 发送解锁请求
    ROS_INFO("尝试解锁ArduSub...");
    if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("ArduSub成功解锁");
    } else {
        ROS_ERROR("解锁ArduSub失败");
        return 1;
    }
    
    // 等待解锁状态（最多3秒）
    start_time = ros::Time::now();
    while(ros::ok() && !current_state.armed && 
          (ros::Time::now() - start_time) < ros::Duration(3.0)) {
        // 继续发送RC消息
        rc_override_pub.publish(rc_override_msg);
        ros::spinOnce();
        rate.sleep();
    }
    
    if(!current_state.armed) {
        ROS_ERROR("无法解锁ArduSub");
        return 1;
    }
    
    ROS_INFO("ArduSub已解锁并处于GUIDED模式");
    
    // 继续发送RC消息一段时间以保持状态
    start_time = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - start_time) < ros::Duration(5.0)) {
        rc_override_pub.publish(rc_override_msg);
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("操作完成");
    
    return 0;
}
