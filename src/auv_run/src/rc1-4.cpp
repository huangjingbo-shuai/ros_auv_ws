#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ctime>

class PIDController {
private:
    double kp, ki, kd;
    double setpoint;
    double integral, prev_error;
    double output_min, output_max;
    double integral_limit; // 添加积分限制
    ros::Time last_time;
    bool first_call;

public:
    PIDController(double _kp, double _ki, double _kd, double _setpoint, 
                 double _output_min, double _output_max) 
        : kp(_kp), ki(_ki), kd(_kd), setpoint(_setpoint),
          integral(0.0), prev_error(0.0), 
          output_min(_output_min), output_max(_output_max),
          integral_limit(100.0), // 默认积分限制
          first_call(true) {}

    double compute(double measurement) {
        ros::Time current_time = ros::Time::now();
        double dt;
        
        if (first_call) {
            dt = 0.0;
            first_call = false;
        } else {
            dt = (current_time - last_time).toSec();
        }
        last_time = current_time;
        
        // Avoid division by zero
        if (dt <= 0.0) return 0.0;
        
        // Calculate error
        double error = setpoint - measurement;
        
        // Proportional term
        double p_term = kp * error;
        
        // Integral term with anti-windup
        integral += error * dt;
        
        // Apply integral limits to prevent windup
        if (integral > integral_limit) {
            integral = integral_limit;
        } else if (integral < -integral_limit) {
            integral = -integral_limit;
        }
        
        double i_term = ki * integral;
        
        // Derivative term
        double derivative = 0.0;
        if (dt > 0) {
            derivative = (error - prev_error) / dt;
        }
        double d_term = kd * derivative;
        
        // Calculate total output
        double output = p_term + i_term + d_term;
        
        // Apply output limits
        if (output > output_max) {
            output = output_max;
        } else if (output < output_min) {
            output = output_min;
        }
        
        // Save error for next iteration
        prev_error = error;
        
        // 输出各项贡献，便于调试
        ROS_DEBUG("PID components: P=%.2f, I=%.2f, D=%.2f", p_term, i_term, d_term);
        
        return output;
    }
    
    void reset() {
        integral = 0.0;
        prev_error = 0.0;
        first_call = true;
    }
    
    // 设置积分限制
    void setIntegralLimit(double limit) {
        integral_limit = limit;
    }
};

class FlightControllerInterface {
private:
    ros::NodeHandle nh;
    ros::Publisher rc_pub;
    ros::ServiceClient arming_client;
    ros::Subscriber imu_sub;
    ros::Rate* rate;
    
    // PID controller for pitch stabilization
    PIDController* pitch_pid;
    
    // Current pitch angle in radians
    double current_pitch;
    
    // Base throttle value
    uint16_t base_throttle;
    
    // 电机输出系数，用于调整电机响应差异
    double motor3_factor;
    double motor4_factor;

public:
    FlightControllerInterface() : current_pitch(0.0), base_throttle(1500),
                                 motor3_factor(1.0), motor4_factor(1.0) {
        // 初始化RC控制发布器
        rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 100);
        
        // 等待解锁服务
        ROS_INFO("等待解锁服务...");
        ros::service::waitForService("/mavros/cmd/arming");
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        ROS_INFO("解锁服务已准备就绪");
        
        // 订阅IMU数据
        imu_sub = nh.subscribe("/mavros/imu/data", 100, &FlightControllerInterface::imuCallback, this);
        ROS_INFO("已订阅IMU数据");

        // 设置发布频率为20Hz
        rate = new ros::Rate(100);
        
        // 初始化PID控制器 (kp, ki, kd, setpoint, min_output, max_output)
        // 这些参数需要根据实际情况调整
        pitch_pid = new PIDController(1000.0, 2, 40.0, 0.0, -400.0, 400.0);
        
        // 设置积分限制
        pitch_pid->setIntegralLimit(200.0);
    }

    ~FlightControllerInterface() {
        delete rate;
        delete pitch_pid;
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 从四元数中提取欧拉角
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        // 保存当前pitch角度
        current_pitch = pitch;
    }

    // 解锁飞控
    bool armVehicle() {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("飞控解锁成功");
            return true;
        } else {
            ROS_WARN("飞控解锁失败");
            return false;
        }
    }

    // 控制电机3和电机4
    void controlMotors(uint16_t throttle_ch3, uint16_t throttle_ch4) {
        mavros_msgs::OverrideRCIn msg;
        
        // 填充所有通道为65535（表示"不改变"）
        for (int i = 0; i < 18; ++i) {
            msg.channels[i] = 65535;
        }
        
        // 设置通道3和通道4的值
        msg.channels[2] = throttle_ch3;  // 通道3
        msg.channels[3] = throttle_ch4;  // 通道4
        
        rc_pub.publish(msg);
    }
    
    // 使用PID控制器稳定pitch
    void stabilizePitch() {
        // 计算PID输出
        double pid_output = pitch_pid->compute(current_pitch);
        
        // 应用电机系数调整
        double adjusted_output3 = pid_output * motor3_factor;
        double adjusted_output4 = pid_output * motor4_factor;
        
        // 根据PID输出调整两个电机的差值
        uint16_t throttle_ch3 = base_throttle - adjusted_output3;
        uint16_t throttle_ch4 = base_throttle + adjusted_output4;
        
        // 确保油门值在安全范围内
        throttle_ch3 = std::max((uint16_t)1000, std::min((uint16_t)2000, throttle_ch3));
        throttle_ch4 = std::max((uint16_t)1000, std::min((uint16_t)2000, throttle_ch4));
        
        // 控制电机
        controlMotors(throttle_ch3, throttle_ch4);
        
        // 打印调试信息
        printf("\033[1;32m[PID] Pitch: %.2f°, Error: %.2f°, PID: %.2f, Motors: %d, %d\033[0m\n", 
               current_pitch * 180.0 / M_PI, 
               (0.0 - current_pitch) * 180.0 / M_PI,
               pid_output, 
               throttle_ch3, throttle_ch4);
        fflush(stdout);
    }

    // 设置电机响应系数
    void setMotorFactors(double factor3, double factor4) {
        motor3_factor = factor3;
        motor4_factor = factor4;
        ROS_INFO("设置电机系数 - 电机3: %.2f, 电机4: %.2f", motor3_factor, motor4_factor);
    }

    // 运行演示
    void runDemo() {
        if (!armVehicle()) {
            ROS_ERROR("飞控解锁失败，终止演示");
            return;
        }
        
        ROS_INFO("开始PID稳定pitch控制，按Ctrl+C终止");
        try {
            // 等待一段时间以确保收到IMU数据
            ros::Duration(1.0).sleep();
            
            // 如果观察到电机4反应更大，可以调整电机系数
            // 例如: setMotorFactors(1.0, 0.8);
            
            // 持续运行，直到收到终止信号
            while (ros::ok()) {
                stabilizePitch();
                ros::spinOnce();
                rate->sleep();
            }
            
            // 停止电机 (使用1500是停止值)
            controlMotors(1500, 1500);
            ROS_INFO("控制终止");
            
        } catch (ros::Exception& e) {
            ROS_ERROR("发生错误: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");

    ros::init(argc, argv, "flight_controller_interface");
    
    try {
        FlightControllerInterface controller;
        controller.runDemo();
    } catch (ros::Exception& e) {
        ROS_ERROR("错误: %s", e.what());
    }
    
    return 0;
}


////////////////////////////////////////成功控制3，4电机第一版////////////////////////////////////////////////////////
// #include <ros/ros.h>
// #include <mavros_msgs/OverrideRCIn.h>
// #include <mavros_msgs/CommandBool.h>
// #include <ctime>

// class FlightControllerInterface {
// private:
//     ros::NodeHandle nh;
//     ros::Publisher rc_pub;
//     ros::ServiceClient arming_client;
//     ros::Rate* rate;

// public:
//     FlightControllerInterface() {
//         // 初始化RC控制发布器
//         rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
        
//         // 等待解锁服务
//         ROS_INFO("等待解锁服务...");
//         ros::service::waitForService("/mavros/cmd/arming");
//         arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
//         ROS_INFO("解锁服务已准备就绪");

//         // 设置发布频率为20Hz
//         rate = new ros::Rate(20);
//     }

//     ~FlightControllerInterface() {
//         delete rate;
//     }

//     // 解锁飞控
//     bool armVehicle() {
//         mavros_msgs::CommandBool arm_cmd;
//         arm_cmd.request.value = true;

//         if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
//             ROS_INFO("飞控解锁成功");
//             return true;
//         } else {
//             ROS_WARN("飞控解锁失败");
//             return false;
//         }
//     }

//     // 控制电机3和电机4
//     void controlMotors(uint16_t throttle_ch3, uint16_t throttle_ch4) {
//         mavros_msgs::OverrideRCIn msg;
        
//         // 填充所有通道为65535（表示"不改变"）
//         for (int i = 0; i < 18; ++i) {
//             msg.channels[i] = 65535;
//         }
        
//         // 设置通道3和通道4的值
//         msg.channels[2] = throttle_ch3;  // 通道3
//         msg.channels[3] = throttle_ch4;  // 通道4
        
//         rc_pub.publish(msg);
//     }

//     // 运行演示
//     void runDemo() {
//         if (!armVehicle()) {
//             ROS_ERROR("飞控解锁失败，终止演示");
//             return;
//         }
        
//         ROS_INFO("开始控制演示");
//         try {
//             // 运行30秒
//             ros::Time start_time = ros::Time::now();
//             while (ros::ok() && (ros::Time::now() - start_time).toSec() < 30.0) {
//                 controlMotors(1400, 1400);  // 设置两个电机为1400
//                 ros::spinOnce();
//                 rate->sleep();
//             }
            
//             // 停止电机
//             controlMotors(1000, 1000);
//             ROS_INFO("演示完成");
            
//         } catch (ros::Exception& e) {
//             ROS_ERROR("发生错误: %s", e.what());
//         }
//     }
// };

// int main(int argc, char **argv) {
//     setlocale(LC_ALL,"");

//     ros::init(argc, argv, "flight_controller_interface");
    
//     try {
//         FlightControllerInterface controller;
//         controller.runDemo();
//     } catch (ros::Exception& e) {
//         ROS_ERROR("错误: %s", e.what());
//     }
    
//     return 0;
// }
