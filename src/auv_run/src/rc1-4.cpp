#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <sstream>

class PIDController {
private:
    double kp, ki, kd;
    double setpoint;
    double integral, prev_error;
    double output_min, output_max;
    double integral_limit; // 添加积分限制
    ros::Time last_time;
    bool first_call;
    
    // 添加存储各项值的变量
    double p_term_last, i_term_last, d_term_last;

public:
    PIDController(double _kp, double _ki, double _kd, double _setpoint, 
                 double _output_min, double _output_max) 
        : kp(_kp), ki(_ki), kd(_kd), setpoint(_setpoint),
          integral(0.0), prev_error(0.0), 
          output_min(_output_min), output_max(_output_max),
          integral_limit(100.0), // 默认积分限制
          first_call(true),
          p_term_last(0.0), i_term_last(0.0), d_term_last(0.0) {}

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
        p_term_last = p_term;
        
        // Integral term with anti-windup
        integral += error * dt;
        
        // Apply integral limits to prevent windup
        if (integral > integral_limit) {
            integral = integral_limit;
        } else if (integral < -integral_limit) {
            integral = -integral_limit;
        }
        
        double i_term = ki * integral;
        i_term_last = i_term;
        
        // Derivative term
        double derivative = 0.0;
        if (dt > 0) {
            derivative = (error - prev_error) / dt;
        }
        double d_term = kd * derivative;
        d_term_last = d_term;
        
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
        p_term_last = i_term_last = d_term_last = 0.0;
    }
    
    // 设置积分限制
    void setIntegralLimit(double limit) {
        integral_limit = limit;
    }
    
    // 设置目标值
    void setSetpoint(double _setpoint) {
        setpoint = _setpoint;
    }
    
    // 获取各项值的方法
    double getLastPTerm() const { return p_term_last; }
    double getLastITerm() const { return i_term_last; }
    double getLastDTerm() const { return d_term_last; }
};

class FlightControllerInterface {
private:
    ros::NodeHandle nh;
    ros::Publisher rc_pub;
    ros::ServiceClient arming_client;
    ros::Subscriber imu_sub;
    ros::Subscriber altitude_sub;
    ros::Rate* rate;
    
    // PID controllers
    PIDController* pitch_pid;
    PIDController* altitude_pid;
    
    // Current pitch angle in radians
    double current_pitch;
    
    // Current altitude in meters
    double current_altitude;
    
    // Target altitude in meters
    double target_altitude;
    
    // Base throttle value
    uint16_t base_throttle;
    
    // 电机输出系数，用于调整电机响应差异
    double motor3_factor;
    double motor4_factor;
    
    // 数据记录相关成员
    std::ofstream data_file;
    std::string csv_filename;
    bool logging_enabled;
    ros::Time start_time;
    
    // PWM smoothing
    uint16_t current_pwm_ch3;
    uint16_t current_pwm_ch4;
    uint16_t target_pwm_ch3;
    uint16_t target_pwm_ch4;
    double pwm_change_rate; // Maximum PWM change per iteration
    
    // Deadzone range for pitch (in radians)
    double pitch_deadzone_min; // -5 degrees in radians
    double pitch_deadzone_max; // 5 degrees in radians
    
    // Deadzone range for altitude (in meters)
    double altitude_deadzone; // 0.08 meters
    
    // PWM deadzone value
    uint16_t pwm_deadzone; // 5 PWM units
    
    // 生成带时间戳的文件名
    std::string generateTimestampedFilename() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        
        std::stringstream ss;
        ss << "pid_data_";
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
        ss << ".csv";
        
        return ss.str();
    }
    
    // 初始化CSV文件
    void initDataLogging() {
        if (logging_enabled) {
            csv_filename = generateTimestampedFilename();
            data_file.open(csv_filename);
            
            // 写入CSV头部
            data_file << "timestamp,pitch_deg,pitch_error_deg,pitch_p,pitch_i,pitch_d,pitch_output,"
                      << "altitude,altitude_error,altitude_p,altitude_i,altitude_d,altitude_output,"
                      << "motor3_pwm,motor4_pwm" << std::endl;
            
            // 记录开始时间
            start_time = ros::Time::now();
            
            ROS_INFO("数据记录已开始，文件名: %s", csv_filename.c_str());
        }
    }
    
    // 记录一行数据
    void logData(double pitch_deg, double pitch_error_deg, double pitch_p, double pitch_i, double pitch_d, 
                double pitch_output, double altitude, double altitude_error, double altitude_p, 
                double altitude_i, double altitude_d, double altitude_output,
                uint16_t motor3_pwm, uint16_t motor4_pwm) {
        if (logging_enabled && data_file.is_open()) {
            // 计算相对时间戳（秒）
            double elapsed = (ros::Time::now() - start_time).toSec();
            
            // 写入数据行
            data_file << std::fixed << std::setprecision(3) 
                     << elapsed << ","
                     << pitch_deg << ","
                     << pitch_error_deg << ","
                     << pitch_p << ","
                     << pitch_i << ","
                     << pitch_d << ","
                     << pitch_output << ","
                     << altitude << ","
                     << altitude_error << ","
                     << altitude_p << ","
                     << altitude_i << ","
                     << altitude_d << ","
                     << altitude_output << ","
                     << motor3_pwm << ","
                     << motor4_pwm << std::endl;
        }
    }

public:
    FlightControllerInterface() : current_pitch(0.0), current_altitude(0.0), target_altitude(0.2),
                                 base_throttle(1500),
                                 motor3_factor(1.0), motor4_factor(1.0),
                                 logging_enabled(true),
                                 current_pwm_ch3(1500), current_pwm_ch4(1500),
                                 target_pwm_ch3(1500), target_pwm_ch4(1500),
                                 pwm_change_rate(2.50),
                                 pitch_deadzone_min(-5.0 * M_PI / 180.0),
                                 pitch_deadzone_max(5.0 * M_PI / 180.0),
                                 altitude_deadzone(0.08),
                                 pwm_deadzone(5) {
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
        
        // 订阅高度数据
        altitude_sub = nh.subscribe("/mavros/global_position/rel_alt", 100, 
                                   &FlightControllerInterface::altitudeCallback, this);
        ROS_INFO("已订阅高度数据");

        // 设置发布频率为100Hz
        rate = new ros::Rate(100);
        
        // 初始化PID控制器 (kp, ki, kd, setpoint, min_output, max_output)
        pitch_pid = new PIDController(120.0, 0.1, 1.0, 0.0, -100.0, 100.0);
        altitude_pid = new PIDController(820.0, 1.0, 5.0, target_altitude, -400.0, 400.0);
        
        // 设置积分限制
        pitch_pid->setIntegralLimit(200.0);
        altitude_pid->setIntegralLimit(50.0);
        
        // 初始化数据记录
        initDataLogging();
    }

    ~FlightControllerInterface() {
        // 关闭数据文件
        if (data_file.is_open()) {
            data_file.close();
            ROS_INFO("数据记录已结束，文件已保存: %s", csv_filename.c_str());
        }
        
        delete rate;
        delete pitch_pid;
        delete altitude_pid;
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
    
    void altitudeCallback(const std_msgs::Float64::ConstPtr& msg) {
        // 注意：这里的高度是相对高度，负值表示在地面以下
        // 我们将其转换为正值，表示深度
        current_altitude = -msg->data;
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
    
    // 应用PWM死区
    int applyPWMDeadzone(double pid_output) {
        // 如果PID输出的绝对值小于死区值，则输出为0
        if (std::abs(pid_output) < pwm_deadzone) {
            return 0;
        } else {
            // 否则，保持原有输出方向，但加上去30电机死区值
            return (pid_output > 0) ? 
                   (pid_output + 30) : 
                   (pid_output - 30);
        }
    }
    
    // 设置目标深度
    void setTargetAltitude(double depth) {
        target_altitude = depth;
        altitude_pid->setSetpoint(depth);
        ROS_INFO("目标深度设置为: %.2f 米", depth);
    }
    
    // 使用PID控制器稳定pitch和深度
    void stabilize() {
        // 计算pitch控制
        double pitch_output = 0.0;
        double pitch_p = 0.0, pitch_i = 0.0, pitch_d = 0.0;
        
        // 计算角度（度）
        double pitch_deg = current_pitch * 180.0 / M_PI;
        double pitch_error_deg = (0.0 - current_pitch) * 180.0 / M_PI;
        
        // 只有当pitch超出死区范围时才应用PID控制
        if (current_pitch < pitch_deadzone_min || current_pitch > pitch_deadzone_max) {
            // 计算PID输出
            pitch_output = pitch_pid->compute(current_pitch);
            
            // 获取PID各项值
            pitch_p = pitch_pid->getLastPTerm();
            pitch_i = pitch_pid->getLastITerm();
            pitch_d = pitch_pid->getLastDTerm();
        } else {
            // 在死区内，重置PID控制器以避免积分累积
            pitch_pid->reset();
        }
        
        // 计算深度控制
        double altitude_output = 0.0;
        double altitude_p = 0.0, altitude_i = 0.0, altitude_d = 0.0;
        double altitude_error = target_altitude - current_altitude;
        
        // 只有当深度误差超出死区范围时才应用PID控制
        if (std::abs(altitude_error) > altitude_deadzone) {
            // 计算PID输出
            altitude_output = altitude_pid->compute(current_altitude);
            
            // 获取PID各项值
            altitude_p = altitude_pid->getLastPTerm();
            altitude_i = altitude_pid->getLastITerm();
            altitude_d = altitude_pid->getLastDTerm();
        } else {
            // 在死区内，重置PID控制器以避免积分累积
            altitude_pid->reset();
        }
        
        // 应用PWM死区到pitch输出
        double adjusted_pitch_output = applyPWMDeadzone(pitch_output);
        
        // 应用电机系数调整
        double adjusted_output3 = adjusted_pitch_output * motor3_factor;
        double adjusted_output4 = adjusted_pitch_output * motor4_factor;
        
        // 设置目标PWM值，结合pitch和深度控制
        // 深度控制会同时影响两个电机
        // Pitch暂时不调，先深度
        // target_pwm_ch3 = base_throttle + adjusted_output3 - altitude_output;
        // target_pwm_ch4 = base_throttle - adjusted_output4 - altitude_output;
        target_pwm_ch3 = base_throttle - altitude_output;
        target_pwm_ch4 = base_throttle - altitude_output;        
        
        // 确保目标油门值在安全范围内
        target_pwm_ch3 = std::max((uint16_t)1000, std::min((uint16_t)2000, target_pwm_ch3));
        target_pwm_ch4 = std::max((uint16_t)1000, std::min((uint16_t)2000, target_pwm_ch4));
        
        // 平滑过渡到目标PWM值
        smoothPWMTransition();
        
        // 记录数据
        logData(pitch_deg, pitch_error_deg, pitch_p, pitch_i, pitch_d, pitch_output,
                current_altitude, altitude_error, altitude_p, altitude_i, altitude_d, altitude_output,
                current_pwm_ch3, current_pwm_ch4);
        
        // 控制电机
        controlMotors(current_pwm_ch3, current_pwm_ch4);
        
        // 打印调试信息
        printf("\033[1;32m[PID] Pitch: %.2f°, Error: %.2f°, PID: %.2f, Depth: %.3fm, Target: %.3fm, Error: %.3fm, PID: %.2f, Motors: %d, %d\033[0m\n", 
               pitch_deg, pitch_error_deg, pitch_output, 
               current_altitude, target_altitude, altitude_error, altitude_output,
               current_pwm_ch3, current_pwm_ch4);
        fflush(stdout);
    }
    
    // 平滑过渡PWM值
    void smoothPWMTransition() {
        // 平滑过渡通道3
        if (current_pwm_ch3 < target_pwm_ch3) {
            current_pwm_ch3 = std::min(target_pwm_ch3, static_cast<uint16_t>(current_pwm_ch3 + pwm_change_rate));
        } else if (current_pwm_ch3 > target_pwm_ch3) {
            current_pwm_ch3 = std::max(target_pwm_ch3, static_cast<uint16_t>(current_pwm_ch3 - pwm_change_rate));
        }
        
        // 平滑过渡通道4
        if (current_pwm_ch4 < target_pwm_ch4) {
            current_pwm_ch4 = std::min(target_pwm_ch4, static_cast<uint16_t>(current_pwm_ch4 + pwm_change_rate));
        } else if (current_pwm_ch4 > target_pwm_ch4) {
            current_pwm_ch4 = std::max(target_pwm_ch4, static_cast<uint16_t>(current_pwm_ch4 - pwm_change_rate));
        }
    }

    // 设置电机响应系数
    void setMotorFactors(double factor3, double factor4) {
        motor3_factor = factor3;
        motor4_factor = factor4;
        ROS_INFO("设置电机系数 - 电机3: %.2f, 电机4: %.2f", motor3_factor, motor4_factor);
    }
    
    // 设置PWM变化率
    void setPWMChangeRate(double rate) {
        pwm_change_rate = rate;
        ROS_INFO("PWM变化率已设置为: %.2f", pwm_change_rate);
    }
    
    // 设置PWM死区
    void setPWMDeadzone(uint16_t deadzone) {
        pwm_deadzone = deadzone;
        ROS_INFO("PWM死区已设置为: %d", pwm_deadzone);
    }
    
    // 设置深度控制死区
    void setAltitudeDeadzone(double deadzone) {
        altitude_deadzone = deadzone;
        ROS_INFO("深度控制死区已设置为: %.3f 米", altitude_deadzone);
    }
    
    // 启用/禁用数据记录
    void enableDataLogging(bool enable) {
        if (enable && !logging_enabled) {
            // 如果之前禁用，现在启用，创建新文件
            logging_enabled = true;
            initDataLogging();
        } else if (!enable && logging_enabled) {
            // 如果之前启用，现在禁用，关闭文件
            logging_enabled = false;
            if (data_file.is_open()) {
                data_file.close();
                ROS_INFO("数据记录已停止，文件已保存: %s", csv_filename.c_str());
            }
        }
    }

    // 运行演示
    void runDemo() {
        if (!armVehicle()) {
            ROS_ERROR("飞控解锁失败，终止演示");
            return;
        }
        
        ROS_INFO("开始PID稳定控制，按Ctrl+C终止");
        ROS_INFO("Pitch死区范围: -5° 到 5°");
        ROS_INFO("深度死区范围: ±%.3f 米", altitude_deadzone);
        ROS_INFO("PWM死区值: %d", pwm_deadzone);
        ROS_INFO("目标深度: %.2f 米", target_altitude);
        
        try {
            // 等待一段时间以确保收到IMU和高度数据
            ros::Duration(1.0).sleep();
            
            // 持续运行，直到收到终止信号
            while (ros::ok()) {
                stabilize();
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
/////////////////////////////////////////////////////////第一次尝试定深控制,现象很晃,这版代码我把Pitch调整注释了,只有定深,还没试/////////////////////////////////////////////////////////////
// #include <ros/ros.h>
// #include <mavros_msgs/OverrideRCIn.h>
// #include <mavros_msgs/CommandBool.h>
// #include <sensor_msgs/Imu.h>
// #include <std_msgs/Float64.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <ctime>
// #include <fstream>
// #include <iomanip>
// #include <chrono>
// #include <sstream>

// class PIDController {
// private:
//     double kp, ki, kd;
//     double setpoint;
//     double integral, prev_error;
//     double output_min, output_max;
//     double integral_limit; // 添加积分限制
//     ros::Time last_time;
//     bool first_call;
    
//     // 添加存储各项值的变量
//     double p_term_last, i_term_last, d_term_last;

// public:
//     PIDController(double _kp, double _ki, double _kd, double _setpoint, 
//                  double _output_min, double _output_max) 
//         : kp(_kp), ki(_ki), kd(_kd), setpoint(_setpoint),
//           integral(0.0), prev_error(0.0), 
//           output_min(_output_min), output_max(_output_max),
//           integral_limit(100.0), // 默认积分限制
//           first_call(true),
//           p_term_last(0.0), i_term_last(0.0), d_term_last(0.0) {}

//     double compute(double measurement) {
//         ros::Time current_time = ros::Time::now();
//         double dt;
        
//         if (first_call) {
//             dt = 0.0;
//             first_call = false;
//         } else {
//             dt = (current_time - last_time).toSec();
//         }
//         last_time = current_time;
        
//         // Avoid division by zero
//         if (dt <= 0.0) return 0.0;
        
//         // Calculate error
//         double error = setpoint - measurement;
        
//         // Proportional term
//         double p_term = kp * error;
//         p_term_last = p_term;
        
//         // Integral term with anti-windup
//         integral += error * dt;
        
//         // Apply integral limits to prevent windup
//         if (integral > integral_limit) {
//             integral = integral_limit;
//         } else if (integral < -integral_limit) {
//             integral = -integral_limit;
//         }
        
//         double i_term = ki * integral;
//         i_term_last = i_term;
        
//         // Derivative term
//         double derivative = 0.0;
//         if (dt > 0) {
//             derivative = (error - prev_error) / dt;
//         }
//         double d_term = kd * derivative;
//         d_term_last = d_term;
        
//         // Calculate total output
//         double output = p_term + i_term + d_term;
        
//         // Apply output limits
//         if (output > output_max) {
//             output = output_max;
//         } else if (output < output_min) {
//             output = output_min;
//         }
        
//         // Save error for next iteration
//         prev_error = error;
        
//         // 输出各项贡献，便于调试
//         ROS_DEBUG("PID components: P=%.2f, I=%.2f, D=%.2f", p_term, i_term, d_term);
        
//         return output;
//     }
    
//     void reset() {
//         integral = 0.0;
//         prev_error = 0.0;
//         first_call = true;
//         p_term_last = i_term_last = d_term_last = 0.0;
//     }
    
//     // 设置积分限制
//     void setIntegralLimit(double limit) {
//         integral_limit = limit;
//     }
    
//     // 设置目标值
//     void setSetpoint(double _setpoint) {
//         setpoint = _setpoint;
//     }
    
//     // 获取各项值的方法
//     double getLastPTerm() const { return p_term_last; }
//     double getLastITerm() const { return i_term_last; }
//     double getLastDTerm() const { return d_term_last; }
// };

// class FlightControllerInterface {
// private:
//     ros::NodeHandle nh;
//     ros::Publisher rc_pub;
//     ros::ServiceClient arming_client;
//     ros::Subscriber imu_sub;
//     ros::Subscriber altitude_sub;
//     ros::Rate* rate;
    
//     // PID controllers
//     PIDController* pitch_pid;
//     PIDController* altitude_pid;
    
//     // Current pitch angle in radians
//     double current_pitch;
    
//     // Current altitude in meters
//     double current_altitude;
    
//     // Target altitude in meters
//     double target_altitude;
    
//     // Base throttle value
//     uint16_t base_throttle;
    
//     // 电机输出系数，用于调整电机响应差异
//     double motor3_factor;
//     double motor4_factor;
    
//     // 数据记录相关成员
//     std::ofstream data_file;
//     std::string csv_filename;
//     bool logging_enabled;
//     ros::Time start_time;
    
//     // PWM smoothing
//     uint16_t current_pwm_ch3;
//     uint16_t current_pwm_ch4;
//     uint16_t target_pwm_ch3;
//     uint16_t target_pwm_ch4;
//     double pwm_change_rate; // Maximum PWM change per iteration
    
//     // Deadzone range for pitch (in radians)
//     double pitch_deadzone_min; // -3 degrees in radians
//     double pitch_deadzone_max; // 3 degrees in radians
    
//     // PWM deadzone value
//     uint16_t pwm_deadzone; // 10PWM units
    
//     // 生成带时间戳的文件名
//     std::string generateTimestampedFilename() {
//         auto now = std::chrono::system_clock::now();
//         auto in_time_t = std::chrono::system_clock::to_time_t(now);
        
//         std::stringstream ss;
//         ss << "pid_data_";
//         ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
//         ss << ".csv";
        
//         return ss.str();
//     }
    
//     // 初始化CSV文件
//     void initDataLogging() {
//         if (logging_enabled) {
//             csv_filename = generateTimestampedFilename();
//             data_file.open(csv_filename);
            
//             // 写入CSV头部
//             data_file << "timestamp,pitch_deg,pitch_error_deg,pitch_p,pitch_i,pitch_d,pitch_output,"
//                       << "altitude,altitude_error,altitude_p,altitude_i,altitude_d,altitude_output,"
//                       << "motor3_pwm,motor4_pwm" << std::endl;
            
//             // 记录开始时间
//             start_time = ros::Time::now();
            
//             ROS_INFO("数据记录已开始，文件名: %s", csv_filename.c_str());
//         }
//     }
    
//     // 记录一行数据
//     void logData(double pitch_deg, double pitch_error_deg, double pitch_p, double pitch_i, double pitch_d, 
//                 double pitch_output, double altitude, double altitude_error, double altitude_p, 
//                 double altitude_i, double altitude_d, double altitude_output,
//                 uint16_t motor3_pwm, uint16_t motor4_pwm) {
//         if (logging_enabled && data_file.is_open()) {
//             // 计算相对时间戳（秒）
//             double elapsed = (ros::Time::now() - start_time).toSec();
            
//             // 写入数据行
//             data_file << std::fixed << std::setprecision(3) 
//                      << elapsed << ","
//                      << pitch_deg << ","
//                      << pitch_error_deg << ","
//                      << pitch_p << ","
//                      << pitch_i << ","
//                      << pitch_d << ","
//                      << pitch_output << ","
//                      << altitude << ","
//                      << altitude_error << ","
//                      << altitude_p << ","
//                      << altitude_i << ","
//                      << altitude_d << ","
//                      << altitude_output << ","
//                      << motor3_pwm << ","
//                      << motor4_pwm << std::endl;
//         }
//     }

// public:
//     FlightControllerInterface() : current_pitch(0.0), current_altitude(0.0), target_altitude(2),
//                                  base_throttle(1500),
//                                  motor3_factor(1.0), motor4_factor(1.0),
//                                  logging_enabled(true),
//                                  current_pwm_ch3(1500), current_pwm_ch4(1500),
//                                  target_pwm_ch3(1500), target_pwm_ch4(1500),
//                                 //  pwm_change_rate(5.0),
//                                  pwm_change_rate(2.50),
//                                  pitch_deadzone_min(-5.0 * M_PI / 180.0),
//                                  pitch_deadzone_max(5.0 * M_PI / 180.0),
//                                  pwm_deadzone(5) {
//         // 初始化RC控制发布器
//         rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 100);
        
//         // 等待解锁服务
//         ROS_INFO("等待解锁服务...");
//         ros::service::waitForService("/mavros/cmd/arming");
//         arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
//         ROS_INFO("解锁服务已准备就绪");
        
//         // 订阅IMU数据
//         imu_sub = nh.subscribe("/mavros/imu/data", 100, &FlightControllerInterface::imuCallback, this);
//         ROS_INFO("已订阅IMU数据");
        
//         // 订阅高度数据
//         altitude_sub = nh.subscribe("/mavros/global_position/rel_alt", 100, 
//                                    &FlightControllerInterface::altitudeCallback, this);
//         ROS_INFO("已订阅高度数据");

//         // 设置发布频率为100Hz
//         rate = new ros::Rate(100);
        
//         // 初始化PID控制器 (kp, ki, kd, setpoint, min_output, max_output)
//         // pitch_pid = new PIDController(350.0, 30, 40.0, 0.0, -400.0, 400.0);
//         pitch_pid = new PIDController(120.0, 0.1, 1.0, 0.0, -100.0, 100.0);
//         altitude_pid = new PIDController(820.0, 1.0, 5.0, target_altitude, -400.0, 400.0);
        
//         // 设置积分限制
//         pitch_pid->setIntegralLimit(200.0);
//         altitude_pid->setIntegralLimit(50.0);
        
//         // 初始化数据记录
//         initDataLogging();
//     }

//     ~FlightControllerInterface() {
//         // 关闭数据文件
//         if (data_file.is_open()) {
//             data_file.close();
//             ROS_INFO("数据记录已结束，文件已保存: %s", csv_filename.c_str());
//         }
        
//         delete rate;
//         delete pitch_pid;
//         delete altitude_pid;
//     }
    
//     void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
//         // 从四元数中提取欧拉角
//         tf2::Quaternion q(
//             msg->orientation.x,
//             msg->orientation.y,
//             msg->orientation.z,
//             msg->orientation.w
//         );
        
//         double roll, pitch, yaw;
//         tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
//         // 保存当前pitch角度
//         current_pitch = pitch;
//     }
    
//     void altitudeCallback(const std_msgs::Float64::ConstPtr& msg) {
//         // 注意：这里的高度是相对高度，负值表示在地面以下
//         // 我们将其转换为正值，表示深度
//         current_altitude = -msg->data;
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
    
//     // 应用PWM死区
//     int applyPWMDeadzone(double pid_output) {
//         // 如果PID输出的绝对值小于死区值，则输出为0
//         if (std::abs(pid_output) < pwm_deadzone) {
//             return 0;
//         } else {
//             // 否则，保持原有输出方向，但加上去30电机死区值
//             return (pid_output > 0) ? 
//                    (pid_output + 30) : 
//                    (pid_output - 30);
//         }
//     }
    
//     // 设置目标深度
//     void setTargetAltitude(double depth) {
//         target_altitude = depth;
//         altitude_pid->setSetpoint(depth);
//         ROS_INFO("目标深度设置为: %.2f 米", depth);
//     }
    
//     // 使用PID控制器稳定pitch和深度
//     void stabilize() {
//         // 计算pitch控制
//         double pitch_output = 0.0;
//         double pitch_p = 0.0, pitch_i = 0.0, pitch_d = 0.0;
        
//         // 计算角度（度）
//         double pitch_deg = current_pitch * 180.0 / M_PI;
//         double pitch_error_deg = (0.0 - current_pitch) * 180.0 / M_PI;
        
//         // 只有当pitch超出死区范围时才应用PID控制
//         if (current_pitch < pitch_deadzone_min || current_pitch > pitch_deadzone_max) {
//             // 计算PID输出
//             pitch_output = pitch_pid->compute(current_pitch);
            
//             // 获取PID各项值
//             pitch_p = pitch_pid->getLastPTerm();
//             pitch_i = pitch_pid->getLastITerm();
//             pitch_d = pitch_pid->getLastDTerm();
//         } else {
//             // 在死区内，重置PID控制器以避免积分累积
//             pitch_pid->reset();
//         }
        
//         // 计算深度控制
//         double altitude_output = altitude_pid->compute(current_altitude);
//         double altitude_p = altitude_pid->getLastPTerm();
//         double altitude_i = altitude_pid->getLastITerm();
//         double altitude_d = altitude_pid->getLastDTerm();
//         double altitude_error = target_altitude - current_altitude;
        
//         // 应用PWM死区到pitch输出
//         double adjusted_pitch_output = applyPWMDeadzone(pitch_output);
        
//         // 应用电机系数调整
//         double adjusted_output3 = adjusted_pitch_output * motor3_factor;
//         double adjusted_output4 = adjusted_pitch_output * motor4_factor;
        
//         // 设置目标PWM值，结合pitch和深度控制
//         // 深度控制会同时影响两个电机
//         // target_pwm_ch3 = base_throttle + adjusted_output3 - altitude_output;
//         // target_pwm_ch4 = base_throttle - adjusted_output4 - altitude_output;
//         //Pitch暂时不调，先深度
//         target_pwm_ch3 = base_throttle  - altitude_output;
//         target_pwm_ch4 = base_throttle  - altitude_output;        
//         // 确保目标油门值在安全范围内
//         target_pwm_ch3 = std::max((uint16_t)1000, std::min((uint16_t)2000, target_pwm_ch3));
//         target_pwm_ch4 = std::max((uint16_t)1000, std::min((uint16_t)2000, target_pwm_ch4));
        
//         // 平滑过渡到目标PWM值
//         smoothPWMTransition();
        
//         // 记录数据
//         logData(pitch_deg, pitch_error_deg, pitch_p, pitch_i, pitch_d, pitch_output,
//                 current_altitude, altitude_error, altitude_p, altitude_i, altitude_d, altitude_output,
//                 current_pwm_ch3, current_pwm_ch4);
        
//         // 控制电机
//         controlMotors(current_pwm_ch3, current_pwm_ch4);
        
//         // 打印调试信息
//         printf("\033[1;32m[PID] Pitch: %.2f°, Error: %.2f°, PID: %.2f, Depth: %.3fm, Target: %.3fm, PID: %.2f, Motors: %d, %d\033[0m\n", 
//                pitch_deg, pitch_error_deg, pitch_output, 
//                current_altitude, target_altitude, altitude_output,
//                current_pwm_ch3, current_pwm_ch4);
//         fflush(stdout);
//     }
    
//     // 平滑过渡PWM值
//     void smoothPWMTransition() {
//         // 平滑过渡通道3
//         if (current_pwm_ch3 < target_pwm_ch3) {
//             current_pwm_ch3 = std::min(target_pwm_ch3, static_cast<uint16_t>(current_pwm_ch3 + pwm_change_rate));
//         } else if (current_pwm_ch3 > target_pwm_ch3) {
//             current_pwm_ch3 = std::max(target_pwm_ch3, static_cast<uint16_t>(current_pwm_ch3 - pwm_change_rate));
//         }
        
//         // 平滑过渡通道4
//         if (current_pwm_ch4 < target_pwm_ch4) {
//             current_pwm_ch4 = std::min(target_pwm_ch4, static_cast<uint16_t>(current_pwm_ch4 + pwm_change_rate));
//         } else if (current_pwm_ch4 > target_pwm_ch4) {
//             current_pwm_ch4 = std::max(target_pwm_ch4, static_cast<uint16_t>(current_pwm_ch4 - pwm_change_rate));
//         }
//     }

//     // 设置电机响应系数
//     void setMotorFactors(double factor3, double factor4) {
//         motor3_factor = factor3;
//         motor4_factor = factor4;
//         ROS_INFO("设置电机系数 - 电机3: %.2f, 电机4: %.2f", motor3_factor, motor4_factor);
//     }
    
//     // 设置PWM变化率
//     void setPWMChangeRate(double rate) {
//         pwm_change_rate = rate;
//         ROS_INFO("PWM变化率已设置为: %.2f", pwm_change_rate);
//     }
    
//     // 设置PWM死区
//     void setPWMDeadzone(uint16_t deadzone) {
//         pwm_deadzone = deadzone;
//         ROS_INFO("PWM死区已设置为: %d", pwm_deadzone);
//     }
    
//     // 启用/禁用数据记录
//     void enableDataLogging(bool enable) {
//         if (enable && !logging_enabled) {
//             // 如果之前禁用，现在启用，创建新文件
//             logging_enabled = true;
//             initDataLogging();
//         } else if (!enable && logging_enabled) {
//             // 如果之前启用，现在禁用，关闭文件
//             logging_enabled = false;
//             if (data_file.is_open()) {
//                 data_file.close();
//                 ROS_INFO("数据记录已停止，文件已保存: %s", csv_filename.c_str());
//             }
//         }
//     }

//     // 运行演示
//     void runDemo() {
//         if (!armVehicle()) {
//             ROS_ERROR("飞控解锁失败，终止演示");
//             return;
//         }
        
//         ROS_INFO("开始PID稳定控制，按Ctrl+C终止");
//         ROS_INFO("Pitch死区范围: -3° 到 3°");
//         ROS_INFO("PWM死区值: %d", pwm_deadzone);
//         ROS_INFO("目标深度: %.2f 米", target_altitude);
        
//         try {
//             // 等待一段时间以确保收到IMU和高度数据
//             ros::Duration(1.0).sleep();
            
//             // 持续运行，直到收到终止信号
//             while (ros::ok()) {
//                 stabilize();
//                 ros::spinOnce();
//                 rate->sleep();
//             }
            
//             // 停止电机 (使用1500是停止值)
//             controlMotors(1500, 1500);
//             ROS_INFO("控制终止");
            
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

// #include <ros/ros.h>
// #include <mavros_msgs/OverrideRCIn.h>
// #include <mavros_msgs/CommandBool.h>
// #include <sensor_msgs/Imu.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <ctime>
// #include <fstream>
// #include <iomanip>
// #include <chrono>
// #include <sstream>

// class PIDController {
// private:
//     double kp, ki, kd;
//     double setpoint;
//     double integral, prev_error;
//     double output_min, output_max;
//     double integral_limit; // 添加积分限制
//     ros::Time last_time;
//     bool first_call;
    
//     // 添加存储各项值的变量
//     double p_term_last, i_term_last, d_term_last;

// public:
//     PIDController(double _kp, double _ki, double _kd, double _setpoint, 
//                  double _output_min, double _output_max) 
//         : kp(_kp), ki(_ki), kd(_kd), setpoint(_setpoint),
//           integral(0.0), prev_error(0.0), 
//           output_min(_output_min), output_max(_output_max),
//           integral_limit(100.0), // 默认积分限制
//           first_call(true),
//           p_term_last(0.0), i_term_last(0.0), d_term_last(0.0) {}

//     double compute(double measurement) {
//         ros::Time current_time = ros::Time::now();
//         double dt;
        
//         if (first_call) {
//             dt = 0.0;
//             first_call = false;
//         } else {
//             dt = (current_time - last_time).toSec();
//         }
//         last_time = current_time;
        
//         // Avoid division by zero
//         if (dt <= 0.0) return 0.0;
        
//         // Calculate error
//         double error = setpoint - measurement;
        
//         // Proportional term
//         double p_term = kp * error;
//         p_term_last = p_term;
        
//         // Integral term with anti-windup
//         integral += error * dt;
        
//         // Apply integral limits to prevent windup
//         if (integral > integral_limit) {
//             integral = integral_limit;
//         } else if (integral < -integral_limit) {
//             integral = -integral_limit;
//         }
        
//         double i_term = ki * integral;
//         i_term_last = i_term;
        
//         // Derivative term
//         double derivative = 0.0;
//         if (dt > 0) {
//             derivative = (error - prev_error) / dt;
//         }
//         double d_term = kd * derivative;
//         d_term_last = d_term;
        
//         // Calculate total output
//         double output = p_term + i_term + d_term;
        
//         // Apply output limits
//         if (output > output_max) {
//             output = output_max;
//         } else if (output < output_min) {
//             output = output_min;
//         }
        
//         // Save error for next iteration
//         prev_error = error;
        
//         // 输出各项贡献，便于调试
//         ROS_DEBUG("PID components: P=%.2f, I=%.2f, D=%.2f", p_term, i_term, d_term);
        
//         return output;
//     }
    
//     void reset() {
//         integral = 0.0;
//         prev_error = 0.0;
//         first_call = true;
//         p_term_last = i_term_last = d_term_last = 0.0;
//     }
    
//     // 设置积分限制
//     void setIntegralLimit(double limit) {
//         integral_limit = limit;
//     }
    
//     // 获取各项值的方法
//     double getLastPTerm() const { return p_term_last; }
//     double getLastITerm() const { return i_term_last; }
//     double getLastDTerm() const { return d_term_last; }
// };

// class FlightControllerInterface {
// private:
//     ros::NodeHandle nh;
//     ros::Publisher rc_pub;
//     ros::ServiceClient arming_client;
//     ros::Subscriber imu_sub;
//     ros::Rate* rate;
    
//     // PID controller for pitch stabilization
//     PIDController* pitch_pid;
    
//     // Current pitch angle in radians
//     double current_pitch;
    
//     // Base throttle value
//     uint16_t base_throttle;
    
//     // 电机输出系数，用于调整电机响应差异
//     double motor3_factor;
//     double motor4_factor;
    
//     // 数据记录相关成员
//     std::ofstream data_file;
//     std::string csv_filename;
//     bool logging_enabled;
//     ros::Time start_time;
    
//     // PWM smoothing
//     uint16_t current_pwm_ch3;
//     uint16_t current_pwm_ch4;
//     uint16_t target_pwm_ch3;
//     uint16_t target_pwm_ch4;
//     double pwm_change_rate; // Maximum PWM change per iteration
    
//     // Deadzone range for pitch (in radians)
//     double pitch_deadzone_min; // -3 degrees in radians
//     double pitch_deadzone_max; // 3 degrees in radians
    
//     // PWM deadzone value
//     uint16_t pwm_deadzone;
    
//     // 生成带时间戳的文件名
//     std::string generateTimestampedFilename() {
//         auto now = std::chrono::system_clock::now();
//         auto in_time_t = std::chrono::system_clock::to_time_t(now);
        
//         std::stringstream ss;
//         ss << "pid_data_";
//         ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
//         ss << ".csv";
        
//         return ss.str();
//     }
    
//     // 初始化CSV文件
//     void initDataLogging() {
//         if (logging_enabled) {
//             csv_filename = generateTimestampedFilename();
//             data_file.open(csv_filename);
            
//             // 写入CSV头部
//             data_file << "timestamp,pitch_deg,error_deg,p_term,i_term,d_term,pid_output,motor3_pwm,motor4_pwm" << std::endl;
            
//             // 记录开始时间
//             start_time = ros::Time::now();
            
//             ROS_INFO("数据记录已开始，文件名: %s", csv_filename.c_str());
//         }
//     }
    
//     // 记录一行数据
//     void logData(double pitch_deg, double error_deg, double p_term, double i_term, double d_term, 
//                 double pid_output, uint16_t motor3_pwm, uint16_t motor4_pwm) {
//         if (logging_enabled && data_file.is_open()) {
//             // 计算相对时间戳（秒）
//             double elapsed = (ros::Time::now() - start_time).toSec();
            
//             // 写入数据行
//             data_file << std::fixed << std::setprecision(3) 
//                      << elapsed << ","
//                      << pitch_deg << ","
//                      << error_deg << ","
//                      << p_term << ","
//                      << i_term << ","
//                      << d_term << ","
//                      << pid_output << ","
//                      << motor3_pwm << ","
//                      << motor4_pwm << std::endl;
//         }
//     }

// public:
//     FlightControllerInterface() : current_pitch(0.0), base_throttle(1500),
//                                  motor3_factor(1.0), motor4_factor(1.0),
//                                  logging_enabled(true),
//                                  current_pwm_ch3(1500), current_pwm_ch4(1500),
//                                  target_pwm_ch3(1500), target_pwm_ch4(1500),
//                                 //  pwm_change_rate(5),
//                                  pwm_change_rate(2.50),
//                                  pitch_deadzone_min(-5.0 * M_PI / 180.0),
//                                  pitch_deadzone_max(5.0 * M_PI / 180.0),
//                                  pwm_deadzone(5) {
//         // 初始化RC控制发布器
//         rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 100);
        
//         // 等待解锁服务
//         ROS_INFO("等待解锁服务...");
//         ros::service::waitForService("/mavros/cmd/arming");
//         arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
//         ROS_INFO("解锁服务已准备就绪");
        
//         // 订阅IMU数据
//         imu_sub = nh.subscribe("/mavros/imu/data", 100, &FlightControllerInterface::imuCallback, this);
//         ROS_INFO("已订阅IMU数据");

//         // 设置发布频率为100Hz
//         rate = new ros::Rate(100);
        
//         // 初始化PID控制器 (kp, ki, kd, setpoint, min_output, max_output)
//         // pitch_pid = new PIDController(350.0, 30, 40.0, 0.0, -400.0, 400.0);
//         pitch_pid = new PIDController(120.0, 0.1, 1.0, 0.0, -400.0, 400.0);

        
//         // 设置积分限制
//         pitch_pid->setIntegralLimit(200.0);
        
//         // 初始化数据记录
//         initDataLogging();
//     }

//     ~FlightControllerInterface() {
//         // 关闭数据文件
//         if (data_file.is_open()) {
//             data_file.close();
//             ROS_INFO("数据记录已结束，文件已保存: %s", csv_filename.c_str());
//         }
        
//         delete rate;
//         delete pitch_pid;
//     }
    
//     void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
//         // 从四元数中提取欧拉角
//         tf2::Quaternion q(
//             msg->orientation.x,
//             msg->orientation.y,
//             msg->orientation.z,
//             msg->orientation.w
//         );
        
//         double roll, pitch, yaw;
//         tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
//         // 保存当前pitch角度
//         current_pitch = pitch;
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
    
//     // 应用PWM死区
//     int applyPWMDeadzone(double pid_output) {
//         // 如果PID输出的绝对值小于死区值，则输出为0
//         if (std::abs(pid_output) < pwm_deadzone) {
//             return 0;
//         } else {
//             // 否则，保持原有输出方向，但加上去30电机死区值
//             return (pid_output > 0) ? 
//                    (pid_output + 30) : 
//                    (pid_output - 30);
//         }
//     }
    
//     // 使用PID控制器稳定pitch
//     void stabilizePitch() {
//         double pid_output = 0.0;
//         double p_term = 0.0, i_term = 0.0, d_term = 0.0;
        
//         // 计算角度（度）
//         double pitch_deg = current_pitch * 180.0 / M_PI;
//         double error_deg = (0.0 - current_pitch) * 180.0 / M_PI;
        
//         // 只有当pitch超出死区范围时才应用PID控制
//         if (current_pitch < pitch_deadzone_min || current_pitch > pitch_deadzone_max) {
//             // 计算PID输出
//             pid_output = pitch_pid->compute(current_pitch);
            
//             // 获取PID各项值
//             p_term = pitch_pid->getLastPTerm();
//             i_term = pitch_pid->getLastITerm();
//             d_term = pitch_pid->getLastDTerm();
            
//             // 应用PWM死区
//             double adjusted_pid_output = applyPWMDeadzone(pid_output);
            
//             // 应用电机系数调整
//             double adjusted_output3 = adjusted_pid_output * motor3_factor;
//             double adjusted_output4 = adjusted_pid_output * motor4_factor;
            
//             // 设置目标PWM值
//             target_pwm_ch3 = base_throttle + adjusted_output3;
//             target_pwm_ch4 = base_throttle - adjusted_output4;
//         } else {
//             // 在死区内，保持当前状态
//             pitch_pid->reset(); // 重置PID控制器以避免积分累积
//             // 目标值保持不变
//         }
        
//         // 确保目标油门值在安全范围内
//         target_pwm_ch3 = std::max((uint16_t)1000, std::min((uint16_t)2000, target_pwm_ch3));
//         target_pwm_ch4 = std::max((uint16_t)1000, std::min((uint16_t)2000, target_pwm_ch4));
        
//         // 平滑过渡到目标PWM值
//         smoothPWMTransition();
        
//         // 记录数据
//         logData(pitch_deg, error_deg, p_term, i_term, d_term, pid_output, current_pwm_ch3, current_pwm_ch4);
        
//         // 控制电机
//         controlMotors(current_pwm_ch3, current_pwm_ch4);
        
//         // 打印调试信息
//         printf("\033[1;32m[PID] Pitch: %.2f°, Error: %.2f°, PID: %.2f, Motors: %d, %d\033[0m\n", 
//                pitch_deg, error_deg, pid_output, current_pwm_ch3, current_pwm_ch4);
//         fflush(stdout);
//     }
    
//     // 平滑过渡PWM值，后续可以采用更加高级的指数平滑过渡
//     void smoothPWMTransition() {
//         // 平滑过渡通道3
//         if (current_pwm_ch3 < target_pwm_ch3) {
//             current_pwm_ch3 = std::min(target_pwm_ch3, static_cast<uint16_t>(current_pwm_ch3 + pwm_change_rate));
//         } else if (current_pwm_ch3 > target_pwm_ch3) {
//             current_pwm_ch3 = std::max(target_pwm_ch3, static_cast<uint16_t>(current_pwm_ch3 - pwm_change_rate));
//         }
        
//         // 平滑过渡通道4
//         if (current_pwm_ch4 < target_pwm_ch4) {
//             current_pwm_ch4 = std::min(target_pwm_ch4, static_cast<uint16_t>(current_pwm_ch4 + pwm_change_rate));
//         } else if (current_pwm_ch4 > target_pwm_ch4) {
//             current_pwm_ch4 = std::max(target_pwm_ch4, static_cast<uint16_t>(current_pwm_ch4 - pwm_change_rate));
//         }
//     }

//     // 设置电机响应系数
//     void setMotorFactors(double factor3, double factor4) {
//         motor3_factor = factor3;
//         motor4_factor = factor4;
//         ROS_INFO("设置电机系数 - 电机3: %.2f, 电机4: %.2f", motor3_factor, motor4_factor);
//     }
    
//     // 设置PWM变化率
//     void setPWMChangeRate(double rate) {
//         pwm_change_rate = rate;
//         ROS_INFO("PWM变化率已设置为: %.2f", pwm_change_rate);
//     }
    
//     // 设置PWM死区
//     void setPWMDeadzone(uint16_t deadzone) {
//         pwm_deadzone = deadzone;
//         ROS_INFO("PWM死区已设置为: %d", pwm_deadzone);
//     }
    
//     // 启用/禁用数据记录
//     void enableDataLogging(bool enable) {
//         if (enable && !logging_enabled) {
//             // 如果之前禁用，现在启用，创建新文件
//             logging_enabled = true;
//             initDataLogging();
//         } else if (!enable && logging_enabled) {
//             // 如果之前启用，现在禁用，关闭文件
//             logging_enabled = false;
//             if (data_file.is_open()) {
//                 data_file.close();
//                 ROS_INFO("数据记录已停止，文件已保存: %s", csv_filename.c_str());
//             }
//         }
//     }

//     // 运行演示
//     void runDemo() {
//         if (!armVehicle()) {
//             ROS_ERROR("飞控解锁失败，终止演示");
//             return;
//         }
        
//         ROS_INFO("开始PID稳定pitch控制，按Ctrl+C终止");
//         ROS_INFO("Pitch死区范围: -3° 到 3°");
//         ROS_INFO("PWM死区值: %d", pwm_deadzone);
        
//         try {
//             // 等待一段时间以确保收到IMU数据
//             ros::Duration(1.0).sleep();
            
//             // 如果观察到电机4反应更大，可以调整电机系数
//             // 例如: setMotorFactors(1.0, 0.8);
            
//             // 持续运行，直到收到终止信号
//             while (ros::ok()) {
//                 stabilizePitch();
//                 ros::spinOnce();
//                 rate->sleep();
//             }
            
//             // 停止电机 (使用1500是停止值)
//             controlMotors(1500, 1500);
//             ROS_INFO("控制终止");
            
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

//////////////////////////////////////////////////加入死区，并且pitch改为-3到3/////////////////////////////////////////////////////////
// #include <ros/ros.h>
// #include <mavros_msgs/OverrideRCIn.h>
// #include <mavros_msgs/CommandBool.h>
// #include <sensor_msgs/Imu.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <ctime>
// #include <fstream>
// #include <iomanip>
// #include <chrono>
// #include <sstream>

// class PIDController {
// private:
//     double kp, ki, kd;
//     double setpoint;
//     double integral, prev_error;
//     double output_min, output_max;
//     double integral_limit; // 添加积分限制
//     ros::Time last_time;
//     bool first_call;
    
//     // 添加存储各项值的变量
//     double p_term_last, i_term_last, d_term_last;

// public:
//     PIDController(double _kp, double _ki, double _kd, double _setpoint, 
//                  double _output_min, double _output_max) 
//         : kp(_kp), ki(_ki), kd(_kd), setpoint(_setpoint),
//           integral(0.0), prev_error(0.0), 
//           output_min(_output_min), output_max(_output_max),
//           integral_limit(100.0), // 默认积分限制
//           first_call(true),
//           p_term_last(0.0), i_term_last(0.0), d_term_last(0.0) {}

//     double compute(double measurement) {
//         ros::Time current_time = ros::Time::now();
//         double dt;
        
//         if (first_call) {
//             dt = 0.0;
//             first_call = false;
//         } else {
//             dt = (current_time - last_time).toSec();
//         }
//         last_time = current_time;
        
//         // Avoid division by zero
//         if (dt <= 0.0) return 0.0;
        
//         // Calculate error
//         double error = setpoint - measurement;
        
//         // Proportional term
//         double p_term = kp * error;
//         p_term_last = p_term;
        
//         // Integral term with anti-windup
//         integral += error * dt;
        
//         // Apply integral limits to prevent windup
//         if (integral > integral_limit) {
//             integral = integral_limit;
//         } else if (integral < -integral_limit) {
//             integral = -integral_limit;
//         }
        
//         double i_term = ki * integral;
//         i_term_last = i_term;
        
//         // Derivative term
//         double derivative = 0.0;
//         if (dt > 0) {
//             derivative = (error - prev_error) / dt;
//         }
//         double d_term = kd * derivative;
//         d_term_last = d_term;
        
//         // Calculate total output
//         double output = p_term + i_term + d_term;
        
//         // Apply output limits
//         if (output > output_max) {
//             output = output_max;
//         } else if (output < output_min) {
//             output = output_min;
//         }
        
//         // Save error for next iteration
//         prev_error = error;
        
//         // 输出各项贡献，便于调试
//         ROS_DEBUG("PID components: P=%.2f, I=%.2f, D=%.2f", p_term, i_term, d_term);
        
//         return output;
//     }
    
//     void reset() {
//         integral = 0.0;
//         prev_error = 0.0;
//         first_call = true;
//         p_term_last = i_term_last = d_term_last = 0.0;
//     }
    
//     // 设置积分限制
//     void setIntegralLimit(double limit) {
//         integral_limit = limit;
//     }
    
//     // 获取各项值的方法
//     double getLastPTerm() const { return p_term_last; }
//     double getLastITerm() const { return i_term_last; }
//     double getLastDTerm() const { return d_term_last; }
// };

// class FlightControllerInterface {
// private:
//     ros::NodeHandle nh;
//     ros::Publisher rc_pub;
//     ros::ServiceClient arming_client;
//     ros::Subscriber imu_sub;
//     ros::Rate* rate;
    
//     // PID controller for pitch stabilization
//     PIDController* pitch_pid;
    
//     // Current pitch angle in radians
//     double current_pitch;
    
//     // Base throttle value
//     uint16_t base_throttle;
    
//     // 电机输出系数，用于调整电机响应差异
//     double motor3_factor;
//     double motor4_factor;
    
//     // 数据记录相关成员
//     std::ofstream data_file;
//     std::string csv_filename;
//     bool logging_enabled;
//     ros::Time start_time;
    
//     // PWM smoothing
//     uint16_t current_pwm_ch3;
//     uint16_t current_pwm_ch4;
//     uint16_t target_pwm_ch3;
//     uint16_t target_pwm_ch4;
//     double pwm_change_rate; // Maximum PWM change per iteration
    
//     // Deadzone range for pitch (in radians)
//     double pitch_deadzone_min; // -3 degrees in radians
//     double pitch_deadzone_max; // 3 degrees in radians
    
//     // PWM deadzone value
//     uint16_t pwm_deadzone; // 30 PWM units
    
//     // 生成带时间戳的文件名
//     std::string generateTimestampedFilename() {
//         auto now = std::chrono::system_clock::now();
//         auto in_time_t = std::chrono::system_clock::to_time_t(now);
        
//         std::stringstream ss;
//         ss << "pid_data_";
//         ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
//         ss << ".csv";
        
//         return ss.str();
//     }
    
//     // 初始化CSV文件
//     void initDataLogging() {
//         if (logging_enabled) {
//             csv_filename = generateTimestampedFilename();
//             data_file.open(csv_filename);
            
//             // 写入CSV头部
//             data_file << "timestamp,pitch_deg,error_deg,p_term,i_term,d_term,pid_output,motor3_pwm,motor4_pwm" << std::endl;
            
//             // 记录开始时间
//             start_time = ros::Time::now();
            
//             ROS_INFO("数据记录已开始，文件名: %s", csv_filename.c_str());
//         }
//     }
    
//     // 记录一行数据
//     void logData(double pitch_deg, double error_deg, double p_term, double i_term, double d_term, 
//                 double pid_output, uint16_t motor3_pwm, uint16_t motor4_pwm) {
//         if (logging_enabled && data_file.is_open()) {
//             // 计算相对时间戳（秒）
//             double elapsed = (ros::Time::now() - start_time).toSec();
            
//             // 写入数据行
//             data_file << std::fixed << std::setprecision(3) 
//                      << elapsed << ","
//                      << pitch_deg << ","
//                      << error_deg << ","
//                      << p_term << ","
//                      << i_term << ","
//                      << d_term << ","
//                      << pid_output << ","
//                      << motor3_pwm << ","
//                      << motor4_pwm << std::endl;
//         }
//     }

// public:
//     FlightControllerInterface() : current_pitch(0.0), base_throttle(1500),
//                                  motor3_factor(1.0), motor4_factor(1.0),
//                                  logging_enabled(true),
//                                  current_pwm_ch3(1500), current_pwm_ch4(1500),
//                                  target_pwm_ch3(1500), target_pwm_ch4(1500),
//                                  pwm_change_rate(5.0),
//                                  pitch_deadzone_min(-3.0 * M_PI / 180.0),
//                                  pitch_deadzone_max(3.0 * M_PI / 180.0),
//                                  pwm_deadzone(30) {
//         // 初始化RC控制发布器
//         rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 100);
        
//         // 等待解锁服务
//         ROS_INFO("等待解锁服务...");
//         ros::service::waitForService("/mavros/cmd/arming");
//         arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
//         ROS_INFO("解锁服务已准备就绪");
        
//         // 订阅IMU数据
//         imu_sub = nh.subscribe("/mavros/imu/data", 100, &FlightControllerInterface::imuCallback, this);
//         ROS_INFO("已订阅IMU数据");

//         // 设置发布频率为100Hz
//         rate = new ros::Rate(100);
        
//         // 初始化PID控制器 (kp, ki, kd, setpoint, min_output, max_output)
//         pitch_pid = new PIDController(350.0, 30, 40.0, 0.0, -400.0, 400.0);
        
//         // 设置积分限制
//         pitch_pid->setIntegralLimit(200.0);
        
//         // 初始化数据记录
//         initDataLogging();
//     }

//     ~FlightControllerInterface() {
//         // 关闭数据文件
//         if (data_file.is_open()) {
//             data_file.close();
//             ROS_INFO("数据记录已结束，文件已保存: %s", csv_filename.c_str());
//         }
        
//         delete rate;
//         delete pitch_pid;
//     }
    
//     void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
//         // 从四元数中提取欧拉角
//         tf2::Quaternion q(
//             msg->orientation.x,
//             msg->orientation.y,
//             msg->orientation.z,
//             msg->orientation.w
//         );
        
//         double roll, pitch, yaw;
//         tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
//         // 保存当前pitch角度
//         current_pitch = pitch;
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
    
//     // 使用PID控制器稳定pitch
//     void stabilizePitch() {
//         double pid_output = 0.0;
//         double p_term = 0.0, i_term = 0.0, d_term = 0.0;
        
//         // 计算角度（度）
//         double pitch_deg = current_pitch * 180.0 / M_PI;
//         double error_deg = (0.0 - current_pitch) * 180.0 / M_PI;
        
//         // 只有当pitch超出死区范围时才应用PID控制
//         if (current_pitch < pitch_deadzone_min || current_pitch > pitch_deadzone_max) {
//             // 计算PID输出
//             pid_output = pitch_pid->compute(current_pitch);
            
//             // 获取PID各项值
//             p_term = pitch_pid->getLastPTerm();
//             i_term = pitch_pid->getLastITerm();
//             d_term = pitch_pid->getLastDTerm();
            
//             // 应用电机系数调整
//             double adjusted_output3 = pid_output * motor3_factor;
//             double adjusted_output4 = pid_output * motor4_factor;
            
//             // 设置目标PWM值
//             target_pwm_ch3 = base_throttle - adjusted_output3;
//             target_pwm_ch4 = base_throttle - adjusted_output4;
//         } else {
//             // 在死区内，保持当前状态
//             pitch_pid->reset(); // 重置PID控制器以避免积分累积
//             // 目标值保持不变
//         }
        
//         // 确保目标油门值在安全范围内
//         target_pwm_ch3 = std::max((uint16_t)1000, std::min((uint16_t)2000, target_pwm_ch3));
//         target_pwm_ch4 = std::max((uint16_t)1000, std::min((uint16_t)2000, target_pwm_ch4));
        
//         // 平滑过渡到目标PWM值
//         smoothPWMTransition();
        
//         // 记录数据
//         logData(pitch_deg, error_deg, p_term, i_term, d_term, pid_output, current_pwm_ch3, current_pwm_ch4);
        
//         // 控制电机
//         controlMotors(current_pwm_ch3, current_pwm_ch4);
        
//         // 打印调试信息
//         printf("\033[1;32m[PID] Pitch: %.2f°, Error: %.2f°, PID: %.2f, Motors: %d, %d\033[0m\n", 
//                pitch_deg, error_deg, pid_output, current_pwm_ch3, current_pwm_ch4);
//         fflush(stdout);
//     }
    
//     // 平滑过渡PWM值
//     void smoothPWMTransition() {
//         // 计算当前值与目标值的差异
//         int diff_ch3 = std::abs(static_cast<int>(target_pwm_ch3) - static_cast<int>(current_pwm_ch3));
//         int diff_ch4 = std::abs(static_cast<int>(target_pwm_ch4) - static_cast<int>(current_pwm_ch4));
        
//         // 如果差异小于PWM死区，则不调整
//         if (diff_ch3 <= pwm_deadzone) {
//             // 在PWM死区内，不调整
//         } else {
//             // 平滑过渡通道3
//             if (current_pwm_ch3 < target_pwm_ch3) {
//                 current_pwm_ch3 = std::min(target_pwm_ch3, static_cast<uint16_t>(current_pwm_ch3 + pwm_change_rate));
//             } else if (current_pwm_ch3 > target_pwm_ch3) {
//                 current_pwm_ch3 = std::max(target_pwm_ch3, static_cast<uint16_t>(current_pwm_ch3 - pwm_change_rate));
//             }
//         }
        
//         // 如果差异小于PWM死区，则不调整
//         if (diff_ch4 <= pwm_deadzone) {
//             // 在PWM死区内，不调整
//         } else {
//             // 平滑过渡通道4
//             if (current_pwm_ch4 < target_pwm_ch4) {
//                 current_pwm_ch4 = std::min(target_pwm_ch4, static_cast<uint16_t>(current_pwm_ch4 + pwm_change_rate));
//             } else if (current_pwm_ch4 > target_pwm_ch4) {
//                 current_pwm_ch4 = std::max(target_pwm_ch4, static_cast<uint16_t>(current_pwm_ch4 - pwm_change_rate));
//             }
//         }
//     }

//     // 设置电机响应系数
//     void setMotorFactors(double factor3, double factor4) {
//         motor3_factor = factor3;
//         motor4_factor = factor4;
//         ROS_INFO("设置电机系数 - 电机3: %.2f, 电机4: %.2f", motor3_factor, motor4_factor);
//     }
    
//     // 设置PWM变化率
//     void setPWMChangeRate(double rate) {
//         pwm_change_rate = rate;
//         ROS_INFO("PWM变化率已设置为: %.2f", pwm_change_rate);
//     }
    
//     // 设置PWM死区
//     void setPWMDeadzone(uint16_t deadzone) {
//         pwm_deadzone = deadzone;
//         ROS_INFO("PWM死区已设置为: %d", pwm_deadzone);
//     }
    
//     // 启用/禁用数据记录
//     void enableDataLogging(bool enable) {
//         if (enable && !logging_enabled) {
//             // 如果之前禁用，现在启用，创建新文件
//             logging_enabled = true;
//             initDataLogging();
//         } else if (!enable && logging_enabled) {
//             // 如果之前启用，现在禁用，关闭文件
//             logging_enabled = false;
//             if (data_file.is_open()) {
//                 data_file.close();
//                 ROS_INFO("数据记录已停止，文件已保存: %s", csv_filename.c_str());
//             }
//         }
//     }

//     // 运行演示
//     void runDemo() {
//         if (!armVehicle()) {
//             ROS_ERROR("飞控解锁失败，终止演示");
//             return;
//         }
        
//         ROS_INFO("开始PID稳定pitch控制，按Ctrl+C终止");
//         ROS_INFO("Pitch死区范围: -3° 到 3°");
//         ROS_INFO("PWM死区值: %d", pwm_deadzone);
        
//         try {
//             // 等待一段时间以确保收到IMU数据
//             ros::Duration(1.0).sleep();
            
//             // 如果观察到电机4反应更大，可以调整电机系数
//             // 例如: setMotorFactors(1.0, 0.8);
            
//             // 持续运行，直到收到终止信号
//             while (ros::ok()) {
//                 stabilizePitch();
//                 ros::spinOnce();
//                 rate->sleep();
//             }
            
//             // 停止电机 (使用1500是停止值)
//             controlMotors(1500, 1500);
//             ROS_INFO("控制终止");
            
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

/////////////////////////////////////////////////////加入平滑处理和调整区间//////////////////////////////////////////////////////
// #include <ros/ros.h>
// #include <mavros_msgs/OverrideRCIn.h>
// #include <mavros_msgs/CommandBool.h>
// #include <sensor_msgs/Imu.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <ctime>
// #include <fstream>
// #include <iomanip>
// #include <chrono>
// #include <sstream>

// class PIDController {
// private:
//     double kp, ki, kd;
//     double setpoint;
//     double integral, prev_error;
//     double output_min, output_max;
//     double integral_limit; // 添加积分限制
//     ros::Time last_time;
//     bool first_call;
    
//     // 添加存储各项值的变量
//     double p_term_last, i_term_last, d_term_last;

// public:
//     PIDController(double _kp, double _ki, double _kd, double _setpoint, 
//                  double _output_min, double _output_max) 
//         : kp(_kp), ki(_ki), kd(_kd), setpoint(_setpoint),
//           integral(0.0), prev_error(0.0), 
//           output_min(_output_min), output_max(_output_max),
//           integral_limit(100.0), // 默认积分限制
//           first_call(true),
//           p_term_last(0.0), i_term_last(0.0), d_term_last(0.0) {}

//     double compute(double measurement) {
//         ros::Time current_time = ros::Time::now();
//         double dt;
        
//         if (first_call) {
//             dt = 0.0;
//             first_call = false;
//         } else {
//             dt = (current_time - last_time).toSec();
//         }
//         last_time = current_time;
        
//         // Avoid division by zero
//         if (dt <= 0.0) return 0.0;
        
//         // Calculate error
//         double error = setpoint - measurement;
        
//         // Proportional term
//         double p_term = kp * error;
//         p_term_last = p_term;
        
//         // Integral term with anti-windup
//         integral += error * dt;
        
//         // Apply integral limits to prevent windup
//         if (integral > integral_limit) {
//             integral = integral_limit;
//         } else if (integral < -integral_limit) {
//             integral = -integral_limit;
//         }
        
//         double i_term = ki * integral;
//         i_term_last = i_term;
        
//         // Derivative term
//         double derivative = 0.0;
//         if (dt > 0) {
//             derivative = (error - prev_error) / dt;
//         }
//         double d_term = kd * derivative;
//         d_term_last = d_term;
        
//         // Calculate total output
//         double output = p_term + i_term + d_term;
        
//         // Apply output limits
//         if (output > output_max) {
//             output = output_max;
//         } else if (output < output_min) {
//             output = output_min;
//         }
        
//         // Save error for next iteration
//         prev_error = error;
        
//         // 输出各项贡献，便于调试
//         ROS_DEBUG("PID components: P=%.2f, I=%.2f, D=%.2f", p_term, i_term, d_term);
        
//         return output;
//     }
    
//     void reset() {
//         integral = 0.0;
//         prev_error = 0.0;
//         first_call = true;
//         p_term_last = i_term_last = d_term_last = 0.0;
//     }
    
//     // 设置积分限制
//     void setIntegralLimit(double limit) {
//         integral_limit = limit;
//     }
    
//     // 获取各项值的方法
//     double getLastPTerm() const { return p_term_last; }
//     double getLastITerm() const { return i_term_last; }
//     double getLastDTerm() const { return d_term_last; }
// };

// class FlightControllerInterface {
// private:
//     ros::NodeHandle nh;
//     ros::Publisher rc_pub;
//     ros::ServiceClient arming_client;
//     ros::Subscriber imu_sub;
//     ros::Rate* rate;
    
//     // PID controller for pitch stabilization
//     PIDController* pitch_pid;
    
//     // Current pitch angle in radians
//     double current_pitch;
    
//     // Base throttle value
//     uint16_t base_throttle;
    
//     // 电机输出系数，用于调整电机响应差异
//     double motor3_factor;
//     double motor4_factor;
    
//     // 数据记录相关成员
//     std::ofstream data_file;
//     std::string csv_filename;
//     bool logging_enabled;
//     ros::Time start_time;
    
//     // Add new members for PWM smoothing
//     uint16_t current_pwm_ch3;
//     uint16_t current_pwm_ch4;
//     uint16_t target_pwm_ch3;
//     uint16_t target_pwm_ch4;
//     double pwm_change_rate; // Maximum PWM change per iteration
    
//     // Add deadzone range
//     double pitch_deadzone_min; // -5 degrees in radians
//     double pitch_deadzone_max; // 5 degrees in radians
    
//     // 生成带时间戳的文件名
//     std::string generateTimestampedFilename() {
//         auto now = std::chrono::system_clock::now();
//         auto in_time_t = std::chrono::system_clock::to_time_t(now);
        
//         std::stringstream ss;
//         ss << "pid_data_";
//         ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
//         ss << ".csv";
        
//         return ss.str();
//     }
    
//     // 初始化CSV文件
//     void initDataLogging() {
//         if (logging_enabled) {
//             csv_filename = generateTimestampedFilename();
//             data_file.open(csv_filename);
            
//             // 写入CSV头部
//             data_file << "timestamp,pitch_deg,error_deg,p_term,i_term,d_term,pid_output,motor3_pwm,motor4_pwm" << std::endl;
            
//             // 记录开始时间
//             start_time = ros::Time::now();
            
//             ROS_INFO("数据记录已开始，文件名: %s", csv_filename.c_str());
//         }
//     }
    
//     // 记录一行数据
//     void logData(double pitch_deg, double error_deg, double p_term, double i_term, double d_term, 
//                 double pid_output, uint16_t motor3_pwm, uint16_t motor4_pwm) {
//         if (logging_enabled && data_file.is_open()) {
//             // 计算相对时间戳（秒）
//             double elapsed = (ros::Time::now() - start_time).toSec();
            
//             // 写入数据行
//             data_file << std::fixed << std::setprecision(3) 
//                      << elapsed << ","
//                      << pitch_deg << ","
//                      << error_deg << ","
//                      << p_term << ","
//                      << i_term << ","
//                      << d_term << ","
//                      << pid_output << ","
//                      << motor3_pwm << ","
//                      << motor4_pwm << std::endl;
//         }
//     }

// public:
//     FlightControllerInterface() : current_pitch(0.0), base_throttle(1500),
//                                  motor3_factor(1.0), motor4_factor(1.0),
//                                  logging_enabled(true),
//                                  current_pwm_ch3(1500), current_pwm_ch4(1500),
//                                  target_pwm_ch3(1500), target_pwm_ch4(1500),
//                                  pwm_change_rate(5.0),
//                                  pitch_deadzone_min(-5.0 * M_PI / 180.0),
//                                  pitch_deadzone_max(5.0 * M_PI / 180.0) {
//         // 初始化RC控制发布器
//         rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 100);
        
//         // 等待解锁服务
//         ROS_INFO("等待解锁服务...");
//         ros::service::waitForService("/mavros/cmd/arming");
//         arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
//         ROS_INFO("解锁服务已准备就绪");
        
//         // 订阅IMU数据
//         imu_sub = nh.subscribe("/mavros/imu/data", 100, &FlightControllerInterface::imuCallback, this);
//         ROS_INFO("已订阅IMU数据");

//         // 设置发布频率为100Hz
//         rate = new ros::Rate(100);
        
//         // 初始化PID控制器 (kp, ki, kd, setpoint, min_output, max_output)
//         pitch_pid = new PIDController(350.0, 2, 100.0, 0.0, -400.0, 400.0);
        
//         // 设置积分限制
//         pitch_pid->setIntegralLimit(200.0);
        
//         // 初始化数据记录
//         initDataLogging();
//     }

//     ~FlightControllerInterface() {
//         // 关闭数据文件
//         if (data_file.is_open()) {
//             data_file.close();
//             ROS_INFO("数据记录已结束，文件已保存: %s", csv_filename.c_str());
//         }
        
//         delete rate;
//         delete pitch_pid;
//     }
    
//     void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
//         // 从四元数中提取欧拉角
//         tf2::Quaternion q(
//             msg->orientation.x,
//             msg->orientation.y,
//             msg->orientation.z,
//             msg->orientation.w
//         );
        
//         double roll, pitch, yaw;
//         tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
//         // 保存当前pitch角度
//         current_pitch = pitch;
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
    
//     // 使用PID控制器稳定pitch
//     void stabilizePitch() {
//         double pid_output = 0.0;
//         double p_term = 0.0, i_term = 0.0, d_term = 0.0;
        
//         // 计算角度（度）
//         double pitch_deg = current_pitch * 180.0 / M_PI;
//         double error_deg = (0.0 - current_pitch) * 180.0 / M_PI;
        
//         // 只有当pitch超出死区范围时才应用PID控制
//         if (current_pitch < pitch_deadzone_min || current_pitch > pitch_deadzone_max) {
//             // 计算PID输出
//             pid_output = pitch_pid->compute(current_pitch);
            
//             // 获取PID各项值
//             p_term = pitch_pid->getLastPTerm();
//             i_term = pitch_pid->getLastITerm();
//             d_term = pitch_pid->getLastDTerm();
            
//             // 应用电机系数调整
//             double adjusted_output3 = pid_output * motor3_factor;
//             double adjusted_output4 = pid_output * motor4_factor;
            
//             // 设置目标PWM值
//             target_pwm_ch3 = base_throttle - adjusted_output3;
//             target_pwm_ch4 = base_throttle - adjusted_output4;
//         } else {
//             // 在死区内，保持当前状态
//             pitch_pid->reset(); // 重置PID控制器以避免积分累积
//             // 目标值保持不变
//         }
        
//         // 确保目标油门值在安全范围内
//         target_pwm_ch3 = std::max((uint16_t)1000, std::min((uint16_t)2000, target_pwm_ch3));
//         target_pwm_ch4 = std::max((uint16_t)1000, std::min((uint16_t)2000, target_pwm_ch4));
        
//         // 平滑过渡到目标PWM值
//         smoothPWMTransition();
        
//         // 记录数据
//         logData(pitch_deg, error_deg, p_term, i_term, d_term, pid_output, current_pwm_ch3, current_pwm_ch4);
        
//         // 控制电机
//         controlMotors(current_pwm_ch3, current_pwm_ch4);
        
//         // 打印调试信息
//         printf("\033[1;32m[PID] Pitch: %.2f°, Error: %.2f°, PID: %.2f, Motors: %d, %d\033[0m\n", 
//                pitch_deg, error_deg, pid_output, current_pwm_ch3, current_pwm_ch4);
//         fflush(stdout);
//     }
    
//     // 平滑过渡PWM值
//     void smoothPWMTransition() {
//         // 平滑过渡通道3
//         if (current_pwm_ch3 < target_pwm_ch3) {
//             current_pwm_ch3 = std::min(target_pwm_ch3, static_cast<uint16_t>(current_pwm_ch3 + pwm_change_rate));
//         } else if (current_pwm_ch3 > target_pwm_ch3) {
//             current_pwm_ch3 = std::max(target_pwm_ch3, static_cast<uint16_t>(current_pwm_ch3 - pwm_change_rate));
//         }
        
//         // 平滑过渡通道4
//         if (current_pwm_ch4 < target_pwm_ch4) {
//             current_pwm_ch4 = std::min(target_pwm_ch4, static_cast<uint16_t>(current_pwm_ch4 + pwm_change_rate));
//         } else if (current_pwm_ch4 > target_pwm_ch4) {
//             current_pwm_ch4 = std::max(target_pwm_ch4, static_cast<uint16_t>(current_pwm_ch4 - pwm_change_rate));
//         }
//     }

//     // 设置电机响应系数
//     void setMotorFactors(double factor3, double factor4) {
//         motor3_factor = factor3;
//         motor4_factor = factor4;
//         ROS_INFO("设置电机系数 - 电机3: %.2f, 电机4: %.2f", motor3_factor, motor4_factor);
//     }
    
//     // 设置PWM变化率
//     void setPWMChangeRate(double rate) {
//         pwm_change_rate = rate;
//         ROS_INFO("PWM变化率已设置为: %.2f", pwm_change_rate);
//     }
    
//     // 启用/禁用数据记录
//     void enableDataLogging(bool enable) {
//         if (enable && !logging_enabled) {
//             // 如果之前禁用，现在启用，创建新文件
//             logging_enabled = true;
//             initDataLogging();
//         } else if (!enable && logging_enabled) {
//             // 如果之前启用，现在禁用，关闭文件
//             logging_enabled = false;
//             if (data_file.is_open()) {
//                 data_file.close();
//                 ROS_INFO("数据记录已停止，文件已保存: %s", csv_filename.c_str());
//             }
//         }
//     }

//     // 运行演示
//     void runDemo() {
//         if (!armVehicle()) {
//             ROS_ERROR("飞控解锁失败，终止演示");
//             return;
//         }
        
//         ROS_INFO("开始PID稳定pitch控制，按Ctrl+C终止");
//         try {
//             // 等待一段时间以确保收到IMU数据
//             ros::Duration(1.0).sleep();
            
//             // 如果观察到电机4反应更大，可以调整电机系数
//             // 例如: setMotorFactors(1.0, 0.8);
            
//             // 持续运行，直到收到终止信号
//             while (ros::ok()) {
//                 stabilizePitch();
//                 ros::spinOnce();
//                 rate->sleep();
//             }
            
//             // 停止电机 (使用1500是停止值)
//             controlMotors(1500, 1500);
//             ROS_INFO("控制终止");
            
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

/////////////////////////////////////////////////////////////////////////调整Pitch第一版///////////////////////////////////////////////////////////////////////////////////
// #include <ros/ros.h>
// #include <mavros_msgs/OverrideRCIn.h>
// #include <mavros_msgs/CommandBool.h>
// #include <sensor_msgs/Imu.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <ctime>
// #include <fstream>
// #include <iomanip>
// #include <chrono>
// #include <sstream>

// class PIDController {
// private:
//     double kp, ki, kd;
//     double setpoint;
//     double integral, prev_error;
//     double output_min, output_max;
//     double integral_limit; // 添加积分限制
//     ros::Time last_time;
//     bool first_call;
    
//     // 添加存储各项值的变量
//     double p_term_last, i_term_last, d_term_last;

// public:
//     PIDController(double _kp, double _ki, double _kd, double _setpoint, 
//                  double _output_min, double _output_max) 
//         : kp(_kp), ki(_ki), kd(_kd), setpoint(_setpoint),
//           integral(0.0), prev_error(0.0), 
//           output_min(_output_min), output_max(_output_max),
//           integral_limit(100.0), // 默认积分限制
//           first_call(true),
//           p_term_last(0.0), i_term_last(0.0), d_term_last(0.0) {}

//     double compute(double measurement) {
//         ros::Time current_time = ros::Time::now();
//         double dt;
        
//         if (first_call) {
//             dt = 0.0;
//             first_call = false;
//         } else {
//             dt = (current_time - last_time).toSec();
//         }
//         last_time = current_time;
        
//         // Avoid division by zero
//         if (dt <= 0.0) return 0.0;
        
//         // Calculate error
//         double error = setpoint - measurement;
        
//         // Proportional term
//         double p_term = kp * error;
//         p_term_last = p_term;
        
//         // Integral term with anti-windup
//         integral += error * dt;
        
//         // Apply integral limits to prevent windup
//         if (integral > integral_limit) {
//             integral = integral_limit;
//         } else if (integral < -integral_limit) {
//             integral = -integral_limit;
//         }
        
//         double i_term = ki * integral;
//         i_term_last = i_term;
        
//         // Derivative term
//         double derivative = 0.0;
//         if (dt > 0) {
//             derivative = (error - prev_error) / dt;
//         }
//         double d_term = kd * derivative;
//         d_term_last = d_term;
        
//         // Calculate total output
//         double output = p_term + i_term + d_term;
        
//         // Apply output limits
//         if (output > output_max) {
//             output = output_max;
//         } else if (output < output_min) {
//             output = output_min;
//         }
        
//         // Save error for next iteration
//         prev_error = error;
        
//         // 输出各项贡献，便于调试
//         ROS_DEBUG("PID components: P=%.2f, I=%.2f, D=%.2f", p_term, i_term, d_term);
        
//         return output;
//     }
    
//     void reset() {
//         integral = 0.0;
//         prev_error = 0.0;
//         first_call = true;
//         p_term_last = i_term_last = d_term_last = 0.0;
//     }
    
//     // 设置积分限制
//     void setIntegralLimit(double limit) {
//         integral_limit = limit;
//     }
    
//     // 获取各项值的方法
//     double getLastPTerm() const { return p_term_last; }
//     double getLastITerm() const { return i_term_last; }
//     double getLastDTerm() const { return d_term_last; }
// };

// class FlightControllerInterface {
// private:
//     ros::NodeHandle nh;
//     ros::Publisher rc_pub;
//     ros::ServiceClient arming_client;
//     ros::Subscriber imu_sub;
//     ros::Rate* rate;
    
//     // PID controller for pitch stabilization
//     PIDController* pitch_pid;
    
//     // Current pitch angle in radians
//     double current_pitch;
    
//     // Base throttle value
//     uint16_t base_throttle;
    
//     // 电机输出系数，用于调整电机响应差异
//     double motor3_factor;
//     double motor4_factor;
    
//     // 数据记录相关成员
//     std::ofstream data_file;
//     std::string csv_filename;
//     bool logging_enabled;
//     ros::Time start_time;
    
//     // 生成带时间戳的文件名
//     std::string generateTimestampedFilename() {
//         auto now = std::chrono::system_clock::now();
//         auto in_time_t = std::chrono::system_clock::to_time_t(now);
        
//         std::stringstream ss;
//         ss << "pid_data_";
//         ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
//         ss << ".csv";
        
//         return ss.str();
//     }
    
//     // 初始化CSV文件
//     void initDataLogging() {
//         if (logging_enabled) {
//             csv_filename = generateTimestampedFilename();
//             data_file.open(csv_filename);
            
//             // 写入CSV头部
//             data_file << "timestamp,pitch_deg,error_deg,p_term,i_term,d_term,pid_output,motor3_pwm,motor4_pwm" << std::endl;
            
//             // 记录开始时间
//             start_time = ros::Time::now();
            
//             ROS_INFO("数据记录已开始，文件名: %s", csv_filename.c_str());
//         }
//     }
    
//     // 记录一行数据
//     void logData(double pitch_deg, double error_deg, double p_term, double i_term, double d_term, 
//                 double pid_output, uint16_t motor3_pwm, uint16_t motor4_pwm) {
//         if (logging_enabled && data_file.is_open()) {
//             // 计算相对时间戳（秒）
//             double elapsed = (ros::Time::now() - start_time).toSec();
            
//             // 写入数据行
//             data_file << std::fixed << std::setprecision(3) 
//                      << elapsed << ","
//                      << pitch_deg << ","
//                      << error_deg << ","
//                      << p_term << ","
//                      << i_term << ","
//                      << d_term << ","
//                      << pid_output << ","
//                      << motor3_pwm << ","
//                      << motor4_pwm << std::endl;
//         }
//     }

// public:
//     FlightControllerInterface() : current_pitch(0.0), base_throttle(1500),
//                                  motor3_factor(1.0), motor4_factor(1.0),
//                                  logging_enabled(true) {
//         // 初始化RC控制发布器
//         rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 100);
        
//         // 等待解锁服务
//         ROS_INFO("等待解锁服务...");
//         ros::service::waitForService("/mavros/cmd/arming");
//         arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
//         ROS_INFO("解锁服务已准备就绪");
        
//         // 订阅IMU数据
//         imu_sub = nh.subscribe("/mavros/imu/data", 100, &FlightControllerInterface::imuCallback, this);
//         ROS_INFO("已订阅IMU数据");

//         // 设置发布频率为100Hz
//         rate = new ros::Rate(100);
        
//         // 初始化PID控制器 (kp, ki, kd, setpoint, min_output, max_output)
//         pitch_pid = new PIDController(350.0, 2, 100.0, 0.0, -400.0, 400.0);
        
//         // 设置积分限制
//         pitch_pid->setIntegralLimit(200.0);
        
//         // 初始化数据记录
//         initDataLogging();
//     }

//     ~FlightControllerInterface() {
//         // 关闭数据文件
//         if (data_file.is_open()) {
//             data_file.close();
//             ROS_INFO("数据记录已结束，文件已保存: %s", csv_filename.c_str());
//         }
        
//         delete rate;
//         delete pitch_pid;
//     }
    
//     void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
//         // 从四元数中提取欧拉角
//         tf2::Quaternion q(
//             msg->orientation.x,
//             msg->orientation.y,
//             msg->orientation.z,
//             msg->orientation.w
//         );
        
//         double roll, pitch, yaw;
//         tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
//         // 保存当前pitch角度
//         current_pitch = pitch;
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
    
//     // 使用PID控制器稳定pitch
//     void stabilizePitch() {
//         // 计算PID输出
//         double pid_output = pitch_pid->compute(current_pitch);
        
//         // 获取PID各项值
//         double p_term = pitch_pid->getLastPTerm();
//         double i_term = pitch_pid->getLastITerm();
//         double d_term = pitch_pid->getLastDTerm();
        
//         // 应用电机系数调整
//         double adjusted_output3 = pid_output * motor3_factor;
//         double adjusted_output4 = pid_output * motor4_factor;
        
//         // 根据PID输出调整两个电机的差值
//         uint16_t throttle_ch3 = base_throttle - adjusted_output3;
//         uint16_t throttle_ch4 = base_throttle - adjusted_output4;
        
//         // 确保油门值在安全范围内
//         throttle_ch3 = std::max((uint16_t)1000, std::min((uint16_t)2000, throttle_ch3));
//         throttle_ch4 = std::max((uint16_t)1000, std::min((uint16_t)2000, throttle_ch4));
        
//         // 计算角度（度）
//         double pitch_deg = current_pitch * 180.0 / M_PI;
//         double error_deg = (0.0 - current_pitch) * 180.0 / M_PI;
        
//         // 记录数据
//         logData(pitch_deg, error_deg, p_term, i_term, d_term, pid_output, throttle_ch3, throttle_ch4);
        
//         // 控制电机
//         controlMotors(throttle_ch3, throttle_ch4);
        
//         // 打印调试信息
//         printf("\033[1;32m[PID] Pitch: %.2f°, Error: %.2f°, PID: %.2f, Motors: %d, %d\033[0m\n", 
//                pitch_deg, error_deg, pid_output, throttle_ch3, throttle_ch4);
//         fflush(stdout);
//     }

//     // 设置电机响应系数
//     void setMotorFactors(double factor3, double factor4) {
//         motor3_factor = factor3;
//         motor4_factor = factor4;
//         ROS_INFO("设置电机系数 - 电机3: %.2f, 电机4: %.2f", motor3_factor, motor4_factor);
//     }
    
//     // 启用/禁用数据记录
//     void enableDataLogging(bool enable) {
//         if (enable && !logging_enabled) {
//             // 如果之前禁用，现在启用，创建新文件
//             logging_enabled = true;
//             initDataLogging();
//         } else if (!enable && logging_enabled) {
//             // 如果之前启用，现在禁用，关闭文件
//             logging_enabled = false;
//             if (data_file.is_open()) {
//                 data_file.close();
//                 ROS_INFO("数据记录已停止，文件已保存: %s", csv_filename.c_str());
//             }
//         }
//     }

//     // 运行演示
//     void runDemo() {
//         if (!armVehicle()) {
//             ROS_ERROR("飞控解锁失败，终止演示");
//             return;
//         }
        
//         ROS_INFO("开始PID稳定pitch控制，按Ctrl+C终止");
//         try {
//             // 等待一段时间以确保收到IMU数据
//             ros::Duration(1.0).sleep();
            
//             // 如果观察到电机4反应更大，可以调整电机系数
//             // 例如: setMotorFactors(1.0, 0.8);
            
//             // 持续运行，直到收到终止信号
//             while (ros::ok()) {
//                 stabilizePitch();
//                 ros::spinOnce();
//                 rate->sleep();
//             }
            
//             // 停止电机 (使用1500是停止值)
//             controlMotors(1500, 1500);
//             ROS_INFO("控制终止");
            
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
