#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <sstream>
#include <deque>

// PID调试模式枚举
enum class PIDDebugMode {
    ACCELERATION_ONLY = 1,    // 只调试加速度环（内环）
    VELOCITY_ACCEL = 2,       // 调试速度环+加速度环（内环+中环）
    FULL_CASCADE = 3          // 全三环调试
};

class PIDController {
private:
    double kp, ki, kd;
    double setpoint;
    double integral, prev_error;
    double output_min, output_max;
    double integral_limit;
    ros::Time last_time;
    bool first_call;
    
    double p_term_last, i_term_last, d_term_last;

public:
    PIDController(double _kp, double _ki, double _kd, double _setpoint, 
                 double _output_min, double _output_max) 
        : kp(_kp), ki(_ki), kd(_kd), setpoint(_setpoint),
          integral(0.0), prev_error(0.0), 
          output_min(_output_min), output_max(_output_max),
          integral_limit(100.0),
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
        
        if (dt <= 0.0) return 0.0;
        
        double error = setpoint - measurement;
        
        double p_term = kp * error;
        p_term_last = p_term;
        
        integral += error * dt;
        if (integral > integral_limit) {
            integral = integral_limit;
        } else if (integral < -integral_limit) {
            integral = -integral_limit;
        }
        
        double i_term = ki * integral;
        i_term_last = i_term;
        
        double derivative = 0.0;
        if (dt > 0) {
            derivative = (error - prev_error) / dt;
        }
        double d_term = kd * derivative;
        d_term_last = d_term;
        
        double output = p_term + i_term + d_term;
        
        if (output > output_max) {
            output = output_max;
        } else if (output < output_min) {
            output = output_min;
        }
        
        prev_error = error;
        
        return output;
    }
    
    void reset() {
        integral = 0.0;
        prev_error = 0.0;
        first_call = true;
        p_term_last = i_term_last = d_term_last = 0.0;
    }
    
    void resetIntegralOnly() {
        integral = 0.0;
    }
    
    void setIntegralLimit(double limit) {
        integral_limit = limit;
    }
    
    void setSetpoint(double _setpoint) {
        setpoint = _setpoint;
    }
    
    // 动态更新PID参数
    void updateParams(double _kp, double _ki, double _kd) {
        kp = _kp;
        ki = _ki;
        kd = _kd;
    }
    
    double getLastPTerm() const { return p_term_last; }
    double getLastITerm() const { return i_term_last; }
    double getLastDTerm() const { return d_term_last; }
    double getSetpoint() const { return setpoint; }
};

// 可调试的三环PID控制器
class DebuggableCascadedPIDController {
private:
    PIDController* position_pid;
    PIDController* velocity_pid;
    PIDController* accel_pid;
    
    double position_setpoint;
    double current_position;
    double current_velocity;
    double current_acceleration;
    
    double velocity_filter_alpha;
    double accel_filter_alpha;
    double output_min, output_max;
    
    double position_output_last;
    double velocity_output_last;
    double accel_output_last;
    
    // 调试模式
    PIDDebugMode debug_mode;
    
    // 手动设定点（用于调试）
    double manual_velocity_setpoint;
    double manual_accel_setpoint;
    bool use_manual_setpoints;

public:
    DebuggableCascadedPIDController(double pos_kp, double pos_ki, double pos_kd,
                                   double vel_kp, double vel_ki, double vel_kd,
                                   double acc_kp, double acc_ki, double acc_kd,
                                   double _position_setpoint,
                                   double _output_min, double _output_max)
        : position_setpoint(_position_setpoint),
          current_position(0.0), current_velocity(0.0), current_acceleration(0.0),
          output_min(_output_min), output_max(_output_max),
          position_output_last(0.0), velocity_output_last(0.0), accel_output_last(0.0),
          velocity_filter_alpha(0.3), accel_filter_alpha(0.2),
          debug_mode(PIDDebugMode::FULL_CASCADE),
          manual_velocity_setpoint(0.0), manual_accel_setpoint(0.0),
          use_manual_setpoints(false) {
        
        position_pid = new PIDController(pos_kp, pos_ki, pos_kd, position_setpoint, -10.0, 10.0);
        velocity_pid = new PIDController(vel_kp, vel_ki, vel_kd, 0.0, -200.0, 200.0);
        accel_pid = new PIDController(acc_kp, acc_ki, acc_kd, 0.0, output_min, output_max);
        
        position_pid->setIntegralLimit(5.0);
        velocity_pid->setIntegralLimit(10.0);
        accel_pid->setIntegralLimit(50.0);
    }
    
    ~DebuggableCascadedPIDController() {
        delete position_pid;
        delete velocity_pid;
        delete accel_pid;
    }
    
    // 设置调试模式
    void setDebugMode(PIDDebugMode mode) {
        debug_mode = mode;
        ROS_INFO("PID调试模式设置为: %d (1=只加速度环, 2=速度+加速度环, 3=全三环)", (int)mode);
    }
    
    // 手动设置各环的目标值（用于调试）
    void setManualSetpoints(double vel_setpoint, double accel_setpoint, bool enable = true) {
        manual_velocity_setpoint = vel_setpoint;
        manual_accel_setpoint = accel_setpoint;
        use_manual_setpoints = enable;
        
        if (enable) {
            ROS_INFO("启用手动设定点 - 速度目标: %.3f, 加速度目标: %.3f", vel_setpoint, accel_setpoint);
        } else {
            ROS_INFO("禁用手动设定点，使用级联PID");
        }
    }
    
    // 动态更新PID参数
    void updatePositionPID(double kp, double ki, double kd) {
        position_pid->updateParams(kp, ki, kd);
        ROS_INFO("位置环PID更新为: P=%.3f, I=%.3f, D=%.3f", kp, ki, kd);
    }
    
    void updateVelocityPID(double kp, double ki, double kd) {
        velocity_pid->updateParams(kp, ki, kd);
        ROS_INFO("速度环PID更新为: P=%.3f, I=%.3f, D=%.3f", kp, ki, kd);
    }
    
    void updateAccelPID(double kp, double ki, double kd) {
        accel_pid->updateParams(kp, ki, kd);
        ROS_INFO("加速度环PID更新为: P=%.3f, I=%.3f, D=%.3f", kp, ki, kd);
    }
    
    void setPositionSetpoint(double setpoint) {
        position_setpoint = setpoint;
        position_pid->setSetpoint(setpoint);
    }
    
    void setFilterCoefficients(double vel_alpha, double acc_alpha) {
        velocity_filter_alpha = vel_alpha;
        accel_filter_alpha = acc_alpha;
    }
    
    void setMeasurements(double position, double velocity, double acceleration) {
        current_position = position;
        current_velocity = velocity_filter_alpha * velocity + (1 - velocity_filter_alpha) * current_velocity;
        current_acceleration = accel_filter_alpha * acceleration + (1 - accel_filter_alpha) * current_acceleration;
    }
    
    // 根据调试模式计算输出
    double compute() {
        double velocity_setpoint = 0.0;
        double accel_setpoint = 0.0;
        double final_output = 0.0;
        
        switch (debug_mode) {
            case PIDDebugMode::ACCELERATION_ONLY:
                // 只调试加速度环，使用手动设定的加速度目标值
                if (use_manual_setpoints) {
                    accel_setpoint = manual_accel_setpoint;
                } else {
                    accel_setpoint = 0.0; // 默认目标加速度为0
                }
                
                accel_pid->setSetpoint(accel_setpoint);
                final_output = accel_pid->compute(current_acceleration);
                accel_output_last = final_output;
                
                // 其他环的输出设为0
                position_output_last = 0.0;
                velocity_output_last = 0.0;
                break;
                
            case PIDDebugMode::VELOCITY_ACCEL:
                // 调试速度环和加速度环
                if (use_manual_setpoints) {
                    velocity_setpoint = manual_velocity_setpoint;
                } else {
                    velocity_setpoint = 0.0; // 默认目标速度为0
                }
                
                // 速度环计算
                velocity_pid->setSetpoint(velocity_setpoint);
                accel_setpoint = velocity_pid->compute(current_velocity);
                velocity_output_last = accel_setpoint;
                
                // 加速度环计算
                accel_pid->setSetpoint(accel_setpoint);
                final_output = accel_pid->compute(current_acceleration);
                accel_output_last = final_output;
                
                // 位置环输出设为0
                position_output_last = 0.0;
                break;
                
            case PIDDebugMode::FULL_CASCADE:
            default:
                // 全三环级联控制
                velocity_setpoint = position_pid->compute(current_position);
                position_output_last = velocity_setpoint;
                
                velocity_pid->setSetpoint(velocity_setpoint);
                accel_setpoint = velocity_pid->compute(current_velocity);
                velocity_output_last = accel_setpoint;
                
                accel_pid->setSetpoint(accel_setpoint);
                final_output = accel_pid->compute(current_acceleration);
                accel_output_last = final_output;
                break;
        }
        
        // 应用输出限制
        if (final_output > output_max) {
            final_output = output_max;
        } else if (final_output < output_min) {
            final_output = output_min;
        }
        
        return final_output;
    }
    
    void reset() {
        position_pid->reset();
        velocity_pid->reset();
        accel_pid->reset();
        position_output_last = 0.0;
        velocity_output_last = 0.0;
        accel_output_last = 0.0;
    }
    
    void resetIntegralOnly() {
        position_pid->resetIntegralOnly();
        velocity_pid->resetIntegralOnly();
        accel_pid->resetIntegralOnly();
    }
    
    // 获取当前测量值
    double getCurrentPosition() const { return current_position; }
    double getCurrentVelocity() const { return current_velocity; }
    double getCurrentAcceleration() const { return current_acceleration; }
    
    // 获取调试模式
    PIDDebugMode getDebugMode() const { return debug_mode; }
    
    // 获取各环节的输出
    double getPositionOutput() const { return position_output_last; }
    double getVelocityOutput() const { return velocity_output_last; }
    double getAccelOutput() const { return accel_output_last; }
    
    // 获取各PID控制器的P、I、D项
    double getPositionPTerm() const { return position_pid->getLastPTerm(); }
    double getPositionITerm() const { return position_pid->getLastITerm(); }
    double getPositionDTerm() const { return position_pid->getLastDTerm(); }
    
    double getVelocityPTerm() const { return velocity_pid->getLastPTerm(); }
    double getVelocityITerm() const { return velocity_pid->getLastITerm(); }
    double getVelocityDTerm() const { return velocity_pid->getLastDTerm(); }
    
    double getAccelPTerm() const { return accel_pid->getLastPTerm(); }
    double getAccelITerm() const { return accel_pid->getLastITerm(); }
    double getAccelDTerm() const { return accel_pid->getLastDTerm(); }
    
    // 获取当前设定点
    double getPositionSetpoint() const { return position_pid->getSetpoint(); }
    double getVelocitySetpoint() const { return velocity_pid->getSetpoint(); }
    double getAccelSetpoint() const { return accel_pid->getSetpoint(); }
};

class FlightControllerInterface {
private:
    ros::NodeHandle nh;
    ros::Publisher rc_pub;
    ros::ServiceClient arming_client;
    ros::Subscriber imu_sub;
    ros::Subscriber altitude_sub;
    ros::Subscriber velocity_sub;
    ros::Rate* rate;
    
    // 可调试的PID控制器
    DebuggableCascadedPIDController* altitude_pid;
    
    double current_altitude;
    double current_velocity_z;
    double current_accel_z;
    double target_altitude;
    uint16_t base_throttle;
    
    // 数据记录
    std::ofstream data_file;
    std::string csv_filename;
    bool logging_enabled;
    ros::Time start_time;
    
    // 修正后的PWM控制变量
    uint16_t current_pwm_ch3;
    uint16_t current_pwm_ch4;
    uint16_t target_pwm_ch3;
    uint16_t target_pwm_ch4;
    
    // PWM平滑控制
    double pwm_base_change_rate;
    double pwm_max_change_rate;
    
    // PWM累积机制（解决小数截断问题）
    double pwm_accumulator_ch3;
    double pwm_accumulator_ch4;
    
    uint16_t last_stable_pwm_ch3;
    uint16_t last_stable_pwm_ch4;
    bool in_deadzone_last_time;
    
    double altitude_deadzone;
    int count;
    uint16_t pwm_deadzone;
    
    // PID计算控制
    bool pid_calculation_enabled;
    int pid_calculation_interval;
    int pid_calculation_counter;
    double last_altitude_output;
    
    // 调试相关
    int debug_print_counter;
    bool verbose_debug;

    std::string generateTimestampedFilename() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        
        std::stringstream ss;
        ss << "pid_debug_data_";
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
        ss << ".csv";
        
        return ss.str();
    }
    
    void initDataLogging() {
        if (logging_enabled) {
            csv_filename = generateTimestampedFilename();
            data_file.open(csv_filename);
            
            data_file << "timestamp,debug_mode,altitude,altitude_error,velocity_z,accel_z,"
                      << "pos_setpoint,vel_setpoint,acc_setpoint,"
                      << "pos_p,pos_i,pos_d,vel_p,vel_i,vel_d,acc_p,acc_i,acc_d,"
                      << "pos_out,vel_out,acc_out,final_output,motor3_pwm,motor4_pwm,"
                      << "pid_calc_enabled" << std::endl;
            
            start_time = ros::Time::now();
            ROS_INFO("调试数据记录已开始，文件名: %s", csv_filename.c_str());
        }
    }
    
    void logData(int debug_mode, double altitude, double altitude_error, 
                double velocity_z, double accel_z,
                double pos_setpoint, double vel_setpoint, double acc_setpoint,
                double pos_p, double pos_i, double pos_d,
                double vel_p, double vel_i, double vel_d,
                double acc_p, double acc_i, double acc_d,
                double pos_out, double vel_out, double acc_out, double final_output,
                uint16_t motor3_pwm, uint16_t motor4_pwm,
                bool pid_calc_enabled) {
        if (logging_enabled && data_file.is_open()) {
            double elapsed = (ros::Time::now() - start_time).toSec();
            
            data_file << std::fixed << std::setprecision(4) 
                     << elapsed << ","
                     << debug_mode << ","
                     << altitude << ","
                     << altitude_error << ","
                     << velocity_z << ","
                     << accel_z << ","
                     << pos_setpoint << ","
                     << vel_setpoint << ","
                     << acc_setpoint << ","
                     << pos_p << "," << pos_i << "," << pos_d << ","
                     << vel_p << "," << vel_i << "," << vel_d << ","
                     << acc_p << "," << acc_i << "," << acc_d << ","
                     << pos_out << "," << vel_out << "," << acc_out << ","
                     << final_output << ","
                     << motor3_pwm << "," << motor4_pwm << ","
                     << (pid_calc_enabled ? 1 : 0) << std::endl;
        }
    }

    // 动态计算PWM变化速率
    double calculatePWMChangeRate() {
        uint16_t max_diff = std::max(
            std::abs((int)current_pwm_ch3 - (int)target_pwm_ch3),
            std::abs((int)current_pwm_ch4 - (int)target_pwm_ch4)
        );
        
        if (max_diff <= 5) {
            return pwm_base_change_rate;  // 小误差时使用基础速率
        } else if (max_diff <= 20) {
            return pwm_base_change_rate * 2;  // 中等误差时加速
        } else if (max_diff <= 50) {
            return pwm_base_change_rate * 5;  // 大误差时大幅加速
        } else {
            return pwm_max_change_rate;  // 极大误差时使用最大速率
        }
    }

public:
    FlightControllerInterface() : current_altitude(0.0), current_velocity_z(0.0), current_accel_z(0.0), 
                                 target_altitude(0.5), base_throttle(1500),
                                 logging_enabled(true),
                                 current_pwm_ch3(1500), current_pwm_ch4(1500),
                                 target_pwm_ch3(1500), target_pwm_ch4(1500),
                                 last_stable_pwm_ch3(1500), last_stable_pwm_ch4(1500),
                                 in_deadzone_last_time(false),
                                 pwm_base_change_rate(3.0), pwm_max_change_rate(15.0),
                                 altitude_deadzone(0.0),
                                 pwm_deadzone(30), count(0),
                                 pid_calculation_enabled(true), pid_calculation_interval(5), 
                                 pid_calculation_counter(0), last_altitude_output(0.0),
                                 debug_print_counter(0), verbose_debug(false) {
        
        rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 100);
        
        ROS_INFO("等待解锁服务...");
        ros::service::waitForService("/mavros/cmd/arming");
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        ROS_INFO("解锁服务已准备就绪");
        
        imu_sub = nh.subscribe("/mavros/imu/data", 100, &FlightControllerInterface::imuCallback, this);
        altitude_sub = nh.subscribe("/mavros/global_position/rel_alt", 100, 
                                   &FlightControllerInterface::altitudeCallback, this);
        velocity_sub = nh.subscribe("/mavros/global_position/local", 100,
                                   &FlightControllerInterface::odometryCallback, this);

        rate = new ros::Rate(100);
        
        // 从ROS参数服务器读取PID参数，如果没有设置则使用默认值
        double pos_kp, pos_ki, pos_kd;
        double vel_kp, vel_ki, vel_kd;
        double acc_kp, acc_ki, acc_kd;
        
        // 位置环参数
        nh.param("position_pid/kp", pos_kp, 3.0);
        nh.param("position_pid/ki", pos_ki, 0.0);
        nh.param("position_pid/kd", pos_kd, 0.0);
        
        // 速度环参数
        nh.param("velocity_pid/kp", vel_kp, 8.0);
        nh.param("velocity_pid/ki", vel_ki, 0.0);
        nh.param("velocity_pid/kd", vel_kd, 0.0);
        
        // 加速度环参数
        nh.param("acceleration_pid/kp", acc_kp, 0.5);
        nh.param("acceleration_pid/ki", acc_ki, 0.1);
        nh.param("acceleration_pid/kd", acc_kd, 0.5);
        
        // 目标深度
        nh.param("target_altitude", target_altitude, 0.5);
        
        // PWM平滑参数
        nh.param("pwm_base_change_rate", pwm_base_change_rate, 3.0);
        nh.param("pwm_max_change_rate", pwm_max_change_rate, 15.0);
        nh.param("pid_calculation_interval", pid_calculation_interval, 5);
        
        ROS_INFO("=== 载入PID参数 ===");
        ROS_INFO("位置环PID: P=%.3f, I=%.3f, D=%.3f", pos_kp, pos_ki, pos_kd);
        ROS_INFO("速度环PID: P=%.3f, I=%.3f, D=%.3f", vel_kp, vel_ki, vel_kd);
        ROS_INFO("加速度环PID: P=%.3f, I=%.3f, D=%.3f", acc_kp, acc_ki, acc_kd);
        ROS_INFO("目标深度: %.2f 米", target_altitude);
        ROS_INFO("PWM平滑参数: 基础速率=%.1f, 最大速率=%.1f", 
                 pwm_base_change_rate, pwm_max_change_rate);
        ROS_INFO("PID计算间隔: %d 个周期", pid_calculation_interval);
        
        // 使用参数服务器的值初始化PID控制器
        altitude_pid = new DebuggableCascadedPIDController(
            pos_kp, pos_ki, pos_kd,     // 位置环参数
            vel_kp, vel_ki, vel_kd,     // 速度环参数
            acc_kp, acc_ki, acc_kd,     // 加速度环参数
            target_altitude,            // 目标深度
            -400.0, 400.0                // 输出限制
        );
        
        altitude_pid->setFilterCoefficients(0.2, 0.3);
        initDataLogging();
    }

    ~FlightControllerInterface() {
        if (data_file.is_open()) {
            data_file.close();
            ROS_INFO("调试数据记录已结束，文件已保存: %s", csv_filename.c_str());
        }
        
        delete rate;
        delete altitude_pid;
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        current_accel_z = 9.9 - msg->linear_acceleration.z;
        // current_accel_z = msg->linear_acceleration.z;
    }
    
    void altitudeCallback(const std_msgs::Float64::ConstPtr& msg) {
        current_altitude = -msg->data;
    }
    
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_velocity_z = msg->twist.twist.linear.z;
    }
    
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

    void controlMotors(uint16_t throttle_ch3, uint16_t throttle_ch4) {
        mavros_msgs::OverrideRCIn msg;
        
        for (int i = 0; i < 18; ++i) {
            msg.channels[i] = 65535;
        }
        
        msg.channels[2] = throttle_ch3;
        msg.channels[3] = throttle_ch4;
        
        rc_pub.publish(msg);
    }
    
    // PID调试接口函数
    void setDebugMode(int mode) {
        altitude_pid->setDebugMode(static_cast<PIDDebugMode>(mode));
    }
    
    void setManualSetpoints(double vel_setpoint, double accel_setpoint) {
        altitude_pid->setManualSetpoints(vel_setpoint, accel_setpoint, true);
    }
    
    void disableManualSetpoints() {
        altitude_pid->setManualSetpoints(0.0, 0.0, false);
    }
    
    void updatePositionPID(double kp, double ki, double kd) {
        altitude_pid->updatePositionPID(kp, ki, kd);
    }
    
    void updateVelocityPID(double kp, double ki, double kd) {
        altitude_pid->updateVelocityPID(kp, ki, kd);
    }
    
    void updateAccelPID(double kp, double ki, double kd) {
        altitude_pid->updateAccelPID(kp, ki, kd);
    }
    
    void setVerboseDebug(bool enable) {
        verbose_debug = enable;
    }
    
    // 动态更新PID参数（从ROS参数服务器读取）
    void updatePIDParamsFromServer() {
        double pos_kp, pos_ki, pos_kd;
        double vel_kp, vel_ki, vel_kd;
        double acc_kp, acc_ki, acc_kd;
        
        // 读取位置环参数
        if (nh.getParam("position_pid/kp", pos_kp) &&
            nh.getParam("position_pid/ki", pos_ki) &&
            nh.getParam("position_pid/kd", pos_kd)) {
            altitude_pid->updatePositionPID(pos_kp, pos_ki, pos_kd);
        }
        
        // 读取速度环参数  
        if (nh.getParam("velocity_pid/kp", vel_kp) &&
            nh.getParam("velocity_pid/ki", vel_ki) &&
            nh.getParam("velocity_pid/kd", vel_kd)) {
            altitude_pid->updateVelocityPID(vel_kp, vel_ki, vel_kd);
        }
        
        // 读取加速度环参数
        if (nh.getParam("acceleration_pid/kp", acc_kp) &&
            nh.getParam("acceleration_pid/ki", acc_ki) &&
            nh.getParam("acceleration_pid/kd", acc_kd)) {
            altitude_pid->updateAccelPID(acc_kp, acc_ki, acc_kd);
        }
        
        // 读取目标深度
        double new_target;
        if (nh.getParam("target_altitude", new_target) && new_target != target_altitude) {
            setTargetAltitude(new_target);
        }
        
        // 读取调试模式
        int debug_mode;
        if (nh.getParam("debug_mode", debug_mode)) {
            setDebugMode(debug_mode);
        }
        
        // 读取手动设定点
        double manual_vel, manual_accel;
        bool use_manual;
        if (nh.getParam("manual_setpoints/velocity", manual_vel) &&
            nh.getParam("manual_setpoints/acceleration", manual_accel) &&
            nh.getParam("manual_setpoints/enable", use_manual)) {
            if (use_manual) {
                setManualSetpoints(manual_vel, manual_accel);
            } else {
                disableManualSetpoints();
            }
        }
        
        // 读取PWM平滑参数
        double new_base_rate, new_max_rate;
        int new_interval;
        if (nh.getParam("pwm_base_change_rate", new_base_rate)) {
            pwm_base_change_rate = new_base_rate;
        }
        if (nh.getParam("pwm_max_change_rate", new_max_rate)) {
            pwm_max_change_rate = new_max_rate;
        }
        if (nh.getParam("pid_calculation_interval", new_interval)) {
            pid_calculation_interval = new_interval;
        }
    }
    
    void setTargetAltitude(double depth) {
        target_altitude = depth;
        altitude_pid->setPositionSetpoint(depth);
        ROS_INFO("目标深度设置为: %.2f 米", depth);
    }
    
    void stabilize() {
        altitude_pid->setMeasurements(current_altitude, current_velocity_z, current_accel_z);
        
        double altitude_output = last_altitude_output;  // 默认使用上次的输出
        double altitude_error = target_altitude - current_altitude;
        bool in_deadzone_now = std::abs(altitude_error) <= altitude_deadzone;
        
        // 固定周期PID计算逻辑：每5个周期计算一次PID
        bool should_calculate_pid = false;
        
        pid_calculation_counter++;
        if (pid_calculation_counter >= pid_calculation_interval) {
            should_calculate_pid = true;
            pid_calculation_counter = 0;
        }
        
        // 只有在需要计算PID时才进行计算
        if (should_calculate_pid && !in_deadzone_now) {
            altitude_output = altitude_pid->compute();
            last_altitude_output = altitude_output;  // 保存这次的输出

            if (altitude_output > 0) {
                altitude_output += pwm_deadzone;
            } else if (altitude_output < 0) {
                altitude_output -= pwm_deadzone;
            }
            
            target_pwm_ch3 = base_throttle + (altitude_output);
            target_pwm_ch4 = base_throttle + (altitude_output);
            
            target_pwm_ch3 = std::max((uint16_t)1100, std::min((uint16_t)1900, target_pwm_ch3));
            target_pwm_ch4 = std::max((uint16_t)1100, std::min((uint16_t)1900, target_pwm_ch4));
            
            last_stable_pwm_ch3 = target_pwm_ch3;
            last_stable_pwm_ch4 = target_pwm_ch4;
            
            pid_calculation_enabled = true;
        } else if (in_deadzone_now) {
            if (!in_deadzone_last_time) {
                altitude_pid->resetIntegralOnly();
                ROS_INFO("进入深度死区，保持最后稳定PWM值: %d, %d", last_stable_pwm_ch3, last_stable_pwm_ch4);
            }
            
            target_pwm_ch3 = last_stable_pwm_ch3;
            target_pwm_ch4 = last_stable_pwm_ch4;
            pid_calculation_enabled = false;
        } else {
            // PWM还在调整中或者还没到PID计算时间，保持当前目标不变
            pid_calculation_enabled = false;
        }
        
        in_deadzone_last_time = in_deadzone_now;
        
        // 获取调试信息
        PIDDebugMode mode = altitude_pid->getDebugMode();
        double pos_setpoint = altitude_pid->getPositionSetpoint();
        double vel_setpoint = altitude_pid->getVelocitySetpoint();
        double acc_setpoint = altitude_pid->getAccelSetpoint();
        
        double pos_p = altitude_pid->getPositionPTerm();
        double pos_i = altitude_pid->getPositionITerm();
        double pos_d = altitude_pid->getPositionDTerm();
        
        double vel_p = altitude_pid->getVelocityPTerm();
        double vel_i = altitude_pid->getVelocityITerm();
        double vel_d = altitude_pid->getVelocityDTerm();
        
        double acc_p = altitude_pid->getAccelPTerm();
        double acc_i = altitude_pid->getAccelITerm();
        double acc_d = altitude_pid->getAccelDTerm();
        
        double pos_out = altitude_pid->getPositionOutput();
        double vel_out = altitude_pid->getVelocityOutput();
        double acc_out = altitude_pid->getAccelOutput();
        
        // 执行PWM平滑调整
        smoothPWMTransition();
        
        // 记录调试数据
        logData((int)mode, current_altitude, altitude_error, current_velocity_z, current_accel_z,
                pos_setpoint, vel_setpoint, acc_setpoint,
                pos_p, pos_i, pos_d, vel_p, vel_i, vel_d, acc_p, acc_i, acc_d,
                pos_out, vel_out, acc_out, last_altitude_output, current_pwm_ch3, current_pwm_ch4,
                should_calculate_pid);
        
        controlMotors(current_pwm_ch3, current_pwm_ch4);
        
        // 调试信息打印
        debug_print_counter++;
        if (debug_print_counter >= 50) {  // 每50次循环打印一次 (0.5秒)
            debug_print_counter = 0;
            
            printf("\033[2J\033[H"); // 清屏并移动光标到顶部
            printf("\033[1;36m=== PID调试信息 ===\033[0m\n");
            printf("\033[1;33m调试模式: %d ", (int)mode);
            switch(mode) {
                case PIDDebugMode::ACCELERATION_ONLY:
                    printf("(仅加速度环)\033[0m\n");
                    break;
                case PIDDebugMode::VELOCITY_ACCEL:
                    printf("(速度+加速度环)\033[0m\n");
                    break;
                case PIDDebugMode::FULL_CASCADE:
                    printf("(全三环级联)\033[0m\n");
                    break;
            }
            
            printf("\033[1;32m当前状态:\033[0m\n");
            printf("  深度: %.3fm, 目标: %.3fm, 误差: %.3fm\n", 
                   current_altitude, target_altitude, altitude_error);
            printf("  速度: %.4fm/s, 加速度: %.3fm/s²\n", 
                   current_velocity_z, current_accel_z);
            
            printf("\033[1;34m设定点:\033[0m\n");
            printf("  位置设定: %.3fm, 速度设定: %.4fm/s, 加速度设定: %.3fm/s²\n", 
                   pos_setpoint, vel_setpoint, acc_setpoint);
            
            printf("\033[1;35mPWM控制状态:\033[0m\n");
            printf("  目标PWM: %d, %d | 当前PWM: %d, %d\n", 
                   target_pwm_ch3, target_pwm_ch4, current_pwm_ch3, current_pwm_ch4);
            printf("  PID计算: %s | 计数器: %d/%d\n", 
                   (should_calculate_pid ? "是" : "否"),
                   pid_calculation_counter, pid_calculation_interval);
            
            if (verbose_debug) {
                printf("\033[1;35mPID分量:\033[0m\n");
                printf("  位置环 - P: %6.3f, I: %6.3f, D: %6.3f → 输出: %6.3f\n", 
                       pos_p, pos_i, pos_d, pos_out);
                printf("  速度环 - P: %6.3f, I: %6.3f, D: %6.3f → 输出: %6.3f\n", 
                       vel_p, vel_i, vel_d, vel_out);
                printf("  加速环 - P: %6.3f, I: %6.3f, D: %6.3f → 输出: %6.3f\n", 
                       acc_p, acc_i, acc_d, acc_out);
            }
            
            printf("\033[1;31m最终输出: %.2f\033[0m\n", last_altitude_output);
            
            printf("\033[1;37m按键提示: 可在另一个终端中使用rostopic或rosparam动态调整参数\033[0m\n");
            fflush(stdout);
        }
    }
    
    void smoothPWMTransition() {
        // 动态计算PWM变化速率
        double change_rate = calculatePWMChangeRate();
        
        // 对ch3进行平滑调整
        if (current_pwm_ch3 < target_pwm_ch3) {
            uint16_t step = std::min((uint16_t)change_rate, 
                                    (uint16_t)(target_pwm_ch3 - current_pwm_ch3));
            current_pwm_ch3 += step;
        } else if (current_pwm_ch3 > target_pwm_ch3) {
            uint16_t step = std::min((uint16_t)change_rate, 
                                    (uint16_t)(current_pwm_ch3 - target_pwm_ch3));
            current_pwm_ch3 -= step;
        }
        
        // 对ch4进行平滑调整
        if (current_pwm_ch4 < target_pwm_ch4) {
            uint16_t step = std::min((uint16_t)change_rate, 
                                    (uint16_t)(target_pwm_ch4 - current_pwm_ch4));
            current_pwm_ch4 += step;
        } else if (current_pwm_ch4 > target_pwm_ch4) {
            uint16_t step = std::min((uint16_t)change_rate, 
                                    (uint16_t)(current_pwm_ch4 - target_pwm_ch4));
            current_pwm_ch4 -= step;
        }
    }
    
    void runDebugDemo() {
        if (!armVehicle()) {
            ROS_ERROR("飞控解锁失败，终止演示");
            return;
        }
        
        ROS_INFO("=== PID调试模式演示开始 ===");
        ROS_INFO("简化的PID控制：固定每%d个周期更新一次PID", pid_calculation_interval);
        ROS_INFO("默认开启全三环模式");
        ROS_INFO("你可以通过以下命令实时调整参数:");
        ROS_INFO("rosparam set /position_pid/kp 5.0");
        ROS_INFO("rosparam set /velocity_pid/kp 10.0");
        ROS_INFO("rosparam set /acceleration_pid/kp 0.8");
        ROS_INFO("rosparam set /debug_mode 1  # 1=加速度环, 2=速度+加速度, 3=全三环");
        ROS_INFO("rosparam set /target_altitude 0.8");
        ROS_INFO("rosparam set /pwm_base_change_rate 5.0  # PWM基础变化速率");
        ROS_INFO("rosparam set /pwm_max_change_rate 20.0  # PWM最大变化速率");
        ROS_INFO("rosparam set /pid_calculation_interval 3  # PID计算间隔(周期数)");
        
        // 从ROS参数获取调试模式
        int debug_mode = 3;
        nh.param("debug_mode", debug_mode, 3);
        setDebugMode(debug_mode);
        
        // 如果是单环或双环调试，可以设置手动设定点
        if (debug_mode == 1) {
            double manual_accel = 0.0;
            bool enable_manual = true;
            nh.param("manual_setpoints/acceleration", manual_accel, 0.0);
            nh.param("manual_setpoints/enable", enable_manual, true);
            if (enable_manual) {
                setManualSetpoints(0.0, manual_accel);
                ROS_INFO("加速度环调试模式，手动加速度设定点: %.3f", manual_accel);
            }
        } else if (debug_mode == 2) {
            double manual_vel = 0.0;
            bool enable_manual = true;
            nh.param("manual_setpoints/velocity", manual_vel, 0.0);
            nh.param("manual_setpoints/enable", enable_manual, true);
            if (enable_manual) {
                setManualSetpoints(manual_vel, 0.0);
                ROS_INFO("速度+加速度环调试模式，手动速度设定点: %.3f", manual_vel);
            }
        }
        
        // 是否启用详细调试输出
        bool verbose = false;
        nh.param("verbose_debug", verbose, false);
        setVerboseDebug(verbose);
        
        // 参数更新计数器
        int param_update_counter = 0;
        const int PARAM_UPDATE_INTERVAL = 500; // 每5秒检查一次参数更新
        
        try {
            ros::Duration(1.0).sleep();
            
            while (ros::ok()) {
                // 定期检查参数更新
                param_update_counter++;
                if (param_update_counter >= PARAM_UPDATE_INTERVAL) {
                    param_update_counter = 0;
                    updatePIDParamsFromServer();
                }
                
                stabilize();
                ros::spinOnce();
                rate->sleep();
            }
            
            controlMotors(1500, 1500);
            ROS_INFO("调试控制终止");
            
        } catch (ros::Exception& e) {
            ROS_ERROR("发生错误: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "pid_debug_controller");
    
    try {
        FlightControllerInterface controller;
        controller.runDebugDemo();
    } catch (ros::Exception& e) {
        ROS_ERROR("错误: %s", e.what());
    }
    
    return 0;
}