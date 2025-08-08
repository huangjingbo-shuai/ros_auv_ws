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
    double getError(double measurement) const { return setpoint - measurement; }
};

// 可调试的三环PID控制器（用于深度控制）
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
        ROS_INFO("深度PID调试模式设置为: %d (1=只加速度环, 2=速度+加速度环, 3=全三环)", (int)mode);
    }
    
    // 手动设置各环的目标值（用于调试）
    void setManualSetpoints(double vel_setpoint, double accel_setpoint, bool enable = true) {
        manual_velocity_setpoint = vel_setpoint;
        manual_accel_setpoint = accel_setpoint;
        use_manual_setpoints = enable;
        
        if (enable) {
            ROS_INFO("深度控制启用手动设定点 - 速度目标: %.3f, 加速度目标: %.3f", vel_setpoint, accel_setpoint);
        } else {
            ROS_INFO("深度控制禁用手动设定点，使用级联PID");
        }
    }
    
    // 动态更新PID参数
    void updatePositionPID(double kp, double ki, double kd) {
        position_pid->updateParams(kp, ki, kd);
        ROS_INFO("深度位置环PID更新为: P=%.3f, I=%.3f, D=%.3f", kp, ki, kd);
    }
    
    void updateVelocityPID(double kp, double ki, double kd) {
        velocity_pid->updateParams(kp, ki, kd);
        ROS_INFO("深度速度环PID更新为: P=%.3f, I=%.3f, D=%.3f", kp, ki, kd);
    }
    
    void updateAccelPID(double kp, double ki, double kd) {
        accel_pid->updateParams(kp, ki, kd);
        ROS_INFO("深度加速度环PID更新为: P=%.3f, I=%.3f, D=%.3f", kp, ki, kd);
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
    
    // 深度控制 - 可调试的三环PID控制器
    DebuggableCascadedPIDController* altitude_pid;
    
    // 俯仰角控制 - 简单的单环PID控制器
    PIDController* pitch_pid;
    
    // 深度控制相关变量
    double current_altitude;
    double current_velocity_z;
    double current_accel_z;
    double target_altitude;
    
    // 俯仰角控制相关变量
    double current_pitch;
    double target_pitch;
    
    uint16_t base_throttle;
    
    // 数据记录
    std::ofstream data_file;
    std::string csv_filename;
    bool logging_enabled;
    ros::Time start_time;
    
    // PWM平滑
    uint16_t current_pwm_ch3;
    uint16_t current_pwm_ch4;
    uint16_t target_pwm_ch3;
    uint16_t target_pwm_ch4;
    double pwm_change_rate;
    
    uint16_t last_stable_pwm_ch3;
    uint16_t last_stable_pwm_ch4;
    bool in_deadzone_last_time;
    
    double altitude_deadzone;
    double pitch_deadzone;
    int count;
    uint16_t pwm_deadzone;
    
    // 调试相关
    int debug_print_counter;
    bool verbose_debug;

    std::string generateTimestampedFilename() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        
        std::stringstream ss;
        ss << "dual_pid_debug_data_";
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
        ss << ".csv";
        
        return ss.str();
    }
    
    void initDataLogging() {
        if (logging_enabled) {
            csv_filename = generateTimestampedFilename();
            data_file.open(csv_filename);
            
            data_file << "timestamp,"
                      << "alt_debug_mode,altitude,altitude_error,velocity_z,accel_z,"
                      << "alt_pos_setpoint,alt_vel_setpoint,alt_acc_setpoint,"
                      << "alt_pos_p,alt_pos_i,alt_pos_d,alt_vel_p,alt_vel_i,alt_vel_d,alt_acc_p,alt_acc_i,alt_acc_d,"
                      << "alt_pos_out,alt_vel_out,alt_acc_out,alt_final_output,"
                      << "pitch,pitch_error,pitch_setpoint,pitch_p,pitch_i,pitch_d,pitch_output,"
                      << "motor3_pwm,motor4_pwm" << std::endl;
            
            start_time = ros::Time::now();
            ROS_INFO("双控制器调试数据记录已开始，文件名: %s", csv_filename.c_str());
        }
    }
    
    void logData(int alt_debug_mode, double altitude, double altitude_error, 
                double velocity_z, double accel_z,
                double alt_pos_setpoint, double alt_vel_setpoint, double alt_acc_setpoint,
                double alt_pos_p, double alt_pos_i, double alt_pos_d,
                double alt_vel_p, double alt_vel_i, double alt_vel_d,
                double alt_acc_p, double alt_acc_i, double alt_acc_d,
                double alt_pos_out, double alt_vel_out, double alt_acc_out, double alt_final_output,
                double pitch, double pitch_error, double pitch_setpoint,
                double pitch_p, double pitch_i, double pitch_d, double pitch_output,
                uint16_t motor3_pwm, uint16_t motor4_pwm) {
        if (logging_enabled && data_file.is_open()) {
            double elapsed = (ros::Time::now() - start_time).toSec();
            
            data_file << std::fixed << std::setprecision(4) 
                     << elapsed << ","
                     << alt_debug_mode << "," << altitude << "," << altitude_error << ","
                     << velocity_z << "," << accel_z << ","
                     << alt_pos_setpoint << "," << alt_vel_setpoint << "," << alt_acc_setpoint << ","
                     << alt_pos_p << "," << alt_pos_i << "," << alt_pos_d << ","
                     << alt_vel_p << "," << alt_vel_i << "," << alt_vel_d << ","
                     << alt_acc_p << "," << alt_acc_i << "," << alt_acc_d << ","
                     << alt_pos_out << "," << alt_vel_out << "," << alt_acc_out << ","
                     << alt_final_output << ","
                     << pitch << "," << pitch_error << "," << pitch_setpoint << ","
                     << pitch_p << "," << pitch_i << "," << pitch_d << "," << pitch_output << ","
                     << motor3_pwm << "," << motor4_pwm << std::endl;
        }
    }

    // 四元数转欧拉角函数
    void quaternionToEuler(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw) {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf_q);
        m.getRPY(roll, pitch, yaw);
    }

public:
    FlightControllerInterface() : current_altitude(0.0), current_velocity_z(0.0), current_accel_z(0.0), 
                                 target_altitude(0.5),
                                 current_pitch(0.0), target_pitch(0.0),
                                 base_throttle(1500),
                                 logging_enabled(true),
                                 current_pwm_ch3(1500), current_pwm_ch4(1500),
                                 target_pwm_ch3(1500), target_pwm_ch4(1500),
                                 last_stable_pwm_ch3(1500), last_stable_pwm_ch4(1500),
                                 in_deadzone_last_time(false),
                                 pwm_change_rate(1),
                                 altitude_deadzone(0.0), pitch_deadzone(0.0),
                                 pwm_deadzone(30), count(0),
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
        
        // 从ROS参数服务器读取深度PID参数
        double alt_pos_kp, alt_pos_ki, alt_pos_kd;
        double alt_vel_kp, alt_vel_ki, alt_vel_kd;
        double alt_acc_kp, alt_acc_ki, alt_acc_kd;
        
        // 深度控制位置环参数
        nh.param("altitude_position_pid/kp", alt_pos_kp, 3.0);
        nh.param("altitude_position_pid/ki", alt_pos_ki, 0.0);
        nh.param("altitude_position_pid/kd", alt_pos_kd, 0.0);
        
        // 深度控制速度环参数
        nh.param("altitude_velocity_pid/kp", alt_vel_kp, 8.0);
        nh.param("altitude_velocity_pid/ki", alt_vel_ki, 0.0);
        nh.param("altitude_velocity_pid/kd", alt_vel_kd, 0.0);
        
        // 深度控制加速度环参数
        nh.param("altitude_acceleration_pid/kp", alt_acc_kp, 0.5);
        nh.param("altitude_acceleration_pid/ki", alt_acc_ki, 0.1);
        nh.param("altitude_acceleration_pid/kd", alt_acc_kd, 0.5);
        
        // 从ROS参数服务器读取俯仰角PID参数（单环）
        double pitch_kp, pitch_ki, pitch_kd;
        
        // 俯仰角控制单环参数
        nh.param("pitch_pid/kp", pitch_kp, 100.0);
        nh.param("pitch_pid/ki", pitch_ki, 0.0);
        nh.param("pitch_pid/kd", pitch_kd, 20.0);
        
        // 目标值
        nh.param("target_altitude", target_altitude, 0.5);
        nh.param("target_pitch", target_pitch, 0.0);  // 现在直接是角度
        
        ROS_INFO("=== 载入深度控制PID参数 ===");
        ROS_INFO("位置环PID: P=%.3f, I=%.3f, D=%.3f", alt_pos_kp, alt_pos_ki, alt_pos_kd);
        ROS_INFO("速度环PID: P=%.3f, I=%.3f, D=%.3f", alt_vel_kp, alt_vel_ki, alt_vel_kd);
        ROS_INFO("加速度环PID: P=%.3f, I=%.3f, D=%.3f", alt_acc_kp, alt_acc_ki, alt_acc_kd);
        ROS_INFO("目标深度: %.2f 米", target_altitude);
        
        ROS_INFO("=== 载入俯仰角控制PID参数 ===");
        ROS_INFO("俯仰角PID: P=%.1f, I=%.3f, D=%.1f", pitch_kp, pitch_ki, pitch_kd);
        ROS_INFO("目标俯仰角: %.2f 度", target_pitch);  // 直接显示角度
        
        // 初始化深度PID控制器（三环级联）
        altitude_pid = new DebuggableCascadedPIDController(
            alt_pos_kp, alt_pos_ki, alt_pos_kd,     // 位置环参数
            alt_vel_kp, alt_vel_ki, alt_vel_kd,     // 速度环参数
            alt_acc_kp, alt_acc_ki, alt_acc_kd,     // 加速度环参数
            target_altitude,                        // 目标深度
            -400.0, 400.0                          // 输出限制
        );
        
        // 初始化俯仰角PID控制器（单环）
        pitch_pid = new PIDController(
            pitch_kp, pitch_ki, pitch_kd,          // PID参数
            target_pitch,                           // 目标俯仰角
            -200.0, 200.0                          // 输出限制（差分控制范围）
        );
        
        altitude_pid->setFilterCoefficients(0.2, 0.3);
        pitch_pid->setIntegralLimit(10.0);
        initDataLogging();
    }

    ~FlightControllerInterface() {
        if (data_file.is_open()) {
            data_file.close();
            ROS_INFO("双控制器调试数据记录已结束，文件已保存: %s", csv_filename.c_str());
        }
        
        delete rate;
        delete altitude_pid;
        delete pitch_pid;
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 提取Z轴加速度（深度控制用）
        current_accel_z = 9.9 - msg->linear_acceleration.z;
        
        // 从四元数提取俯仰角
        double roll, pitch, yaw;
        quaternionToEuler(msg->orientation, roll, pitch, yaw);
        
        // 转换为角度（弧度 → 角度）
        current_pitch = -1 * pitch * 180.0 / M_PI;
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
    
    // 深度PID调试接口函数
    void setAltitudeDebugMode(int mode) {
        altitude_pid->setDebugMode(static_cast<PIDDebugMode>(mode));
    }
    
    void setAltitudeManualSetpoints(double vel_setpoint, double accel_setpoint) {
        altitude_pid->setManualSetpoints(vel_setpoint, accel_setpoint, true);
    }
    
    void disableAltitudeManualSetpoints() {
        altitude_pid->setManualSetpoints(0.0, 0.0, false);
    }
    
    void updateAltitudePositionPID(double kp, double ki, double kd) {
        altitude_pid->updatePositionPID(kp, ki, kd);
    }
    
    void updateAltitudeVelocityPID(double kp, double ki, double kd) {
        altitude_pid->updateVelocityPID(kp, ki, kd);
    }
    
    void updateAltitudeAccelPID(double kp, double ki, double kd) {
        altitude_pid->updateAccelPID(kp, ki, kd);
    }
    
    // 俯仰角PID调试接口函数（简化的单环接口）
    void updatePitchPID(double kp, double ki, double kd) {
        pitch_pid->updateParams(kp, ki, kd);
        ROS_INFO("俯仰角PID更新为: P=%.1f, I=%.3f, D=%.1f", kp, ki, kd);
    }
    
    void setVerboseDebug(bool enable) {
        verbose_debug = enable;
    }
    
    // 动态更新PID参数（从ROS参数服务器读取）
    void updatePIDParamsFromServer() {
        // 更新深度控制PID参数
        double alt_pos_kp, alt_pos_ki, alt_pos_kd;
        double alt_vel_kp, alt_vel_ki, alt_vel_kd;
        double alt_acc_kp, alt_acc_ki, alt_acc_kd;
        
        // 读取深度位置环参数
        if (nh.getParam("altitude_position_pid/kp", alt_pos_kp) &&
            nh.getParam("altitude_position_pid/ki", alt_pos_ki) &&
            nh.getParam("altitude_position_pid/kd", alt_pos_kd)) {
            altitude_pid->updatePositionPID(alt_pos_kp, alt_pos_ki, alt_pos_kd);
        }
        
        // 读取深度速度环参数  
        if (nh.getParam("altitude_velocity_pid/kp", alt_vel_kp) &&
            nh.getParam("altitude_velocity_pid/ki", alt_vel_ki) &&
            nh.getParam("altitude_velocity_pid/kd", alt_vel_kd)) {
            altitude_pid->updateVelocityPID(alt_vel_kp, alt_vel_ki, alt_vel_kd);
        }
        
        // 读取深度加速度环参数
        if (nh.getParam("altitude_acceleration_pid/kp", alt_acc_kp) &&
            nh.getParam("altitude_acceleration_pid/ki", alt_acc_ki) &&
            nh.getParam("altitude_acceleration_pid/kd", alt_acc_kd)) {
            altitude_pid->updateAccelPID(alt_acc_kp, alt_acc_ki, alt_acc_kd);
        }
        
        // 更新俯仰角控制PID参数（单环）
        double pitch_kp, pitch_ki, pitch_kd;
        
        // 读取俯仰角参数
        if (nh.getParam("pitch_pid/kp", pitch_kp) &&
            nh.getParam("pitch_pid/ki", pitch_ki) &&
            nh.getParam("pitch_pid/kd", pitch_kd)) {
            updatePitchPID(pitch_kp, pitch_ki, pitch_kd);
        }
        
        // 读取目标值
        double new_altitude_target, new_pitch_target;
        if (nh.getParam("target_altitude", new_altitude_target) && new_altitude_target != target_altitude) {
            setTargetAltitude(new_altitude_target);
        }
        
        if (nh.getParam("target_pitch", new_pitch_target) && new_pitch_target != target_pitch) {
            setTargetPitch(new_pitch_target);
        }
        
        // 读取深度调试模式
        int alt_debug_mode;
        if (nh.getParam("altitude_debug_mode", alt_debug_mode)) {
            setAltitudeDebugMode(alt_debug_mode);
        }
        
        // 读取深度手动设定点
        double alt_manual_vel, alt_manual_accel;
        bool use_alt_manual;
        
        if (nh.getParam("altitude_manual_setpoints/velocity", alt_manual_vel) &&
            nh.getParam("altitude_manual_setpoints/acceleration", alt_manual_accel) &&
            nh.getParam("altitude_manual_setpoints/enable", use_alt_manual)) {
            if (use_alt_manual) {
                setAltitudeManualSetpoints(alt_manual_vel, alt_manual_accel);
            } else {
                disableAltitudeManualSetpoints();
            }
        }
    }
    
    void setTargetAltitude(double depth) {
        target_altitude = depth;
        altitude_pid->setPositionSetpoint(depth);
        ROS_INFO("目标深度设置为: %.2f 米", depth);
    }
    
    void setTargetPitch(double pitch_angle_degrees) {
        target_pitch = pitch_angle_degrees;
        pitch_pid->setSetpoint(pitch_angle_degrees);
        ROS_INFO("目标俯仰角设置为: %.2f 度", pitch_angle_degrees);
    }
    
    void stabilize() {
        // 设置深度PID控制器的测量值
        altitude_pid->setMeasurements(current_altitude, current_velocity_z, current_accel_z);
        
        double altitude_output = 0.0;
        double pitch_output = 0.0;
        
        double altitude_error = target_altitude - current_altitude;
        double pitch_error = pitch_pid->getError(current_pitch);

        bool in_altitude_deadzone = std::abs(altitude_error) <= altitude_deadzone;
        bool in_pitch_deadzone = std::abs(pitch_error) <= pitch_deadzone;

        // 计算深度控制输出
        if (!in_altitude_deadzone) {
            altitude_output = altitude_pid->compute();

            if (altitude_output > 0) {
                altitude_output += pwm_deadzone;
            } else if (altitude_output < 0) {
                altitude_output += 30;
            }
            
            // 在非死区时，不断更新最后稳定的PWM值（深度控制部分）
            last_stable_pwm_ch3 = base_throttle + altitude_output;
            last_stable_pwm_ch4 = base_throttle + altitude_output;
        } else {
            if (!in_deadzone_last_time) {
                altitude_pid->resetIntegralOnly();
                ROS_INFO("进入深度死区，保持最后稳定PWM值: 前=%d, 后=%d", last_stable_pwm_ch3, last_stable_pwm_ch4);
            }
            // 在死区内，深度输出设为0，保持最后稳定的深度推力
            altitude_output = 0.0;
        }
        
        // 计算俯仰角控制输出（单环PID）
        if (!in_pitch_deadzone) {
            pitch_output = pitch_pid->compute(current_pitch);
        } else {
            pitch_pid->resetIntegralOnly();
        }
        
        // 混合控制：深度控制作为基础，俯仰角控制作为差分
        if (in_altitude_deadzone) {
            // 死区内：使用保持的稳定深度推力 + 俯仰角控制
            target_pwm_ch3 = last_stable_pwm_ch3 - pitch_output;  // 前推进器
            target_pwm_ch4 = last_stable_pwm_ch4 + pitch_output;  // 后推进器
        } else {
            // 非死区：正常的混合控制
            target_pwm_ch3 = base_throttle + altitude_output - pitch_output;  // 前推进器
            target_pwm_ch4 = base_throttle + altitude_output + pitch_output;  // 后推进器
        }
        
        // 应用PWM限制
        target_pwm_ch3 = std::max((uint16_t)1100, std::min((uint16_t)1900, target_pwm_ch3));
        target_pwm_ch4 = std::max((uint16_t)1100, std::min((uint16_t)1900, target_pwm_ch4));
        
        in_deadzone_last_time = in_altitude_deadzone;
        
        // 获取深度控制调试信息
        PIDDebugMode alt_mode = altitude_pid->getDebugMode();
        double alt_pos_setpoint = altitude_pid->getPositionSetpoint();
        double alt_vel_setpoint = altitude_pid->getVelocitySetpoint();
        double alt_acc_setpoint = altitude_pid->getAccelSetpoint();
        
        double alt_pos_p = altitude_pid->getPositionPTerm();
        double alt_pos_i = altitude_pid->getPositionITerm();
        double alt_pos_d = altitude_pid->getPositionDTerm();
        
        double alt_vel_p = altitude_pid->getVelocityPTerm();
        double alt_vel_i = altitude_pid->getVelocityITerm();
        double alt_vel_d = altitude_pid->getVelocityDTerm();
        
        double alt_acc_p = altitude_pid->getAccelPTerm();
        double alt_acc_i = altitude_pid->getAccelITerm();
        double alt_acc_d = altitude_pid->getAccelDTerm();
        
        double alt_pos_out = altitude_pid->getPositionOutput();
        double alt_vel_out = altitude_pid->getVelocityOutput();
        double alt_acc_out = altitude_pid->getAccelOutput();
        
        // 获取俯仰角控制调试信息（单环）
        double pitch_setpoint = pitch_pid->getSetpoint();
        double pitch_p = pitch_pid->getLastPTerm();
        double pitch_i = pitch_pid->getLastITerm();
        double pitch_d = pitch_pid->getLastDTerm();
        
        smoothPWMTransition();
        
        // 记录调试数据
        logData((int)alt_mode, current_altitude, altitude_error, current_velocity_z, current_accel_z,
                alt_pos_setpoint, alt_vel_setpoint, alt_acc_setpoint,
                alt_pos_p, alt_pos_i, alt_pos_d, alt_vel_p, alt_vel_i, alt_vel_d, 
                alt_acc_p, alt_acc_i, alt_acc_d, alt_pos_out, alt_vel_out, alt_acc_out, altitude_output,
                current_pitch, pitch_error, pitch_setpoint, pitch_p, pitch_i, pitch_d, pitch_output,
                current_pwm_ch3, current_pwm_ch4);
        
        controlMotors(current_pwm_ch3, current_pwm_ch4);
        
        // 调试信息打印
        debug_print_counter++;
        if (debug_print_counter >= 50) {  // 每50次循环打印一次 (0.5秒)
            debug_print_counter = 0;
            
            printf("\033[2J\033[H"); // 清屏并移动光标到顶部
            printf("\033[1;36m=== 双控制器PID调试信息 ===\033[0m\n");
            
            printf("\033[1;32m=== 深度控制（三环级联） ===\033[0m\n");
            printf("\033[1;33m调试模式: %d ", (int)alt_mode);
            switch(alt_mode) {
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
            
            printf("当前状态:\n");
            printf("  深度: %.3fm, 目标: %.3fm, 误差: %.3fm\n", 
                   current_altitude, target_altitude, altitude_error);
            printf("  速度: %.4fm/s, 加速度: %.3fm/s²\n", 
                   current_velocity_z, current_accel_z);
            printf("设定点: 位置=%.3fm, 速度=%.4fm/s, 加速度=%.3fm/s²\n", 
                   alt_pos_setpoint, alt_vel_setpoint, alt_acc_setpoint);
            printf("最终输出: %.2f\n", altitude_output);
            
            printf("\033[1;35m=== 俯仰角控制（单环PID） ===\033[0m\n");
            printf("当前状态:\n");
            printf("  俯仰角: %.3f°, 目标: %.3f°, 误差: %.3f°\n", 
                   current_pitch, target_pitch, pitch_error);
            printf("PID分量: P=%.2f, I=%.2f, D=%.2f\n", pitch_p, pitch_i, pitch_d);
            printf("最终输出: %.2f\n", pitch_output);
            
            if (verbose_debug) {
                printf("\033[1;34m深度PID详细分量:\033[0m\n");
                printf("  位置环 - P: %6.3f, I: %6.3f, D: %6.3f → 输出: %6.3f\n", 
                       alt_pos_p, alt_pos_i, alt_pos_d, alt_pos_out);
                printf("  速度环 - P: %6.3f, I: %6.3f, D: %6.3f → 输出: %6.3f\n", 
                       alt_vel_p, alt_vel_i, alt_vel_d, alt_vel_out);
                printf("  加速环 - P: %6.3f, I: %6.3f, D: %6.3f → 输出: %6.3f\n", 
                       alt_acc_p, alt_acc_i, alt_acc_d, alt_acc_out);
            }
            
            printf("\033[1;31m混合控制输出: 前电机PWM=%d, 后电机PWM=%d\033[0m\n", 
                   current_pwm_ch3, current_pwm_ch4);
            
            printf("\033[1;37m按键提示: 可在另一个终端中使用rostopic或rosparam动态调整参数\033[0m\n");
            fflush(stdout);
        }
    }
    
    void smoothPWMTransition() {
        if (current_pwm_ch3 < target_pwm_ch3) {
            current_pwm_ch3 = std::min(target_pwm_ch3, static_cast<uint16_t>(current_pwm_ch3 + pwm_change_rate));
        } else if (current_pwm_ch3 > target_pwm_ch3) {
            current_pwm_ch3 = std::max(target_pwm_ch3, static_cast<uint16_t>(current_pwm_ch3 - pwm_change_rate));
        }
        
        if (current_pwm_ch4 < target_pwm_ch4) {
            current_pwm_ch4 = std::min(target_pwm_ch4, static_cast<uint16_t>(current_pwm_ch4 + pwm_change_rate));
        } else if (current_pwm_ch4 > target_pwm_ch4) {
            current_pwm_ch4 = std::max(target_pwm_ch4, static_cast<uint16_t>(current_pwm_ch4 - pwm_change_rate));
        }
    }
    
    void runDebugDemo() {
        if (!armVehicle()) {
            ROS_ERROR("飞控解锁失败，终止演示");
            return;
        }
        
        ROS_INFO("=== 双控制器PID调试模式演示开始 ===");
        ROS_INFO("深度控制: 三环级联PID，默认开启全三环模式");
        ROS_INFO("俯仰角控制: 单环PID，直接从角度误差到输出");
        ROS_INFO("你可以通过以下命令实时调整参数:");
        ROS_INFO("深度控制:");
        ROS_INFO("  rosparam set /altitude_position_pid/kp 5.0");
        ROS_INFO("  rosparam set /altitude_velocity_pid/kp 10.0");
        ROS_INFO("  rosparam set /altitude_acceleration_pid/kp 0.8");
        ROS_INFO("  rosparam set /altitude_debug_mode 1  # 1=加速度环, 2=速度+加速度, 3=全三环");
        ROS_INFO("  rosparam set /target_altitude 0.8");
        ROS_INFO("俯仰角控制:");
        ROS_INFO("  rosparam set /pitch_pid/kp 100.0");
        ROS_INFO("  rosparam set /pitch_pid/ki 0.0");
        ROS_INFO("  rosparam set /pitch_pid/kd 20.0");
        ROS_INFO("  rosparam set /target_pitch 5.0  # 角度单位，直接输入5度");
        
        // 从ROS参数获取调试模式
        int alt_debug_mode = 3;
        nh.param("altitude_debug_mode", alt_debug_mode, 3);
        setAltitudeDebugMode(alt_debug_mode);
        
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
            ROS_INFO("双控制器调试控制终止");
            
        } catch (ros::Exception& e) {
            ROS_ERROR("发生错误: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "dual_pid_debug_controller");
    
    try {
        FlightControllerInterface controller;
        controller.runDebugDemo();
    } catch (ros::Exception& e) {
        ROS_ERROR("错误: %s", e.what());
    }
    
    return 0;
}