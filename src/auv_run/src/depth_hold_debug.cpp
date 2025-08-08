#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <sstream>
#include <deque>
#include <cmath>

// PID调试模式枚举
enum class PIDDebugMode {
    ACCELERATION_ONLY = 1,    // 只调试加速度环（内环）
    VELOCITY_ACCEL = 2,       // 调试速度环+加速度环（内环+中环）
    FULL_CASCADE = 3          // 全三环调试
};

// 控制模式枚举
enum class ControlMode {
    DEPTH_ONLY = 1,           // 只控制深度
    PITCH_ONLY = 2,           // 只控制俯仰角
    YAW_ONLY = 3,             // 只控制偏航角
    DEPTH_AND_PITCH = 4,      // 控制深度和俯仰角
    DEPTH_AND_YAW = 5,        // 控制深度和偏航角
    PITCH_AND_YAW = 6,        // 控制俯仰角和偏航角
    ALL_CONTROL = 7           // 全控制模式(深度+俯仰+偏航)
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
    
    PIDDebugMode debug_mode;
    
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
    
    void setDebugMode(PIDDebugMode mode) {
        debug_mode = mode;
    }
    
    void setManualSetpoints(double vel_setpoint, double accel_setpoint, bool enable = true) {
        manual_velocity_setpoint = vel_setpoint;
        manual_accel_setpoint = accel_setpoint;
        use_manual_setpoints = enable;
    }
    
    void updatePositionPID(double kp, double ki, double kd) {
        position_pid->updateParams(kp, ki, kd);
    }
    
    void updateVelocityPID(double kp, double ki, double kd) {
        velocity_pid->updateParams(kp, ki, kd);
    }
    
    void updateAccelPID(double kp, double ki, double kd) {
        accel_pid->updateParams(kp, ki, kd);
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
    
    double compute() {
        double velocity_setpoint = 0.0;
        double accel_setpoint = 0.0;
        double final_output = 0.0;
        
        switch (debug_mode) {
            case PIDDebugMode::ACCELERATION_ONLY:
                if (use_manual_setpoints) {
                    accel_setpoint = manual_accel_setpoint;
                } else {
                    accel_setpoint = 0.0;
                }
                
                accel_pid->setSetpoint(accel_setpoint);
                final_output = accel_pid->compute(current_acceleration);
                accel_output_last = final_output;
                
                position_output_last = 0.0;
                velocity_output_last = 0.0;
                break;
                
            case PIDDebugMode::VELOCITY_ACCEL:
                if (use_manual_setpoints) {
                    velocity_setpoint = manual_velocity_setpoint;
                } else {
                    velocity_setpoint = 0.0;
                }
                
                velocity_pid->setSetpoint(velocity_setpoint);
                accel_setpoint = velocity_pid->compute(current_velocity);
                velocity_output_last = accel_setpoint;
                
                accel_pid->setSetpoint(accel_setpoint);
                final_output = accel_pid->compute(current_acceleration);
                accel_output_last = final_output;
                
                position_output_last = 0.0;
                break;
                
            case PIDDebugMode::FULL_CASCADE:
            default:
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
    
    // 获取调试信息的方法
    double getCurrentPosition() const { return current_position; }
    double getCurrentVelocity() const { return current_velocity; }
    double getCurrentAcceleration() const { return current_acceleration; }
    PIDDebugMode getDebugMode() const { return debug_mode; }
    double getPositionOutput() const { return position_output_last; }
    double getVelocityOutput() const { return velocity_output_last; }
    double getAccelOutput() const { return accel_output_last; }
    
    double getPositionPTerm() const { return position_pid->getLastPTerm(); }
    double getPositionITerm() const { return position_pid->getLastITerm(); }
    double getPositionDTerm() const { return position_pid->getLastDTerm(); }
    
    double getVelocityPTerm() const { return velocity_pid->getLastPTerm(); }
    double getVelocityITerm() const { return velocity_pid->getLastITerm(); }
    double getVelocityDTerm() const { return velocity_pid->getLastDTerm(); }
    
    double getAccelPTerm() const { return accel_pid->getLastPTerm(); }
    double getAccelITerm() const { return accel_pid->getLastITerm(); }
    double getAccelDTerm() const { return accel_pid->getLastDTerm(); }
    
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
    
    // 深度控制PID
    DebuggableCascadedPIDController* altitude_pid;
    
    // Pitch角控制PID
    PIDController* pitch_pid;
    
    // Yaw角控制PID
    PIDController* yaw_pid;
    
    // 控制模式
    ControlMode control_mode;
    
    // 深度相关变量
    double current_altitude;
    double current_velocity_z;
    double current_accel_z;
    double target_altitude;
    
    // Pitch角相关变量
    double current_pitch;
    double current_pitch_rate;
    double target_pitch;
    double pitch_filter_alpha;
    
    // Yaw角相关变量
    double current_yaw;
    double current_yaw_rate;
    double target_yaw;
    double initial_yaw;          // 初始偏航角（作为参考零点）
    double yaw_filter_alpha;
    bool yaw_initialized;
    
    // PWM控制变量
    uint16_t base_throttle;
    uint16_t current_pwm_ch1, current_pwm_ch2;  // CH1右推进器, CH2左推进器 (Yaw控制)
    uint16_t current_pwm_ch3, current_pwm_ch4;  // CH3前推进器, CH4后推进器 (深度+Pitch控制)
    uint16_t target_pwm_ch1, target_pwm_ch2;
    uint16_t target_pwm_ch3, target_pwm_ch4;
    
    // PWM平滑控制
    double pwm_base_change_rate;
    double pwm_max_change_rate;
    
    uint16_t last_stable_pwm_ch1, last_stable_pwm_ch2;
    uint16_t last_stable_pwm_ch3, last_stable_pwm_ch4;
    bool in_deadzone_last_time_altitude;
    bool in_deadzone_last_time_pitch;
    bool in_deadzone_last_time_yaw;
    
    double altitude_deadzone;
    double pitch_deadzone;
    double yaw_deadzone;
    uint16_t pwm_deadzone;
    
    // PID计算控制
    bool pid_calculation_enabled;
    int pid_calculation_interval;
    int pid_calculation_counter;
    double last_altitude_output;
    double last_pitch_output;
    double last_yaw_output;
    
    // 数据记录
    std::ofstream data_file;
    std::string csv_filename;
    bool logging_enabled;
    ros::Time start_time;
    
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
            
            data_file << "timestamp,control_mode,debug_mode,"
                      << "altitude,altitude_error,velocity_z,accel_z,"
                      << "pitch,pitch_error,pitch_rate,"
                      << "yaw,yaw_error,yaw_rate,"
                      << "alt_pos_setpoint,alt_vel_setpoint,alt_acc_setpoint,"
                      << "pitch_setpoint,pitch_output,yaw_setpoint,yaw_output,"
                      << "alt_pos_p,alt_pos_i,alt_pos_d,"
                      << "alt_vel_p,alt_vel_i,alt_vel_d,"
                      << "alt_acc_p,alt_acc_i,alt_acc_d,"
                      << "pitch_p,pitch_i,pitch_d,"
                      << "yaw_p,yaw_i,yaw_d,"
                      << "alt_pos_out,alt_vel_out,alt_acc_out,alt_final_output,"
                      << "motor1_pwm,motor2_pwm,motor3_pwm,motor4_pwm,"
                      << "pid_calc_enabled" << std::endl;
            
            start_time = ros::Time::now();
            ROS_INFO("调试数据记录已开始，文件名: %s", csv_filename.c_str());
        }
    }
    
    void logData(int control_mode, int debug_mode, 
                double altitude, double altitude_error, double velocity_z, double accel_z,
                double pitch, double pitch_error, double pitch_rate,
                double yaw, double yaw_error, double yaw_rate,
                double alt_pos_setpoint, double alt_vel_setpoint, double alt_acc_setpoint,
                double pitch_setpoint, double pitch_output, double yaw_setpoint, double yaw_output,
                double alt_pos_p, double alt_pos_i, double alt_pos_d,
                double alt_vel_p, double alt_vel_i, double alt_vel_d,
                double alt_acc_p, double alt_acc_i, double alt_acc_d,
                double pitch_p, double pitch_i, double pitch_d,
                double yaw_p, double yaw_i, double yaw_d,
                double alt_pos_out, double alt_vel_out, double alt_acc_out, double alt_final_output,
                uint16_t motor1_pwm, uint16_t motor2_pwm, uint16_t motor3_pwm, uint16_t motor4_pwm,
                bool pid_calc_enabled) {
        if (logging_enabled && data_file.is_open()) {
            double elapsed = (ros::Time::now() - start_time).toSec();
            
            data_file << std::fixed << std::setprecision(4) 
                     << elapsed << ","
                     << control_mode << "," << debug_mode << ","
                     << altitude << "," << altitude_error << "," << velocity_z << "," << accel_z << ","
                     << pitch << "," << pitch_error << "," << pitch_rate << ","
                     << yaw << "," << yaw_error << "," << yaw_rate << ","
                     << alt_pos_setpoint << "," << alt_vel_setpoint << "," << alt_acc_setpoint << ","
                     << pitch_setpoint << "," << pitch_output << "," << yaw_setpoint << "," << yaw_output << ","
                     << alt_pos_p << "," << alt_pos_i << "," << alt_pos_d << ","
                     << alt_vel_p << "," << alt_vel_i << "," << alt_vel_d << ","
                     << alt_acc_p << "," << alt_acc_i << "," << alt_acc_d << ","
                     << pitch_p << "," << pitch_i << "," << pitch_d << ","
                     << yaw_p << "," << yaw_i << "," << yaw_d << ","
                     << alt_pos_out << "," << alt_vel_out << "," << alt_acc_out << ","
                     << alt_final_output << ","
                     << motor1_pwm << "," << motor2_pwm << "," << motor3_pwm << "," << motor4_pwm << ","
                     << (pid_calc_enabled ? 1 : 0) << std::endl;
        }
    }

    // 动态计算PWM变化速率
    double calculatePWMChangeRate() {
        uint16_t max_diff = std::max({
            std::abs((int)current_pwm_ch1 - (int)target_pwm_ch1),
            std::abs((int)current_pwm_ch2 - (int)target_pwm_ch2),
            std::abs((int)current_pwm_ch3 - (int)target_pwm_ch3),
            std::abs((int)current_pwm_ch4 - (int)target_pwm_ch4)
        });
        
        if (max_diff <= 5) {
            return pwm_base_change_rate;
        } else if (max_diff <= 20) {
            return pwm_base_change_rate * 2;
        } else if (max_diff <= 50) {
            return pwm_base_change_rate * 5;
        } else {
            return pwm_max_change_rate;
        }
    }

    // 角度标准化函数，将角度限制在[-180, 180]范围内
    double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

public:
    FlightControllerInterface() : 
        current_altitude(0.0), current_velocity_z(0.0), current_accel_z(0.0), target_altitude(0.5),
        current_pitch(0.0), current_pitch_rate(0.0), target_pitch(0.0), pitch_filter_alpha(0.3),
        current_yaw(0.0), current_yaw_rate(0.0), target_yaw(0.0), initial_yaw(0.0), 
        yaw_filter_alpha(0.3), yaw_initialized(false),
        base_throttle(1500), logging_enabled(true),
        current_pwm_ch1(1500), current_pwm_ch2(1500), current_pwm_ch3(1500), current_pwm_ch4(1500),
        target_pwm_ch1(1500), target_pwm_ch2(1500), target_pwm_ch3(1500), target_pwm_ch4(1500),
        last_stable_pwm_ch1(1500), last_stable_pwm_ch2(1500),
        last_stable_pwm_ch3(1500), last_stable_pwm_ch4(1500),
        in_deadzone_last_time_altitude(false), in_deadzone_last_time_pitch(false), in_deadzone_last_time_yaw(false),
        pwm_base_change_rate(3.0), pwm_max_change_rate(15.0),
        altitude_deadzone(0.0), pitch_deadzone(2.0), yaw_deadzone(5.0), pwm_deadzone(30),
        pid_calculation_enabled(true), pid_calculation_interval(5), 
        pid_calculation_counter(0), last_altitude_output(0.0), last_pitch_output(0.0), last_yaw_output(0.0),
        debug_print_counter(0), verbose_debug(false),
        control_mode(ControlMode::ALL_CONTROL) {
        
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
        
        // 从ROS参数服务器读取PID参数
        double pos_kp, pos_ki, pos_kd;
        double vel_kp, vel_ki, vel_kd;
        double acc_kp, acc_ki, acc_kd;
        double pitch_kp, pitch_ki, pitch_kd;
        double yaw_kp, yaw_ki, yaw_kd;
        
        // 深度控制PID参数
        nh.param("position_pid/kp", pos_kp, 3.0);
        nh.param("position_pid/ki", pos_ki, 0.0);
        nh.param("position_pid/kd", pos_kd, 0.0);
        
        nh.param("velocity_pid/kp", vel_kp, 8.0);
        nh.param("velocity_pid/ki", vel_ki, 0.0);
        nh.param("velocity_pid/kd", vel_kd, 0.0);
        
        nh.param("acceleration_pid/kp", acc_kp, 0.5);
        nh.param("acceleration_pid/ki", acc_ki, 0.1);
        nh.param("acceleration_pid/kd", acc_kd, 0.5);
        
        // Pitch角控制PID参数
        nh.param("pitch_pid/kp", pitch_kp, 2.0);
        nh.param("pitch_pid/ki", pitch_ki, 0.1);
        nh.param("pitch_pid/kd", pitch_kd, 0.5);
        
        // Yaw角控制PID参数
        nh.param("yaw_pid/kp", yaw_kp, 1.5);
        nh.param("yaw_pid/ki", yaw_ki, 0.05);
        nh.param("yaw_pid/kd", yaw_kd, 0.3);
        
        // 目标值
        nh.param("target_altitude", target_altitude, 0.5);
        nh.param("target_pitch", target_pitch, 0.0);
        nh.param("target_yaw", target_yaw, 0.0);
        
        // 控制模式
        int mode;
        nh.param("control_mode", mode, 7);
        control_mode = static_cast<ControlMode>(mode);
        
        // PWM平滑参数
        nh.param("pwm_base_change_rate", pwm_base_change_rate, 3.0);
        nh.param("pwm_max_change_rate", pwm_max_change_rate, 15.0);
        nh.param("pid_calculation_interval", pid_calculation_interval, 5);
        
        // 死区参数
        nh.param("altitude_deadzone", altitude_deadzone, 0.02);
        nh.param("pitch_deadzone", pitch_deadzone, 2.0);
        nh.param("yaw_deadzone", yaw_deadzone, 5.0);
        
        ROS_INFO("=== 载入PID参数 ===");
        ROS_INFO("深度控制 - 位置环PID: P=%.3f, I=%.3f, D=%.3f", pos_kp, pos_ki, pos_kd);
        ROS_INFO("深度控制 - 速度环PID: P=%.3f, I=%.3f, D=%.3f", vel_kp, vel_ki, vel_kd);
        ROS_INFO("深度控制 - 加速度环PID: P=%.3f, I=%.3f, D=%.3f", acc_kp, acc_ki, acc_kd);
        ROS_INFO("俯仰角控制PID: P=%.3f, I=%.3f, D=%.3f", pitch_kp, pitch_ki, pitch_kd);
        ROS_INFO("偏航角控制PID: P=%.3f, I=%.3f, D=%.3f", yaw_kp, yaw_ki, yaw_kd);
        ROS_INFO("目标深度: %.2f 米, 目标俯仰角: %.1f 度, 目标偏航角: %.1f 度", target_altitude, target_pitch, target_yaw);
        ROS_INFO("控制模式: %d (1=仅深度, 2=仅俯仰, 3=仅偏航, 4=深度+俯仰, 5=深度+偏航, 6=俯仰+偏航, 7=全控制)", (int)control_mode);
        
        // 初始化PID控制器
        altitude_pid = new DebuggableCascadedPIDController(
            pos_kp, pos_ki, pos_kd,
            vel_kp, vel_ki, vel_kd,
            acc_kp, acc_ki, acc_kd,
            target_altitude,
            -400.0, 400.0
        );
        
        pitch_pid = new PIDController(
            pitch_kp, pitch_ki, pitch_kd,
            target_pitch,
            -400.0, 400.0
        );
        
        yaw_pid = new PIDController(
            yaw_kp, yaw_ki, yaw_kd,
            target_yaw,
            -400.0, 400.0
        );
        
        altitude_pid->setFilterCoefficients(0.2, 0.3);
        pitch_pid->setIntegralLimit(50.0);
        yaw_pid->setIntegralLimit(50.0);
        
        initDataLogging();
    }

    ~FlightControllerInterface() {
        if (data_file.is_open()) {
            data_file.close();
            ROS_INFO("调试数据记录已结束，文件已保存: %s", csv_filename.c_str());
        }
        
        delete rate;
        delete altitude_pid;
        delete pitch_pid;
        delete yaw_pid;
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 获取加速度
        current_accel_z = 9.9 - msg->linear_acceleration.z;
        
        // 直接从四元数计算俯仰角和偏航角，避免tf2依赖问题
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;
        
        // 从四元数计算俯仰角（弧度）
        double pitch_rad = std::asin(2.0 * (qw * qy - qz * qx));
        
        // 从四元数计算偏航角（弧度）
        double yaw_rad = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        
        // 将弧度转换为度并应用滤波
        double pitch_deg = pitch_rad * 180.0 / M_PI;
        double yaw_deg = yaw_rad * 180.0 / M_PI;
        
        pitch_deg = normalizeAngle(pitch_deg);
        yaw_deg = normalizeAngle(yaw_deg);
        
        // 初始化yaw零点
        if (!yaw_initialized) {
            initial_yaw = yaw_deg;
            yaw_initialized = true;
            ROS_INFO("偏航角零点已初始化: %.1f 度", initial_yaw);
        }
        
        // 计算相对于初始位置的偏航角
        double relative_yaw = normalizeAngle(yaw_deg - initial_yaw);
        
        current_pitch = pitch_filter_alpha * pitch_deg + (1 - pitch_filter_alpha) * current_pitch;
        current_yaw = yaw_filter_alpha * relative_yaw + (1 - yaw_filter_alpha) * current_yaw;
        
        // 计算角速率（简单差分）
        static double last_pitch = current_pitch;
        static double last_yaw = current_yaw;
        static ros::Time last_time = ros::Time::now();
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        
        if (dt > 0) {
            current_pitch_rate = (current_pitch - last_pitch) / dt;
            current_yaw_rate = normalizeAngle(current_yaw - last_yaw) / dt;
        }
        
        last_pitch = current_pitch;
        last_yaw = current_yaw;
        last_time = current_time;
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

    void controlMotors(uint16_t throttle_ch1, uint16_t throttle_ch2,
                      uint16_t throttle_ch3, uint16_t throttle_ch4) {
        mavros_msgs::OverrideRCIn msg;
        
        for (int i = 0; i < 18; ++i) {
            msg.channels[i] = 65535;
        }
        
        msg.channels[0] = throttle_ch1;  // CH1: 右推进器 (机器人右边，用于Yaw控制)
        msg.channels[1] = throttle_ch2;  // CH2: 左推进器 (机器人左边，用于Yaw控制)
        msg.channels[2] = throttle_ch3;  // CH3: 前推进器 (机器人正前方，用于深度+Pitch控制)
        msg.channels[3] = throttle_ch4;  // CH4: 后推进器 (机器人正后方，用于深度+Pitch控制)
        
        rc_pub.publish(msg);
    }
    
    // 设置控制模式
    void setControlMode(int mode) {
        control_mode = static_cast<ControlMode>(mode);
        ROS_INFO("控制模式设置为: %d (1=仅深度, 2=仅俯仰, 3=深度+俯仰)", mode);
    }
    
    // PID调试接口函数
    void setDebugMode(int mode) {
        altitude_pid->setDebugMode(static_cast<PIDDebugMode>(mode));
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
    
    void updatePitchPID(double kp, double ki, double kd) {
        pitch_pid->updateParams(kp, ki, kd);
        ROS_INFO("俯仰角PID更新为: P=%.3f, I=%.3f, D=%.3f", kp, ki, kd);
    }
    
    void updateYawPID(double kp, double ki, double kd) {
        yaw_pid->updateParams(kp, ki, kd);
        ROS_INFO("偏航角PID更新为: P=%.3f, I=%.3f, D=%.3f", kp, ki, kd);
    }
    
    void setTargetAltitude(double depth) {
        target_altitude = depth;
        altitude_pid->setPositionSetpoint(depth);
        ROS_INFO("目标深度设置为: %.2f 米", depth);
    }
    
    void setTargetPitch(double pitch) {
        target_pitch = normalizeAngle(pitch);
        pitch_pid->setSetpoint(target_pitch);
        ROS_INFO("目标俯仰角设置为: %.1f 度", target_pitch);
    }
    
    void setTargetYaw(double yaw) {
        target_yaw = normalizeAngle(yaw);
        yaw_pid->setSetpoint(target_yaw);
        ROS_INFO("目标偏航角设置为: %.1f 度", target_yaw);
    }
    
    void resetYawZero() {
        if (yaw_initialized) {
            initial_yaw = current_yaw + initial_yaw;  // 将当前角度作为新的零点
            target_yaw = 0.0;
            yaw_pid->setSetpoint(0.0);
            yaw_pid->reset();
            ROS_INFO("偏航角零点已重置，当前角度设为新零点");
        }
    }
    
    void setVerboseDebug(bool enable) {
        verbose_debug = enable;
    }
    
    // 动态更新PID参数（从ROS参数服务器读取）
    void updatePIDParamsFromServer() {
        double pos_kp, pos_ki, pos_kd;
        double vel_kp, vel_ki, vel_kd;
        double acc_kp, acc_ki, acc_kd;
        double pitch_kp, pitch_ki, pitch_kd;
        double yaw_kp, yaw_ki, yaw_kd;
        
        // 读取深度控制PID参数
        if (nh.getParam("position_pid/kp", pos_kp) &&
            nh.getParam("position_pid/ki", pos_ki) &&
            nh.getParam("position_pid/kd", pos_kd)) {
            altitude_pid->updatePositionPID(pos_kp, pos_ki, pos_kd);
        }
        
        if (nh.getParam("velocity_pid/kp", vel_kp) &&
            nh.getParam("velocity_pid/ki", vel_ki) &&
            nh.getParam("velocity_pid/kd", vel_kd)) {
            altitude_pid->updateVelocityPID(vel_kp, vel_ki, vel_kd);
        }
        
        if (nh.getParam("acceleration_pid/kp", acc_kp) &&
            nh.getParam("acceleration_pid/ki", acc_ki) &&
            nh.getParam("acceleration_pid/kd", acc_kd)) {
            altitude_pid->updateAccelPID(acc_kp, acc_ki, acc_kd);
        }
        
        // 读取俯仰角PID参数
        if (nh.getParam("pitch_pid/kp", pitch_kp) &&
            nh.getParam("pitch_pid/ki", pitch_ki) &&
            nh.getParam("pitch_pid/kd", pitch_kd)) {
            updatePitchPID(pitch_kp, pitch_ki, pitch_kd);
        }
        
        // 读取偏航角PID参数
        if (nh.getParam("yaw_pid/kp", yaw_kp) &&
            nh.getParam("yaw_pid/ki", yaw_ki) &&
            nh.getParam("yaw_pid/kd", yaw_kd)) {
            updateYawPID(yaw_kp, yaw_ki, yaw_kd);
        }
        
        // 读取目标值
        double new_target_altitude, new_target_pitch, new_target_yaw;
        if (nh.getParam("target_altitude", new_target_altitude) && 
            new_target_altitude != target_altitude) {
            setTargetAltitude(new_target_altitude);
        }
        
        if (nh.getParam("target_pitch", new_target_pitch) && 
            new_target_pitch != target_pitch) {
            setTargetPitch(new_target_pitch);
        }
        
        if (nh.getParam("target_yaw", new_target_yaw) && 
            new_target_yaw != target_yaw) {
            setTargetYaw(new_target_yaw);
        }
        
        // 读取控制模式
        int mode;
        if (nh.getParam("control_mode", mode)) {
            setControlMode(mode);
        }
        
        // 读取调试模式
        int debug_mode;
        if (nh.getParam("debug_mode", debug_mode)) {
            setDebugMode(debug_mode);
        }
        
        // 检查是否需要重置yaw零点
        bool reset_yaw;
        if (nh.getParam("reset_yaw_zero", reset_yaw) && reset_yaw) {
            resetYawZero();
            // 重置参数避免重复执行
            nh.setParam("reset_yaw_zero", false);
        }
    }
    
    void stabilize() {
        altitude_pid->setMeasurements(current_altitude, current_velocity_z, current_accel_z);
        
        double altitude_output = last_altitude_output;
        double pitch_output = last_pitch_output;
        double yaw_output = last_yaw_output;
        
        double altitude_error = target_altitude - current_altitude;
        double pitch_error = normalizeAngle(target_pitch - current_pitch);
        double yaw_error = normalizeAngle(target_yaw - current_yaw);
        
        bool in_altitude_deadzone = std::abs(altitude_error) <= altitude_deadzone;
        bool in_pitch_deadzone = std::abs(pitch_error) <= pitch_deadzone;
        bool in_yaw_deadzone = std::abs(yaw_error) <= yaw_deadzone;
        
        // PID计算逻辑
        bool should_calculate_pid = false;
        
        pid_calculation_counter++;
        if (pid_calculation_counter >= pid_calculation_interval) {
            should_calculate_pid = true;
            pid_calculation_counter = 0;
        }
        
        // 深度控制计算
        if (should_calculate_pid && !in_altitude_deadzone && 
            (control_mode == ControlMode::DEPTH_ONLY || control_mode == ControlMode::DEPTH_AND_PITCH ||
             control_mode == ControlMode::DEPTH_AND_YAW || control_mode == ControlMode::ALL_CONTROL)) {
            altitude_output = altitude_pid->compute();
            last_altitude_output = altitude_output;
            
            if (altitude_output > 0) {
                altitude_output += pwm_deadzone;
            } else if (altitude_output < 0) {
                altitude_output -= pwm_deadzone;
            }
        } else if (in_altitude_deadzone) {
            if (!in_deadzone_last_time_altitude) {
                altitude_pid->resetIntegralOnly();
            }
            altitude_output = 0;
        }
        
        // Pitch角控制计算
        if (should_calculate_pid && !in_pitch_deadzone && 
            (control_mode == ControlMode::PITCH_ONLY || control_mode == ControlMode::DEPTH_AND_PITCH ||
             control_mode == ControlMode::PITCH_AND_YAW || control_mode == ControlMode::ALL_CONTROL)) {
            pitch_output = pitch_pid->compute(current_pitch);
            last_pitch_output = pitch_output;
            
            if (pitch_output > 0) {
                pitch_output += pwm_deadzone;
            } else if (pitch_output < 0) {
                pitch_output -= pwm_deadzone;
            }
        } else if (in_pitch_deadzone) {
            if (!in_deadzone_last_time_pitch) {
                pitch_pid->resetIntegralOnly();
            }
            pitch_output = 0;
        }
        
        // Yaw角控制计算
        if (should_calculate_pid && !in_yaw_deadzone && 
            (control_mode == ControlMode::YAW_ONLY || control_mode == ControlMode::DEPTH_AND_YAW ||
             control_mode == ControlMode::PITCH_AND_YAW || control_mode == ControlMode::ALL_CONTROL)) {
            yaw_output = yaw_pid->compute(current_yaw);
            last_yaw_output = yaw_output;
            
            if (yaw_output > 0) {
                yaw_output += pwm_deadzone;
            } else if (yaw_output < 0) {
                yaw_output -= pwm_deadzone;
            }
        } else if (in_yaw_deadzone) {
            if (!in_deadzone_last_time_yaw) {
                yaw_pid->resetIntegralOnly();
            }
            yaw_output = 0;
        }
        
        in_deadzone_last_time_altitude = in_altitude_deadzone;
        in_deadzone_last_time_pitch = in_pitch_deadzone;
        in_deadzone_last_time_yaw = in_yaw_deadzone;
        
        // 根据控制模式分配PWM值
        // CH1: 右推进器, CH2: 左推进器 (Yaw控制)
        // CH3: 前推进器, CH4: 后推进器 (深度+Pitch控制)
        // Yaw控制逻辑：右转(yaw > 0)：右电机后推，左电机前推
        switch (control_mode) {
            case ControlMode::DEPTH_ONLY:
                // 只控制深度
                target_pwm_ch1 = base_throttle;
                target_pwm_ch2 = base_throttle;
                target_pwm_ch3 = base_throttle + altitude_output;
                target_pwm_ch4 = base_throttle + altitude_output;
                break;
                
            case ControlMode::PITCH_ONLY:
                // 只控制Pitch
                target_pwm_ch1 = base_throttle;
                target_pwm_ch2 = base_throttle;
                target_pwm_ch3 = base_throttle - pitch_output;   // 前推进器
                target_pwm_ch4 = base_throttle + pitch_output;   // 后推进器
                break;
                
            case ControlMode::YAW_ONLY:
                // 只控制Yaw
                target_pwm_ch1 = base_throttle - yaw_output;     // 右推进器
                target_pwm_ch2 = base_throttle + yaw_output;     // 左推进器
                target_pwm_ch3 = base_throttle;
                target_pwm_ch4 = base_throttle;
                break;
                
            case ControlMode::DEPTH_AND_PITCH:
                // 深度+俯仰角控制
                target_pwm_ch1 = base_throttle;
                target_pwm_ch2 = base_throttle;
                target_pwm_ch3 = base_throttle + altitude_output - pitch_output;   // 前推进器
                target_pwm_ch4 = base_throttle + altitude_output + pitch_output;   // 后推进器
                break;
                
            case ControlMode::DEPTH_AND_YAW:
                // 深度+偏航角控制
                target_pwm_ch1 = base_throttle - yaw_output;     // 右推进器
                target_pwm_ch2 = base_throttle + yaw_output;     // 左推进器
                target_pwm_ch3 = base_throttle + altitude_output;
                target_pwm_ch4 = base_throttle + altitude_output;
                break;
                
            case ControlMode::PITCH_AND_YAW:
                // 俯仰角+偏航角控制
                target_pwm_ch1 = base_throttle - yaw_output;     // 右推进器
                target_pwm_ch2 = base_throttle + yaw_output;     // 左推进器
                target_pwm_ch3 = base_throttle - pitch_output;   // 前推进器
                target_pwm_ch4 = base_throttle + pitch_output;   // 后推进器
                break;
                
            case ControlMode::ALL_CONTROL:
            default:
                // 全控制模式：深度+俯仰角+偏航角
                target_pwm_ch1 = base_throttle - yaw_output;                      // 右推进器
                target_pwm_ch2 = base_throttle + yaw_output;                      // 左推进器
                target_pwm_ch3 = base_throttle + altitude_output - pitch_output;  // 前推进器
                target_pwm_ch4 = base_throttle + altitude_output + pitch_output;  // 后推进器
                break;
        }
        
        // PWM限制
        target_pwm_ch1 = std::max((uint16_t)1100, std::min((uint16_t)1900, target_pwm_ch1));
        target_pwm_ch2 = std::max((uint16_t)1100, std::min((uint16_t)1900, target_pwm_ch2));
        target_pwm_ch3 = std::max((uint16_t)1100, std::min((uint16_t)1900, target_pwm_ch3));
        target_pwm_ch4 = std::max((uint16_t)1100, std::min((uint16_t)1900, target_pwm_ch4));
        
        // 保存稳定PWM值
        if (should_calculate_pid) {
            last_stable_pwm_ch1 = target_pwm_ch1;
            last_stable_pwm_ch2 = target_pwm_ch2;
            last_stable_pwm_ch3 = target_pwm_ch3;
            last_stable_pwm_ch4 = target_pwm_ch4;
        }
        
        // 获取调试信息
        PIDDebugMode debug_mode = altitude_pid->getDebugMode();
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
        
        double pitch_p = pitch_pid->getLastPTerm();
        double pitch_i = pitch_pid->getLastITerm();
        double pitch_d = pitch_pid->getLastDTerm();
        
        double yaw_p = yaw_pid->getLastPTerm();
        double yaw_i = yaw_pid->getLastITerm();
        double yaw_d = yaw_pid->getLastDTerm();
        
        // 执行PWM平滑调整
        smoothPWMTransition();
        
        // 记录调试数据
        logData((int)control_mode, (int)debug_mode, 
                current_altitude, altitude_error, current_velocity_z, current_accel_z,
                current_pitch, pitch_error, current_pitch_rate,
                current_yaw, yaw_error, current_yaw_rate,
                alt_pos_setpoint, alt_vel_setpoint, alt_acc_setpoint,
                target_pitch, last_pitch_output, target_yaw, last_yaw_output,
                alt_pos_p, alt_pos_i, alt_pos_d,
                alt_vel_p, alt_vel_i, alt_vel_d,
                alt_acc_p, alt_acc_i, alt_acc_d,
                pitch_p, pitch_i, pitch_d, yaw_p, yaw_i, yaw_d,
                alt_pos_out, alt_vel_out, alt_acc_out, last_altitude_output,
                current_pwm_ch1, current_pwm_ch2, current_pwm_ch3, current_pwm_ch4,
                should_calculate_pid);
        
        controlMotors(current_pwm_ch1, current_pwm_ch2, current_pwm_ch3, current_pwm_ch4);
        
        // 调试信息打印
        debug_print_counter++;
        if (debug_print_counter >= 50) {
            debug_print_counter = 0;
            
            printf("\033[2J\033[H"); // 清屏
            printf("\033[1;36m=== ROV 6DOF PID调试信息 ===\033[0m\n");
            
            printf("\033[1;33m控制模式: %d ", (int)control_mode);
            switch(control_mode) {
                case ControlMode::DEPTH_ONLY:
                    printf("(仅深度控制)\033[0m\n");
                    break;
                case ControlMode::PITCH_ONLY:
                    printf("(仅俯仰角控制)\033[0m\n");
                    break;
                case ControlMode::YAW_ONLY:
                    printf("(仅偏航角控制)\033[0m\n");
                    break;
                case ControlMode::DEPTH_AND_PITCH:
                    printf("(深度+俯仰角控制)\033[0m\n");
                    break;
                case ControlMode::DEPTH_AND_YAW:
                    printf("(深度+偏航角控制)\033[0m\n");
                    break;
                case ControlMode::PITCH_AND_YAW:
                    printf("(俯仰角+偏航角控制)\033[0m\n");
                    break;
                case ControlMode::ALL_CONTROL:
                    printf("(全控制模式)\033[0m\n");
                    break;
            }
            
            printf("\033[1;33m深度调试模式: %d ", (int)debug_mode);
            switch(debug_mode) {
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
            printf("  俯仰角: %.1f°, 目标: %.1f°, 误差: %.1f°\n", 
                   current_pitch, target_pitch, pitch_error);
            printf("  俯仰角速率: %.2f°/s\n", current_pitch_rate);
            printf("  偏航角: %.1f°, 目标: %.1f°, 误差: %.1f°\n", 
                   current_yaw, target_yaw, yaw_error);
            printf("  偏航角速率: %.2f°/s\n", current_yaw_rate);
            
            printf("\033[1;34m设定点:\033[0m\n");
            printf("  深度 - 位置: %.3fm, 速度: %.4fm/s, 加速度: %.3fm/s²\n", 
                   alt_pos_setpoint, alt_vel_setpoint, alt_acc_setpoint);
            printf("  俯仰角设定点: %.1f°\n", target_pitch);
            printf("  偏航角设定点: %.1f°\n", target_yaw);
            
            printf("\033[1;35mPWM控制状态:\033[0m\n");
            printf("  目标PWM - 右(CH1):%d, 左(CH2):%d, 前(CH3):%d, 后(CH4):%d\n", 
                   target_pwm_ch1, target_pwm_ch2, target_pwm_ch3, target_pwm_ch4);
            printf("  当前PWM - 右(CH1):%d, 左(CH2):%d, 前(CH3):%d, 后(CH4):%d\n", 
                   current_pwm_ch1, current_pwm_ch2, current_pwm_ch3, current_pwm_ch4);
            printf("  PID计算: %s | 计数器: %d/%d\n", 
                   (should_calculate_pid ? "是" : "否"),
                   pid_calculation_counter, pid_calculation_interval);
            
            if (verbose_debug) {
                printf("\033[1;35m深度PID分量:\033[0m\n");
                printf("  位置环 - P: %6.3f, I: %6.3f, D: %6.3f → 输出: %6.3f\n", 
                       alt_pos_p, alt_pos_i, alt_pos_d, alt_pos_out);
                printf("  速度环 - P: %6.3f, I: %6.3f, D: %6.3f → 输出: %6.3f\n", 
                       alt_vel_p, alt_vel_i, alt_vel_d, alt_vel_out);
                printf("  加速环 - P: %6.3f, I: %6.3f, D: %6.3f → 输出: %6.3f\n", 
                       alt_acc_p, alt_acc_i, alt_acc_d, alt_acc_out);
                printf("\033[1;35m俯仰角PID分量:\033[0m\n");
                printf("  俯仰环 - P: %6.3f, I: %6.3f, D: %6.3f → 输出: %6.3f\n", 
                       pitch_p, pitch_i, pitch_d, last_pitch_output);
                printf("\033[1;35m偏航角PID分量:\033[0m\n");
                printf("  偏航环 - P: %6.3f, I: %6.3f, D: %6.3f → 输出: %6.3f\n", 
                       yaw_p, yaw_i, yaw_d, last_yaw_output);
            }
            
            printf("\033[1;31m最终输出 - 深度: %.2f, 俯仰: %.2f, 偏航: %.2f\033[0m\n", 
                   last_altitude_output, last_pitch_output, last_yaw_output);
            
            printf("\033[1;37m按键提示: 可在另一个终端中使用rostopic或rosparam动态调整参数\033[0m\n");
            fflush(stdout);
        }
    }
    
    void smoothPWMTransition() {
        double change_rate = calculatePWMChangeRate();
        
        // 对所有4个通道进行平滑调整
        auto smoothChannel = [&](uint16_t& current, uint16_t target) {
            if (current < target) {
                uint16_t step = std::min((uint16_t)change_rate, (uint16_t)(target - current));
                current += step;
            } else if (current > target) {
                uint16_t step = std::min((uint16_t)change_rate, (uint16_t)(current - target));
                current -= step;
            }
        };
        
        smoothChannel(current_pwm_ch1, target_pwm_ch1);
        smoothChannel(current_pwm_ch2, target_pwm_ch2);
        smoothChannel(current_pwm_ch3, target_pwm_ch3);
        smoothChannel(current_pwm_ch4, target_pwm_ch4);
    }
    
    void runDebugDemo() {
        if (!armVehicle()) {
            ROS_ERROR("飞控解锁失败，终止演示");
            return;
        }
        
        ROS_INFO("=== ROV 6DOF PID调试模式演示开始 ===");
        ROS_INFO("固定每%d个周期更新一次PID", pid_calculation_interval);
        ROS_INFO("默认开启全控制模式(深度+俯仰角+偏航角)");
        ROS_INFO("你可以通过以下命令实时调整参数:");
        ROS_INFO("深度控制:");
        ROS_INFO("  rosparam set /position_pid/kp 5.0");
        ROS_INFO("  rosparam set /velocity_pid/kp 10.0");
        ROS_INFO("  rosparam set /acceleration_pid/kp 0.8");
        ROS_INFO("俯仰角控制:");
        ROS_INFO("  rosparam set /pitch_pid/kp 2.5");
        ROS_INFO("  rosparam set /pitch_pid/ki 0.1");
        ROS_INFO("  rosparam set /pitch_pid/kd 0.8");
        ROS_INFO("偏航角控制:");
        ROS_INFO("  rosparam set /yaw_pid/kp 1.8");
        ROS_INFO("  rosparam set /yaw_pid/ki 0.05");
        ROS_INFO("  rosparam set /yaw_pid/kd 0.5");
        ROS_INFO("目标设定:");
        ROS_INFO("  rosparam set /target_altitude 0.8");
        ROS_INFO("  rosparam set /target_pitch 10.0");
        ROS_INFO("  rosparam set /target_yaw 30.0");
        ROS_INFO("控制模式:");
        ROS_INFO("  rosparam set /control_mode 7  # 1=仅深度, 2=仅俯仰, 3=仅偏航");
        ROS_INFO("                                # 4=深度+俯仰, 5=深度+偏航, 6=俯仰+偏航, 7=全控制");
        ROS_INFO("  rosparam set /debug_mode 1    # 1=加速度环, 2=速度+加速度, 3=全三环");
        ROS_INFO("偏航角零点重置:");
        ROS_INFO("  rosparam set /reset_yaw_zero true  # 将当前角度设为新零点");
        
        // 从ROS参数获取控制模式
        int ctrl_mode = 7;
        nh.param("control_mode", ctrl_mode, 7);
        setControlMode(ctrl_mode);
        
        int debug_mode = 3;
        nh.param("debug_mode", debug_mode, 3);
        setDebugMode(debug_mode);
        
        bool verbose = false;
        nh.param("verbose_debug", verbose, false);
        setVerboseDebug(verbose);
        
        // 参数更新计数器
        int param_update_counter = 0;
        const int PARAM_UPDATE_INTERVAL = 500;
        
        try {
            ros::Duration(1.0).sleep();
            
            while (ros::ok()) {
                param_update_counter++;
                if (param_update_counter >= PARAM_UPDATE_INTERVAL) {
                    param_update_counter = 0;
                    updatePIDParamsFromServer();
                }
                
                stabilize();
                ros::spinOnce();
                rate->sleep();
            }
            
            controlMotors(1500, 1500, 1500, 1500);
            ROS_INFO("调试控制终止");
            
        } catch (ros::Exception& e) {
            ROS_ERROR("发生错误: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "enhanced_rov_pid_controller");
    
    try {
        FlightControllerInterface controller;
        controller.runDebugDemo();
    } catch (ros::Exception& e) {
        ROS_ERROR("错误: %s", e.what());
    }
    
    return 0;
}