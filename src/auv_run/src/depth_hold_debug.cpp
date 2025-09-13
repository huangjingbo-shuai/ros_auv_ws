#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
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
    YAW_ONLY = 2,             // 只控制偏航角
    DOCK_TRACKING = 3         // Dock跟踪模式（深度保持+前进+视觉Yaw）
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
          integral_limit(500.0),
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
        accel_pid->setIntegralLimit(400.0);
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
    ros::Subscriber yolo_pixel_sub;  // 订阅YOLO像素坐标
    ros::Rate* rate;
    
    // 深度控制PID
    DebuggableCascadedPIDController* altitude_pid;
    
    // 视觉Yaw角控制PID
    PIDController* visual_yaw_pid;
    
    // 控制模式
    ControlMode control_mode;
    
    // 深度相关变量
    double current_altitude;
    double current_velocity_z;
    double current_accel_z;
    double target_altitude;
    
    // 视觉Yaw控制相关变量
    double current_pixel_x;        // 当前Dock中心像素X坐标
    double current_pixel_y;        // 当前Dock中心像素Y坐标
    double target_pixel_x;         // 目标像素X坐标（相机中心）
    double target_pixel_y;         // 目标像素Y坐标（相机中心）
    double pixel_error_x;          // 像素误差X
    double pixel_error_y;          // 像素误差Y
    double dock_confidence;        // YOLO检测置信度
    bool dock_detected;           // Dock是否检测到
    ros::Time last_dock_time;     // 上次检测到Dock的时间
    double dock_timeout;          // Dock超时时间
    
    // 前进控制参数
    double forward_thrust;         // Dock跟踪模式的前进推力
    
    // 相机参数
    double camera_cx;              // 相机光心X坐标
    double camera_cy;              // 相机光心Y坐标
    double camera_fx;              // X方向焦距
    double camera_fy;              // Y方向焦距
    
    // PWM控制变量
    uint16_t base_throttle;
    uint16_t current_pwm_ch1, current_pwm_ch2;  // CH1右推进器, CH2左推进器 (Yaw控制+前进)
    uint16_t current_pwm_ch3, current_pwm_ch4;  // CH3前推进器, CH4后推进器 (深度控制)
    uint16_t target_pwm_ch1, target_pwm_ch2;
    uint16_t target_pwm_ch3, target_pwm_ch4;
    
    // PWM平滑控制
    double pwm_base_change_rate;
    double pwm_max_change_rate;
    
    uint16_t last_stable_pwm_ch1, last_stable_pwm_ch2;
    uint16_t last_stable_pwm_ch3, last_stable_pwm_ch4;
    bool in_deadzone_last_time_altitude;
    bool in_deadzone_last_time_visual_yaw;
    
    double altitude_deadzone;
    double visual_yaw_deadzone;
    uint16_t pwm_deadzone;
    
    // PID计算控制
    bool pid_calculation_enabled;
    int pid_calculation_interval;
    int pid_calculation_counter;
    double last_altitude_output;
    double last_visual_yaw_output;
    
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
        ss << "rov_dock_tracking_data_";
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
                      << "pixel_x,pixel_y,pixel_error_x,pixel_error_y,dock_detected,dock_confidence,"
                      << "alt_pos_setpoint,alt_vel_setpoint,alt_acc_setpoint,"
                      << "visual_yaw_setpoint,visual_yaw_output,forward_thrust,"
                      << "alt_pos_p,alt_pos_i,alt_pos_d,"
                      << "alt_vel_p,alt_vel_i,alt_vel_d,"
                      << "alt_acc_p,alt_acc_i,alt_acc_d,"
                      << "visual_yaw_p,visual_yaw_i,visual_yaw_d,"
                      << "alt_pos_out,alt_vel_out,alt_acc_out,alt_final_output,"
                      << "motor1_pwm,motor2_pwm,motor3_pwm,motor4_pwm,"
                      << "pid_calc_enabled" << std::endl;
            
            start_time = ros::Time::now();
            ROS_INFO("调试数据记录已开始，文件名: %s", csv_filename.c_str());
        }
    }
    
    void logData(int control_mode, int debug_mode, 
                double altitude, double altitude_error, double velocity_z, double accel_z,
                double pixel_x, double pixel_y, double pixel_error_x, double pixel_error_y, 
                bool dock_detected, double dock_confidence,
                double alt_pos_setpoint, double alt_vel_setpoint, double alt_acc_setpoint,
                double visual_yaw_setpoint, double visual_yaw_output, double forward_thrust,
                double alt_pos_p, double alt_pos_i, double alt_pos_d,
                double alt_vel_p, double alt_vel_i, double alt_vel_d,
                double alt_acc_p, double alt_acc_i, double alt_acc_d,
                double visual_yaw_p, double visual_yaw_i, double visual_yaw_d,
                double alt_pos_out, double alt_vel_out, double alt_acc_out, double alt_final_output,
                uint16_t motor1_pwm, uint16_t motor2_pwm, uint16_t motor3_pwm, uint16_t motor4_pwm,
                bool pid_calc_enabled) {
        if (logging_enabled && data_file.is_open()) {
            double elapsed = (ros::Time::now() - start_time).toSec();
            
            data_file << std::fixed << std::setprecision(4) 
                     << elapsed << ","
                     << control_mode << "," << debug_mode << ","
                     << altitude << "," << altitude_error << "," << velocity_z << "," << accel_z << ","
                     << pixel_x << "," << pixel_y << "," << pixel_error_x << "," << pixel_error_y << "," 
                     << (dock_detected ? 1 : 0) << "," << dock_confidence << ","
                     << alt_pos_setpoint << "," << alt_vel_setpoint << "," << alt_acc_setpoint << ","
                     << visual_yaw_setpoint << "," << visual_yaw_output << "," << forward_thrust << ","
                     << alt_pos_p << "," << alt_pos_i << "," << alt_pos_d << ","
                     << alt_vel_p << "," << alt_vel_i << "," << alt_vel_d << ","
                     << alt_acc_p << "," << alt_acc_i << "," << alt_acc_d << ","
                     << visual_yaw_p << "," << visual_yaw_i << "," << visual_yaw_d << ","
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

public:
    FlightControllerInterface() : 
        current_altitude(0.0), current_velocity_z(0.0), current_accel_z(0.0), target_altitude(0.6),
        current_pixel_x(0.0), current_pixel_y(0.0), target_pixel_x(318.509118), target_pixel_y(247.737583),
        pixel_error_x(0.0), pixel_error_y(0.0), dock_confidence(0.0), dock_detected(false), dock_timeout(2.0),
        forward_thrust(100.0),  // 默认前进推力
        camera_cx(318.509118), camera_cy(247.737583), camera_fx(410.971988), camera_fy(412.393625),
        base_throttle(1500), logging_enabled(true),
        current_pwm_ch1(1500), current_pwm_ch2(1500), current_pwm_ch3(1500), current_pwm_ch4(1500),
        target_pwm_ch1(1500), target_pwm_ch2(1500), target_pwm_ch3(1500), target_pwm_ch4(1500),
        last_stable_pwm_ch1(1500), last_stable_pwm_ch2(1500),
        last_stable_pwm_ch3(1500), last_stable_pwm_ch4(1500),
        in_deadzone_last_time_altitude(false), in_deadzone_last_time_visual_yaw(false),
        pwm_base_change_rate(3.0), pwm_max_change_rate(15.0),
        altitude_deadzone(0.02), visual_yaw_deadzone(10.0), pwm_deadzone(30),
        pid_calculation_enabled(true), pid_calculation_interval(5), 
        pid_calculation_counter(0), last_altitude_output(0.0), last_visual_yaw_output(0.0),
        debug_print_counter(0), verbose_debug(true),
        control_mode(ControlMode::DOCK_TRACKING) {
        
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
        
        // 订阅YOLO Dock像素坐标
        yolo_pixel_sub = nh.subscribe("/yolo_pixel", 10, 
                                     &FlightControllerInterface::yoloPixelCallback, this);

        rate = new ros::Rate(100);
        
        // 从ROS参数服务器读取PID参数，使用配置文件中的默认值
        double pos_kp, pos_ki, pos_kd;
        double vel_kp, vel_ki, vel_kd;
        double acc_kp, acc_ki, acc_kd;
        double visual_yaw_kp, visual_yaw_ki, visual_yaw_kd;
        
        // 深度控制PID参数 - 使用配置文件值
        nh.param("position_pid/kp", pos_kp, 20.0);
        nh.param("position_pid/ki", pos_ki, 0.0);
        nh.param("position_pid/kd", pos_kd, 0.0);
        
        nh.param("velocity_pid/kp", vel_kp, 10.0);
        nh.param("velocity_pid/ki", vel_ki, 0.0);
        nh.param("velocity_pid/kd", vel_kd, 0.0);
        
        nh.param("acceleration_pid/kp", acc_kp, 0.8);
        nh.param("acceleration_pid/ki", acc_ki, 0.03);
        nh.param("acceleration_pid/kd", acc_kd, 0.8);
        
        // 视觉Yaw控制PID参数
        nh.param("visual_yaw_pid/kp", visual_yaw_kp, 2.0);
        nh.param("visual_yaw_pid/ki", visual_yaw_ki, 0.05);
        nh.param("visual_yaw_pid/kd", visual_yaw_kd, 0.8);
        
        // 目标值 - 使用配置文件值
        nh.param("target_altitude", target_altitude, 0.6);
        
        // 前进推力
        nh.param("forward_thrust", forward_thrust, 100.0);
        
        // 控制模式
        int mode;
        nh.param("control_mode", mode, 3);
        control_mode = static_cast<ControlMode>(mode);
        
        // PWM平滑参数
        nh.param("pwm_base_change_rate", pwm_base_change_rate, 3.0);
        nh.param("pwm_max_change_rate", pwm_max_change_rate, 15.0);
        nh.param("pid_calculation_interval", pid_calculation_interval, 5);
        
        // 死区参数
        nh.param("altitude_deadzone", altitude_deadzone, 0.02);
        nh.param("visual_yaw_deadzone", visual_yaw_deadzone, 10.0);
        
        // 调试输出设置 - 使用配置文件值
        nh.param("verbose_debug", verbose_debug, true);
        
        // 相机参数
        nh.param("camera_cx", camera_cx, 318.509118);
        nh.param("camera_cy", camera_cy, 247.737583);
        nh.param("camera_fx", camera_fx, 410.971988);
        nh.param("camera_fy", camera_fy, 412.393625);
        
        target_pixel_x = camera_cx;
        target_pixel_y = camera_cy;
        
        ROS_INFO("=== 载入PID参数 ===");
        ROS_INFO("深度控制 - 位置环PID: P=%.3f, I=%.3f, D=%.3f", pos_kp, pos_ki, pos_kd);
        ROS_INFO("深度控制 - 速度环PID: P=%.3f, I=%.3f, D=%.3f", vel_kp, vel_ki, vel_kd);
        ROS_INFO("深度控制 - 加速度环PID: P=%.3f, I=%.3f, D=%.3f", acc_kp, acc_ki, acc_kd);
        ROS_INFO("视觉Yaw控制PID: P=%.3f, I=%.3f, D=%.3f", visual_yaw_kp, visual_yaw_ki, visual_yaw_kd);
        ROS_INFO("目标深度: %.2f 米", target_altitude);
        ROS_INFO("前进推力: %.1f", forward_thrust);
        ROS_INFO("控制模式: %d (1=仅深度, 2=仅视觉Yaw, 3=Dock跟踪)", (int)control_mode);
        ROS_INFO("详细调试输出: %s", verbose_debug ? "开启" : "关闭");
        ROS_INFO("相机参数: 光心(%.1f, %.1f), 焦距(%.1f, %.1f)", camera_cx, camera_cy, camera_fx, camera_fy);
        
        // 初始化PID控制器
        altitude_pid = new DebuggableCascadedPIDController(
            pos_kp, pos_ki, pos_kd,
            vel_kp, vel_ki, vel_kd,
            acc_kp, acc_ki, acc_kd,
            target_altitude,
            -400.0, 400.0
        );
        
        visual_yaw_pid = new PIDController(
            visual_yaw_kp, visual_yaw_ki, visual_yaw_kd,
            0.0,  // 目标像素误差为0
            -400.0, 400.0
        );
        
        altitude_pid->setFilterCoefficients(0.2, 0.3);
        visual_yaw_pid->setIntegralLimit(100.0);
        
        // 设置手动设定点（如果启用）
        bool manual_enable;
        double manual_vel, manual_accel;
        nh.param("manual_setpoints/enable", manual_enable, true);
        nh.param("manual_setpoints/velocity", manual_vel, 0.13);
        nh.param("manual_setpoints/acceleration", manual_accel, 1.0);
        
        if (manual_enable) {
            altitude_pid->setManualSetpoints(manual_vel, manual_accel, true);
            ROS_INFO("手动设定点已启用 - 速度: %.3f m/s, 加速度: %.2f m/s²", manual_vel, manual_accel);
        }
        
        initDataLogging();
    }

    ~FlightControllerInterface() {
        if (data_file.is_open()) {
            data_file.close();
            ROS_INFO("调试数据记录已结束，文件已保存: %s", csv_filename.c_str());
        }
        
        delete rate;
        delete altitude_pid;
        delete visual_yaw_pid;
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 获取加速度
        current_accel_z = 9.9 - msg->linear_acceleration.z;
    }
    
    void altitudeCallback(const std_msgs::Float64::ConstPtr& msg) {
        current_altitude = -msg->data;
    }
    
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_velocity_z = msg->twist.twist.linear.z;
    }
    
    // YOLO Dock像素坐标回调函数
    void yoloPixelCallback(const geometry_msgs::Point::ConstPtr& msg) {
        current_pixel_x = msg->x;
        current_pixel_y = msg->y;
        dock_confidence = msg->z;  // YOLO检测置信度
        
        // 计算像素误差
        pixel_error_x = current_pixel_x - target_pixel_x;
        pixel_error_y = current_pixel_y - target_pixel_y;
        
        dock_detected = true;
        last_dock_time = ros::Time::now();
        
        ROS_DEBUG("Dock检测到: 像素(%.1f, %.1f), 置信度: %.3f, 误差(%.1f, %.1f)", 
                  current_pixel_x, current_pixel_y, dock_confidence, pixel_error_x, pixel_error_y);
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
        
        msg.channels[0] = throttle_ch1;  // CH1: 右推进器 (用于Yaw控制+前进)
        msg.channels[1] = throttle_ch2;  // CH2: 左推进器 (用于Yaw控制+前进)
        msg.channels[2] = throttle_ch3;  // CH3: 前推进器 (用于深度控制)
        msg.channels[3] = throttle_ch4;  // CH4: 后推进器 (用于深度控制)
        
        rc_pub.publish(msg);
    }
    
    // 设置控制模式
    void setControlMode(int mode) {
        control_mode = static_cast<ControlMode>(mode);
        ROS_INFO("控制模式设置为: %d (1=仅深度, 2=仅视觉Yaw, 3=Dock跟踪)", mode);
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
    
    void updateVisualYawPID(double kp, double ki, double kd) {
        visual_yaw_pid->updateParams(kp, ki, kd);
        ROS_INFO("视觉Yaw PID更新为: P=%.3f, I=%.3f, D=%.3f", kp, ki, kd);
    }
    
    void setTargetAltitude(double depth) {
        target_altitude = depth;
        altitude_pid->setPositionSetpoint(depth);
        ROS_INFO("目标深度设置为: %.2f 米", depth);
    }
    
    void setVerboseDebug(bool enable) {
        verbose_debug = enable;
    }
    
    void setForwardThrust(double thrust) {
        forward_thrust = thrust;
        ROS_INFO("前进推力设置为: %.1f", thrust);
    }
    
    // 动态更新PID参数（从ROS参数服务器读取）
    void updatePIDParamsFromServer() {
        double pos_kp, pos_ki, pos_kd;
        double vel_kp, vel_ki, vel_kd;
        double acc_kp, acc_ki, acc_kd;
        double visual_yaw_kp, visual_yaw_ki, visual_yaw_kd;
        
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
        
        // 读取视觉Yaw PID参数
        if (nh.getParam("visual_yaw_pid/kp", visual_yaw_kp) &&
            nh.getParam("visual_yaw_pid/ki", visual_yaw_ki) &&
            nh.getParam("visual_yaw_pid/kd", visual_yaw_kd)) {
            updateVisualYawPID(visual_yaw_kp, visual_yaw_ki, visual_yaw_kd);
        }
        
        // 读取目标值
        double new_target_altitude;
        if (nh.getParam("target_altitude", new_target_altitude) && 
            new_target_altitude != target_altitude) {
            setTargetAltitude(new_target_altitude);
        }
        
        // 读取前进推力
        double new_forward_thrust;
        if (nh.getParam("forward_thrust", new_forward_thrust) && 
            new_forward_thrust != forward_thrust) {
            setForwardThrust(new_forward_thrust);
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
        
        // 读取手动设定点
        bool manual_enable;
        double manual_vel, manual_accel;
        if (nh.getParam("manual_setpoints/enable", manual_enable) &&
            nh.getParam("manual_setpoints/velocity", manual_vel) &&
            nh.getParam("manual_setpoints/acceleration", manual_accel)) {
            altitude_pid->setManualSetpoints(manual_vel, manual_accel, manual_enable);
        }
    }
    
    void stabilize() {
        // 检查Dock超时
        if (dock_detected && (ros::Time::now() - last_dock_time).toSec() > dock_timeout) {
            dock_detected = false;
            ROS_WARN("Dock检测超时，视觉控制暂停");
        }
        
        altitude_pid->setMeasurements(current_altitude, current_velocity_z, current_accel_z);
        
        double altitude_output = last_altitude_output;
        double visual_yaw_output = last_visual_yaw_output;
        
        double altitude_error = target_altitude - current_altitude;
        
        bool in_altitude_deadzone = std::abs(altitude_error) <= altitude_deadzone;
        bool in_visual_yaw_deadzone = dock_detected ? (std::abs(pixel_error_x) <= visual_yaw_deadzone) : true;
        
        // PID计算逻辑
        bool should_calculate_pid = false;
        
        pid_calculation_counter++;
        if (pid_calculation_counter >= pid_calculation_interval) {
            should_calculate_pid = true;
            pid_calculation_counter = 0;
        }
        
        // 深度控制计算
        if (should_calculate_pid && !in_altitude_deadzone && 
            (control_mode == ControlMode::DEPTH_ONLY || control_mode == ControlMode::DOCK_TRACKING)) {
            altitude_output = altitude_pid->compute();
            last_altitude_output = altitude_output;
            
            if (altitude_output > 0) {
                altitude_output += pwm_deadzone;
            } else if (altitude_output < 0) {
                altitude_output -= pwm_deadzone;
            }
        } else if (in_altitude_deadzone) {
            if (!in_deadzone_last_time_altitude) {
                // altitude_pid->resetIntegralOnly();
            }
        }
        
        // 视觉Yaw控制计算
        if (should_calculate_pid && dock_detected && !in_visual_yaw_deadzone && 
            (control_mode == ControlMode::YAW_ONLY || control_mode == ControlMode::DOCK_TRACKING)) {
            // 使用像素误差X作为PID输入
            visual_yaw_output = visual_yaw_pid->compute(pixel_error_x);
            last_visual_yaw_output = visual_yaw_output;
            
            if (visual_yaw_output > 0) {
                visual_yaw_output += pwm_deadzone;
            } else if (visual_yaw_output < 0) {
                visual_yaw_output -= pwm_deadzone;
            }
        } else if (in_visual_yaw_deadzone || !dock_detected) {
            if (!in_deadzone_last_time_visual_yaw) {
                // visual_yaw_pid->resetIntegralOnly();
            }
            if (!dock_detected) {
                visual_yaw_output = 0; // 没有检测到目标时停止yaw控制
            }
        }
        
        in_deadzone_last_time_altitude = in_altitude_deadzone;
        in_deadzone_last_time_visual_yaw = in_visual_yaw_deadzone;
        
        // 根据控制模式分配PWM值
        // CH1: 右推进器, CH2: 左推进器 (视觉Yaw控制+前进)
        // CH3: 前推进器, CH4: 后推进器 (深度控制)
        switch (control_mode) {
            case ControlMode::DEPTH_ONLY:
                // 只控制深度
                target_pwm_ch1 = base_throttle;
                target_pwm_ch2 = base_throttle;
                target_pwm_ch3 = base_throttle + altitude_output;
                target_pwm_ch4 = base_throttle + altitude_output;
                break;
                
            case ControlMode::YAW_ONLY:
                // 只控制视觉Yaw
                target_pwm_ch1 = base_throttle + visual_yaw_output;   // 右推进器
                target_pwm_ch2 = base_throttle - visual_yaw_output;   // 左推进器
                target_pwm_ch3 = base_throttle;
                target_pwm_ch4 = base_throttle;
                break;
                
            case ControlMode::DOCK_TRACKING:
            default:
                // Dock跟踪模式：深度保持+前进+视觉Yaw跟踪
                target_pwm_ch1 = base_throttle + forward_thrust + visual_yaw_output;   // 右推进器: 前进+yaw
                target_pwm_ch2 = base_throttle + forward_thrust - visual_yaw_output;   // 左推进器: 前进+yaw
                target_pwm_ch3 = base_throttle + altitude_output;    // 垂直推进器: 深度控制
                target_pwm_ch4 = base_throttle + altitude_output;    // 垂直推进器: 深度控制
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
        
        double visual_yaw_p = visual_yaw_pid->getLastPTerm();
        double visual_yaw_i = visual_yaw_pid->getLastITerm();
        double visual_yaw_d = visual_yaw_pid->getLastDTerm();
        
        // 执行PWM平滑调整
        smoothPWMTransition();
        
        // 记录调试数据
        logData((int)control_mode, (int)debug_mode, 
                current_altitude, altitude_error, current_velocity_z, current_accel_z,
                current_pixel_x, current_pixel_y, pixel_error_x, pixel_error_y, 
                dock_detected, dock_confidence,
                alt_pos_setpoint, alt_vel_setpoint, alt_acc_setpoint,
                0.0, last_visual_yaw_output, forward_thrust,
                alt_pos_p, alt_pos_i, alt_pos_d,
                alt_vel_p, alt_vel_i, alt_vel_d,
                alt_acc_p, alt_acc_i, alt_acc_d,
                visual_yaw_p, visual_yaw_i, visual_yaw_d,
                alt_pos_out, alt_vel_out, alt_acc_out, last_altitude_output,
                current_pwm_ch1, current_pwm_ch2, current_pwm_ch3, current_pwm_ch4,
                should_calculate_pid);
        
        controlMotors(current_pwm_ch1, current_pwm_ch2, current_pwm_ch3, current_pwm_ch4);
        
        // 调试信息打印
        debug_print_counter++;
        if (debug_print_counter >= 50) {
            debug_print_counter = 0;
            
            printf("\033[2J\033[H"); // 清屏
            printf("\033[1;36m=== ROV Dock跟踪 PID调试信息 ===\033[0m\n");
            
            printf("\033[1;33m控制模式: %d ", (int)control_mode);
            switch(control_mode) {
                case ControlMode::DEPTH_ONLY:
                    printf("(仅深度控制)\033[0m\n");
                    break;
                case ControlMode::YAW_ONLY:
                    printf("(仅视觉Yaw控制)\033[0m\n");
                    break;
                case ControlMode::DOCK_TRACKING:
                    printf("(Dock跟踪模式: 深度+前进+Yaw)\033[0m\n");
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
            
            printf("\033[1;32mDock检测状态:\033[0m\n");
            if (dock_detected) {
                printf("  YOLO检测: \033[1;32mDock已检测到\033[0m\n");
                printf("  像素坐标: (%.1f, %.1f), 目标: (%.1f, %.1f)\n",
                       current_pixel_x, current_pixel_y, target_pixel_x, target_pixel_y);
                printf("  像素误差: X=%.1f, Y=%.1f\n", pixel_error_x, pixel_error_y);
                printf("  检测置信度: %.3f\n", dock_confidence);
                
                // 计算角度误差用于显示
                double angle_error_x = atan(pixel_error_x / camera_fx) * 180.0 / M_PI;
                double angle_error_y = atan(pixel_error_y / camera_fy) * 180.0 / M_PI;
                printf("  角度误差: 水平=%.2f°, 垂直=%.2f°\n", angle_error_x, angle_error_y);
            } else {
                printf("  YOLO检测: \033[1;31mDock未检测到\033[0m\n");
            }
            
            printf("\033[1;34m控制输出:\033[0m\n");
            printf("  前进推力: %.1f\n", forward_thrust);
            printf("  深度控制输出: %.2f\n", last_altitude_output);
            printf("  视觉Yaw输出: %.2f\n", last_visual_yaw_output);
            
            printf("\033[1;34m设定点:\033[0m\n");
            printf("  深度 - 位置: %.3fm, 速度: %.4fm/s, 加速度: %.3fm/s²\n", 
                   alt_pos_setpoint, alt_vel_setpoint, alt_acc_setpoint);
            printf("  视觉Yaw设定点: %.1f (像素误差为0)\n", 0.0);
            
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
                printf("\033[1;35m视觉Yaw PID分量:\033[0m\n");
                printf("  视觉Yaw - P: %6.3f, I: %6.3f, D: %6.3f → 输出: %6.3f\n", 
                       visual_yaw_p, visual_yaw_i, visual_yaw_d, last_visual_yaw_output);
            }
            
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
    
    void runDockTrackingDemo() {
        if (!armVehicle()) {
            ROS_ERROR("飞控解锁失败，终止演示");
            return;
        }
        
        ROS_INFO("=== ROV Dock跟踪演示开始 ===");
        ROS_INFO("固定每%d个周期更新一次PID", pid_calculation_interval);
        ROS_INFO("默认开启Dock跟踪模式(深度保持+前进+视觉Yaw跟踪)");
        ROS_INFO("订阅话题: /yolo_pixel 获取YOLO Dock像素坐标");
        ROS_INFO("你可以通过以下命令实时调整参数:");
        ROS_INFO("深度控制:");
        ROS_INFO("  rosparam set /position_pid/kp 20.0");
        ROS_INFO("  rosparam set /velocity_pid/kp 10.0");
        ROS_INFO("  rosparam set /acceleration_pid/kp 0.8");
        ROS_INFO("视觉Yaw控制:");
        ROS_INFO("  rosparam set /visual_yaw_pid/kp 2.0");
        ROS_INFO("  rosparam set /visual_yaw_pid/ki 0.05");
        ROS_INFO("  rosparam set /visual_yaw_pid/kd 0.8");
        ROS_INFO("前进控制:");
        ROS_INFO("  rosparam set /forward_thrust 120.0");
        ROS_INFO("目标设定:");
        ROS_INFO("  rosparam set /target_altitude 0.8");
        ROS_INFO("控制模式:");
        ROS_INFO("  rosparam set /control_mode 3  # 1=仅深度, 2=仅视觉Yaw, 3=Dock跟踪");
        ROS_INFO("  rosparam set /debug_mode 1    # 1=加速度环, 2=速度+加速度, 3=全三环");
        
        // 从ROS参数获取控制模式
        int ctrl_mode = 3;
        nh.param("control_mode", ctrl_mode, 3);
        setControlMode(ctrl_mode);
        
        int debug_mode = 3;
        nh.param("debug_mode", debug_mode, 3);
        setDebugMode(debug_mode);
        
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
            ROS_INFO("Dock跟踪控制终止");
            
        } catch (ros::Exception& e) {
            ROS_ERROR("发生错误: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "rov_dock_tracking_controller");
    
    try {
        FlightControllerInterface controller;
        controller.runDockTrackingDemo();
    } catch (ros::Exception& e) {
        ROS_ERROR("错误: %s", e.what());
    }
    
    return 0;
}