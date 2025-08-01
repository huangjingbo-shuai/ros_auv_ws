#include "attitude_control_node_APM.h"
#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>

AttitudeControlNode::AttitudeControlNode(ros::NodeHandle& nh) : nh_(nh) {
    loadParameters();
    setupPublishersSubscribers();
    initializeDataLogging();

    // 设置动态参数回调
    config_callback_ = boost::bind(&AttitudeControlNode::configCallback, this, _1, _2);
    config_server_.setCallback(config_callback_);
    
    last_dock_time_ = ros::Time::now();
    last_boat_time_ = ros::Time::now();
    
    // 初始化PWM平滑控制变量
    current_yaw_pwm_ = 1500;  // 中心值
    current_throttle_pwm_ = 1500;  // 中心值
    target_yaw_pwm_ = 1500;
    target_throttle_pwm_ = 1500;
    
    ROS_INFO("Attitude Control Node initialized");

    ROS_INFO("Initial PWM values - yaw: %d, throttle: %d", current_yaw_pwm_, current_throttle_pwm_);
    ROS_INFO("PWM limits - min: %d, max: %d, center: %d", pwm_min_, pwm_max_, pwm_center_);
}

AttitudeControlNode::~AttitudeControlNode() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

// 动态参数回调函数
void AttitudeControlNode::configCallback(usv_run::AttitudeControlConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: attitude_angle=%.2f, flag_control=%d, use_3d_angle=%s, kp_boat_angle=%.2f", 
             config.attitude_angle, config.flag_control, 
             config.use_3d_angle ? "True" : "False", config.kp_boat_angle);
    
    // 更新基本控制参数
    control_params_.attitude_angle = config.attitude_angle;
    control_params_.flag_control = config.flag_control;
    control_params_.use_3d_angle = config.use_3d_angle;
    control_params_.kp_boat_angle = config.kp_boat_angle;
    control_params_.default_throttle_output = config.default_throttle_output;
    
    // 动态更新YAW PID参数（包含所有参数）
    PIDParams yaw_params;
    yaw_params.kp = config.yaw_kp;
    yaw_params.ki = config.yaw_ki;
    yaw_params.kd = config.yaw_kd;
    
    yaw_pid_.updateParams(yaw_params);
    
    ROS_INFO("Updated YAW PID: Kp=%.3f, Ki=%.6f, Kd=%.6f", 
             config.yaw_kp, config.yaw_ki, config.yaw_kd);
    
    // 动态更新Throttle PID参数（包含所有参数）
    PIDParams throttle_params;
    throttle_params.kp = config.throttle_kp;
    throttle_params.ki = config.throttle_ki;
    throttle_params.kd = config.throttle_kd;
    
    throttle_pid_.updateParams(throttle_params);
    
    ROS_INFO("Updated Throttle PID: Kp=%.3f, Ki=%.6f, Kd=%.6f", 
             config.throttle_kp, config.throttle_ki, config.throttle_kd);
    
    // 如果是姿态控制模式，立即更新期望航向
    if (control_params_.flag_control == 0) {
        current_yaw_record_flag_ = true;
        ros::Duration(0.5).sleep();
        updateDesiredYaw();
        ROS_INFO("Updated attitude angle to %.2f degrees", config.attitude_angle);
    }
}

void AttitudeControlNode::loadParameters() {
    // Load control parameters
    nh_.param("control/kp_boat_angle", control_params_.kp_boat_angle, 1.0);
    nh_.param("control/attitude_angle", control_params_.attitude_angle, 0.0);
    nh_.param("control/use_3d_angle", control_params_.use_3d_angle, true);
    nh_.param("control/flag_control", control_params_.flag_control, 0);
    nh_.param("control/csv_path", control_params_.csv_path, std::string("boat_data"));
    nh_.param("default_throttle_output", control_params_.default_throttle_output, 0.0);
    
    // Load PWM smooth control parameters
    nh_.param("pwm/smooth_step", pwm_smooth_step_, 10);  // 每次调整的PWM步长
    nh_.param("pwm/min_value", pwm_min_, 1100);
    nh_.param("pwm/max_value", pwm_max_, 1900);
    nh_.param("pwm/center_value", pwm_center_, 1500);
    
    // Load timeout configuration
    nh_.param("timeout/dock_timeout", timeout_config_.dock_timeout, 5.0);
    nh_.param("timeout/boat_timeout", timeout_config_.boat_timeout, 2.0);
    
    // Load PID parameters for yaw control
    PIDParams yaw_params;
    nh_.param("yaw/kp", yaw_params.kp, 1.5f);
    nh_.param("yaw/ki", yaw_params.ki, 0.001f);
    nh_.param("yaw/kd", yaw_params.kd, 0.015f);
    nh_.param("yaw/output_limit_min", yaw_params.output_limit_min, -1.0f);
    nh_.param("yaw/output_limit_max", yaw_params.output_limit_max, 1.0f);
    nh_.param("yaw/integral_limit", yaw_params.integral_limit, 2.0f);
    yaw_pid_.updateParams(yaw_params);
    
    // Load PID parameters for throttle control
    PIDParams throttle_params;
    nh_.param("throttle/kp", throttle_params.kp, 1.0f);
    nh_.param("throttle/ki", throttle_params.ki, 0.0f);
    nh_.param("throttle/kd", throttle_params.kd, 0.0f);
    nh_.param("throttle/output_limit_min", throttle_params.output_limit_min, 0.0f);
    nh_.param("throttle/output_limit_max", throttle_params.output_limit_max, 1.0f);
    nh_.param("throttle/integral_limit", throttle_params.integral_limit, 2.0f);
    throttle_pid_.updateParams(throttle_params);
    
    ROS_INFO("Parameters loaded - Control mode: %d, Use 3D angle: %s", 
             control_params_.flag_control, control_params_.use_3d_angle ? "true" : "false");
}

void AttitudeControlNode::setupPublishersSubscribers() {
    // Subscribers
    state_sub_ = nh_.subscribe("/mavros/state", 10, &AttitudeControlNode::stateCallback, this);
    pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &AttitudeControlNode::poseCallback, this);
    global_sub_ = nh_.subscribe("/mavros/global_position/global", 10, &AttitudeControlNode::globalCallback, this);
    velocity_sub_ = nh_.subscribe("/mavros/local_position/velocity_body", 10, &AttitudeControlNode::velocityCallback, this);
    dock_angle_3d_sub_ = nh_.subscribe("/yolo/dock_angle_3D", 10, &AttitudeControlNode::dockAngle3DCallback, this);
    dock_angle_2d_sub_ = nh_.subscribe("/yolo/dock_angle_2D", 10, &AttitudeControlNode::dockAngle2DCallback, this);
    boat_angle_sub_ = nh_.subscribe("/yolo/boat_angle", 10, &AttitudeControlNode::boatAngleCallback, this);
    
    // Publishers - 修改为RC override
    rc_pub_ = nh_.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
    
    // Service clients
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    
    // Timers
    dock_timer_ = nh_.createTimer(ros::Duration(1.0), &AttitudeControlNode::checkDockTimeout, this);
    boat_timer_ = nh_.createTimer(ros::Duration(1.0), &AttitudeControlNode::checkBoatTimeout, this);
}

void AttitudeControlNode::initializeDataLogging() {
    std::string log_path = "/home/shuai/ros_boat_ws/" + control_params_.csv_path + ".csv";
    log_file_.open(log_path, std::ios::app);
    
    if (log_file_.is_open()) {
        log_file_ << "time,latitude,longitude,position_x,position_y,position_z,"
                  << "roll_angle,pitch_angle,yaw_angle,v_body_x,"
                  << "dock_angle,dock_angle_2D,dock_angle_3D,boat_angle,"
                  << "current_yaw,desired_yaw,yaw_output,throttle_output,"
                  << "yaw_pwm,throttle_pwm\n";
    } else {
        ROS_ERROR("Failed to open CSV file for logging: %s", log_path.c_str());
    }
}

void AttitudeControlNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

void AttitudeControlNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    position_x_ = msg->pose.position.x;
    position_y_ = msg->pose.position.y;
    position_z_ = msg->pose.position.z;
    
    double roll, pitch, yaw;
    quaternionToEuler(msg->pose.orientation, roll, pitch, yaw);
    
    if (yaw < 0) yaw += 2 * M_PI;
    current_yaw_ = yaw;
    
    roll_angle_ = roll * 180.0 / M_PI;
    pitch_angle_ = pitch * 180.0 / M_PI;
    yaw_angle_ = yaw * 180.0 / M_PI;

    if (control_params_.flag_control == 0 && current_yaw_record_flag_ == true){
        current_yaw_record_ = yaw;
        current_yaw_record_flag_ = false;
    }
}

void AttitudeControlNode::globalCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    latitude_ = msg->latitude;
    longitude_ = msg->longitude;
}

void AttitudeControlNode::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    v_body_x_ = msg->twist.linear.x;
}

void AttitudeControlNode::dockAngle3DCallback(const std_msgs::Float64::ConstPtr& msg) {
    dock_angle_3d_ = msg->data;
    last_dock_time_ = ros::Time::now();
    
    if (control_params_.use_3d_angle) {
        dock_angle_ = dock_angle_3d_;
    }
}

void AttitudeControlNode::dockAngle2DCallback(const std_msgs::Float64::ConstPtr& msg) {
    dock_angle_2d_ = msg->data;
    last_dock_time_ = ros::Time::now();
    
    if (!control_params_.use_3d_angle) {
        dock_angle_ = dock_angle_2d_;
    }
}

void AttitudeControlNode::boatAngleCallback(const std_msgs::Float64::ConstPtr& msg) {
    boat_angle_ = msg->data;
    last_boat_time_ = ros::Time::now();
}

void AttitudeControlNode::checkDockTimeout(const ros::TimerEvent&) {
    if ((ros::Time::now() - last_dock_time_).toSec() > timeout_config_.dock_timeout) {
        dock_angle_ = -1;
        ROS_WARN_THROTTLE(5, "Dock angle timeout - no signal received");
    }
}

void AttitudeControlNode::checkBoatTimeout(const ros::TimerEvent&) {
    if ((ros::Time::now() - last_boat_time_).toSec() > timeout_config_.boat_timeout) {
        boat_angle_ = 0;
        ROS_WARN_THROTTLE(5, "Boat angle timeout - setting to 0");
    }
}

void AttitudeControlNode::updateDesiredYaw() {
    if (control_params_.flag_control == 1 && dock_angle_ != -1) {
        // Vision control mode
        double dock_angle_filtered = (std::abs(dock_angle_) < 2) ? 0 : dock_angle_;
        double boat_angle_filtered = (std::abs(boat_angle_) < 2) ? 0 : boat_angle_;
        
        desired_yaw_ = -1 * (dock_angle_filtered + control_params_.kp_boat_angle * boat_angle_filtered) 
                      * M_PI / 180.0 + current_yaw_;
    } else if (control_params_.flag_control == 0) {
        // Attitude control mode
        desired_yaw_ = control_params_.attitude_angle * M_PI / 180.0 + current_yaw_record_;
    }
}

// 新增：将PID输出(-1到1)映射到PWM值(1100到1900)
int AttitudeControlNode::mapToPWM(double pid_output) {
    // 限制输入范围到-1到1
    pid_output = std::max(-1.0, std::min(1.0, pid_output));
    
    // 线性映射：-1对应1100，1对应1900，0对应1500
    int pwm_value = static_cast<int>(pwm_center_ + pid_output * (pwm_max_ - pwm_center_));

    ROS_DEBUG("mapToPWM: input=%.3f, output=%d", pid_output, pwm_value);
    
    // 确保PWM值在有效范围内
    return std::max(pwm_min_, std::min(pwm_max_, pwm_value));

    ROS_DEBUG("mapToPWM: input=%.3f, output=%d", pid_output, pwm_value);
}

// 新增：PWM平滑控制函数
int AttitudeControlNode::smoothPWM(int current_pwm, int target_pwm) {
    int diff = target_pwm - current_pwm;
    
    // 如果差值在步长范围内，直接返回目标值
    if (std::abs(diff) <= pwm_smooth_step_) {
        return target_pwm;
    }
    
    // 否则按步长逐步调整
    if (diff > 0) {
        return current_pwm + pwm_smooth_step_;
    } else {
        return current_pwm - pwm_smooth_step_;
    }
}

// 修改后的sendActuatorControl函数，现在使用RC override
void AttitudeControlNode::sendActuatorControl() {
    ROS_INFO("=== Control Debug Info ===");
    ROS_INFO("dock_angle_: %.2f, flag_control: %d", dock_angle_, control_params_.flag_control);
    ROS_INFO("current_yaw_: %.2f deg, desired_yaw_: %.2f deg", 
            current_yaw_ * 180.0 / M_PI, desired_yaw_ * 180.0 / M_PI);
            
    mavros_msgs::OverrideRCIn rc_msg;
    
    // 初始化所有通道为65535（不改变）
    std::fill(rc_msg.channels.begin(), rc_msg.channels.end(), 65535);

    // 检测角度跳跃并调整desired_yaw_
    detectAndCompensateYawJump();
    
    if (dock_angle_ != -1 || control_params_.flag_control == 0) {
        // Normal control
        double dt = 0.01;
        yaw_output_ = -yaw_pid_.calculate(current_yaw_, desired_yaw_, dt);
        throttle_output_ = control_params_.default_throttle_output;
        
        // 将PID输出映射到PWM值
        target_yaw_pwm_ = mapToPWM(yaw_output_);
        target_throttle_pwm_ = mapToPWM(throttle_output_);

        ROS_INFO("PID Output - yaw_output_: %.3f, throttle_output_: %.3f", yaw_output_, throttle_output_);
        ROS_INFO("PWM Target - yaw: %d, throttle: %d", target_yaw_pwm_, target_throttle_pwm_);
        
    } else {
        // Emergency/lost signal mode
        yaw_output_ = -1.0;
        throttle_output_ = 0.0;
        
        // 紧急模式：yaw设为最小值，throttle设为中心值（停止）
        target_yaw_pwm_ = pwm_min_;
        target_throttle_pwm_ = pwm_center_;
    }
    
    // 平滑PWM控制
    current_yaw_pwm_ = smoothPWM(current_yaw_pwm_, target_yaw_pwm_);
    current_throttle_pwm_ = smoothPWM(current_throttle_pwm_, target_throttle_pwm_);
    ROS_INFO("PWM Current - yaw: %d, throttle: %d", current_yaw_pwm_, current_throttle_pwm_);
    
    // 设置RC通道
    rc_msg.channels[3] = current_yaw_pwm_;      // 通道4：左右yaw控制
    rc_msg.channels[2] = current_throttle_pwm_; // 通道3：前后油门控制
    
    ROS_INFO("Publishing RC - Channel[2]: %d, Channel[3]: %d", 
        rc_msg.channels[2], rc_msg.channels[3]);
    ROS_INFO("RC Publisher valid: %s, subscribers: %d", 
        rc_pub_ ? "true" : "false", rc_pub_.getNumSubscribers());
            
    // 发布RC override消息
    rc_pub_.publish(rc_msg);

    // 更新previous_yaw_为当前值
    previous_yaw_ = current_yaw_;
    first_yaw_update_ = false;
    
    // 调试输出
    ROS_DEBUG("PWM - Yaw: %d (target: %d), Throttle: %d (target: %d)", 
              current_yaw_pwm_, target_yaw_pwm_, current_throttle_pwm_, target_throttle_pwm_);
}

void AttitudeControlNode::detectAndCompensateYawJump() {
    // 如果是第一次更新，不进行跳跃检测
    if (first_yaw_update_) {
        return;
    }
    
    // 将弧度转换为度数进行检测（便于理解和调试）
    double current_yaw_deg = current_yaw_ * 180.0 / M_PI;
    double previous_yaw_deg = previous_yaw_ * 180.0 / M_PI;
    
    // 计算角度差值
    double yaw_diff = current_yaw_deg - previous_yaw_deg;
    
    // 设置跳跃检测阈值（度数）
    const double JUMP_THRESHOLD = 180.0;  // 如果角度变化超过180度，认为发生了跳跃
    
    // 检测从360度跳跃到0度的情况（实际上是从接近360跳到接近0）
    if (yaw_diff < -JUMP_THRESHOLD) {
        // current_yaw从360附近跳跃到0附近，desired_yaw减去360度
        desired_yaw_ -= 2 * M_PI;  // 减去2π弧度（等于360度）
        ROS_INFO("Detected yaw jump from ~360 to ~0, adjusted desired_yaw by -360 degrees");
        ROS_INFO("Previous yaw: %.2f deg, Current yaw: %.2f deg, Desired yaw adjusted to: %.2f deg", 
                 previous_yaw_deg, current_yaw_deg, desired_yaw_ * 180.0 / M_PI);
    }
    // 检测从0度跳跃到360度的情况（实际上是从接近0跳到接近360）
    else if (yaw_diff > JUMP_THRESHOLD) {
        // current_yaw从0附近跳跃到360附近，desired_yaw加上360度
        desired_yaw_ += 2 * M_PI;  // 加上2π弧度（等于360度）
        ROS_INFO("Detected yaw jump from ~0 to ~360, adjusted desired_yaw by +360 degrees");
        ROS_INFO("Previous yaw: %.2f deg, Current yaw: %.2f deg, Desired yaw adjusted to: %.2f deg", 
                 previous_yaw_deg, current_yaw_deg, desired_yaw_ * 180.0 / M_PI);
    }
}

void AttitudeControlNode::setOffboardMode() {
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = "OFFBOARD";
    
    if (set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent) {
        ROS_INFO("Offboard mode set successfully");
    } else {
        ROS_ERROR("Failed to set Offboard mode");
    }
}

void AttitudeControlNode::armDrone() {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed successfully");
    } else {
        ROS_ERROR("Failed to arm vehicle");
    }
}

void AttitudeControlNode::quaternionToEuler(const geometry_msgs::Quaternion& q, 
                                           double& roll, double& pitch, double& yaw) {
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);
    else
        pitch = asin(sinp);

    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw = atan2(siny_cosp, cosy_cosp);
}

void AttitudeControlNode::logData() {
    if (!log_file_.is_open()) return;
    
    std::string timestamp = getCurrentTimeString();
    
    log_file_ << timestamp << ","
              << latitude_ << "," << longitude_ << ","
              << position_x_ << "," << position_y_ << "," << position_z_ << ","
              << roll_angle_ << "," << pitch_angle_ << "," << yaw_angle_ << ","
              << v_body_x_ << ","
              << dock_angle_ << "," << dock_angle_2d_ << "," << dock_angle_3d_ << ","
              << boat_angle_ << ","
              << (current_yaw_ * 180.0 / M_PI) << ","
              << (desired_yaw_ * 180.0 / M_PI) << ","
              << yaw_output_ << "," << throttle_output_ << ","
              << current_yaw_pwm_ << "," << current_throttle_pwm_ << "\n";
}

std::string AttitudeControlNode::getCurrentTimeString() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    std::time_t time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&time_t);
    
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d=%H:%M:%S") 
        << "." << std::setw(3) << std::setfill('0') << ms.count();
    
    return oss.str();
}

void AttitudeControlNode::run() {
    ROS_INFO("=== Starting run() function ===");
    ros::Rate rate(20.0);
    
    ROS_INFO("Waiting for connection...");
    // Wait for connection
    while (ros::ok() && !current_state_.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connection established! Starting main operations...");
    
    // // Send initial commands
    // for (int i = 100; ros::ok() && i > 0; --i) {
    //     sendActuatorControl();
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    
    // Set mode and arm
    // setOffboardMode();
    ROS_INFO("About to call armDrone()");
    armDrone();
    ROS_INFO("armDrone() completed, entering main loop");
    
    ROS_INFO("Entering main control loop");
    // Main control loop
    while (ros::ok()) {
        if (update_counter_ % 10 == 0) {
            updateDesiredYaw();
            logData();
            update_counter_ = 0;
        }
        
        ROS_INFO_THROTTLE(1, "About to call sendActuatorControl()");
        sendActuatorControl();
        ROS_INFO_THROTTLE(1, "sendActuatorControl() completed");
        
        update_counter_++;

        if (update_counter_ % 50 == 0) {  // 每2.5秒打印一次
            ROS_INFO("Vehicle State - Connected: %s, Armed: %s, Mode: %s", 
                     current_state_.connected ? "true" : "false",
                     current_state_.armed ? "true" : "false", 
                     current_state_.mode.c_str());
        }

        ros::spinOnce();
        rate.sleep();
    }
}