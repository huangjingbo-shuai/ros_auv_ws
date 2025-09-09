#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <serial/serial.h>
#include <memory>
#include <cmath>

// 舵机参数定义
#define SERVO1_MIN_PWM 500    // 360度位置舵机最小PWM (0度)
#define SERVO1_MAX_PWM 2500   // 360度位置舵机最大PWM (360度)
#define SERVO1_CENTER_PWM 1500 // 360度位置舵机中位PWM (180度)

#define SERVO2_MIN_PWM 500    // 180度舵机最小PWM (-90度)
#define SERVO2_MAX_PWM 2500   // 180度舵机最大PWM (+90度)
#define SERVO2_CENTER_PWM 1500 // 180度舵机中位PWM (0度)

// 相机参数定义（基于实际标定参数）
#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480
#define CAMERA_CENTER_X 318.5   // 来自标定参数 cx
#define CAMERA_CENTER_Y 247.7   // 来自标定参数 cy

// 控制状态枚举
enum ControlState {
    INIT,           // 初始化状态
    RESET_TO_CENTER, // 回到中位
    MOVE_SERVO2_TO_UP, // 舵机2转到向上
    SEARCHING,      // 搜索目标
    TRACKING        // 跟踪目标
};

class GimbalController {
public:
    GimbalController(const std::string& port_name)
        : serial_(std::make_shared<serial::Serial>()),
          current_state_(INIT), target_detected_(false),
          servo1_angle_(180.0), servo2_angle_(0.0), // 初始角度位置
          search_direction_(1), search_min_angle_(0.0), search_max_angle_(360.0),
          search_step_(0.3), target_pixel_x_(0), target_pixel_y_(0),  // 降低搜索步长从1.0到0.3
          log_counter_(0)  // 初始化日志计数器
    {
        // 从参数服务器读取P参数
        ros::NodeHandle nh;
        nh.param("kp_x", kp_x_, 0.18);          // 水平P参数
        nh.param("kp_y", kp_y_, 0.25);          // 垂直P参数
        nh.param("max_angle_step", max_angle_step_, 2.5);  // 最大步长
        nh.param("pixel_deadzone_x", pixel_deadzone_x_, 12); // 水平死区
        nh.param("pixel_deadzone_y", pixel_deadzone_y_, 10); // 垂直死区
        
        initSerialPort(port_name);
        
        // 订阅目标检测话题
        target_sub_ = nh_.subscribe("/aruco_pixel", 1, 
                                   &GimbalController::targetCallback, this);
        
        ROS_INFO("Gimbal Controller initialized (Arduino Serial Communication)");
        ROS_INFO("P Parameters: kp_x=%.3f, kp_y=%.3f, deadzone=(%dpx,%dpx)", 
                 kp_x_, kp_y_, pixel_deadzone_x_, pixel_deadzone_y_);
    }
    
    void run() {
        ros::Rate rate(50); // 从140Hz降低到50Hz (20ms周期)
        
        while (ros::ok()) {
            processStateMachine();
            updateServoValues();
            sendServoCommands();
            
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    
    std::shared_ptr<serial::Serial> serial_;
    
    // 控制状态
    ControlState current_state_;
    bool target_detected_;
    ros::Time last_target_time_;
    ros::Time state_start_time_;
    
    // 舵机控制变量
    double servo1_angle_; // 水平360度位置舵机角度 (0 to 360)
    double servo2_angle_; // 垂直180度舵机角度 (-90 to +90)
    int servo1_pwm_, servo2_pwm_; // PWM值
    
    // 搜索参数
    int search_direction_; // 1 或 -1
    double search_min_angle_, search_max_angle_;
    double search_step_;
    
    // 目标像素坐标
    int target_pixel_x_, target_pixel_y_;
    
    // 比例控制参数（P参数）
    double kp_x_, kp_y_;           // 比例系数
    double max_angle_step_;        // 最大角度步长
    int pixel_deadzone_x_, pixel_deadzone_y_; // 像素死区
    
    // PWM记录相关
    int log_counter_;  // 日志计数器，用于控制输出频率
    
    void initSerialPort(const std::string& port_name) {
        try {
            serial_->setPort(port_name);
            serial_->setBaudrate(115200);  // Arduino标准波特率
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_->setTimeout(timeout);
            serial_->open();
            
            if (serial_->isOpen()) {
                ROS_INFO_STREAM("Serial port opened: " << port_name);
                // Arduino重启需要时间，等待更长时间
                ROS_INFO("Waiting for Arduino to initialize...");
                ros::Duration(3.0).sleep();  // 增加到3秒
                
                // 读取并清空Arduino启动信息
                try {
                    if (serial_->available()) {
                        std::string startup_msg = serial_->read(serial_->available());
                        ROS_INFO_STREAM("Arduino startup message: " << startup_msg);
                    }
                } catch (const std::exception& e) {
                    ROS_WARN_STREAM("Could not read startup message: " << e.what());
                }
                
                // 发送一个测试命令而不是INIT
                serial_->write("1500,1500\n");  // 直接发送中位命令
                serial_->flush();
                
                ROS_INFO("Arduino initialization complete");
            } else {
                ROS_ERROR_STREAM("Failed to open serial port: " << port_name);
                ros::shutdown();
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Serial port exception: " << e.what());
            ros::shutdown();
        }
    }
    
    void targetCallback(const geometry_msgs::Point::ConstPtr& msg) {
        // 接收目标在图像中的像素坐标
        target_pixel_x_ = static_cast<int>(msg->x);
        target_pixel_y_ = static_cast<int>(msg->y);
        target_detected_ = true;
        last_target_time_ = ros::Time::now();
    }
    
    void processStateMachine() {
        ros::Time current_time = ros::Time::now();
        
        // 检查目标丢失 (超过1秒没有收到目标信息)
        if (target_detected_ && 
            (current_time - last_target_time_).toSec() > 1.0) {
            target_detected_ = false;
            ROS_WARN("Target lost, switching to search mode");
            current_state_ = SEARCHING;
            state_start_time_ = current_time;
        }
        
        // 添加状态机调试信息
        static int debug_counter = 0;
        debug_counter++;
        if (debug_counter % 50 == 0) {  // 每50次循环(约350ms)输出一次
            double elapsed = (current_time - state_start_time_).toSec();
            ROS_INFO("State: %s, Elapsed: %.2fs", getStateString(current_state_).c_str(), elapsed);
        }
        
        switch (current_state_) {
            case INIT:
                ROS_INFO("Initializing gimbal...");
                current_state_ = RESET_TO_CENTER;
                state_start_time_ = current_time;
                break;
                
            case RESET_TO_CENTER:
                servo1_angle_ = 180.0; // 水平舵机中位 (180度)
                servo2_angle_ = 0.0;   // 垂直舵机中位 (0度)
                
                if ((current_time - state_start_time_).toSec() > 2.0) {
                    ROS_INFO("Moving servo2 to up position...");
                    current_state_ = MOVE_SERVO2_TO_UP;
                    state_start_time_ = current_time;  // 重要：重置时间
                }
                break;
                
            case MOVE_SERVO2_TO_UP:
                servo2_angle_ = 60.0; // 垂直向上60度
                
                // 立即切换，不等待 - 先确保状态机工作
                ROS_INFO("Servo2 moved to 60 degrees, starting search immediately...");
                current_state_ = SEARCHING;
                state_start_time_ = current_time;
                // 重置搜索参数
                servo1_angle_ = search_min_angle_;
                search_direction_ = 1;
                break;
                
            case SEARCHING:
                if (target_detected_) {
                    ROS_INFO("Target found! Switching to tracking mode.");
                    current_state_ = TRACKING;
                    state_start_time_ = current_time;
                } else {
                    // 搜索模式：在指定角度范围内来回扫描
                    performSearch();
                }
                break;
                
            case TRACKING:
                if (!target_detected_) {
                    ROS_WARN("Target lost during tracking");
                    current_state_ = SEARCHING;
                    state_start_time_ = current_time;
                    // 重置搜索到当前位置附近
                    search_min_angle_ = std::max(0.0, servo1_angle_ - 60.0);
                    search_max_angle_ = std::min(360.0, servo1_angle_ + 60.0);
                    search_direction_ = 1;
                } else {
                    // 简单的比例控制
                    trackTargetProportional();
                }
                break;
        }
    }
    
    void performSearch() {
        // 按步长移动舵机
        servo1_angle_ += search_direction_ * search_step_;
        
        // 检查是否到达边界，如果是则改变方向
        if (servo1_angle_ >= search_max_angle_) {
            servo1_angle_ = search_max_angle_;
            search_direction_ = -1;
        } else if (servo1_angle_ <= search_min_angle_) {
            servo1_angle_ = search_min_angle_;
            search_direction_ = 1;
        }
    }
    
    void trackTargetProportional() {
        // 计算像素误差（相对于相机光心）
        double error_x = target_pixel_x_ - CAMERA_CENTER_X; // 正值表示目标在右侧
        double error_y = target_pixel_y_ - CAMERA_CENTER_Y; // 正值表示目标在下方
        
        // 水平方向P控制 - 修复方向反转问题
        if (std::abs(error_x) > pixel_deadzone_x_) {
            // 根据舵机2角度决定水平控制方向
            // 当舵机2角度小于0度时，相机视角倒转，需要反向控制
            double direction_factor = (servo2_angle_ < 0.0) ? 1.0 : -1.0;
            
            // P控制：角度调整量 = 方向系数 × Kp × 像素误差
            double angle_adjustment_x = direction_factor * kp_x_ * error_x;
            
            // 限制最大角度步长，防止过冲
            angle_adjustment_x = std::max(-max_angle_step_, 
                                        std::min(max_angle_step_, angle_adjustment_x));
            
            // 更新水平舵机角度
            servo1_angle_ += angle_adjustment_x;
            
            // 确保角度在有效范围内 (0-360度)
            while (servo1_angle_ >= 360.0) servo1_angle_ -= 360.0;
            while (servo1_angle_ < 0.0) servo1_angle_ += 360.0;
            
            ROS_INFO("水平控制: 误差=%.1f, 调整=%.2f°, 当前角度=%.1f°, servo2_angle=%.1f°", 
                     error_x, angle_adjustment_x, servo1_angle_, servo2_angle_);
        }
        
        // 垂直方向P控制
        if (std::abs(error_y) > pixel_deadzone_y_) {
            // P控制：角度调整量 = -Kp × 像素误差 （加负号反转方向）
            double angle_adjustment_y = -kp_y_ * error_y;
            
            // 限制最大角度步长
            angle_adjustment_y = std::max(-max_angle_step_, 
                                        std::min(max_angle_step_, angle_adjustment_y));
            
            // 更新垂直舵机角度
            servo2_angle_ += angle_adjustment_y;
            servo2_angle_ = std::max(-90.0, std::min(90.0, servo2_angle_));
            
            ROS_INFO("垂直控制: 误差=%.1f, 调整=%.2f°, 当前角度=%.1f°", 
                     error_y, angle_adjustment_y, servo2_angle_);
        }
    }
    
    void updateServoValues() {
        // 舵机1 (360度位置舵机): 0° to 360° 映射到 500-2500us
        servo1_pwm_ = static_cast<int>(
            SERVO1_MIN_PWM + (servo1_angle_ / 360.0) * 2000);
        servo1_pwm_ = std::max(SERVO1_MIN_PWM, std::min(SERVO1_MAX_PWM, servo1_pwm_));
        
        // 舵机2 (180度舵机): -90° to +90° 映射到 500-2500us  
        servo2_pwm_ = static_cast<int>(
            SERVO2_MIN_PWM + ((servo2_angle_ + 90.0) / 180.0) * 2000);
        servo2_pwm_ = std::max(SERVO2_MIN_PWM, std::min(SERVO2_MAX_PWM, servo2_pwm_));
        
        // PWM值记录和输出 - 降低频率
        log_counter_++;
        if (log_counter_ >= 50) {  // 每50次循环输出一次（约1秒间隔）
            ROS_INFO("=== PWM值记录 ===");
            ROS_INFO("舵机1: 角度=%.2f°, PWM=%dus", servo1_angle_, servo1_pwm_);
            ROS_INFO("舵机2: 角度=%.2f°, PWM=%dus", servo2_angle_, servo2_pwm_);
            ROS_INFO("状态: %s", getStateString(current_state_).c_str());
            if (target_detected_) {
                ROS_INFO("目标像素: (%d, %d)", target_pixel_x_, target_pixel_y_);
            }
            ROS_INFO("================");
            log_counter_ = 0;
        }
    }
    
    // 获取状态字符串的辅助函数
    std::string getStateString(ControlState state) {
        switch (state) {
            case INIT: return "初始化";
            case RESET_TO_CENTER: return "回到中位";
            case MOVE_SERVO2_TO_UP: return "舵机2上移";
            case SEARCHING: return "搜索目标";
            case TRACKING: return "跟踪目标";
            default: return "未知状态";
        }
    }
    
    void sendServoCommands() {
        if (!serial_->isOpen()) {
            ROS_ERROR("Serial port is not open!");
            return;
        }
        
        try {
            // 发送PWM命令格式: PWM1,PWM2\n
            std::string command = std::to_string(servo1_pwm_) + "," + 
                                 std::to_string(servo2_pwm_) + "\n";
            
            serial_->write(command);
            serial_->flush();
            
            // 大幅降低调试信息输出频率
            static int serial_log_counter = 0;
            serial_log_counter++;
            if (serial_log_counter >= 50) {  // 每50次记录一次（约1秒间隔）
                ROS_INFO(">>> 发送命令: %s", command.c_str());
                ROS_INFO(">>> PWM计算: servo1=%d, servo2=%d", servo1_pwm_, servo2_pwm_);
                ROS_INFO(">>> 角度: servo1=%.1f°, servo2=%.1f°", servo1_angle_, servo2_angle_);
                serial_log_counter = 0;
            }
            
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Serial write error: " << e.what());
        }
    }
};

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "gimbal_arduino_controller");
    ros::NodeHandle nh;
    
    // 获取串口参数
    std::string serial_port;
    nh.param<std::string>("serial_port", serial_port, "/dev/ttyACM0");
    
    ROS_INFO("Starting Gimbal Arduino Controller on port: %s", 
             serial_port.c_str());
    
    try {
        GimbalController controller(serial_port);
        controller.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in gimbal controller: %s", e.what());
        return -1;
    }
    
    return 0;
}