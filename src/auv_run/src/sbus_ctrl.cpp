#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <QSerialPort>
#include <QThread>
#include <memory>
#include <cmath>

#define SBUS_SCALE_OFFSET 880.0f
#define SBUS_SCALE_FACTOR 0.625f

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
        : values_(), serial_(std::make_shared<QSerialPort>()),
          current_state_(INIT), target_detected_(false),
          servo1_angle_(180.0), servo2_angle_(0.0), // 初始角度位置
          search_direction_(1), search_min_angle_(90.0), search_max_angle_(270.0),
          search_step_(0.3), target_pixel_x_(0), target_pixel_y_(0)  // 降低搜索步长从1.0到0.3
    {
        // 初始化通道默认中位值 1500us
        for (int i = 0; i < 16; i++) {
            values_[i] = 1500;
        }
        
        // 从参数服务器读取P参数
        ros::NodeHandle nh;
        nh.param("kp_x", kp_x_, 0.18);          // 水平P参数
        nh.param("kp_y", kp_y_, 0.25);          // 垂直P参数
        nh.param("max_angle_step", max_angle_step_, 2.5);  // 最大步长
        nh.param("pixel_deadzone_x", pixel_deadzone_x_, 12); // 水平死区
        nh.param("pixel_deadzone_y", pixel_deadzone_y_, 10); // 垂直死区
        
        initSerialPort(port_name);
        
        // 订阅目标检测话题
        // target_sub_ = nh_.subscribe("/person_pixel", 1, 
        target_sub_ = nh_.subscribe("/aruco_pixel", 1, 
                                   &GimbalController::targetCallback, this);
        
        ROS_INFO("Gimbal Controller initialized (Simple P Control)");
        ROS_INFO("P Parameters: kp_x=%.3f, kp_y=%.3f, deadzone=(%dpx,%dpx)", 
                 kp_x_, kp_y_, pixel_deadzone_x_, pixel_deadzone_y_);
    }
    
    void run() {
        ros::Rate rate(140); // 约7ms周期
        
        while (ros::ok()) {
            processStateMachine();
            updateServoValues();
            sendSbusFrame();
            
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    
    int values_[16];
    std::shared_ptr<QSerialPort> serial_;
    
    // 控制状态
    ControlState current_state_;
    bool target_detected_;
    ros::Time last_target_time_;
    ros::Time state_start_time_;
    
    // 舵机控制变量
    double servo1_angle_; // 水平360度位置舵机角度 (0 to 360)
    double servo2_angle_; // 垂直180度舵机角度 (-90 to +90)
    
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
    
    void initSerialPort(const std::string& port_name) {
        serial_->setPortName(QString::fromStdString(port_name));
        serial_->setBaudRate(100000);
        serial_->setDataBits(QSerialPort::Data8);
        serial_->setParity(QSerialPort::EvenParity);
        serial_->setStopBits(QSerialPort::TwoStop);
        serial_->setFlowControl(QSerialPort::NoFlowControl);
        
        if (!serial_->open(QIODevice::ReadWrite)) {
            ROS_ERROR_STREAM("Failed to open serial port: " << port_name);
            ros::shutdown();
        } else {
            ROS_INFO_STREAM("Serial port opened: " << port_name);
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
                    state_start_time_ = current_time;
                }
                break;
                
            case MOVE_SERVO2_TO_UP:
                servo2_angle_ = 60.0; // 垂直向上60度
                
                if ((current_time - state_start_time_).toSec() > 2.0) {
                    ROS_INFO("Starting target search...");
                    current_state_ = SEARCHING;
                    state_start_time_ = current_time;
                    // 重置搜索参数
                    servo1_angle_ = search_min_angle_;
                    search_direction_ = 1;
                }
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
        int servo1_pwm = static_cast<int>(
            SERVO1_MIN_PWM + (servo1_angle_ / 360.0) * 2000);
        servo1_pwm = std::max(SERVO1_MIN_PWM, std::min(SERVO1_MAX_PWM, servo1_pwm));
        values_[0] = servo1_pwm;
        
        // 舵机2 (180度舵机): -90° to +90° 映射到 500-2500us  
        int servo2_pwm = static_cast<int>(
            SERVO2_MIN_PWM + ((servo2_angle_ + 90.0) / 180.0) * 2000);
        servo2_pwm = std::max(SERVO2_MIN_PWM, std::min(SERVO2_MAX_PWM, servo2_pwm));
        values_[1] = servo2_pwm;
    }
    
    void sendSbusFrame() {
        uint8_t oframe[25] = {0};
        oframe[0] = 0x0F;    // 起始字节
        oframe[24] = 0x00;   // 结束字节
        
        uint8_t byteIndex = 1;
        uint8_t offset = 0;
        
        for (int i = 0; i < 16; i++) {
            uint16_t value = static_cast<uint16_t>(
                (values_[i] - SBUS_SCALE_OFFSET) / SBUS_SCALE_FACTOR + 0.5f);
            if (value > 0x07FF) value = 0x07FF;
            
            while (offset >= 8) {
                byteIndex++;
                offset -= 8;
            }
            
            oframe[byteIndex]     |= (value << offset) & 0xFF;
            oframe[byteIndex + 1] |= (value >> (8 - offset)) & 0xFF;
            oframe[byteIndex + 2] |= (value >> (16 - offset)) & 0xFF;
            offset += 11;
        }
        
        serial_->write(reinterpret_cast<const char*>(oframe), 25);
        serial_->waitForBytesWritten(1);
        QThread::msleep(6);  // 补足7ms周期
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gimbal_sbus_controller");
    ros::NodeHandle nh;
    
    // 获取串口参数
    std::string serial_port;
    nh.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    
    ROS_INFO("Starting Gimbal SBUS Controller (Simple P Control) on port: %s", 
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
