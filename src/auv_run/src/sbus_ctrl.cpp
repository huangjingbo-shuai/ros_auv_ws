#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <QSerialPort>
#include <QThread>
#include <memory>
#include <cmath>

// CRSF协议参数定义
#define CRSF_SYNC_BYTE 0xC8
#define CRSF_FRAME_SIZE 26
#define CRSF_PAYLOAD_SIZE 22
#define CRSF_TYPE_RC_CHANNELS 0x16
#define CRSF_MIN_VALUE 0      // 对应500us
#define CRSF_MAX_VALUE 2047     // 对应2500us

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
          search_direction_(1), search_min_angle_(0.0), search_max_angle_(360.0),
          search_step_(0.3), target_pixel_x_(0), target_pixel_y_(0),  // 降低搜索步长从1.0到0.3
          log_counter_(0)  // 初始化日志计数器
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
            sendCrsfFrame();
            
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
    
    // PWM记录相关
    int log_counter_;  // 日志计数器，用于控制输出频率
    
    void initSerialPort(const std::string& port_name) {
        serial_->setPortName(QString::fromStdString(port_name));
        serial_->setBaudRate(420000);  // CRSF协议使用420000波特率
        serial_->setDataBits(QSerialPort::Data8);
        serial_->setParity(QSerialPort::NoParity);  // CRSF使用无校验
        serial_->setStopBits(QSerialPort::OneStop);  // CRSF使用1个停止位
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
        
        // PWM值记录和输出 - 每20次循环输出一次（约140ms间隔）
        log_counter_++;
        if (log_counter_ >= 20) {
            ROS_INFO("=== PWM值记录 ===");
            ROS_INFO("舵机1: 角度=%.2f°, PWM=%dus", servo1_angle_, servo1_pwm);
            ROS_INFO("舵机2: 角度=%.2f°, PWM=%dus", servo2_angle_, servo2_pwm);
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
    
    // CRSF CRC计算函数
    uint8_t calculateCrsfCrc(const uint8_t* data, uint8_t length) {
        static const uint8_t crc8_tab[256] = {
            0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
            0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
            0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
            0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
            0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
            0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
            0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
            0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
            0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
            0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
            0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
            0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
            0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
            0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
            0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
            0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
        };
        
        uint8_t crc = 0;
        for (uint8_t i = 0; i < length; i++) {
            crc = crc8_tab[crc ^ data[i]];
        }
        return crc;
    }
    
    void sendCrsfFrame() {
        uint8_t frame[CRSF_FRAME_SIZE];
        
        // CRSF帧头
        frame[0] = CRSF_SYNC_BYTE;           // 同步字节 0xC8
        frame[1] = CRSF_PAYLOAD_SIZE + 2;    // 长度 (payload + type + crc)
        frame[2] = CRSF_TYPE_RC_CHANNELS;    // 类型 0x16
        
        // 将PWM值转换为CRSF值 (500-2500us -> 172-1811)
        uint16_t crsf_values[16];
        for (int i = 0; i < 16; i++) {
            // 将500-2500us PWM值映射到172-1811范围
            crsf_values[i] = static_cast<uint16_t>(
                CRSF_MIN_VALUE + ((values_[i] - 500) * (CRSF_MAX_VALUE - CRSF_MIN_VALUE)) / 2000);
            
            // 确保值在有效范围内
            if (crsf_values[i] < CRSF_MIN_VALUE) crsf_values[i] = CRSF_MIN_VALUE;
            if (crsf_values[i] > CRSF_MAX_VALUE) crsf_values[i] = CRSF_MAX_VALUE;
        }
        
        // 打包16个11位通道数据到22字节中
        uint8_t* data = &frame[3];
        uint16_t bits = 0;
        uint8_t bitsAvailable = 0;
        uint8_t byteIndex = 0;
        
        for (int i = 0; i < 16; i++) {
            bits |= (crsf_values[i] << bitsAvailable);
            bitsAvailable += 11;
            
            while (bitsAvailable >= 8) {
                data[byteIndex++] = bits & 0xFF;
                bits >>= 8;
                bitsAvailable -= 8;
            }
        }
        
        // 如果还有剩余位，写入最后一个字节
        if (bitsAvailable > 0) {
            data[byteIndex] = bits & 0xFF;
        }
        
        // 计算并添加CRC
        frame[CRSF_FRAME_SIZE - 1] = calculateCrsfCrc(&frame[2], CRSF_PAYLOAD_SIZE + 1);
        
        // 发送CRSF帧
        serial_->write(reinterpret_cast<const char*>(frame), CRSF_FRAME_SIZE);
        serial_->waitForBytesWritten(1);
        QThread::msleep(6);  // 保持7ms周期
        
        // 调试信息输出（降低频率）
        static int crsf_log_counter = 0;
        crsf_log_counter++;
        if (crsf_log_counter >= 200) {  // 每200次记录一次（约1.4秒间隔）
            ROS_DEBUG("CRSF帧发送: CH1=%d, CH2=%d (对应PWM: %dus, %dus)", 
                     crsf_values[0], crsf_values[1], values_[0], values_[1]);
            crsf_log_counter = 0;
        }
    }
};

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "gimbal_crsf_controller");
    ros::NodeHandle nh;
    
    // 获取串口参数
    std::string serial_port;
    nh.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    
    ROS_INFO("Starting Gimbal CRSF Controller (Simple P Control) on port: %s", 
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