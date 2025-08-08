#include <ros/ros.h>
#include <QSerialPort>
#include <QThread>
#include <memory>

#define SBUS_SCALE_OFFSET 880.0f
#define SBUS_SCALE_FACTOR 0.625f

class SbusController {
public:
    SbusController(const std::string& port_name)
        : values_(), serial_(std::make_shared<QSerialPort>()) {

        // 初始化通道默认中位值 1500us
        for (int i = 0; i < 16; i++) {
            values_[i] = 1500;
        }

        initSerialPort(port_name);
    }

    void run() {
        ros::Rate rate(140); // 大约7ms周期

        int pwm = 1500;
        bool direction = true;

        while (ros::ok()) {
            // 简单示范：CH1 PWM从1000变到2000循环
            if (direction) {
                pwm += 5;
                if (pwm >= 2000) direction = false;
            } else {
                pwm -= 5;
                if (pwm <= 1000) direction = true;
            }

            values_[0] = pwm;  // 只控制第1通道（舵机1）

            sendSbusFrame();

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    int values_[16];
    std::shared_ptr<QSerialPort> serial_;

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

    void sendSbusFrame() {
        uint8_t oframe[25] = {0};
        oframe[0] = 0x0F;    // 起始字节
        oframe[24] = 0x00;   // 结束字节

        uint8_t byteIndex = 1;
        uint8_t offset = 0;

        for (int i = 0; i < 16; i++) {
            uint16_t value = static_cast<uint16_t>((values_[i] - SBUS_SCALE_OFFSET) / SBUS_SCALE_FACTOR + 0.5f);
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
        QThread::msleep(6);  // 补足7ms周期（1ms写入+6ms睡眠）
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "sbus_controller_node");
    ros::NodeHandle nh;

    // 你可以用参数或硬编码串口名
    std::string serial_port = "/dev/ttyUSB0";

    SbusController controller(serial_port);
    controller.run();

    return 0;
}
