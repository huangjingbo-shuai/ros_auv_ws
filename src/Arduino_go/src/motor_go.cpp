#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <serial/serial.h>

serial::Serial ser;

void pwmCallback(const std_msgs::Int32::ConstPtr& msg) {
    int pwm = msg->data;
    std::string send_str = std::to_string(pwm) + "\n";
    ser.write(send_str);
    ROS_INFO_STREAM("[PWMSerialNode] Sent: " << send_str);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pwm_serial_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string port;
    int baudrate;

    pnh.param<std::string>("port", port, "/dev/ttyACM0");
    pnh.param<int>("baudrate", baudrate, 57600);

    try {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("[PWMSerialNode] Unable to open port " << port);
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("[PWMSerialNode] Serial port " << port << " initialized at " << baudrate << " baud.");
    } else {
        return -1;
    }

    ros::Subscriber pwm_sub = nh.subscribe("/pwm_value", 10, pwmCallback);
    ros::spin();

    return 0;
}
