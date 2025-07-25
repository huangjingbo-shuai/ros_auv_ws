#include <ros/ros.h>
#include <boost/asio.hpp>

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "serial_send_w");
    ros::NodeHandle nh("~");

    std::string port;
    int baudrate;

    nh.param<std::string>("port", port, "/dev/ttyUSB0");  // 串口号
    nh.param<int>("baudrate", baudrate, 115200);          // 波特率

    boost::asio::io_service io;
    boost::asio::serial_port serial(io);

    try {
        serial.open(port);
        serial.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        ROS_INFO("串口 %s 打开成功，准备发送 'w'", port.c_str());

        char w = 'w';
        boost::asio::write(serial, boost::asio::buffer(&w, 1));

        ROS_INFO("发送完成：'w'");
        serial.close();
    } catch (boost::system::system_error& e) {
        ROS_ERROR("串口操作失败: %s", e.what());
        return 1;
    }

    return 0;
}
