#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <ros/ros.h>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <QThread>
MainWindow::MainWindow(QWidget *parent,int argc,char** argv)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->btnOpenSerial, &QPushButton::toggled, this, &MainWindow::on_btnOpenSerial_toggled);
    connect(ui->verticalSlider, &QSlider::valueChanged, this, &MainWindow::on_verticalSlider_valueChanged);
    // ✅ 初始化 ROS
    ros::init(argc, argv, "motor_go");
    ros::NodeHandle nh;
}

void MainWindow::appendLog(const QString &message)
{
    ui->textLog->append(message);
    ui->textLog->moveCursor(QTextCursor::End);  // 自动滚动到底部
}

void MainWindow::on_btnOpenSerial_toggled(bool checked)
{
    if (checked) {
        serialPort.setPortName("/dev/ttyACM0");
        serialPort.setBaudRate(QSerialPort::Baud57600);
        serialPort.setDataBits(QSerialPort::Data8);
        serialPort.setParity(QSerialPort::NoParity);
        serialPort.setStopBits(QSerialPort::OneStop);
        serialPort.setFlowControl(QSerialPort::NoFlowControl);

        if (!serialPort.open(QIODevice::ReadWrite)) {
            QString msg = "[错误] 串口打开失败";
            qDebug() << msg;
            appendLog(msg);
            return;
        }

        appendLog("[串口] 已打开 /dev/ttyACM0");

        QString pwmStr = "1500\n";
        serialPort.write(pwmStr.toUtf8());
        serialPort.waitForBytesWritten(100);

        appendLog("[PWM] 初始化 PWM 1500μs 发送完成");

        QThread::sleep(2);  // 延迟等待电调识别
        appendLog("[初始化] 电调初始化完成");
    }
    else {
        serialPort.close();
        appendLog("[串口] 已关闭");
    }
    appendLog("✅ 这是点击按钮时的测试日志");

}

void MainWindow::on_verticalSlider_valueChanged(int value)
{
    int pwm = 1000 + value * 10;
    QString pwmStr = QString::number(pwm) + "\n";

    if (serialPort.isOpen()) {
        serialPort.write(pwmStr.toUtf8());
        appendLog("[PWM] Sent: " + QString::number(pwm));
    } else {
        appendLog("[警告] 串口未打开，无法发送PWM");
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
