#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>  // ✅ 加这句，不要漏！
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    // ✅ 添加这个构造函数以支持 MainWindow(nullptr, argc, argv)
    explicit MainWindow(QWidget *parent = nullptr, int argc = 0, char **argv = nullptr);
    ~MainWindow();

private:
    // Qt相关变量
    Ui::MainWindow *ui;
    QSerialPort serialPort;
    //按钮启动相关
    void on_btnOpenSerial_toggled(bool checked);  // 你 UI 按钮命名为 btnOpenSerial
    void on_verticalSlider_valueChanged(int value);
    //功能函数相关
    void appendLog(const QString &message);
    };

#endif // MAINWINDOW_H
