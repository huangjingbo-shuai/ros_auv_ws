#include <mainwindow.h>
#include <QApplication>
#include <ros/ros.h>
int main(int argc, char **argv)
{
    QApplication a(argc, argv);
    MainWindow w(nullptr,argc,argv);
    w.show();
    return a.exec();
}
