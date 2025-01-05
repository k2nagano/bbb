#include "CameraPanel.hh"
#include <QApplication>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication a(argc, argv);
    CameraPanel w;
    w.show();
    return a.exec();
}

