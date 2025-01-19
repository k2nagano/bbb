#include "Widget.hh"
#include <QApplication>
#include <iostream>
#include <unistd.h>

int
main(int argc, char* argv[])
{
    int sonar_range = 3;
    std::string rtsp_address = "192.168.2.42";
    std::string mkv_input;
    int opt;
    while ((opt = getopt(argc, argv, "r:a:m:")) != -1)
    {
        switch (opt)
        {
        case 'r':
            // std::cout << "sonar_range: " << optarg << std::endl;
            sonar_range = atoi(optarg);
            break;
        case 'a':
            rtsp_address = optarg;
            break;
        case 'm':
            mkv_input = optarg;
            break;
        case '?':
        default:
            std::cerr << "Usage: " << argv[0] << " -r <sonar_range>" << std::endl;
            return 1;
        }
    }
    std::cout << "sonar_range=" << sonar_range << std::endl;
    std::cout << "rtsp_address=" << rtsp_address << std::endl;
    std::cout << "mkv_input=" << mkv_input << std::endl;
    QApplication a(argc, argv);
    Widget w(sonar_range, rtsp_address, mkv_input);
    w.show();
    return a.exec();
}
