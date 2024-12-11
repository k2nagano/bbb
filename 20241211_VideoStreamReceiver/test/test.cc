#include "VideoStreamReceiver.hh"
#include <QApplication>

int
main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    VideoStreamReceiver receiver;
    receiver.resize(640, 480);
    receiver.show();
    return app.exec();
}
