#include <QApplication>
#include "videowidget.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    if (argc != 2) {
        // qCritical() << "Usage: udp_h264_receiver <UDP Port>";
        return -1;
    }

    QString udpPort = argv[1];

    // ビデオウィジェットの作成
    VideoWidget videoWidget;
    videoWidget.resize(800, 600);

    if (!videoWidget.initializePipeline(udpPort)) {
        // qCritical() << "Failed to initialize GStreamer pipeline";
        return -1;
    }

    videoWidget.show();

    return app.exec();
}

