#include <QApplication>
#include "viewer_widget.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    ViewerWidget viewer;
    viewer.resize(1920, 1080);  // Set initial size to Full HD
    viewer.show();

    return app.exec();
}

