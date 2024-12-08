#include <QApplication>
#include "Widget.hh"

int
main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    StreamingWidget widget;
    widget.show();
    return app.exec();
}
