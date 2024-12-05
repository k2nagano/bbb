QT += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = gstreamer_udp_h264
TEMPLATE = app

SOURCES += main.cpp \
           MainWindow.cpp

HEADERS += MainWindow.h

# GStreamerライブラリを追加
LIBS += `pkg-config --libs gstreamer-1.0`
INCLUDEPATH += `pkg-config --cflags gstreamer-1.0`

