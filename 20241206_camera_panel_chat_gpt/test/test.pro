QT += widgets core gui

TARGET = gstreamer_udp_h264
TEMPLATE = app

HEADERS += MainWindow.h
SOURCES += MainWindow.cpp main.cpp

#LIBS += `pkg-config --libs gstreamer-1.0`
#INCLUDEPATH += `pkg-config --cflags gstreamer-1.0`
#QMAKE_CXXFLAGS += `pkg-config --cflags gstreamer-1.0`
QMAKE_CXXFLAGS += -pthread -I/usr/include/gstreamer-1.0 -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include
LIBS += -lgstreamer-1.0 -lgobject-2.0 -lglib-2.0
