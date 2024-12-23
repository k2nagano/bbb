######################################################################
# Automatically generated by qmake (3.1) Sat Dec 21 10:04:07 2024
######################################################################

TEMPLATE = app
TARGET = test
INCLUDEPATH += .
QMAKE_CXXFLAGS += -pthread -I/usr/include/gstreamer-1.0 -I/usr/include/orc-0.4 -I/usr/include/gstreamer-1.0 -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include

# The following define makes your compiler warn you if you use any
# feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

# Input
HEADERS += gstthread.h mainwindow.h
SOURCES += gstthread.cpp \
           mainwindow.cpp \
           main.cpp
QT += core widgets
LIBS += -lgstvideo-1.0 -lgstapp-1.0 -lgstbase-1.0 -lgstreamer-1.0 -lgobject-2.0 -lglib-2.0
