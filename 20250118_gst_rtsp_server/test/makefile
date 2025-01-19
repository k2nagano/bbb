CFLAGS =
CFLAGS += `pkg-config --cflags gstreamer-1.0`
CFLAGS += `pkg-config --cflags gstreamer-rtsp-server-1.0`
CFLAGS +=
LIBS =
LIBS += `pkg-config --libs gstreamer-1.0`
LIBS += `pkg-config --libs gstreamer-rtsp-server-1.0`
LIBS += 
LIBS += 
LIBS += 
all:
	gcc ${CFLAGS} test.c -o test ${LIBS}

