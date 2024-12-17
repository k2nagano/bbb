all:
	# g++ test.cc -o test `pkg-config --cflags --libs qt5widgets qt5multimedia qt5multimediawidgets`
	g++ -std=c++11 -fPIC test.cc -o test -pthread `pkg-config --cflags --libs Qt5Core Qt5Widgets Qt5Multimedia  Qt5MultimediaWidgets`
