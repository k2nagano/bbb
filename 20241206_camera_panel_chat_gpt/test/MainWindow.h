#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <gst/gst.h>

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void startRecording();
    void stopRecording();

private:
    GstElement *pipeline;
    GstElement *sink;

    bool isRecording;
    QPushButton *startButton;
    QPushButton *stopButton;

    void createPipeline();
    void destroyPipeline();
};

#endif // MAINWINDOW_H

