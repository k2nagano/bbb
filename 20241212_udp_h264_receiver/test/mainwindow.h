#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QMenu>
#include <QContextMenuEvent>
#include <QDialog>
#include <gst/gst.h>

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void contextMenuEvent(QContextMenuEvent *event) override;

private slots:
    void startStreaming();
    void pauseStreaming();
    void stopStreaming();
    void openSettings();

private:
    GstElement *pipeline;
    bool isStreaming;
    int udpPort;

    void initGStreamer();
    void stopGStreamer();
};

// SettingsDialogクラスの定義
class SettingsDialog : public QDialog
{
    Q_OBJECT

public:
    SettingsDialog(int currentPort, QWidget *parent = nullptr);
    int getPort() const;

private:
    int port;
};

#endif // MAINWINDOW_H

