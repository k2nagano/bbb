#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets>
#include <QWidget>
#include <QImage>
#include "udpthread.h"

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void updateBackground(const QImage &image);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QImage background;
    UdpThread *udpThread;
    QPushButton *button;
};

#endif // MAINWINDOW_H

