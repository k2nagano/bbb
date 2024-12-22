#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QImage>
#include "gstthread.h"

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    virtual ~MainWindow();

public slots:
    void updateFrame(const QImage &image);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QImage currentFrame;
    GstThread *gstThread;
};

#endif // MAINWINDOW_H

