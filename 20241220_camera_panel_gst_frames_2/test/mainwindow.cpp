#include "mainwindow.h"
#include <QPainter>
#include <QtWidgets>

MainWindow::MainWindow(QWidget *parent)
    : QWidget(parent),
      gstThread(new GstThread(this))
{
    connect(gstThread, &GstThread::newFrame, this, &MainWindow::updateFrame);
    gstThread->start();
    QVBoxLayout* p_layout = new QVBoxLayout();
    setLayout(p_layout);
    QPushButton* p_push_button = new QPushButton("aaa");
    p_layout->addWidget(p_push_button);
}

MainWindow::~MainWindow()
{
    gstThread->quit();
    gstThread->wait();
}

void MainWindow::updateFrame(const QImage &image)
{
    currentFrame = image;
    update(); // 再描画
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    if (!currentFrame.isNull()) {
        // ウィンドウのサイズに合わせて画像を拡大縮小して描画
        painter.drawImage(rect(), currentFrame);
    }
}

