#include "mainwindow.h"
#include <QPushButton>
#include <QPainter>
#include <QVBoxLayout>

MainWindow::MainWindow(QWidget *parent)
    : QWidget(parent),
      udpThread(new UdpThread(this)),
      background(QImage())
{
    // QPushButton を作成し、レイアウトに追加
    button = new QPushButton("Button", this);
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(button);

    // UDP スレッドからの画像データを受け取る
    connect(udpThread, &UdpThread::newImage, this, &MainWindow::updateBackground);

    // UDP スレッドを開始
    udpThread->start();
}

MainWindow::~MainWindow()
{
    udpThread->stop();
    udpThread->wait();
}

void MainWindow::updateBackground(const QImage &image)
{
    // 受信した画像を背景に設定
    background = image;
    update();  // QWidget を再描画
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    if (!background.isNull()) {
        // 背景に画像を描画
        painter.drawImage(rect(), background);
    }
    QWidget::paintEvent(event);
}

