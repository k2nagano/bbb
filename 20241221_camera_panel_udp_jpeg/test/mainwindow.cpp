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
    button = new QPushButton("Start Receiving", this);
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(button);

    // UDPスレッドからのJPEG画像データを受け取る
    connect(udpThread, &UdpThread::newImage, this, &MainWindow::updateBackground);

    // UDPスレッドを開始
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
        // 画像のサイズを取得
        int imgWidth = background.width();
        int imgHeight = background.height();

        // ウィジェットのサイズを取得
        int widgetWidth = width();
        int widgetHeight = height();

        // 画像のアスペクト比を計算
        float imgAspect = static_cast<float>(imgWidth) / imgHeight;
        float widgetAspect = static_cast<float>(widgetWidth) / widgetHeight;

        // 描画領域を計算
        QRect targetRect;
        if (imgAspect > widgetAspect) {
            // 画像がウィジェットよりも横長の場合
            int newWidth = widgetWidth;
            int newHeight = static_cast<int>(widgetWidth / imgAspect);
            int yOffset = (widgetHeight - newHeight) / 2;
            targetRect = QRect(0, yOffset, newWidth, newHeight);
        } else {
            // 画像がウィジェットよりも縦長の場合
            int newHeight = widgetHeight;
            int newWidth = static_cast<int>(widgetHeight * imgAspect);
            int xOffset = (widgetWidth - newWidth) / 2;
            targetRect = QRect(xOffset, 0, newWidth, newHeight);
        }

        // 背景を黒で塗りつぶす
        painter.fillRect(rect(), Qt::black);

        // 画像を描画
        painter.drawImage(targetRect, background);
    }
    QWidget::paintEvent(event);
}

