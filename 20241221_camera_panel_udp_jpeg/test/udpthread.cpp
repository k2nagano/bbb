#include "udpthread.h"
#include <QHostAddress>
#include <QByteArray>
#include <QBuffer>
#include <QImageReader>
#include <QDebug>

UdpThread::UdpThread(QObject *parent)
    : QThread(parent),
      running(false),
      udpSocket(nullptr)
{
}

UdpThread::~UdpThread()
{
    stop();
}

void UdpThread::stop()
{
    running = false;
}

void UdpThread::run()
{
    udpSocket = new QUdpSocket();
    udpSocket->bind(QHostAddress::Any, 5601);

    running = true;

    while (running) {
        if (udpSocket->hasPendingDatagrams()) {
            QByteArray datagram;
            datagram.resize(int(udpSocket->pendingDatagramSize()));
            udpSocket->readDatagram(datagram.data(), datagram.size());

            // JPEGデータをQImageに変換
            QBuffer buffer(&datagram);
            buffer.open(QIODevice::ReadOnly);
            QImageReader reader(&buffer, "JPEG");
            QImage image = reader.read();

            if (!image.isNull()) {
                emit newImage(image);  // 受信した画像をメインスレッドに送信
            } else {
                qDebug() << "Failed to decode JPEG image.";
            }
        }

        QThread::msleep(10);  // 少し待機してCPU負荷を抑える
    }

    delete udpSocket;
}

