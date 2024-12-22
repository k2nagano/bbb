#include "udpthread.h"
#include <QHostAddress>
#include <QByteArray>
#include <QDataStream>
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

            // 先頭4バイト（uint16:width, uint16:height）を読み取る
            if (datagram.size() < 4) {
                continue;  // データが少なすぎる場合はスキップ
            }

            // 先頭4バイトから幅と高さを取得
            uint16_t width = static_cast<uint16_t>((uint8_t(datagram[0]) << 8) | uint8_t(datagram[1]));
            uint16_t height = static_cast<uint16_t>((uint8_t(datagram[2]) << 8) | uint8_t(datagram[3]));

            // 画像データの期待されるサイズ
            int expectedSize = width * height * 3;  // RGB888フォーマットのサイズ計算

            // データサイズが一致するか確認
            if (datagram.size() - 4 == expectedSize) {
                // 画像データを取り出してQImageに変換
                QImage image((const uchar *)(datagram.data() + 4), width, height, QImage::Format_RGB888);
                emit newImage(image);
            } else {
                qDebug() << "Invalid image size: received" << datagram.size() - 4 << "bytes, expected" << expectedSize << "bytes.";
            }
        }

        QThread::msleep(10);  // 少し待機してCPU負荷を抑える
    }

    delete udpSocket;
}

