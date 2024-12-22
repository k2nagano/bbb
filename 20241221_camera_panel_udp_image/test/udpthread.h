#ifndef UDPTHREAD_H
#define UDPTHREAD_H

#include <QThread>
#include <QImage>
#include <QUdpSocket>

class UdpThread : public QThread
{
    Q_OBJECT

public:
    explicit UdpThread(QObject *parent = nullptr);
    ~UdpThread();

    void stop();

signals:
    void newImage(const QImage &image);

protected:
    void run() override;

private:
    std::atomic<bool> running;
    QUdpSocket *udpSocket;
};

#endif // UDPTHREAD_H

