#if !defined(VIDEO_STREAM_RECEIVER_HH)
#define VIDEO_STREAM_RECEIVER_HH

#include <QApplication>
#include <QDateTime>
#include <QFile>
#include <QMediaPlayer>
#include <QMediaRecorder>
#include <QPushButton>
#include <QUdpSocket>
#include <QVBoxLayout>
#include <QVideoWidget>
#include <QWidget>

class VideoStreamReceiver : public QWidget
{
    Q_OBJECT

public:
    VideoStreamReceiver(QWidget* parent = nullptr);
    ~VideoStreamReceiver();

private slots:
    void processPendingDatagrams();
    void toggleRecording();

private:
    QVideoWidget* videoWidget;
    QPushButton* recordButton;
    QUdpSocket* udpSocket;
    QMediaPlayer* mediaPlayer;
    QMediaRecorder* mediaRecorder;
    QFile* outputFile;
    bool isRecording;
};

#endif // !defined(VIDEO_STREAM_RECEIVER_HH)
