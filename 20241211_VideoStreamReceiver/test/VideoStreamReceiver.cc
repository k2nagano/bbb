#include "VideoStreamReceiver.hh"

VideoStreamReceiver::VideoStreamReceiver(QWidget* parent)
    : QWidget(parent), isRecording(false), outputFile(nullptr)
{

    // レイアウトの設定
    QVBoxLayout* layout = new QVBoxLayout(this);

    // QVideoWidgetの作成
    videoWidget = new QVideoWidget(this);
    layout->addWidget(videoWidget);

    // 録画ボタンの作成
    recordButton = new QPushButton("REC.", this);
    recordButton->setStyleSheet("background-color: gray; color: white;");
    layout->addWidget(recordButton);

    // 録画ボタンのクリックイベント接続
    connect(recordButton, &QPushButton::clicked, this, &VideoStreamReceiver::toggleRecording);

    // UDPソケットの設定
    udpSocket = new QUdpSocket(this);
    udpSocket->bind(QHostAddress::Any, 5600);
    connect(udpSocket, &QUdpSocket::readyRead, this, &VideoStreamReceiver::processPendingDatagrams);

    // メディアプレイヤーの設定
    mediaPlayer = new QMediaPlayer(this);
    mediaPlayer->setVideoOutput(videoWidget);

    // メディアレコーダーの設定
    mediaRecorder = new QMediaRecorder(mediaPlayer);
}

VideoStreamReceiver::~VideoStreamReceiver()
{
    if (outputFile)
    {
        outputFile->close();
        delete outputFile;
    }
}
void
VideoStreamReceiver::processPendingDatagrams()
{
    while (udpSocket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(int(udpSocket->pendingDatagramSize()));
        udpSocket->readDatagram(datagram.data(), datagram.size());

        // ここで受信したデータを処理（例：メディアプレイヤーに渡す）
        // サンプルではQMediaPlayerを使用していますが、実際にはFFmpegなどでデコードが必要かもしれません。
        // mediaPlayer->play();  // データの再生を行う処理を追加
    }
}
void
VideoStreamReceiver::toggleRecording()
{
    if (isRecording)
    {
        // 録画停止
        mediaRecorder->stop();
        recordButton->setText("REC.");
        recordButton->setStyleSheet("background-color: gray; color: white;");
        isRecording = false;
        if (outputFile)
        {
            outputFile->close();
            delete outputFile;
            outputFile = nullptr;
        }
    }
    else
    {
        // 録画開始
        QString fileName = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".mp4";
        outputFile = new QFile(fileName, this);
        if (outputFile->open(QIODevice::WriteOnly))
        {
            mediaRecorder->setOutputLocation(QUrl::fromLocalFile(fileName));
            mediaRecorder->record();
            recordButton->setText("REC. (録画中)");
            recordButton->setStyleSheet("background-color: red; color: white;");
            isRecording = true;
        }
        else
        {
            delete outputFile;
            outputFile = nullptr;
        }
    }
}
