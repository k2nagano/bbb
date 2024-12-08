#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QMediaRecorder>
#include <QUrl>
#include <QComboBox>
#include <QLabel>
#include <QMessageBox>
#include <QDateTime>

class StreamPlayer : public QWidget {
    Q_OBJECT

public:
    StreamPlayer(QWidget *parent = nullptr) : QWidget(parent) {
        // レイアウトの設定
        QVBoxLayout *layout = new QVBoxLayout(this);

        // QMediaPlayer と QVideoWidget の設定
        videoWidget = new QVideoWidget(this);
        player = new QMediaPlayer(this);
        player->setVideoOutput(videoWidget);

        // 録画用のメディアレコーダーを設定
        recorder = new QMediaRecorder(player, this);

        // ボタンの作成
        QPushButton *playButton = new QPushButton("Start", this);
        QPushButton *pauseButton = new QPushButton("Pause", this);
        QPushButton *stopButton = new QPushButton("Stop", this);
        QPushButton *startRecordingButton = new QPushButton("Start Recording", this);
        QPushButton *stopRecordingButton = new QPushButton("Stop Recording", this);

        // 録画形式を選択するドロップダウンリスト
        QLabel *label = new QLabel("Select format for recording:", this);
        formatComboBox = new QComboBox(this);
        formatComboBox->addItem("MP4", "mp4");
        formatComboBox->addItem("MKV", "mkv");

        // ボタンのシグナル/スロット接続
        connect(playButton, &QPushButton::clicked, this, &StreamPlayer::playStream);
        connect(pauseButton, &QPushButton::clicked, player, &QMediaPlayer::pause);
        connect(stopButton, &QPushButton::clicked, player, &QMediaPlayer::stop);
        connect(startRecordingButton, &QPushButton::clicked, this, &StreamPlayer::startRecording);
        connect(stopRecordingButton, &QPushButton::clicked, this, &StreamPlayer::stopRecording);

        // レイアウトにウィジェットを追加
        layout->addWidget(videoWidget);
        layout->addWidget(playButton);
        layout->addWidget(pauseButton);
        layout->addWidget(stopButton);
        layout->addWidget(label);
        layout->addWidget(formatComboBox);
        layout->addWidget(startRecordingButton);
        layout->addWidget(stopRecordingButton);

        setLayout(layout);
        setWindowTitle("UDP H.264 Stream Player");

        // ストリームの設定
        // player->setMedia(QUrl("udp://127.0.0.1:5000"));  // UDPポート5000で受信
        player->setMedia(QUrl("gst-pipeline: udpsrc port=5600 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! xvimagesink name=qtvideosink"));  // UDPポート5000で受信
    }

public slots:
    // ストリームを再生するスロット
    void playStream() {
        player->play();

        // ストリームが再生されない場合、エラーメッセージを詳細に表示
        if (player->error() != QMediaPlayer::NoError) {
            QMessageBox::critical(this, "Error", "Failed to start streaming: " + player->errorString());
        }
    }

    // 録画を開始するスロット
    void startRecording() {
        // 現在の日時を使ってファイル名を自動生成
        QString selectedFormat = formatComboBox->currentData().toString();
        QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
        QString fileName = QString("recording_%1.%2").arg(timestamp, selectedFormat);

        // 保存場所の指定がない場合、ファイルを自動的に作成
        recorder->setOutputLocation(QUrl::fromLocalFile(fileName));
        recorder->record();
    }

    // 録画を停止するスロット
    void stopRecording() {
        recorder->stop();
    }

private:
    QMediaPlayer *player;
    QVideoWidget *videoWidget;
    QMediaRecorder *recorder;
    QComboBox *formatComboBox;
};

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    StreamPlayer player;
    player.resize(800, 600);
    player.show();

    return app.exec();
}

#include "main.moc"

