#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QFileDialog>
#include <QMediaRecorder>
#include <QUrl>
#include <QComboBox>
#include <QLabel>
#include <QMessageBox>

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
        player->setMedia(QUrl("udp://0.0.0.0:5000"));  // UDPポート5000で受信
    }

public slots:
    // ストリームを再生するスロット
    void playStream() {
        player->play();

        // ストリームが再生されない場合、エラーメッセージを表示
        if (player->error() != QMediaPlayer::NoError) {
            QMessageBox::critical(this, "Error", "Failed to start streaming: " + player->errorString());
        }
    }

    // 録画を開始するスロット
    void startRecording() {
        // 録画ファイルの形式に応じたファイル保存場所の選択
        QString selectedFormat = formatComboBox->currentData().toString();
        QString fileFilter = QString("%1 Files (*.%1)").arg(selectedFormat.toUpper());
        QString fileName = QFileDialog::getSaveFileName(this, "Save Recording", "", fileFilter);
        
        if (!fileName.isEmpty()) {
            if (!fileName.endsWith("." + selectedFormat)) {
                fileName += "." + selectedFormat;
            }

            recorder->setOutputLocation(QUrl::fromLocalFile(fileName));
            recorder->record();
        }
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

