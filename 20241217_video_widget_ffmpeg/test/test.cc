#include <QApplication>
#include <QVideoWidget>
#include <QMediaPlayer>
#include <QStackedLayout>
#include <QPushButton>
#include <QProcess>
#include <QTimer>
#include <QWidget>

class VideoRecorder : public QWidget {
    Q_OBJECT

public:
    VideoRecorder(QWidget *parent = nullptr) : QWidget(parent), recording(false) {
        // 動画再生用のウィジェットを設定
        videoWidget = new QVideoWidget(this);
        player = new QMediaPlayer(this);
        player->setVideoOutput(videoWidget);

        // UDPストリーミングを設定（サンプルURL）
        player->setMedia(QUrl("udp://@:1234"));  // ここで適切なUDPストリームのURLに置き換えてください
        player->play();

        // 録画開始・停止ボタン（スライドボタンの代わり）
        recButton = new QPushButton("OFF", this);
        recButton->setCheckable(true);
        recButton->setStyleSheet("background-color: red; color: white;");

        // ボタンのクリックで録画を切り替え
        connect(recButton, &QPushButton::clicked, this, &VideoRecorder::toggleRecording);

        // レイアウト設定（QStackedLayoutで重ねて配置）
        QStackedLayout *stackedLayout = new QStackedLayout(this);
        stackedLayout->addWidget(videoWidget); // 下に映像
        stackedLayout->addWidget(recButton);   // 上にボタン
        recButton->setGeometry(20, 20, 100, 50); // ボタンの位置を設定

        // FFmpegプロセス
        ffmpegProcess = new QProcess(this);
    }

private slots:
    void toggleRecording(bool checked) {
        if (checked) {
            startRecording();
        } else {
            stopRecording();
        }
    }

    void startRecording() {
        recButton->setText("ON");
        recButton->setStyleSheet("background-color: green; color: white;");
        recording = true;

        // FFmpegコマンドを設定して録画開始
        QStringList arguments;
        arguments << "-y"  // 上書き
                  << "-i" << "udp://@:1234"  // ストリームソース
                  << "-c:v" << "copy"       // エンコードしないでコピー
                  << "-f" << "mp4" << "output.mp4";  // 出力ファイル

        ffmpegProcess->start("ffmpeg", arguments);
    }

    void stopRecording() {
        recButton->setText("OFF");
        recButton->setStyleSheet("background-color: red; color: white;");
        recording = false;

        // FFmpegプロセスを終了して録画停止
        if (ffmpegProcess->state() == QProcess::Running) {
            ffmpegProcess->terminate();  // 終了
            ffmpegProcess->waitForFinished();
        }
    }

private:
    QMediaPlayer *player;
    QVideoWidget *videoWidget;
    QPushButton *recButton;
    QProcess *ffmpegProcess;
    bool recording;
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    VideoRecorder recorder;
    recorder.show();

    return app.exec();
}

#include "main.moc"

