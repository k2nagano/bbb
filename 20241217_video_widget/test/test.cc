#include <QApplication>
#include <QMediaPlayer>
#include <QProcess>
#include <QPushButton>
#include <QStackedLayout>
#include <QTimer>
#include <QVideoWidget>
#include <QWidget>

class VideoRecorder : public QWidget
{
    Q_OBJECT

public:
    explicit VideoRecorder(QWidget* parent = nullptr) : QWidget(parent), recording(false)
    {
        // 動画再生用のウィジェットを設定
        videoWidget = new QVideoWidget(this);
        player = new QMediaPlayer(this);
        player->setVideoOutput(videoWidget);

        // UDPストリーミングを設定（サンプルURL）
        player->setMedia(
            QUrl("udp://@:1234")); // ここで適切なUDPストリームのURLに置き換えてください
        player->play();

        // 録画開始・停止ボタン
        recButton = new QPushButton("OFF", this);
        recButton->setCheckable(true);
        recButton->setStyleSheet("background-color: red; color: white;");

        // ボタンのクリックで録画を切り替え
        connect(recButton, &QPushButton::clicked, this, &VideoRecorder::toggleRecording);

        // レイアウト設定（QStackedLayoutで重ねて配置）
        QStackedLayout* stackedLayout = new QStackedLayout(this);
        stackedLayout->addWidget(videoWidget);   // 下に映像
        stackedLayout->addWidget(recButton);     // 上にボタン
        recButton->setGeometry(20, 20, 100, 50); // ボタンの位置を設定

        // GStreamerプロセス
        gstProcess = new QProcess(this);
    }

    virtual ~VideoRecorder()
    {
    }

private slots:
    void
    toggleRecording(bool checked)
    {
        if (checked)
        {
            startRecording();
        }
        else
        {
            stopRecording();
        }
    }

    void
    startRecording()
    {
        recButton->setText("ON");
        recButton->setStyleSheet("background-color: green; color: white;");
        recording = true;

        // GStreamerコマンドを設定して録画開始
        QStringList arguments;
        arguments << "gst-launch-1.0"
                  << "udpsrc" << "port=1234"
                  << "!" << "application/x-rtp,encoding-name=H264,payload=96"
                  << "!" << "rtpjitterbuffer"
                  << "!" << "rtph264depay"
                  << "!" << "h264parse"
                  << "!" << "mp4mux"
                  << "!" << "filesink" << "location=output.mp4";

        gstProcess->start("gst-launch-1.0", arguments);
    }

    void
    stopRecording()
    {
        recButton->setText("OFF");
        recButton->setStyleSheet("background-color: red; color: white;");
        recording = false;

        // GStreamerプロセスを終了して録画停止
        if (gstProcess->state() == QProcess::Running)
        {
            gstProcess->terminate(); // 終了
            gstProcess->waitForFinished();
        }
    }

private:
    QMediaPlayer* player;
    QVideoWidget* videoWidget;
    QPushButton* recButton;
    QProcess* gstProcess;
    bool recording;
};

int
main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    VideoRecorder recorder;
    recorder.show();

    return app.exec();
}

// #include "main.moc"
