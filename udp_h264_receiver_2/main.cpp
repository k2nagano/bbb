#include <QApplication>
#include <QFileDialog>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
extern "C"
{
#include <gst/gst.h>
#include <gst/video/videooverlay.h>
}

class VideoWidget : public QWidget
{
    Q_OBJECT

public:
    VideoWidget(QWidget* parent = nullptr) : QWidget(parent), pipeline(nullptr), playing(false)
    {
        QVBoxLayout* layout = new QVBoxLayout(this);

        // ボタンの作成
        QPushButton* startButton = new QPushButton("Start Streaming", this);
        QPushButton* stopButton = new QPushButton("Stop Streaming", this);
        QPushButton* pauseButton = new QPushButton("Pause Streaming", this);
        QPushButton* saveButton = new QPushButton("Save to MKV", this);

        layout->addWidget(startButton);
        layout->addWidget(pauseButton);
        layout->addWidget(stopButton);
        layout->addWidget(saveButton);

        connect(startButton, &QPushButton::clicked, this, &VideoWidget::startStreaming);
        connect(stopButton, &QPushButton::clicked, this, &VideoWidget::stopStreaming);
        connect(pauseButton, &QPushButton::clicked, this, &VideoWidget::pauseStreaming);
        connect(saveButton, &QPushButton::clicked, this, &VideoWidget::saveToMKV);

        // GStreamerの初期化
        gst_init(nullptr, nullptr);
    }

    ~VideoWidget()
    {
        if (pipeline)
        {
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
        }
    }

protected:
    void
    paintEvent(QPaintEvent* event) override
    {
        QWidget::paintEvent(event);
        // テキストをオーバーレイ
        QPainter painter(this);
        painter.setPen(Qt::white);
        painter.setFont(QFont("Arial", 30));
        painter.drawText(rect(), Qt::AlignCenter, "H.264 UDP Streaming");
    }

private slots:
    void
    startStreaming()
    {
        if (playing)
            return;

        // GStreamerパイプライン作成
        const char* pipelineStr = "udpsrc port=5000 ! application/x-rtp, payload=96 ! rtph264depay "
                                  "! h264parse ! avdec_h264 ! videoconvert ! autovideosink";
        GError* error = nullptr;
        pipeline = gst_parse_launch(pipelineStr, &error);

        if (error)
        {
            g_printerr("Error: %s\n", error->message);
            g_error_free(error);
            return;
        }

        // ビデオをQWidgetに描画するための設定
        GstElement* videoSink = gst_bin_get_by_interface(GST_BIN(pipeline), GST_TYPE_VIDEO_OVERLAY);
        if (videoSink)
        {
            gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), this->winId());
        }

        gst_element_set_state(pipeline, GST_STATE_PLAYING);
        playing = true;
    }

    void
    stopStreaming()
    {
        if (pipeline && playing)
        {
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
            pipeline = nullptr;
            playing = false;
            update();
        }
    }

    void
    pauseStreaming()
    {
        if (pipeline && playing)
        {
            gst_element_set_state(pipeline, GST_STATE_PAUSED);
        }
    }

    void
    saveToMKV()
    {
        // ファイルダイアログで保存先を選択
        QString filename = QFileDialog::getSaveFileName(this, "Save to MKV", "", "*.mkv");
        if (filename.isEmpty())
            return;

        // 新しいパイプラインを作成して保存
        QString pipelineStr =
            QString("udpsrc port=5000 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! "
                    "matroskamux ! filesink location=%1")
                .arg(filename);

        GError* error = nullptr;
        GstElement* savePipeline = gst_parse_launch(pipelineStr.toUtf8().constData(), &error);
        if (error)
        {
            g_printerr("Error: %s\n", error->message);
            g_error_free(error);
            return;
        }

        gst_element_set_state(savePipeline, GST_STATE_PLAYING);
    }

private:
    GstElement* pipeline;
    bool playing;
};

int
main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    VideoWidget widget;
    widget.resize(800, 600);
    widget.show();

    return app.exec();
}
