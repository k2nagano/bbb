#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QTimer>
#include <gst/gst.h>
#include <gst/video/videooverlay.h>

class VideoRecorder : public QWidget {
    Q_OBJECT

public:
    VideoRecorder(QWidget *parent = nullptr) : QWidget(parent), recording(false), pipeline(nullptr) {
        // レイアウトとボタン
        QVBoxLayout *layout = new QVBoxLayout(this);
        QPushButton *recButton = new QPushButton("Start Recording", this);
        recButton->setCheckable(true);
        layout->addWidget(recButton);

        // GStreamerの初期化
        gst_init(nullptr, nullptr);

        // 録画開始・停止ボタンの接続
        connect(recButton, &QPushButton::clicked, this, &VideoRecorder::toggleRecording);

        // GStreamerパイプラインの作成（映像表示用）
        createPipeline();
    }

    ~VideoRecorder() {
        if (pipeline) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
        }
    }

private slots:
    void toggleRecording(bool checked) {
        if (checked) {
            startRecording();
        } else {
            stopRecording();
        }
    }

private:
    GstElement *pipeline;
    GstElement *videoSink;
    bool recording;

    void createPipeline() {
        // パイプラインの構築
        pipeline = gst_parse_launch(
            "udpsrc port=1234 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink", 
            nullptr
        );

        if (!pipeline) {
            qFatal("GStreamer pipeline creation failed!");
        }

        // ビデオシンク（ウィンドウに映像を表示）
        videoSink = gst_bin_get_by_interface(GST_BIN(pipeline), GST_TYPE_VIDEO_OVERLAY);
        if (!videoSink) {
            qFatal("Failed to get video sink");
        }

        // QtのウィンドウIDを取得してGStreamerに設定
        WId winId = winId();
        gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), winId);

        // パイプラインを再生状態に
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
    }

    void startRecording() {
        if (recording) return;

        // 録画用のパイプラインを追加（teeエレメントで分岐させる）
        GstElement *tee = gst_element_factory_make("tee", "tee");
        GstElement *fileSink = gst_element_factory_make("filesink", "filesink");
        g_object_set(fileSink, "location", "output.mp4", nullptr);

        GstElement *muxer = gst_element_factory_make("mp4mux", "mux");
        GstElement *queue = gst_element_factory_make("queue", "queue");

        // パイプラインに要素を追加
        gst_bin_add_many(GST_BIN(pipeline), tee, queue, muxer, fileSink, nullptr);
        gst_element_link_many(tee, queue, muxer, fileSink, nullptr);

        // 分岐させて録画を開始
        recording = true;
    }

    void stopRecording() {
        if (!recording) return;

        // パイプラインを停止
        gst_element_set_state(pipeline, GST_STATE_NULL);
        recording = false;
    }
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    VideoRecorder recorder;
    recorder.show();

    return app.exec();
}

