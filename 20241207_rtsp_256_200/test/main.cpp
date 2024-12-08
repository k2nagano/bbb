#include <QApplication>
#include <QWidget>
#include <QTimer>
#include <QImage>
#include <QPainter>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <iostream>
#include <vector>

class VideoReceiver : public QWidget
{
    Q_OBJECT

public:
    VideoReceiver(QWidget *parent = nullptr) : QWidget(parent), pipeline(nullptr), appsink(nullptr)
    {
        setMinimumSize(256, 200);  // ウィジェットサイズを256x200に設定

        // GStreamerの初期化
        gst_init(nullptr, nullptr);

        // GStreamerパイプラインの作成
        createPipeline();

        // 定期的にGStreamerのバスをポーリングして映像を取得
        QTimer *timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, &VideoReceiver::onTimeout);
        timer->start(30);  // 30msごとに更新（約33FPS）
    }

    ~VideoReceiver()
    {
        if (pipeline) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
        }
    }

protected:
    void paintEvent(QPaintEvent *event) override
    {
        QPainter painter(this);
        if (!frame.isNull()) {
            // 受信したフレームをウィジェットに描画
            painter.drawImage(0, 0, frame);
        }
    }

private slots:
    void onTimeout()
    {
        if (!appsink) {
            return;
        }

        // appsinkから映像フレームを取得
        GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
        if (sample) {
            GstBuffer *buffer = gst_sample_get_buffer(sample);
            GstCaps *caps = gst_sample_get_caps(sample);
            GstMapInfo map;

            if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
                // 256x200のグレースケール画像のデータを取得
                processFrame(map.data, map.size);
                gst_buffer_unmap(buffer, &map);
            }

            gst_sample_unref(sample);
        }

        update();  // 再描画をリクエスト
    }

private:
    GstElement *pipeline;
    GstElement *appsink;
    QImage frame;

    void createPipeline()
    {
        // GStreamerパイプラインを作成してRTSPストリームを受信
        const char *pipeline_str = "rtspsrc location=rtsp://localhost:8554/test ! decodebin ! videoconvert ! video/x-raw,format=GRAY8,width=256,height=200 ! appsink name=sink";

        GError *error = nullptr;
        pipeline = gst_parse_launch(pipeline_str, &error);
        if (!pipeline || error) {
            std::cerr << "Failed to create pipeline: " << (error ? error->message : "unknown error") << std::endl;
            if (error) g_error_free(error);
            return;
        }

        // appsinkエレメントを取得
        appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
        gst_app_sink_set_emit_signals(GST_APP_SINK(appsink), true);
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
    }

    void processFrame(const guint8 *data, gsize size)
    {
        // グレースケールのフレームデータを処理し、QImageに変換
        if (size == 256 * 200) {
            frame = QImage(data, 256, 200, QImage::Format_Grayscale8);

            // 各画素の濃淡値を表示（必要に応じてデバッグ用）
            for (int y = 0; y < 200; ++y) {
                for (int x = 0; x < 256; ++x) {
                    int pixelValue = data[y * 256 + x];
                    // ここで、画素値に対する処理が可能
                    std::cout << "Pixel (" << x << ", " << y << ") = " << pixelValue << std::endl;
                }
            }
        }
    }
};

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    VideoReceiver receiver;
    receiver.show();

    return app.exec();
}

#include "main.moc"

