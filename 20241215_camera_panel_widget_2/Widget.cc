#include "Widget.hh"
#include <gst/video/videooverlay.h>

// explicit
Widget::Widget(QWidget* pParent)
    : QWidget(pParent), pipeline(nullptr), sink(nullptr), isRecording(false), playing(false),
      mUdpPort(5600)
{
    // GStreamerの初期化
    gst_init(nullptr, nullptr);
    startStreaming();
}

// virtual
Widget::~Widget()
{
    if (pipeline)
    {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
    }
}

void
Widget::contextMenuEvent(QContextMenuEvent* event)
{
    QMenu contextMenu(this);
    QAction* startAction = contextMenu.addAction("Start Streaming");
    QAction* pauseAction = contextMenu.addAction("Pause Streaming");
    QAction* stopAction = contextMenu.addAction("Stop Streaming");
    QAction* settingsAction = contextMenu.addAction("Settings");
    QAction* startRecordingAction = contextMenu.addAction("Start Recording");
    QAction* stopRecordingAction = contextMenu.addAction("Stop Recording");

    // スロットをアクションに接続
    connect(startAction, &QAction::triggered, this, &Widget::startStreaming);
    connect(pauseAction, &QAction::triggered, this, &Widget::pauseStreaming);
    connect(stopAction, &QAction::triggered, this, &Widget::stopStreaming);
    connect(settingsAction, &QAction::triggered, this, &Widget::openSettings);
    connect(startRecordingAction, &QAction::triggered, this, &Widget::startRecording);
    connect(stopRecordingAction, &QAction::triggered, this, &Widget::stopRecording);

    // コンテキストメニューを表示
    contextMenu.exec(event->globalPos());
}

void
Widget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    if (pipeline)
    {
        GstElement* videoSink = gst_bin_get_by_name(GST_BIN(pipeline), "videosink");
        if (videoSink)
        {
            gst_video_overlay_expose(GST_VIDEO_OVERLAY(videoSink));
            gst_object_unref(videoSink);
        }
    }
}

void
Widget::showEvent(QShowEvent* event)
{
    QWidget::showEvent(event);
    if (pipeline)
    {
        GstElement* videoSink = gst_bin_get_by_name(GST_BIN(pipeline), "videosink");
        if (videoSink)
        {
            gst_video_overlay_expose(GST_VIDEO_OVERLAY(videoSink));
            gst_object_unref(videoSink);
        }
    }
}
void
Widget::startStreaming()
{
    if (playing)
        return;

    // GStreamerパイプライン作成
    const char* pipelineStr = "udpsrc port=5600 ! application/x-rtp, payload=96 ! rtph264depay "
                              "! h264parse ! avdec_h264 ! xvimagesink name=videosink";
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
Widget::stopStreaming()
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
Widget::pauseStreaming()
{
    if (pipeline && playing)
    {
        gst_element_set_state(pipeline, GST_STATE_PAUSED);
    }
}

void
Widget::openSettings()
{
}

void
Widget::startRecording()
{
    if (isRecording)
        return;

    // mkvファイルに保存するための新しいsinkエレメントを追加
    sink = gst_parse_bin_from_description("h264parse ! matroskamux ! filesink location=output.mkv",
                                          TRUE, nullptr);

    if (!sink)
    {
        g_printerr("Failed to create recording sink\n");
        return;
    }

    // パイプラインに録画用のsinkを追加
    GstElement* pipeline_sink = gst_bin_get_by_name(GST_BIN(pipeline), "videosink");
    gst_element_set_state(pipeline_sink, GST_STATE_NULL); // 表示用のシンクを一時停止
    gst_bin_add(GST_BIN(pipeline), sink);
    gst_element_link(gst_bin_get_by_name(GST_BIN(pipeline), "videoconvert"), sink);
    gst_element_set_state(sink, GST_STATE_PLAYING); // 録画開始

    isRecording = true;
}
void
Widget::stopRecording()
{
    if (!isRecording)
        return;

    // 録画を終了してファイルを閉じる
    gst_element_set_state(sink, GST_STATE_NULL);
    gst_bin_remove(GST_BIN(pipeline), sink);
    gst_object_unref(sink);
    sink = nullptr;

    // 表示用のシンクを再度有効化
    GstElement* pipeline_sink = gst_bin_get_by_name(GST_BIN(pipeline), "videosink");
    gst_element_set_state(pipeline_sink, GST_STATE_PLAYING);

    isRecording = false;
}
