#include "Widget.hh"
#include <gst/video/videooverlay.h>

// explicit
Widget::Widget(QWidget* pParent) : QWidget(pParent), pipeline(nullptr)
{
    gst_init(nullptr, nullptr);
    startPipeline();
}

// virtual
Widget::~Widget()
{
    stopPipeline();
}

void
Widget::contextMenuEvent(QContextMenuEvent* event)
{
    QMenu contextMenu(this);
    QAction* startAction = contextMenu.addAction("Start");
    QAction* pauseAction = contextMenu.addAction("Pause");
    QAction* stopAction = contextMenu.addAction("Stop");

    // スロットをアクションに接続
    connect(startAction, &QAction::triggered, this, &Widget::startPipeline);
    connect(pauseAction, &QAction::triggered, this, &Widget::pausePipeline);
    connect(stopAction, &QAction::triggered, this, &Widget::stopPipeline);

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
Widget::initPipeline()
{
    // int udpPort = 5600;
    // QString pipeline_str =
    //     QString("udpsrc port=%1 ! application/x-rtp,encoding-name=H264,payload=96 ! "
    //             "rtph264depay ! avdec_h264 ! "
    //             "xvimagesink name=videosink")
    //         .arg(udpPort);

    int udpPort = 5600;
    QString pipeline_str =
        QString("udpsrc port=%1 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! tee name=t "
        "t. ! queue ! avdec_h264 ! xvimagesink name=videosink "
        "t. ! queue ! matroskamux ! filesink location=output.mkv")
            .arg(udpPort);

    GError* error = nullptr;
    pipeline = gst_parse_launch(pipeline_str.toUtf8().data(), &error);

    if (error)
    {
        printf("GStreamer pipeline creation failed: %s\n", error->message);
        g_error_free(error);
        return;
    }

    // ウィンドウIDを取得してGStreamerに渡す
    GstElement* videoSink = gst_bin_get_by_name(GST_BIN(pipeline), "videosink");
    if (!videoSink)
    {
        printf("Failed to get video sink\n");
        return;
    }

    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), winId());
}
void
Widget::startPipeline()
{
    if (pipeline == nullptr)
    {
        initPipeline();
    }
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
}
void
Widget::pausePipeline()
{
    gst_element_set_state(pipeline, GST_STATE_PAUSED);
}
void
Widget::stopPipeline()
{
    if (pipeline)
    {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        pipeline = nullptr;
    }
}
