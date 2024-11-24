#include "videowidget.h"
#include <QResizeEvent>
#include <QShowEvent>
#include <QDebug>

VideoWidget::VideoWidget(QWidget *parent) : QWidget(parent), pipeline(nullptr) {
    // GStreamerの初期化
    gst_init(nullptr, nullptr);
}

VideoWidget::~VideoWidget() {
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
    }
}

bool VideoWidget::initializePipeline(const QString &udpPort) {
    // GStreamerパイプラインの作成
    QString pipelineStr = QString(
        "udpsrc port=%1 ! application/x-rtp,encoding-name=H264,payload=96 ! "
        // "rtph264depay ! avdec_h264 ! videoconvert ! "
        "rtph264depay ! avdec_h264 ! "
        "xvimagesink name=videosink").arg(udpPort);

    GError *error = nullptr;
    pipeline = gst_parse_launch(pipelineStr.toUtf8().data(), &error);

    if (error) {
        qWarning() << "GStreamer pipeline creation failed:" << error->message;
        g_error_free(error);
        return false;
    }

    // ウィンドウIDを取得してGStreamerに渡す
    GstElement *videoSink = gst_bin_get_by_name(GST_BIN(pipeline), "videosink");
    if (!videoSink) {
        qWarning() << "Failed to get video sink";
        return false;
    }

#if defined(Q_OS_WIN)
    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), (guintptr)winId());
#else
    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), winId());
#endif

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    return true;
}

void VideoWidget::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    if (pipeline) {
        GstElement *videoSink = gst_bin_get_by_name(GST_BIN(pipeline), "videosink");
        if (videoSink) {
            gst_video_overlay_expose(GST_VIDEO_OVERLAY(videoSink));
            gst_object_unref(videoSink);
        }
    }
}

void VideoWidget::showEvent(QShowEvent *event) {
    QWidget::showEvent(event);
    if (pipeline) {
        GstElement *videoSink = gst_bin_get_by_name(GST_BIN(pipeline), "videosink");
        if (videoSink) {
            gst_video_overlay_expose(GST_VIDEO_OVERLAY(videoSink));
            gst_object_unref(videoSink);
        }
    }
}

