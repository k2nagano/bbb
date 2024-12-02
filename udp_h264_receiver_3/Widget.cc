#include "Widget.hh"
#include <QDateTime>
#include <QGst/Utils/ApplicationSink>
#include <QMenu>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

StreamingWidget::StreamingWidget(QWidget* parent) : QWidget(parent)
{
    // GStreamer の初期化
    gst_init(nullptr, nullptr);

    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, &QWidget::customContextMenuRequested, this, &StreamingWidget::showContextMenu);

    // GStreamer パイプラインの初期化
    gstPipeline = gst_pipeline_new("udp-stream");
    gstSink = gst_element_factory_make("qt5sink", "video-output");

    GstElement* src = gst_element_factory_make("udpsrc", "udp-source");
    GstCaps* caps =
        gst_caps_from_string("application/x-rtp, media=(string)video, encoding-name=(string)H264");
    g_object_set(src, "caps", caps, NULL);
    g_object_set(src, "port", 5000, NULL);

    GstElement* depay = gst_element_factory_make("rtph264depay", "depay");
    GstElement* decode = gst_element_factory_make("avdec_h264", "decode");

    gst_bin_add_many(GST_BIN(gstPipeline), src, depay, decode, gstSink, NULL);
    gst_element_link_many(src, depay, decode, gstSink, NULL);

    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(gstSink), winId());
    gst_element_set_state(gstPipeline, GST_STATE_PLAYING);

    // 時刻表示のためのタイマー
    overlayTimer = new QTimer(this);
    connect(overlayTimer, &QTimer::timeout, this, &StreamingWidget::updateOverlay);
    overlayTimer->start(1000);
}

StreamingWidget::~StreamingWidget()
{
    gst_element_set_state(gstPipeline, GST_STATE_NULL);
    gst_object_unref(gstPipeline);
}

// コンテキストメニューの表示
void
StreamingWidget::showContextMenu(const QPoint& pos)
{
    QMenu contextMenu(this);

    QAction startAction("Start Recording", this);
    QAction stopAction("Stop Recording", this);
    QAction captureAction("Capture Screenshot", this);
    QAction fullScreenAction("Toggle Fullscreen", this);

    connect(&startAction, &QAction::triggered, this, &StreamingWidget::startRecording);
    connect(&stopAction, &QAction::triggered, this, &StreamingWidget::stopRecording);
    connect(&captureAction, &QAction::triggered, this, &StreamingWidget::captureScreenshot);
    connect(&fullScreenAction, &QAction::triggered, this, &StreamingWidget::toggleFullScreen);

    contextMenu.addAction(&startAction);
    contextMenu.addAction(&stopAction);
    contextMenu.addAction(&captureAction);
    contextMenu.addAction(&fullScreenAction);

    contextMenu.exec(mapToGlobal(pos));
}

// override
void
StreamingWidget::paintEvent(QPaintEvent* event)
{
    QWidget::paintEvent(event);
    // GStreamerの映像は自動で描画される
}

void
StreamingWidget::startRecording()
{
    // 録画を開始する処理
}

void
StreamingWidget::stopRecording()
{
    // 録画を停止する処理
}

void
StreamingWidget::captureScreenshot()
{
    QPixmap pixmap = this->grab(); // QWidgetをキャプチャ
    QString fileName = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss") + ".png";
    pixmap.save(fileName, "PNG");
}

void
StreamingWidget::toggleFullScreen()
{
    if (isFullScreen())
    {
        showNormal();
    }
    else
    {
        showFullScreen();
    }
}

void
StreamingWidget::updateOverlay()
{
    // 現在時刻を表示する処理（QPainterなどを使って描画）
    QPainter painter(this);
    painter.setPen(Qt::white);
    painter.setFont(QFont("Arial", 16));
    painter.drawText(rect(), Qt::AlignTop | Qt::AlignRight,
                     QDateTime::currentDateTime().toString("hh:mm:ss"));
}
