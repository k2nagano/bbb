#include "Widget.hh"
#include <QAction>
#include <QMenu>
#include <QMouseEvent>
#include <QTimer>
#include <QWidget>
#include <ctime>
#include <gst/video/videooverlay.h>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace
{
    // 日付付きのファイル名を生成する関数
    std::string
    generateFilename()
    {
        // 現在の日時を取得
        std::time_t now = std::time(nullptr);
        std::tm* localTime = std::localtime(&now);

        // YYYYMMDD-hhmmss形式の文字列を生成
        std::ostringstream filename;
        filename << "recording_" << std::put_time(localTime, "%Y%m%d-%H%M%S") << ".mp4";

        return filename.str();
    }

} // namespace

// explicit
Widget::Widget(QWidget* pParent)
    : QWidget(pParent), pipeline(nullptr), recording(false)
{
    // GStreamerの初期化
    gst_init(nullptr, nullptr);

    // GStreamerパイプラインを作成（再生用）
    pipeline = gst_parse_launch("udpsrc port=5600 ! application/x-rtp, payload=96 ! "
                                "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
                                "xvimagesink name=videosink",
                                nullptr);

    if (!pipeline)
    {
        std::cerr << "Failed to create pipeline." << std::endl;
        return;
    }

    // GStreamerのビデオをQWidgetに表示する設定
    GstElement* video_sink = gst_bin_get_by_interface(GST_BIN(pipeline), GST_TYPE_VIDEO_OVERLAY);
    if (video_sink)
    {
        gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(video_sink), winId());
    }
    gst_object_unref(video_sink);

    // 再生開始
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
}

// virtual
Widget::~Widget()
{
    // パイプラインの停止と解放
    if (pipeline)
    {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
    }
}

void
Widget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::RightButton)
    {
        QMenu menu(this);
        QAction* startRecordingAction = new QAction("Start Recording", this);
        QAction* stopRecordingAction = new QAction("Stop Recording", this);

        connect(startRecordingAction, &QAction::triggered, this, &Widget::startRecording);
        connect(stopRecordingAction, &QAction::triggered, this, &Widget::stopRecording);

        menu.addAction(startRecordingAction);
        menu.addAction(stopRecordingAction);

        menu.exec(event->globalPos());
    }
}

void
Widget::startRecording()
{
    if (recording)
    {
        std::cout << "Already recording." << std::endl;
        return;
    }

    // 録画ファイル名に現在の日付と時間を付加する
    std::string filename = generateFilename();

    // 録画用のファイルを作成し、GStreamerパイプラインに録画エレメントを追加
    std::string recordingPipelineDesc = "udpsrc port=5600 ! application/x-rtp, payload=96 ! "
                                        "rtph264depay ! h264parse ! mp4mux ! filesink location=" +
                                        filename;
    recording_pipeline = gst_parse_launch(recordingPipelineDesc.c_str(), nullptr);

    if (!recording_pipeline)
    {
        std::cerr << "Failed to create recording pipeline." << std::endl;
        return;
    }

    gst_element_set_state(recording_pipeline, GST_STATE_PLAYING);
    recording = true;
    std::cout << "Recording started. Saving to: " << filename << std::endl;
}

void
Widget::stopRecording()
{
    if (!recording)
    {
        std::cout << "Not recording." << std::endl;
        return;
    }

    // 録画パイプラインを停止して解放
    gst_element_set_state(recording_pipeline, GST_STATE_NULL);
    gst_object_unref(recording_pipeline);
    recording_pipeline = nullptr;
    recording = false;
    std::cout << "Recording stopped." << std::endl;
}
