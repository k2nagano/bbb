#include "MainWindow.h"
#include <QVBoxLayout>
#include <QWidget>
#include <gst/gst.h>

MainWindow::MainWindow(QWidget *parent) 
    : QMainWindow(parent), isRecording(false), pipeline(nullptr), sink(nullptr) {
    
    // ウィジェットとレイアウトの設定
    QWidget *centralWidget = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(centralWidget);

    startButton = new QPushButton("Start Recording", this);
    stopButton = new QPushButton("Stop Recording", this);
    stopButton->setEnabled(false);  // 初期状態では停止ボタンは無効

    layout->addWidget(startButton);
    layout->addWidget(stopButton);
    setCentralWidget(centralWidget);

    // シグナルとスロットの接続
    connect(startButton, &QPushButton::clicked, this, &MainWindow::startRecording);
    connect(stopButton, &QPushButton::clicked, this, &MainWindow::stopRecording);

    // GStreamerの初期化
    gst_init(nullptr, nullptr);
    createPipeline();
}

MainWindow::~MainWindow() {
    destroyPipeline();
    gst_deinit();
}

void MainWindow::createPipeline() {
    // GStreamerパイプラインの作成
    pipeline = gst_parse_launch(
        "udpsrc port=5000 caps=\"application/x-rtp, media=video, encoding-name=H264, payload=96\" ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink name=videosink", nullptr);

    if (!pipeline) {
        g_printerr("Failed to create GStreamer pipeline\n");
        return;
    }

    // パイプラインを再生状態にする
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
}

void MainWindow::destroyPipeline() {
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        pipeline = nullptr;
    }
}

void MainWindow::startRecording() {
    if (isRecording) return;

    // mkvファイルに保存するための新しいsinkエレメントを追加
    sink = gst_parse_bin_from_description(
        "h264parse ! matroskamux ! filesink location=output.mkv", TRUE, nullptr);
    
    if (!sink) {
        g_printerr("Failed to create recording sink\n");
        return;
    }

    // パイプラインに録画用のsinkを追加
    GstElement *pipeline_sink = gst_bin_get_by_name(GST_BIN(pipeline), "videosink");
    gst_element_set_state(pipeline_sink, GST_STATE_NULL); // 表示用のシンクを一時停止
    gst_bin_add(GST_BIN(pipeline), sink);
    gst_element_link(gst_bin_get_by_name(GST_BIN(pipeline), "videoconvert"), sink);
    gst_element_set_state(sink, GST_STATE_PLAYING); // 録画開始

    isRecording = true;
    startButton->setEnabled(false);
    stopButton->setEnabled(true);
}

void MainWindow::stopRecording() {
    if (!isRecording) return;

    // 録画を終了してファイルを閉じる
    gst_element_set_state(sink, GST_STATE_NULL);
    gst_bin_remove(GST_BIN(pipeline), sink);
    gst_object_unref(sink);
    sink = nullptr;

    // 表示用のシンクを再度有効化
    GstElement *pipeline_sink = gst_bin_get_by_name(GST_BIN(pipeline), "videosink");
    gst_element_set_state(pipeline_sink, GST_STATE_PLAYING);

    isRecording = false;
    startButton->setEnabled(true);
    stopButton->setEnabled(false);
}

