#include "mainwindow.h"
#include <QVBoxLayout>
#include <QMenu>
#include <QDebug>
#include <QAction>
#include <QMessageBox>
#include <QInputDialog>

MainWindow::MainWindow(QWidget *parent)
    : QWidget(parent), pipeline(nullptr), isStreaming(false), udpPort(5000)
{
    // レイアウト設定
    QVBoxLayout *layout = new QVBoxLayout(this);
    setLayout(layout);

    // GStreamer初期化
    gst_init(nullptr, nullptr);
}

MainWindow::~MainWindow() {
    stopGStreamer();
}

void MainWindow::contextMenuEvent(QContextMenuEvent *event) {
    QMenu contextMenu(this);
    QAction *startAction = contextMenu.addAction("Start Streaming");
    QAction *pauseAction = contextMenu.addAction("Pause Streaming");
    QAction *stopAction = contextMenu.addAction("Stop Streaming");
    QAction *settingsAction = contextMenu.addAction("Settings");

    // スロットをアクションに接続
    connect(startAction, &QAction::triggered, this, &MainWindow::startStreaming);
    connect(pauseAction, &QAction::triggered, this, &MainWindow::pauseStreaming);
    connect(stopAction, &QAction::triggered, this, &MainWindow::stopStreaming);
    connect(settingsAction, &QAction::triggered, this, &MainWindow::openSettings);

    // コンテキストメニューを表示
    contextMenu.exec(event->globalPos());
}

void MainWindow::startStreaming() {
    qDebug() << "Starting streaming...";

    // GStreamerのパイプラインを作成して、H.264ストリーミングを受信
    if (!isStreaming) {
        initGStreamer();
        isStreaming = true;
    } else {
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
    }
}

void MainWindow::pauseStreaming() {
    qDebug() << "Pausing streaming...";
    if (pipeline && isStreaming) {
        gst_element_set_state(pipeline, GST_STATE_PAUSED);
    }
}

void MainWindow::stopStreaming() {
    qDebug() << "Stopping streaming...";
    stopGStreamer();
    isStreaming = false;
}

void MainWindow::openSettings() {
    qDebug() << "Opening settings...";
    
    // 設定ダイアログを表示してUDPポートを設定
    SettingsDialog dialog(udpPort, this);
    if (dialog.exec() == QDialog::Accepted) {
        udpPort = dialog.getPort();
        qDebug() << "New UDP port set to:" << udpPort;

        // ポート変更後にストリーミングを再開
        if (isStreaming) {
            stopStreaming();
            startStreaming();
        }
    }
}

void MainWindow::initGStreamer() {
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
    }

    // GStreamerのパイプライン構築 (UDPポートを設定)
    QString pipelineStr = QString("udpsrc port=%1 ! application/x-rtp, payload=96 ! "
                                  "rtph264depay ! avdec_h264 ! videoconvert ! autovideosink").arg(udpPort);
    
    pipeline = gst_parse_launch(pipelineStr.toUtf8().constData(), nullptr);

    // パイプライン開始
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
}

void MainWindow::stopGStreamer() {
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        pipeline = nullptr;
    }
}

// SettingsDialogの実装
SettingsDialog::SettingsDialog(int currentPort, QWidget *parent)
    : QDialog(parent), port(currentPort)
{
    setWindowTitle("Settings");

    // UDPポートの入力
    bool ok;
    int newPort = QInputDialog::getInt(this, "UDP Port Setting", "Set UDP Port:", port, 1024, 65535, 1, &ok);
    if (ok) {
        port = newPort;
    }
}

int SettingsDialog::getPort() const {
    return port;
}

