#ifndef VIDEOWIDGET_H
#define VIDEOWIDGET_H

#include <QWidget>
#include <gst/gst.h>
#include <gst/video/videooverlay.h>

class VideoWidget : public QWidget {
    Q_OBJECT
public:
    explicit VideoWidget(QWidget *parent = nullptr);
    ~VideoWidget();

    // GStreamerパイプラインの初期化
    bool initializePipeline(const QString &udpPort);

protected:
    // ウィンドウIDを取得してGStreamerに渡すために再実装
    void resizeEvent(QResizeEvent *event) override;
    void showEvent(QShowEvent *event) override;

private:
    GstElement *pipeline;
};

#endif // VIDEOWIDGET_H

