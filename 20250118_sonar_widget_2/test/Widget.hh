#ifndef WIDGET_HH
#define WIDGET_HH

#include <QWidgets>
// #include <QTimer>
#include <vector>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

class Widget : public QWidget {
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    ~Widget();

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    // GStreamer用
    GstElement *pipeline;
    GstElement *appsink;

    // ソナーデータ
    std::vector<std::vector<int>> frame_data;
    int max_range;
    int smath;

    QColor foreground;
    QColor background;

    // GStreamer関連の初期化と停止
    bool initializeGStreamer();
    void stopGStreamer();

    // GStreamerのフレーム取得
    static GstFlowReturn on_new_sample(GstAppSink *sink, gpointer user_data);
    void processFrame(GstSample *sample);

    // ソナーの描画処理
    void drawSonarData(QPainter &painter, int cx, int cy, int radius);
    void drawGuidelines(QPainter &painter, int cx, int cy, int radius);
    void drawRangeLabels(QPainter &painter, int cx, int cy, int radius);
    void drawAngleLine(QPainter &painter, int cx, int cy, int radius, float angle);

    // ヘルパー関数
    QColor getColorFromIntensity(int intensity);
};

#endif // WIDGET_HH

