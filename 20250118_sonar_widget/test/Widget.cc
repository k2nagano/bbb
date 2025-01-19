#include "Widget.hh"
#include <QDebug>
#include <QPainter>
#include <QResizeEvent>
#include <QTimer>
#include <cmath>
#include <gst/gst.h>

Widget::Widget(QWidget* parent)
    : QWidget(parent), max_range(3), smath(120), pipeline(nullptr), appsink(nullptr)
{
    // GStreamerの初期化
    if (!initializeGStreamer())
    {
        qDebug() << "Failed to initialize GStreamer pipeline.";
    }
}

Widget::~Widget()
{
    stopGStreamer();
}

bool
Widget::initializeGStreamer()
{
    gst_init(nullptr, nullptr);

    // パイプライン作成
    pipeline =
        gst_parse_launch("rtspsrc location=rtsp://127.0.0.1:8554/raw latency=0 buffer-mode=0 ! "
                         "decodebin ! videoconvert ! appsink name=mysink drop=1 sync=false",
                         nullptr);

    if (!pipeline)
    {
        qDebug() << "Failed to create GStreamer pipeline.";
        return false;
    }

    // appsinkの取得
    appsink = gst_bin_get_by_name(GST_BIN(pipeline), "mysink");
    if (!appsink)
    {
        qDebug() << "Failed to get appsink from pipeline.";
        gst_object_unref(pipeline);
        pipeline = nullptr;
        return false;
    }

    // appsinkの設定
    gst_app_sink_set_emit_signals((GstAppSink*)appsink, true);
    g_signal_connect(appsink, "new-sample", G_CALLBACK(Widget::on_new_sample), this);

    // パイプラインの再生
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    return true;
}

void
Widget::stopGStreamer()
{
    if (pipeline)
    {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        pipeline = nullptr;
    }
}

GstFlowReturn
Widget::on_new_sample(GstAppSink* sink, gpointer user_data)
{
    Widget* self = static_cast<Widget*>(user_data);
    GstSample* sample = gst_app_sink_pull_sample(sink);

    if (sample)
    {
        self->processFrame(sample);
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    return GST_FLOW_ERROR;
}

void
Widget::processFrame(GstSample* sample)
{
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;

    if (gst_buffer_map(buffer, &map, GST_MAP_READ))
    {
        // バッファから生データを取得
        unsigned char* data = (unsigned char*)map.data;
        size_t data_size = map.size; // データ全体のバイトサイズ

        // frame_data のサイズを推定して設定
        int width = 256; // 固定のビーム数

        // I420フォーマットに基づいて高さを計算
        int height = static_cast<int>(data_size * 2 / (width * 3));

        // frame_data のサイズを設定
        frame_data.resize(width, std::vector<int>(height, 0));

        // 受け取ったデータを frame_data に変換 (Yプレーンのみ使用する想定)
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                // Yプレーンのデータのみを使用
                frame_data[i][j] = data[i + j * width];
            }
        }

        // 描画の更新を要求
        update();

        gst_buffer_unmap(buffer, &map);
    }
}

void
Widget::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int widget_width = width();
    int widget_height = height();

    // マージンの設定
    int margin = 20;
    int center_x = widget_width / 2;
    int center_y = widget_height - margin;

    // 扇形の半径
    int radius = std::min(widget_width / 2, widget_height - 2 * margin);

    // 扇形の描画
    drawSonarData(painter, center_x, center_y, radius);
    drawGuidelines(painter, center_x, center_y, radius);
    drawRangeLabels(painter, center_x, center_y, radius);
}

void
Widget::resizeEvent(QResizeEvent* event)
{
    // リサイズ時の再描画
    update();
    QWidget::resizeEvent(event);
}

// ソナーデータの描画
void
Widget::drawSonarData(QPainter& painter, int cx, int cy, int radius)
{
    if (frame_data.empty())
        return;

    int width = frame_data.size();
    int height = frame_data[0].size();

    // 扇形の角度幅（度）
    float angle_range = smath;
    float angle_step = angle_range / width;

    // 各ビームごとに描画
    for (int i = 0; i < width; ++i)
    {
        for (int j = 0; j < height; ++j)
        {
            // 強度をカラーにマッピング
            int intensity = frame_data[i][j];
            QColor color = getColorFromIntensity(intensity);

            // 各ビームの角度と距離
            float angle = -angle_range / 2 + i * angle_step;
            float distance = (float)j / height * radius;

            // 描画する点の座標
            int x = cx + distance * std::sin(angle * M_PI / 180.0);
            int y = cy - distance * std::cos(angle * M_PI / 180.0);

            // 強度に応じた点を描画
            painter.setPen(color);
            painter.drawPoint(x, y);
        }
    }
}

// 補助線や円弧の描画
void
Widget::drawGuidelines(QPainter& painter, int cx, int cy, int radius)
{
    painter.setPen(Qt::white);

    // 扇形の半径に応じてガイドラインを描画
    int step = radius / max_range;
    for (int i = 1; i <= max_range; ++i)
    {
        int r = i * step;
        painter.drawArc(cx - r, cy - r, r * 2, r * 2, 0, 180 * 16);
    }

    // 中心から扇形の両端に向かう直線を描画
    float angle_range = smath;
    float left_angle = -angle_range / 2;
    float right_angle = angle_range / 2;

    drawAngleLine(painter, cx, cy, radius, left_angle);
    drawAngleLine(painter, cx, cy, radius, right_angle);
}

// 扇形のガイドラインを描画する補助関数
void
Widget::drawAngleLine(QPainter& painter, int cx, int cy, int radius, float angle)
{
    int x = cx + radius * std::sin(angle * M_PI / 180.0);
    int y = cy - radius * std::cos(angle * M_PI / 180.0);
    painter.drawLine(cx, cy, x, y);
}

// 距離ラベルの描画
void
Widget::drawRangeLabels(QPainter& painter, int cx, int cy, int radius)
{
    // painter.setPen(Qt::white);
    painter.setPen(Qt::black);
    int step = radius / max_range;
    for (int i = 1; i <= max_range; ++i)
    {
        int r = i * step;
        // QString label = QString("%1m").arg(i * 3); // 例として3m単位のラベルを描画
        QString label = QString("%1m").arg(i);
        painter.drawText(cx - r, cy - r, label);
    }
}

// 強度に応じたカラーを返す
QColor
Widget::getColorFromIntensity(int intensity)
{
    // 簡単なrainbow colormapの実装 (0~255の強度に基づく)
    int r = std::min(255, std::max(0, 255 - intensity * 2));
    int g = std::min(255, std::max(0, intensity * 2));
    int b = 255 - std::abs(intensity - 128) * 2;
    return QColor(r, g, b);
}
