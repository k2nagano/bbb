#include "Widget.hh"
// #include <QDebug>
// #include <QPainter>
// #include <QResizeEvent>
// #include <QTimer>
// #include <QWidget>
#include <cmath>
// #include <gst/gstpad.h>

// explicit
Widget::Widget(int maxRange, const std::string& rtspAddress, const std::string& mkvInput,
               QWidget* pParent)
    : QWidget(pParent), pipeline(nullptr), appsink(nullptr), max_range(maxRange), smath(120),
      mRtspAddress(rtspAddress), mMkvInput(mkvInput)
{
    if (!initializeGStreamer())
    {
        qDebug() << "Failed to initialize GStreamer pipeline.";
    }
    if (max_range > 30)
    {
        smath = 90;
    }
    if (max_range > 100)
    {
        max_range = 100;
    }
    // foreground = Qt::black;
    // background = Qt::white;
    foreground = Qt::white;
    background = Qt::black;
    QPalette palette;
    palette.setColor(QPalette::Background, background);
    setPalette(palette);
    setAutoFillBackground(true);
}

// virtual
Widget::~Widget()
{
    stopGStreamer();
}

void
Widget::paintEvent(QPaintEvent* event)
{
    if (frame_data.empty())
        return;

    QPainter painter(this);

    int frame_width = frame_data.size();
    int frame_height = frame_data[0].size();
    int widget_width = event->rect().width();
    int widget_height = event->rect().height();

    // 扇形の中心座標、半径、角度などを計算
    int margin = 20;
    int centerX = widget_width / 2;
    int centerY = widget_height - margin; // marginは画面下からのマージン
    int radius = std::min(widget_width / 2, widget_height - 2 * margin);

    // ソナーデータの範囲と角度
    int startAngle = -smath / 2;                          // 例: -60度
    double angleStep = (double)smath / (frame_width - 1); // 各ビームの角度幅

    // 扇形を描画
    painter.setPen(QPen(foreground));
    // painter.drawArc(centerX - radius, centerY - radius, radius * 2, radius * 2,
    //                 (startAngle + 90) * 16, smath * 16);

    // 角度方向のgrid線 (例: 4分割)
    for (int i = 1; i < 4; ++i)
    {
        double angle = startAngle + 90 + i * smath / 4;
        QLineF line(centerX, centerY, centerX + radius * cos(angle * M_PI / 180),
                    centerY - radius * sin(angle * M_PI / 180));
        painter.drawLine(line);
    }

    // 受信したフレームのデータを元に、各画素に対応する座標を計算し、
    // colorMapを使って色を取得して点を描画
    for (int i = 0; i < frame_width; ++i)
    {
        for (int j = 0; j < frame_height; ++j)
        {
            float angle = -smath / 2 + i * angleStep;
            float distance = (float)(j + 1) / frame_height * radius;

            // 描画する点の座標
            int drawX = centerX + distance * std::sin(angle * M_PI / 180.0);
            int drawY = centerY - distance * std::cos(angle * M_PI / 180.0);

            // 色を取得し、点を描画
            int value = frame_data[i][j];
            QColor color = getColorFromValue(value, 255);
            painter.setPen(color);
            painter.drawPoint(drawX, drawY);
        }
    }

    // 目盛り間隔の計算
    int gridInterval = getGridInterval();

    // 扇形とgrid線の描画
    painter.setPen(foreground);
    for (int i = gridInterval;; i += gridInterval)
    {
        int r = (int)((float)radius / max_range * i);
        if (r > radius)
        {
            break;
        }
        // painter.setPen(QPen(Qt::gray, 1, Qt::DashLine)); // grid線
        painter.drawArc(centerX - r, centerY - r, r * 2, r * 2, (startAngle + 90) * 16, smath * 16);

        // 目盛りの文字列を描画
        QString text = QString::number(i) + "m";
        QFont font = painter.font();
        font.setPointSize(10);
        painter.setFont(font);

        // 目盛りの位置を計算
        // double angle = startAngle + 90 + smath / 2; // 扇形の中心線
        double angle = startAngle + 90; // 扇形の中心線
        int x = centerX + r * cos(angle * M_PI / 180);
        int y = centerY - r * sin(angle * M_PI / 180);

        // 文字列の回転
        painter.save();
        painter.translate(x, y);
        painter.rotate(90 - angle); // 角度を反転
        painter.drawText(0, font.pointSize() / 2, text);
        // painter.drawText(0, 0, text);
        painter.restore();
    }
    QString text = "sonar range=" + QString::number(max_range) + "m";
    painter.drawText(20, centerY - radius - 2, text);
}
void
Widget::resizeEvent(QResizeEvent* event)
{
    update();
    QWidget::resizeEvent(event);
}
bool
Widget::initializeGStreamer()
{
    gst_init(nullptr, nullptr);

    // パイプライン作成
    if (!mMkvInput.empty())
    {
    }
    else
    {
        pipeline =
            gst_parse_launch("rtspsrc location=rtsp://127.0.0.1:8554/raw latency=0 buffer-mode=0 ! "
                             "decodebin ! videoconvert ! appsink name=mysink drop=1 sync=false",
                             nullptr);
    }
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
    g_signal_connect(appsink, "new-sample", G_CALLBACK(Widget::static_on_new_sample), this);

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
Widget::static_on_new_sample(GstAppSink* sink, gpointer user_data)
{
    Widget* self = static_cast<Widget*>(user_data);
    return self->on_new_sample(sink);
}
GstFlowReturn
Widget::on_new_sample(GstAppSink* sink)
{
    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (sample)
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
            // max_range = height / 200 * 3;

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
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
    return GST_FLOW_ERROR;
}
int
Widget::getGridInterval()
{
    if (max_range <= 10)
    {
        return 1;
    }
    else if (max_range <= 20)
    {
        return 5;
    }
    else
    {
        return 10;
    }
}
QColor
Widget::getColorFromValue(int intensity_value, int max_value)
{
    // 色相を値に比例させる
    float hue = intensity_value * 360.0f / max_value;

    // 彩度と明度を調整
    float saturation = 1.0f;
    float value = 0.8f; // 明度を少し下げることで、より暗い色合いになります

    // HSVからRGBに変換
    QColor color = QColor::fromHsvF(hue / 360.0f, saturation, value);
    return color;
}