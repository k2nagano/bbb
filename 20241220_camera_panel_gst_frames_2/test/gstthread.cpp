#include "gstthread.h"
#include <QImage>
#include <gst/app/gstappsink.h>

GstThread::GstThread(QObject* parent) : QThread(parent), stop(false)
{
    gst_init(nullptr, nullptr);
}

GstThread::~GstThread()
{
    stop = true;
    if (pipeline)
    {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
    }
}

void
GstThread::run()
{
    // GStreamer パイプラインの作成
    const char* pipelineDesc = "udpsrc port=5600 ! application/x-rtp, payload=96 ! "
                               "rtph264depay ! avdec_h264 ! videoconvert ! appsink name=appsink";

    GError* error = nullptr;
    pipeline = gst_parse_launch(pipelineDesc, &error);
    if (!pipeline)
    {
        qCritical("Failed to create pipeline: %s", error->message);
        g_error_free(error);
        return;
    }

    GstElement* appsink = gst_bin_get_by_name(GST_BIN(pipeline), "appsink");
    gst_app_sink_set_emit_signals((GstAppSink*)appsink, true);
    gst_app_sink_set_drop((GstAppSink*)appsink, true);
    g_signal_connect(appsink, "new-sample", G_CALLBACK(onNewSample), this);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    bus = gst_element_get_bus(pipeline);
    while (!stop)
    {
        GstMessage* msg = gst_bus_timed_pop_filtered(
            bus, 100 * GST_MSECOND, (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

        if (msg != nullptr)
        {
            GError* err;
            gchar* debugInfo;
            switch (GST_MESSAGE_TYPE(msg))
            {
            case GST_MESSAGE_ERROR:
                gst_message_parse_error(msg, &err, &debugInfo);
                qCritical("Error received from element %s: %s", GST_OBJECT_NAME(msg->src),
                          err->message);
                g_clear_error(&err);
                g_free(debugInfo);
                stop = true;
                break;
            case GST_MESSAGE_EOS:
                qInfo("End of stream");
                stop = true;
                break;
            default:
                break;
            }
            gst_message_unref(msg);
        }
    }

    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
}

gboolean
GstThread::onNewSample(GstElement* sink, GstThread* thread)
{
    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (sample)
    {
        QImage image = thread->gstSampleToQImage(sample);
        gst_sample_unref(sample);

        if (!image.isNull())
        {
            emit thread->newFrame(image);
        }
    }
    return TRUE;
}

QImage GstThread::gstSampleToQImage(GstSample *sample)
{
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *s = gst_caps_get_structure(caps, 0);

    const gchar *format = gst_structure_get_string(s, "format");
    int width, height;
    gst_structure_get_int(s, "width", &width);
    gst_structure_get_int(s, "height", &height);

    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    QImage image;
    if (g_strcmp0(format, "I420") == 0) {
        // Convert I420 to RGB
        image = QImage(width, height, QImage::Format_RGB888);
        const uint8_t *yPlane = map.data;
        const uint8_t *uPlane = yPlane + width * height;
        const uint8_t *vPlane = uPlane + (width * height) / 4;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int yIndex = y * width + x;
                int uIndex = (y / 2) * (width / 2) + (x / 2);
                int vIndex = (y / 2) * (width / 2) + (x / 2);

                uint8_t Y = yPlane[yIndex];
                uint8_t U = uPlane[uIndex];
                uint8_t V = vPlane[vIndex];

                int C = Y - 16;
                int D = U - 128;
                int E = V - 128;

                uint8_t R = qBound(0, (298 * C + 409 * E + 128) >> 8, 255);
                uint8_t G = qBound(0, (298 * C - 100 * D - 208 * E + 128) >> 8, 255);
                uint8_t B = qBound(0, (298 * C + 516 * D + 128) >> 8, 255);

                image.setPixel(x, y, qRgb(R, G, B));
            }
        }
    } else if (g_strcmp0(format, "Y42B") == 0) {
        // Convert Y42B to RGB
        image = QImage(width, height, QImage::Format_RGB888);
        const uint8_t *yPlane = map.data;
        const uint8_t *uPlane = yPlane + width * height;
        const uint8_t *vPlane = uPlane + (width * height) / 2;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int yIndex = y * width + x;
                int uIndex = (y * width + x) / 2;
                int vIndex = (y * width + x) / 2;

                uint8_t Y = yPlane[yIndex];
                uint8_t U = uPlane[uIndex];
                uint8_t V = vPlane[vIndex];

                int C = Y - 16;
                int D = U - 128;
                int E = V - 128;

                uint8_t R = qBound(0, (298 * C + 409 * E + 128) >> 8, 255);
                uint8_t G = qBound(0, (298 * C - 100 * D - 208 * E + 128) >> 8, 255);
                uint8_t B = qBound(0, (298 * C + 516 * D + 128) >> 8, 255);

                image.setPixel(x, y, qRgb(R, G, B));
            }
        }
    } else if (g_strcmp0(format, "RGB") == 0) {
        image = QImage(map.data, width, height, QImage::Format_RGB888);
    } else {
        qWarning("Unsupported format: %s", format);
        gst_buffer_unmap(buffer, &map);
        return QImage();
    }

    gst_buffer_unmap(buffer, &map);

    return image.copy();
}
