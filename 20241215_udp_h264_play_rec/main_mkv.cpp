#include <gst/gst.h>
#include <iostream>

GstElement *pipeline;
GMainLoop *loop;

// EOSを送信してパイプラインを停止するコールバック関数
static gboolean stop_pipeline(gpointer data) {
    std::cout << "Stopping pipeline..." << std::endl;
    
    // EOS (End of Stream) を送信してファイルを正しく終了させる
    gst_element_send_event(pipeline, gst_event_new_eos());
    return FALSE;
}

// EOS受信後にパイプラインを終了するコールバック
static gboolean handle_bus_message(GstBus *bus, GstMessage *message, gpointer data) {
    switch (GST_MESSAGE_TYPE(message)) {
        case GST_MESSAGE_EOS:
            std::cout << "EOS received, stopping main loop..." << std::endl;
            g_main_loop_quit(loop);
            break;
        case GST_MESSAGE_ERROR:
            GError *err;
            gchar *debug;
            gst_message_parse_error(message, &err, &debug);
            std::cerr << "Error: " << err->message << std::endl;
            g_error_free(err);
            g_free(debug);
            g_main_loop_quit(loop);
            break;
        default:
            break;
    }
    return TRUE;
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    // パイプラインの作成
    pipeline = gst_parse_launch(
        "udpsrc port=5000 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! tee name=t "
        "t. ! queue ! avdec_h264 ! videoconvert ! autovideosink "
        "t. ! queue ! matroskamux ! filesink location=output.mkv",
        nullptr);

    if (!pipeline) {
        std::cerr << "Failed to create pipeline." << std::endl;
        return -1;
    }

    // パイプラインを再生状態に設定
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // メインループの作成
    loop = g_main_loop_new(nullptr, FALSE);

    // バスのメッセージを処理する
    GstBus *bus = gst_element_get_bus(pipeline);
    gst_bus_add_watch(bus, handle_bus_message, nullptr);

    // 10秒後に録画を停止するためのタイマー
    g_timeout_add_seconds(10, stop_pipeline, nullptr);

    // メインループを開始
    g_main_loop_run(loop);

    // パイプラインの停止
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(bus);
    gst_object_unref(pipeline);
    g_main_loop_unref(loop);

    return 0;
}

