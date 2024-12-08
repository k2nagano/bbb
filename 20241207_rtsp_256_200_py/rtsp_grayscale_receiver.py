#!/usr/bin/env python3

import gi
import sys

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# GStreamerの初期化
Gst.init(None)

# RTSPストリームからデータを取得するためのGStreamerパイプラインを作成
pipeline_str = (
    "rtspsrc location=rtsp://localhost:8554/test ! decodebin ! videoconvert "
    "! video/x-raw,format=GRAY8,width=256,height=200 ! appsink name=sink"
)

# パイプラインの作成
pipeline = Gst.parse_launch(pipeline_str)

# appsinkを取得
appsink = pipeline.get_by_name("sink")

# appsinkのプロパティ設定
appsink.set_property("emit-signals", True)
appsink.set_property("sync", False)

# フレームデータを取得したときに呼ばれるコールバック
def on_new_sample(sink):
    sample = sink.emit("pull-sample")
    if not sample:
        return Gst.FlowReturn.ERROR

    # バッファ（映像データ）の取得
    buf = sample.get_buffer()
    caps = sample.get_caps()

    # 256x200サイズの映像フレームを取得
    success, map_info = buf.map(Gst.MapFlags.READ)
    if success:
        data = map_info.data  # グレースケールの画素データ
        width = 256
        height = 200

        # 各ピクセルのグレースケール値（0-255）を表示
        for y in range(height):
            for x in range(width):
                pixel_value = data[y * width + x]
                print(f"Pixel ({x}, {y}): {pixel_value}")
        
        buf.unmap(map_info)
    
    return Gst.FlowReturn.OK

# コールバックの接続
appsink.connect("new-sample", on_new_sample)

# パイプラインの再生開始
pipeline.set_state(Gst.State.PLAYING)

# メインループの開始
loop = GLib.MainLoop()

try:
    loop.run()
except KeyboardInterrupt:
    pass

# パイプラインの停止
pipeline.set_state(Gst.State.NULL)

