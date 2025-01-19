import cv2

# 出力ファイル名とコーデック設定
output_file = 'output.mkv'
fourcc = cv2.VideoWriter_fourcc(*'X264')

# フレームサイズ（640x480）、FPS（20.0）
out = cv2.VideoWriter(output_file, fourcc, 20.0, (640, 480))

# カメラからキャプチャする（またはファイルから読み込む）
cap = cv2.VideoCapture(0)

while (cap.isOpened()):
    ret, frame = cap.read()
    if ret:
        # フレームを書き込む
        out.write(frame)
        cv2.imshow('frame', frame)

        # 'q'キーで終了
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# 終了処理
cap.release()
out.release()
cv2.destroyAllWindows()
