import os
import cv2

title = os.path.splitext(os.path.basename(__file__))[0]
output_filename = title + '.mkv'

cap = cv2.VideoCapture(0)
fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
fps = int(cap.get(cv2.CAP_PROP_FPS))                    # カメラのFPSを取得
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))              # カメラの横幅を取得
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))             # カメラの縦幅を取得
print(fourcc, fps, w, h)

fourcc = cv2.VideoWriter_fourcc(*'X264')
print(fourcc)
out = cv2.VideoWriter(output_filename, fourcc, fps, (w, h))

while True:
    ret, frame = cap.read()
    if ret == False:
        break
    if frame is None:
        break
    out.write(frame)
    cv2.imshow(title, frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
