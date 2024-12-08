#if !defined(WIDGET_HH)
#define WIDGET_HH

#include <QtWidgets>
#include <gst/gst.h>
#include <gst/video/videooverlay.h>

class StreamingWidget : public QWidget
{
    Q_OBJECT
public:
    StreamingWidget(QWidget* parent = nullptr);
    ~StreamingWidget();

protected:
    void showContextMenu(const QPoint& pos);
    void paintEvent(QPaintEvent* event) override;

private slots:
    void startRecording();
    void stopRecording();
    void captureScreenshot();
    void toggleFullScreen();
    void updateOverlay();

private:
    GstElement* gstPipeline;
    GstElement* gstSink;
    QTimer* overlayTimer;
};

#endif // !defined(WIDGET_HH)
