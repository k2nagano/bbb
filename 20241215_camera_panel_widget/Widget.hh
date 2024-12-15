#if !defined(WIDGET_HH)
#define WIDGET_HH

#include <QtWidgets>
#include <gst/gst.h>

class Widget : public QWidget
{
public:
    explicit Widget(QWidget* pParent = nullptr);
    virtual ~Widget();

protected:
    void contextMenuEvent(QContextMenuEvent *event) override;

protected:
    void resizeEvent(QResizeEvent* event) override;
    void showEvent(QShowEvent* event) override;

private:
    void startStreaming();
    void stopStreaming();
    void pauseStreaming();
    void openSettings();
    void saveToMKV();
    void startRecording();
    void stopRecording();

private:
    GstElement* pipeline;
    GstElement *sink;
    bool isRecording;
    bool playing;
    int mUdpPort;
};

#endif // !defined(WIDGET_HH)
