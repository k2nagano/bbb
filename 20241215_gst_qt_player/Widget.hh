#if !defined(MY_WIDGET_HH)
#define MY_WIDGET_HH

#include <QtWidgets>
#include <gst/gst.h>

class Widget : public QWidget
{
    Q_OBJECT
public:
    explicit Widget(QWidget* pParent = nullptr);
    virtual ~Widget();

protected:
    void mousePressEvent(QMouseEvent* event) override;

private slots:
    void startRecording();
    void stopRecording();

private:
    GstElement* pipeline;
    GstElement* recording_pipeline;
    bool recording;
};

#endif // !defined(MY_WIDGET_HH)
