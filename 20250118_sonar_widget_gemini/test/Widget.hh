#if !defined(WIDGET_HH)
#define WIDGET_HH

// #include <QTimer>
// #include <QWidget>
#include <QtWidgets>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <vector>

class Widget : public QWidget
{
public:
    explicit Widget(int maxRange = 3, const std::string& rtspAddress = "192.168.2.42",
                    const std::string& mkvInput = "", QWidget* pParent = nullptr);
    virtual ~Widget();

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    bool initializeGStreamer();
    void stopGStreamer();

    static GstFlowReturn static_on_new_sample(GstAppSink* sink, gpointer user_data);
    GstFlowReturn on_new_sample(GstAppSink* sink);

    int getGridInterval();
    QColor getColorFromValue(int intensity_value, int max_value);

private:
    GstElement* pipeline;
    GstElement* appsink;

    std::vector<std::vector<int>> frame_data;
    int max_range;
    int smath;
    QColor foreground;
    QColor background;
    std::string mRtspAddress;
    std::string mMkvInput;
};

#endif // !defined(WIDGET_HH)
