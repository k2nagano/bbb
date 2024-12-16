#if !defined(WIDGET_HH)
#define WIDGET_HH

#include <QtWidgets>
#include <gst/gst.h>

class Widget : public QWidget
{
    Q_OBJECT
public:
    explicit Widget(QWidget* pParent = nullptr);
    virtual ~Widget();

protected:
    void contextMenuEvent(QContextMenuEvent *event) override;
    void resizeEvent(QResizeEvent* event) override;
    void showEvent(QShowEvent* event) override;

private:
    void initPipeline();
    void startPipeline();
    void pausePipeline();
    void stopPipeline();

private:
    GstElement* pipeline;
};

#endif // !defined(WIDGET_HH)
