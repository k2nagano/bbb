#if !defined(VIDEO_WIDGET_HH)
#define VIDEO_WIDGET_HH

#include <QVideoWidget>

class VideoWidget : public QVideoWidget
{
public:
    explicit VideoWidget(QWidget* pParent = nullptr);
    virtual ~VideoWidget();
};

#endif // #if !defined(VIDEO_WIDGET_HH)