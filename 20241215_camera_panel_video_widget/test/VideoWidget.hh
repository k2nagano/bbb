#if !defined(VIDEO_WIDGET_HH)
#define VIDEO_WIDGET_HH

#include <QtWidgets>
#include <QtMultimediaWidgets>

class VideoWidget : public QVideoWidget
{
    Q_OBJECT
public:
    explicit VideoWidget(QWidget* pParent = nullptr);
    virtual ~VideoWidget();
};

#endif // !defined(VIDEO_WIDGET_HH)
