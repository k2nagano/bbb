#if !defined(WIDGET_HH)
#define WIDGET_HH

#include <QtWidgets>

class VideoWidget;
class QMediaPlayer;
class Widget : public QWidget
{
    Q_OBJECT
public:
    explicit Widget(QWidget* pParent = nullptr);
    virtual ~Widget();
private:
    VideoWidget* mpVideoWidget;
    QMediaPlayer* mpMediaPlayer;
};

#endif // !defined(WIDGET_HH)
