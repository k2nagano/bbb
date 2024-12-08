#if !defined(MY_WIDGET_HH)
#define MY_WIDGET_HH

#include <QtWidgets>

class Widget : public QWidget
{
public:
    explicit Widget(QWidget* pParent = nullptr);
    virtual ~Widget();

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    void drawLineAtAngle(QPainter& painter, qreal angle, qreal radius1, qreal radius2);
    void drawAxes(QPainter& painter);
};

#endif // !defined(MY_WIDGET_HH)
