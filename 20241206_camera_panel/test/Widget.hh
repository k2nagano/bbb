#if !defined(MY_WIDGET_HH)
#define MY_WIDGET_HH

#include <QtWidgets>

class Widget : public QWidget
{
public:
    explicit Widget(QWidget* pParent = nullptr);
    virtual ~Widget();
};

#endif // !defined(MY_WIDGET_HH)
