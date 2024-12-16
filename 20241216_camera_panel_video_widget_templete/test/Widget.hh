#if !defined(WIDGET_HH)
#define WIDGET_HH

#include <QtWidgets>

class Widget : public QWidget
{
public:
    explicit Widget(QWidget* pParent = nullptr);
    virtual ~Widget();
};

#endif // !defined(WIDGET_HH)
