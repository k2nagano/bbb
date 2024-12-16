#if !defined(CELL_WIDGET_HH)
#define CELL_WIDGET_HH

#include <QWidget>

class CellWidget : public QWidget
{
public:
    explicit CellWidget(QWidget* pParent = nullptr);
    virtual ~CellWidget();
};

#endif // #if !defined(CELL_WIDGET_HH)