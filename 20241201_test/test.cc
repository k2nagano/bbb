#include <QtWidgets>

class Widget : public QWidget
{
public:
    Widget(QWidget* parent)
    {
    }
    ~Widget()
    {
    }
};

int
main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    QWidget w;
    w.show();
    return a.exec();
}
