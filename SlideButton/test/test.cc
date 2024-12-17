#include <QApplication>
#include <QVBoxLayout>
#include "SlideButton.hh"  // カスタムウィジェットのヘッダー

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    QWidget window;
    QVBoxLayout *layout = new QVBoxLayout(&window);

    SlideButton *slideButton = new SlideButton();
    layout->addWidget(slideButton);

    window.setLayout(layout);
    window.show();

    return app.exec();
}

