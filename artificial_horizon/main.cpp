#include <QApplication>
#include <QSlider>
#include <QVBoxLayout>
#include "horizonwidget.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    // メインウィジェット
    QWidget window;
    window.setWindowTitle("Artificial Horizon");

    // Artificial Horizon Widget
    HorizonWidget *horizonWidget = new HorizonWidget(&window);

    // ピッチ用スライダー
    QSlider *pitchSlider = new QSlider(Qt::Horizontal, &window);
    pitchSlider->setRange(-45, 45); // ピッチ角度を-45度から45度に設定
    QObject::connect(pitchSlider, &QSlider::valueChanged, horizonWidget, &HorizonWidget::setPitch);

    // ロール用スライダー
    QSlider *rollSlider = new QSlider(Qt::Horizontal, &window);
    rollSlider->setRange(-90, 90); // ロール角度を-90度から90度に設定
    QObject::connect(rollSlider, &QSlider::valueChanged, horizonWidget, &HorizonWidget::setRoll);

    // レイアウト
    QVBoxLayout *layout = new QVBoxLayout(&window);
    layout->addWidget(horizonWidget);
    layout->addWidget(pitchSlider);
    layout->addWidget(rollSlider);

    window.setLayout(layout);
    window.show();

    return app.exec();
}

