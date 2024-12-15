#include <QApplication>
#include <QWidget>
#include <QStackedLayout>
#include <QLabel>

class TransparentOverlayWidget : public QWidget {
public:
    TransparentOverlayWidget(QWidget *parent = nullptr) : QWidget(parent) {
        // メインのレイアウト（ウィジェットを重ねるためのレイアウト）
        QStackedLayout *stackedLayout = new QStackedLayout(this);

        // 背景のウィジェット
        QWidget *backgroundWidget = new QWidget(this);
        backgroundWidget->setStyleSheet("background-color: lightblue;");

        // 前景の透明なウィジェット
        QWidget *transparentWidget = new QWidget(this);
        transparentWidget->setAttribute(Qt::WA_TranslucentBackground);  // 背景を透明に設定
        transparentWidget->setStyleSheet("background-color: rgba(0, 0, 0, 0);");  // 完全に透明

        QLabel *label = new QLabel("This is a transparent widget", transparentWidget);
        label->setStyleSheet("color: white; font-size: 20px;");
        label->setAlignment(Qt::AlignCenter);

        // ウィジェットを重ねる
        stackedLayout->addWidget(backgroundWidget);
        stackedLayout->addWidget(transparentWidget);

        // フォアグラウンドのウィジェットを前面に
        stackedLayout->setCurrentIndex(1);
    }
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    TransparentOverlayWidget window;
    window.resize(400, 300);
    window.show();

    return app.exec();
}

