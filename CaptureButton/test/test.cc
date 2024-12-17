#include <QApplication>
#include <QPainter>
#include <QPalette>
#include <QPushButton>
#include <QSoundEffect>
#include <QTimer>

class CaptureButton : public QPushButton
{
    Q_OBJECT

public:
    explicit CaptureButton(QWidget* parent = nullptr)
        : QPushButton(parent), shutterSound(nullptr), flashTimer(nullptr), isFlashing(false)
    {
        setFixedSize(100, 100); // ボタンサイズ設定
        setText("CAP");
        setStyleSheet("background-color: blue; color: white; border-radius: 50px;");

        // シャッター音を設定
        shutterSound = new QSoundEffect(this);
        // shutterSound->setSource(QUrl::fromLocalFile("shutter.wav"));
        shutterSound->setSource(QUrl::fromLocalFile("camera-shutter-6305.mp3"));
        shutterSound->setVolume(0.5);

        // フラッシュ用タイマー設定
        flashTimer = new QTimer(this);
        flashTimer->setInterval(500); // 0.5秒間フラッシュ
        flashTimer->setSingleShot(true);

        // フラッシュ終了後に青に戻す
        connect(flashTimer, &QTimer::timeout, this, &CaptureButton::resetFlash);
    }
    // virtual ~CaptureButton()
    ~CaptureButton()
    {
    }

protected:
    void
    paintEvent(QPaintEvent* event) override
    {
        QPainter painter(this);

        // 背景色の変更（通常は青、フラッシュ中は白）
        if (isFlashing)
        {
            painter.setBrush(QColor(Qt::white));
        }
        else
        {
            painter.setBrush(QColor(Qt::blue));
        }

        painter.setPen(Qt::NoPen);
        painter.drawEllipse(0, 0, width(), height()); // 丸いボタン

        // テキストの描画
        painter.setPen(QColor(Qt::white));
        painter.setFont(QFont("Arial", 20, QFont::Bold));
        painter.drawText(rect(), Qt::AlignCenter, "CAP");
    }

    void
    mousePressEvent(QMouseEvent* event) override
    {
        QPushButton::mousePressEvent(event);

        // シャッター音を再生
        shutterSound->play();

        // 背景を白に変更してフラッシュを開始
        isFlashing = true;
        update();

        // タイマーを開始して0.5秒後に元の色に戻す
        flashTimer->start();
    }

private slots:
    void
    resetFlash()
    {
        isFlashing = false;
        update(); // 元の色に戻して再描画
    }

private:
    QSoundEffect* shutterSound; // シャッター音再生用
    QTimer* flashTimer;         // フラッシュタイマー
    bool isFlashing;            // フラッシュ状態を管理
};

int
main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    CaptureButton button;
    button.show();

    return app.exec();
}

// #include "main.moc"
