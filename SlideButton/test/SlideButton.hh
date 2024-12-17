#if !defined(SLIDE_BUTTON_HH)
#define SLIDE_BUTTON_HH

#include <QtWidgets>

class SlideButton : public QWidget
{
    Q_OBJECT

public:
    explicit SlideButton(QWidget* parent = nullptr) : QWidget(parent), isOn(false), buttonPos(0)
    {
        setFixedSize(100, 40); // ウィジェットのサイズを設定
    }

    virtual ~SlideButton()
    {
    }

protected:
    void
    paintEvent(QPaintEvent* /*event*/) override
    {
        QPainter painter(this);
        painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing |
                               QPainter::HighQualityAntialiasing);

        // スイッチ背景の描画
        if (isOn)
        {
            painter.setBrush(QColor(180, 50, 50)); // ON状態の色
        }
        else
        {
            painter.setBrush(QColor(50, 180, 50)); // OFF状態の色
        }
        painter.setPen(Qt::NoPen);
        painter.drawRoundedRect(0, 0, width(), height(), 20, 20);

        // ボタン部分の描画
        painter.setBrush(Qt::white);
        painter.drawEllipse(buttonPos, 0, 38, 38); // ボタンの位置に基づいて円を描画

        // テキストの描画
        painter.setPen(Qt::white);
        painter.setFont(QFont("Arial", 12, QFont::Bold));

        if (isOn)
        {
            painter.drawText(rect(), Qt::AlignCenter, "ON"); // ONの場合、中央にONと表示
        }
        else
        {
            painter.drawText(rect(), Qt::AlignCenter, "OFF"); // OFFの場合、中央にOFFと表示
        }
    }

    void
    mousePressEvent(QMouseEvent* /*event*/) override
    {
        toggle(); // ボタンがクリックされたらON/OFFを切り替え
    }

private slots:
    void
    toggle()
    {
        isOn = !isOn; // ON/OFF状態を切り替え

        // アニメーションの設定
        QPropertyAnimation* animation = new QPropertyAnimation(this, "buttonPos");
        animation->setDuration(200); // アニメーションの速度
        animation->setStartValue(buttonPos);
        animation->setEndValue(isOn ? width() - 30 : 0); // ONの場合は右、OFFの場合は左に動かす
        animation->start(QAbstractAnimation::DeleteWhenStopped); // アニメーション終了後に自動削除
    }

public:
    int
    getButtonPos() const
    {
        return buttonPos;
    }
    void
    setButtonPos(int pos)
    {
        buttonPos = pos;
        update(); // 再描画
    }

private:
    bool isOn;     // ON/OFF状態を保持
    int buttonPos; // ボタンの現在位置

    // ボタン位置のプロパティを定義
    Q_PROPERTY(int buttonPos READ getButtonPos WRITE setButtonPos)
};

#endif // #if !defined(SLIDE_BUTTON_HH)
