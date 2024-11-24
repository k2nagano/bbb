#ifndef HORIZONWIDGET_H
#define HORIZONWIDGET_H

#include <QWidget>

class HorizonWidget : public QWidget
{
    Q_OBJECT

public:
    explicit HorizonWidget(QWidget* parent = nullptr);

    // ピッチとロールの設定関数
    void setPitch(float pitch); // 度単位でピッチ
    void setRoll(float roll);   // 度単位でロール

protected:
    // 描画のために再実装
    void paintEvent(QPaintEvent* event) override;

private:
    void drawRollScale(QPainter& painter, const QPoint& center);
    void drawPitchScale(QPainter& painter, const QPoint& center);

private:
    float m_pitch; // ピッチ角度
    float m_roll;  // ロール角度
};

#endif // HORIZONWIDGET_H
