#include "horizonwidget.h"
#include <QPainter>
#include <QTransform>
#include <QDebug>
#include <cmath>

#define qDegreesToRadians(deg) (deg/180.0*M_PI)

HorizonWidget::HorizonWidget(QWidget *parent)
    : QWidget(parent), m_pitch(0), m_roll(0)
{
    setMinimumSize(300, 300);
}

void HorizonWidget::setPitch(float pitch)
{
    m_pitch = pitch;
    update();
}

void HorizonWidget::setRoll(float roll)
{
    m_roll = roll;
    update();
}

void HorizonWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    int w = width();
    int h = height();
    QPoint center(w / 2, h / 2);

    // 背景を描画
    painter.setBrush(QBrush(Qt::gray));
    painter.drawRect(0, 0, w, h);

    // ロールを反映して変換
    QTransform transform;
    transform.translate(center.x(), center.y());
    transform.rotate(m_roll);  // ロール角度を反映
    painter.setTransform(transform);

    // 空部分
    painter.setBrush(QBrush(Qt::blue));
    painter.drawRect(-w, -h, 2 * w, h + (m_pitch * 2));

    // 地面部分
    painter.setBrush(QBrush(Qt::green));
    painter.drawRect(-w, h + (m_pitch * 2), 2 * w, 2 * h);

    // 地平線の描画
    painter.setPen(QPen(Qt::white, 2));
    painter.drawLine(-w, 0, w, 0);

    // 中央の水平バー
    painter.resetTransform();
    painter.setPen(QPen(Qt::white, 3));
    painter.drawLine(center.x() - 50, center.y(), center.x() + 50, center.y());

    // ロールの目盛りを描画
    drawRollScale(painter, center);

    // ピッチの目盛りを描画
    drawPitchScale(painter, center);
}

void HorizonWidget::drawRollScale(QPainter &painter, const QPoint &center)
{
    // ロール目盛りの描画 (中心の外側に円弧状に描画)
    painter.setPen(QPen(Qt::white, 2));
    const int radius = 120; // 目盛りの半径
    const int tickLength = 10; // 目盛りの長さ

    // ロール角度 (-90° から +90° まで)
    for (int angle = -90; angle <= 90; angle += 10) {
        float rad = qDegreesToRadians(static_cast<float>(angle));
        int x1 = center.x() + std::cos(rad) * (radius - tickLength);
        int y1 = center.y() - std::sin(rad) * (radius - tickLength);
        int x2 = center.x() + std::cos(rad) * radius;
        int y2 = center.y() - std::sin(rad) * radius;

        painter.drawLine(x1, y1, x2, y2);

        // 主な目盛りに角度を表示
        if (angle % 30 == 0) {
            QString angleText = QString::number(angle);
            int textX = center.x() + std::cos(rad) * (radius + 15);
            int textY = center.y() - std::sin(rad) * (radius + 15);
            painter.drawText(textX - 10, textY + 5, angleText);
        }
    }
}

void HorizonWidget::drawPitchScale(QPainter &painter, const QPoint &center)
{
    // ピッチ目盛りの描画 (上下に線を引く)
    painter.setPen(QPen(Qt::white, 2));

    // ピッチ角度 (-45° から +45° まで)
    for (int angle = -45; angle <= 45; angle += 5) {
        int yOffset = angle * 2; // 目盛りの間隔を調整
        int xStart = center.x() - 30;
        int xEnd = center.x() + 30;

        // 主な角度 (-45°, -30°, ..., 30°, 45°)
        if (angle % 10 == 0) {
            painter.drawLine(xStart, center.y() - yOffset, xEnd, center.y() - yOffset);
            painter.drawText(center.x() + 40, center.y() - yOffset + 5, QString::number(angle));
        } else {
            // 小さな目盛り
            painter.drawLine(center.x() - 15, center.y() - yOffset, center.x() + 15, center.y() - yOffset);
        }
    }
}

