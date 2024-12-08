#include "Widget.hh"

// explicit
Widget::Widget(QWidget* pParent) : QWidget(pParent)
{
}

// virtual
Widget::~Widget()
{
}

void
Widget::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    qreal width = event->rect().width();
    qreal height = event->rect().height();
    // 原点を中央下部に移動
    QTransform transform;
    transform.translate(width / 2, height - 20); // ウィジェットの中央下が原点になるように設定
    painter.setTransform(transform);

    painter.save();
    painter.rotate(-90);

    // 円弧の半径
    // qreal radii[] = {100, 120, 140, 160}; // 半径の配列
    qreal radii[] = {height*0.2, height*0.4, height*0.6, height*0.8}; // 半径の配列
    int startAngle = 0;                   // 0度を中心として
    int spanAngle = 120 * 16;             // 合計120度

    QPen pen;
    pen.setWidth(3);
    painter.setPen(pen);
    painter.setBrush(Qt::transparent);

    // 半径ごとに円弧を描画
    for (qreal radius : radii)
    {
        // -60度から+60度にわたって円弧を描画
        painter.drawArc(-radius, -radius, radius * 2, radius * 2, startAngle - 60 * 16, spanAngle);
    }

    // 円弧の両端の座標を計算
    QPointF start(radii[0] * qCos(qDegreesToRadians(-60.0)),
                  -radii[0] * qSin(qDegreesToRadians(-60.0)));
    QPointF end(radii[0] * qCos(qDegreesToRadians(60.0)),
                -radii[0] * qSin(qDegreesToRadians(60.0)));

    QPointF startOuter(radii[3] * qCos(qDegreesToRadians(-60.0)),
                       -radii[3] * qSin(qDegreesToRadians(-60.0)));
    QPointF endOuter(radii[3] * qCos(qDegreesToRadians(60.0)),
                     -radii[3] * qSin(qDegreesToRadians(60.0)));

    // 円弧の両端を直線で結ぶ
    painter.drawLine(start, startOuter);
    painter.drawLine(end, endOuter);

    // 円弧の両端に "000" を描画
    painter.drawText(start, "000");
    painter.drawText(end, "000");

    // バームクーヘン内に0度、-30度、30度の直線を描画
    drawLineAtAngle(painter, 0, radii[0], radii[3]);   // 0度
    drawLineAtAngle(painter, -30, radii[0], radii[3]); // -30度
    drawLineAtAngle(painter, 30, radii[0], radii[3]);  // 30度

    painter.restore();

    // x軸とy軸の描画
    drawAxes(painter);
}

void
Widget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    update(); // 再描画をリクエスト
}

void
Widget::drawLineAtAngle(QPainter& painter, qreal angle, qreal radius1, qreal radius2)
{
    QPointF start(radius1 * qCos(qDegreesToRadians(angle)),
                  -radius1 * qSin(qDegreesToRadians(angle)));
    QPointF end(radius2 * qCos(qDegreesToRadians(angle)),
                -radius2 * qSin(qDegreesToRadians(angle)));
    painter.drawLine(start, end);
}

void
Widget::drawAxes(QPainter& painter)
{
    // x軸とy軸の描画
    QPen penAxes(Qt::DashLine);
    penAxes.setColor(Qt::black);
    penAxes.setWidth(2);
    painter.setPen(penAxes);

    // x軸 (左右)
    painter.drawLine(-width() / 2, 0, width() / 2, 0);

    // y軸 (上方向)
    painter.drawLine(0, height() - 20, 0, -height());
}