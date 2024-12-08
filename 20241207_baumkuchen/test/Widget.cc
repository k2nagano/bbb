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

    // デカルト座標系の原点を中央下に移動
    QTransform transform;
    transform.translate(width() / 2, height() - 20); // 原点をウィンドウの中央下部に設定
    painter.setTransform(transform);

    // 複数の半径の円弧を描画
    qreal radii[] = {100, 120, 140, 160}; // 半径
    int startAngle = -60 * 16;            // 左右対称に-60度から
    int spanAngle = 120 * 16;             // 120度の円弧

    QPen pen;
    pen.setWidth(3);
    painter.setPen(pen);
    painter.setBrush(Qt::transparent);

    // 各半径に対して円弧を描画
    for (qreal radius : radii)
    {
        painter.drawArc(-radius, -radius, radius * 2, radius * 2, startAngle, spanAngle);
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

    // x軸
    painter.drawLine(-width() / 2, 0, width() / 2, 0);

    // y軸
    painter.drawLine(0, height() - 20, 0, -height());
}