#include "gamepad_widget.h"
#include <algorithm>

GamepadWidget::GamepadWidget(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(180);
}

void GamepadWidget::setSample(const GamepadSample& s) {
    sample_ = s;
    pushHistory();
    update();
}

void GamepadWidget::paintEvent(QPaintEvent* e) {
    QWidget::paintEvent(e);
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    // card background
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 0, 0, 40));
    p.drawRoundedRect(rect().adjusted(0,0,-1,-1), 12, 12);

    const int pad = 14;
    const QRect leftArea(pad, pad + 10, (width() - pad*3)/2, height() - pad*2 - 10);
    const QRect rightArea(leftArea.right() + pad, pad + 10, leftArea.width(), leftArea.height());

    drawStickArea(p, leftArea, sample_.lx, sample_.ly, leftHist_, "Left Stick");
    drawStickArea(p, rightArea, sample_.rx, sample_.ry, rightHist_, "Right Stick");

    // status text
    p.setPen(QColor(229, 231, 235, 220));
    QFont f = p.font();
    f.setPointSize(9);
    f.setBold(true);
    p.setFont(f);

    QString status = sample_.connected ? "Gamepad: CONNECTED" : "Gamepad: DISCONNECTED";
    p.drawText(QRect(10, 6, width()-20, 18), Qt::AlignLeft | Qt::AlignVCenter, status);
}

void GamepadWidget::pushHistory() {
    if (!sample_.connected) {
        leftHist_.clear();
        rightHist_.clear();
        return;
    }
    leftHist_.push_back(QPointF(sample_.lx, sample_.ly));
    rightHist_.push_back(QPointF(sample_.rx, sample_.ry));
    const int N = 80;
    while ((int)leftHist_.size() > N) leftHist_.pop_front();
    while ((int)rightHist_.size() > N) rightHist_.pop_front();
}

QPointF GamepadWidget::stickToPixel(const QRect& area, float x, float y) {
    const float cx = area.center().x();
    const float cy = area.center().y();
    const float r = 0.42f * std::min(area.width(), area.height());
    return QPointF(cx + x * r, cy - y * r);
}

void GamepadWidget::drawStickArea(QPainter& p, const QRect& area, float x, float y,
                                 const std::deque<QPointF>& hist, const QString& title) {
    const QPoint center = area.center();
    const int r = int(0.42 * std::min(area.width(), area.height()));

    p.setPen(QPen(QColor(255,255,255,35), 1));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(center, r, r);

    p.drawLine(center.x()-r, center.y(), center.x()+r, center.y());
    p.drawLine(center.x(), center.y()-r, center.x(), center.y()+r);

    p.setPen(QPen(QColor(59, 130, 246, 120), 2));
    for (size_t i = 1; i < hist.size(); ++i) {
        const QPointF a = stickToPixel(area, (float)hist[i-1].x(), (float)hist[i-1].y());
        const QPointF b = stickToPixel(area, (float)hist[i].x(), (float)hist[i].y());
        p.drawLine(a, b);
    }

    QPointF pt = stickToPixel(area, x, y);
    p.setPen(QPen(QColor(59, 130, 246, 230), 2));
    p.setBrush(QColor(59, 130, 246, 120));
    p.drawEllipse(pt, 8, 8);

    p.setPen(QColor(203,213,225,200));
    QFont f = p.font();
    f.setPointSize(9);
    f.setBold(false);
    p.setFont(f);
    p.drawText(QRect(area.left(), area.top(), area.width(), 16),
               Qt::AlignLeft | Qt::AlignVCenter, title);
}