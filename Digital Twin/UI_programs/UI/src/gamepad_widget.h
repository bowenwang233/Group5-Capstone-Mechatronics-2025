#pragma once
#include <QWidget>
#include <QPainter>
#include <deque>
#include "gamepad.h"

class GamepadWidget : public QWidget {
public:
    explicit GamepadWidget(QWidget* parent = nullptr);

    void setSample(const GamepadSample& s);

protected:
    void paintEvent(QPaintEvent* e) override;

private:
    void pushHistory();
    static QPointF stickToPixel(const QRect& area, float x, float y);
    void drawStickArea(QPainter& p, const QRect& area, float x, float y,
                       const std::deque<QPointF>& hist, const QString& title);

private:
    GamepadSample sample_;
    std::deque<QPointF> leftHist_;
    std::deque<QPointF> rightHist_;
};