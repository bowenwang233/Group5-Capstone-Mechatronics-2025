#pragma once
#include <QWidget>
#include <QImage>
#include <QPainter>
#include <mutex>
#include <vector>
#include "ui_types.h"

class CameraCanvas : public QWidget {
public:
    explicit CameraCanvas(QWidget* parent = nullptr);

    void updateFrameAndBoxes(const QImage& frame, const std::vector<UiBox>& boxes);

protected:
    void paintEvent(QPaintEvent* e) override;

private:
    std::mutex mtx_;
    QImage frame_;
    std::vector<UiBox> boxes_;
};