#include "camera_canvas.h"
#include <algorithm>

CameraCanvas::CameraCanvas(QWidget* parent) : QWidget(parent) {
    setObjectName("camCanvas");
    setMinimumSize(320, 240);
}

void CameraCanvas::updateFrameAndBoxes(const QImage& frame, const std::vector<UiBox>& boxes) {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        frame_ = frame;
        boxes_ = boxes;
    }
    update();
}

void CameraCanvas::paintEvent(QPaintEvent* e) {
    QWidget::paintEvent(e);

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::SmoothPixmapTransform, true);

    QImage frame;
    std::vector<UiBox> boxes;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        frame = frame_;
        boxes = boxes_;
    }

    if (frame.isNull()) {
        p.setPen(QColor(229, 231, 235, 200));
        p.drawText(rect(), Qt::AlignCenter,
                   "Waiting for HTTP stream...\n"
                   "Start Python first:\n"
                   "  http://127.0.0.1:8000/frame.jpg\n"
                   "  http://127.0.0.1:8000/detections");
        return;
    }

    QImage scaled = frame.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    const int x0 = (width() - scaled.width()) / 2;
    const int y0 = (height() - scaled.height()) / 2;
    p.drawImage(QPoint(x0, y0), scaled);

    const float sx = static_cast<float>(scaled.width()) / static_cast<float>(frame.width());
    const float sy = static_cast<float>(scaled.height()) / static_cast<float>(frame.height());

    p.setPen(QPen(QColor(59, 130, 246, 230), 2));
    p.setBrush(Qt::NoBrush);

    QFont font = p.font();
    font.setPointSize(10);
    font.setBold(true);
    p.setFont(font);

    for (const auto& b : boxes) {
        const QRectF r(x0 + b.x * sx, y0 + b.y * sy, b.w * sx, b.h * sy);
        p.drawRect(r);

        QString txt = b.label.isEmpty()
            ? QString("score=%.2f").arg(b.score)
            : QString("%1 (%.2f)").arg(b.label).arg(b.score);

        QRectF tag(r.left(), r.top() - 18, std::min(260.0, r.width()), 18);
        p.fillRect(tag, QColor(0, 0, 0, 140));
        p.setPen(QColor(229, 231, 235, 240));
        p.drawText(tag.adjusted(6, 0, -6, 0), Qt::AlignVCenter | Qt::AlignLeft, txt);
        p.setPen(QPen(QColor(59, 130, 246, 230), 2));
    }
}