#include "http_bridge.h"
#include <QtNetwork/QNetworkReply>
#include <QtNetwork/QNetworkRequest>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

HttpBridge::HttpBridge(const QString& baseUrl, QObject* parent)
    : QObject(parent), baseUrl_(baseUrl) {}

void HttpBridge::start() {
    frameTimer_ = new QTimer(this);
    QObject::connect(frameTimer_, &QTimer::timeout, this, [this]() { fetchFrame(); });
    frameTimer_->start(33);

    detTimer_ = new QTimer(this);
    QObject::connect(detTimer_, &QTimer::timeout, this, [this]() { fetchDetections(); });
    detTimer_->start(100);
}

bool HttpBridge::getLatest(QImage& frame, std::vector<UiBox>& boxes) {
    std::lock_guard<std::mutex> lk(mtx_);
    frame = frame_;
    boxes = boxes_;
    return !frame_.isNull();
}

void HttpBridge::fetchFrame() {
    QNetworkRequest req(QUrl(baseUrl_ + "/frame.jpg"));
    auto* reply = nam_.get(req);
    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        reply->deleteLater();
        if (reply->error() != QNetworkReply::NoError) return;

        const QByteArray data = reply->readAll();
        QImage img;
        img.loadFromData(data, "JPG");
        if (!img.isNull()) {
            std::lock_guard<std::mutex> lk(mtx_);
            frame_ = img;
        }
    });
}

void HttpBridge::fetchDetections() {
    QNetworkRequest req(QUrl(baseUrl_ + "/detections"));
    auto* reply = nam_.get(req);
    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        reply->deleteLater();
        if (reply->error() != QNetworkReply::NoError) return;

        const QByteArray data = reply->readAll();
        const auto doc = QJsonDocument::fromJson(data);
        if (!doc.isObject()) return;

        const auto obj = doc.object();
        const auto arr = obj.value("boxes").toArray();

        std::vector<UiBox> boxes;
        boxes.reserve(arr.size());

        for (const auto& v : arr) {
            if (!v.isObject()) continue;
            const auto b = v.toObject();

            UiBox ub;
            ub.x = float(b.value("x").toDouble());
            ub.y = float(b.value("y").toDouble());
            ub.w = float(b.value("w").toDouble());
            ub.h = float(b.value("h").toDouble());
            ub.label = b.value("label").toString();
            ub.score = float(b.value("score").toDouble());
            boxes.push_back(std::move(ub));
        }

        std::lock_guard<std::mutex> lk(mtx_);
        boxes_ = std::move(boxes);
    });
}