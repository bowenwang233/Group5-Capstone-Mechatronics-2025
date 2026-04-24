#pragma once
#include <QObject>
#include <QImage>
#include <QtNetwork/QNetworkAccessManager>
#include <QTimer>
#include <mutex>
#include <vector>
#include "ui_types.h"

class HttpBridge : public QObject {
public:
    explicit HttpBridge(const QString& baseUrl, QObject* parent = nullptr);

    void start();
    bool getLatest(QImage& frame, std::vector<UiBox>& boxes);

private:
    void fetchFrame();
    void fetchDetections();

private:
    QString baseUrl_;
    QNetworkAccessManager nam_;
    QTimer* frameTimer_ = nullptr;
    QTimer* detTimer_ = nullptr;

    std::mutex mtx_;
    QImage frame_;
    std::vector<UiBox> boxes_;
};