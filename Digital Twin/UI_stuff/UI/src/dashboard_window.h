#pragma once
#include <QMainWindow>
#include <QFrame>
#include <QLabel>
#include <QTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QDateTime>
#include <memory>

#include "camera_canvas.h"
#include "http_bridge.h"
#include "gamepad_widget.h"
#include "gamepad.h"
#include "udp_receiver.h"

class DashboardWindow : public QMainWindow {
public:
    DashboardWindow();

private:
    static QString nowTs();
    void appendActionLogIfNeeded(float cmdVel, float cmdAng, bool manualEnabled, bool estop);

private:
    QFrame* cameraView_ = nullptr;
    CameraCanvas* camCanvas_ = nullptr;
    QProgressBar* batteryBar_ = nullptr;
    QTextEdit* log_ = nullptr;

    QLabel* modeValue_ = nullptr;
    QLabel* connValue_ = nullptr;
    QLabel* speedValue_ = nullptr;
    QLabel* turnValue_ = nullptr;
    QLabel* personDistValue_ = nullptr;

    GamepadWidget* gamepadWidget_ = nullptr;
    QLabel* gpStatus_ = nullptr;
    QLabel* ly_ = nullptr;
    QLabel* rx_ = nullptr;
    QLabel* rb_ = nullptr;

    QPushButton* estopBtn_ = nullptr;
    QPushButton* resetBtn_ = nullptr;
    QTimer* watchdog_;
    QFrame* makeStateItem(const QString& title,
                          const QString& value,
                          QLabel** outValueLabel = nullptr);

    std::unique_ptr<HttpBridge> http_;

    // --- existing state ---
    bool manualEnabled_ = false;
    bool estop_ = false;
    bool prevRbRaw_ = false;

    std::unique_ptr<UdpReceiver> receiver_;

    // for log de-dup
    int lastLyQ_ = 0;
    int lastRxQ_ = 0;
    bool lastManual_ = false;
    bool lastEstop_ = false;
    QString lastAction_;
};
