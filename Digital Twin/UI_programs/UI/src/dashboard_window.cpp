#include "dashboard_window.h"

#include <QApplication>
#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <algorithm>
#include <cmath>



uint8_t voltageToPercent(float voltage) {
    int perc = static_cast<int>((voltage - 10.0f) / (12.3f - 10.0f) * 100.0f);
    if (perc < 0) perc = 0;
    if (perc > 100) perc = 100;
    return static_cast<uint8_t>(perc);
}


QString DashboardWindow::nowTs() {
    return QDateTime::currentDateTime().toString("[HH:mm:ss] ");
}


static QString stateToString(uint8_t state) {
    switch (state) {
        case 0: return "INITIAL";
        case 1: return "IDLE";
        case 2: return "FOLLOW";
        case 3: return "MANUAL";
        case 4: return "NAVIGATION";
        case 5: return "E-STOP";
        default: return "UNKNOWN";
    }
}

DashboardWindow::DashboardWindow() {
    setWindowTitle("Rover Dashboard (HTTP + Gamepad)");
    setMinimumSize(1200, 720);

    auto* root = new QWidget();
    auto* mainLayout = new QHBoxLayout(root);
    mainLayout->setContentsMargins(18, 18, 18, 18);
    mainLayout->setSpacing(14);

    // ---------------- Left: Camera View ----------------
    cameraView_ = new QFrame();
    cameraView_->setObjectName("cameraView");
    cameraView_->setStyleSheet(
        "QFrame#cameraView {"
        "  background: rgba(255,255,255,0.03);"
        "  border: 1px solid rgba(255,255,255,0.10);"
        "  border-radius: 18px;"
        "}"
    );

    auto* camLayout = new QVBoxLayout(cameraView_);
    camLayout->setContentsMargins(16, 14, 16, 16);
    camLayout->setSpacing(10);

    auto* camTitle = new QLabel("Camera View");
    camTitle->setStyleSheet("color: #e5e7eb; font-size: 16px; font-weight: 800;");
    camLayout->addWidget(camTitle);

    camCanvas_ = new CameraCanvas();
    camCanvas_->setStyleSheet(
        "QWidget#camCanvas {"
        "  background: rgba(0,0,0,0.25);"
        "  border: 1px solid rgba(255,255,255,0.08);"
        "  border-radius: 14px;"
        "}"
    );
    camLayout->addWidget(camCanvas_, 1);

    // ---------------- Right: Cards + Rover State ----------------
    auto* rightWrap = new QWidget();
    auto* rightWrapLayout = new QHBoxLayout(rightWrap);
    rightWrapLayout->setContentsMargins(0, 0, 0, 0);
    rightWrapLayout->setSpacing(12);

    auto* cardsCol = new QWidget();
    auto* cardsLayout = new QVBoxLayout(cardsCol);
    cardsLayout->setContentsMargins(0, 0, 0, 0);
    cardsLayout->setSpacing(12);

    // ---- System Status card ----
    auto* sysContent = new QWidget();
    auto* sysLayout = new QVBoxLayout(sysContent);
    sysLayout->setContentsMargins(0, 0, 0, 0);
    sysLayout->setSpacing(10);

    auto* batteryRow = new QHBoxLayout();
    auto* batteryLabel = new QLabel("Battery");
    batteryLabel->setStyleSheet("color:#cbd5e1; font-size: 12px;");
    batteryBar_ = new QProgressBar();
    batteryBar_->setRange(0, 100);
    batteryBar_->setTextVisible(true);
    batteryBar_->setStyleSheet(
        "QProgressBar {"
        "  border: 1px solid rgba(255,255,255,0.12);"
        "  border-radius: 10px;"
        "  background: rgba(255,255,255,0.05);"
        "  height: 16px;"
        "  color: #e5e7eb;"
        "}"
        "QProgressBar::chunk {"
        "  background: rgba(34,197,94,0.7);"
        "  border-radius: 10px;"
        "}"
    );
    batteryRow->addWidget(batteryLabel);
    batteryRow->addStretch(1);
    sysLayout->addLayout(batteryRow);
    sysLayout->addWidget(batteryBar_);

    auto* sysCard = new QFrame();
    sysCard->setStyleSheet(
        "QFrame {"
        "  background: rgba(255,255,255,0.04);"
        "  border: 1px solid rgba(255,255,255,0.10);"
        "  border-radius: 16px;"
        "}"
    );
    auto* sysCardLayout = new QVBoxLayout(sysCard);
    sysCardLayout->setContentsMargins(16, 14, 16, 16);
    sysCardLayout->setSpacing(10);
    auto* sysHeader = new QLabel("System Status");
    sysHeader->setStyleSheet("color: #e5e7eb; font-size: 14px; font-weight: 700;");
    sysCardLayout->addWidget(sysHeader);
    sysCardLayout->addWidget(sysContent);

    // ---- Controls card ----
    auto* ctrlContent = new QWidget();
    auto* ctrlLayout = new QVBoxLayout(ctrlContent);
    ctrlLayout->setContentsMargins(0, 0, 0, 0);
    ctrlLayout->setSpacing(10);

    auto* hint = new QLabel("Manual Control");
    hint->setStyleSheet("color:#cbd5e1; font-size: 12px;");
    ctrlLayout->addWidget(hint);

    gamepadWidget_ = new GamepadWidget();
    ctrlLayout->addWidget(gamepadWidget_);

    auto* nums = new QWidget();
    auto* numsGrid = new QGridLayout(nums);
    numsGrid->setContentsMargins(0, 0, 0, 0);
    numsGrid->setHorizontalSpacing(10);
    numsGrid->setVerticalSpacing(6);

    gpStatus_ = new QLabel("Gamepad: DISCONNECTED");
    gpStatus_->setStyleSheet("color: rgba(229,231,235,0.9); font-size: 12px; font-weight: 700;");
    numsGrid->addWidget(gpStatus_, 0, 0, 1, 2);

    ly_ = new QLabel("Velocity: 0.00");
    rx_ = new QLabel("Angle: 0.00");
    
    // Set minimum widths (adjust number as needed for your max value)
    ly_->setMinimumWidth(80);
    rx_->setMinimumWidth(80);

    auto styleVal = [](QLabel* l) {
        l->setStyleSheet("color: rgba(203,213,225,0.95); font-size: 12px;");
    };
    styleVal(ly_);
    styleVal(rx_);

    numsGrid->addWidget(ly_, 1, 0);
    numsGrid->addWidget(rx_, 1, 1);

    ctrlLayout->addWidget(nums);

    auto* ctrlCard = new QFrame();
    ctrlCard->setStyleSheet(
        "QFrame {"
        "  background: rgba(255,255,255,0.04);"
        "  border: 1px solid rgba(255,255,255,0.10);"
        "  border-radius: 16px;"
        "}"
    );
    auto* ctrlCardLayout = new QVBoxLayout(ctrlCard);
    ctrlCardLayout->setContentsMargins(16, 14, 16, 16);
    ctrlCardLayout->setSpacing(10);
    auto* ctrlHeader = new QLabel("Controls");
    ctrlHeader->setStyleSheet("color: #e5e7eb; font-size: 14px; font-weight: 700;");
    ctrlCardLayout->addWidget(ctrlHeader);
    ctrlCardLayout->addWidget(ctrlContent);
    
 

    // ---- Event Log card ----
    log_ = new QTextEdit();
    log_->setReadOnly(true);
    log_->setStyleSheet(
        "QTextEdit {"
        "  background: rgba(0,0,0,0.22);"
        "  border: 1px solid rgba(255,255,255,0.08);"
        "  border-radius: 12px;"
        "  color: #e5e7eb;"
        "  font-size: 12px;"
        "}"
    );
    log_->append(nowTs() + "Dashboard started.");

    auto* logCard = new QFrame();
    logCard->setStyleSheet(
        "QFrame {"
        "  background: rgba(255,255,255,0.04);"
        "  border: 1px solid rgba(255,255,255,0.10);"
        "  border-radius: 16px;"
        "}"
    );
    auto* logCardLayout = new QVBoxLayout(logCard);
    logCardLayout->setContentsMargins(16, 14, 16, 16);
    logCardLayout->setSpacing(10);
    auto* logHeader = new QLabel("Event Log");
    logHeader->setStyleSheet("color: #e5e7eb; font-size: 14px; font-weight: 700;");
    logCardLayout->addWidget(logHeader);
    logCardLayout->addWidget(log_, 1);

    cardsLayout->addWidget(sysCard);
    cardsLayout->addWidget(ctrlCard);
    cardsLayout->addWidget(logCard, 1);

    // ---- Rover State bar ----
    auto* stateBar = new QFrame();
    stateBar->setObjectName("stateBar");
    stateBar->setFixedWidth(180);
    stateBar->setStyleSheet(
        "QFrame#stateBar {"
        "  background: rgba(255,255,255,0.03);"
        "  border: 1px solid rgba(255,255,255,0.10);"
        "  border-radius: 16px;"
        "}"
    );

    auto* stateLayout = new QVBoxLayout(stateBar);
    stateLayout->setContentsMargins(14, 14, 14, 14);
    stateLayout->setSpacing(10);

    auto* stateTitle = new QLabel("Rover State");
    stateTitle->setStyleSheet("color: #e5e7eb; font-size: 14px; font-weight: 800;");
    stateLayout->addWidget(stateTitle);

    stateLayout->addWidget(makeStateItem("STATE", "AUTO FOLLOW", &modeValue_));
    stateLayout->addWidget(makeStateItem("VIDEO", "NA", &connValue_));
    stateLayout->addWidget(makeStateItem("VELOCITY", "0.00", &speedValue_));
    stateLayout->addWidget(makeStateItem("ANGLE", "0.00", &turnValue_));
    stateLayout->addStretch(1);

    rightWrapLayout->addWidget(cardsCol, 1);
    rightWrapLayout->addWidget(stateBar);

    mainLayout->addWidget(cameraView_, 7);
    mainLayout->addWidget(rightWrap, 3);

    root->setStyleSheet("background: #0b1220;");
    setCentralWidget(root);

    // ---------------- HTTP bridge ----------------
    const QString httpBaseUrl = "http://127.0.0.1:8000";
    http_ = std::make_unique<HttpBridge>(httpBaseUrl);
    http_->start();

    auto* httpTimer = new QTimer(this);
    QObject::connect(httpTimer, &QTimer::timeout, this, [this, httpBaseUrl]() {
    	QImage frame;
    	std::vector<UiBox> boxes;
    	const bool ok = (http_ && http_->getLatest(frame, boxes));

    	if (connValue_) connValue_->setText(ok ? "CONNECTED" : "DISCONNECTED");
    	if (ok && camCanvas_) camCanvas_->updateFrameAndBoxes(frame, boxes);
});
httpTimer->start(33);
    
    watchdog_ = new QTimer(this);
    watchdog_->setInterval(500);

    connect(watchdog_, &QTimer::timeout, this, [this]() {
    	if (gpStatus_) {
        gpStatus_->setText("Gamepad: DISCONNECTED");
    }
});

watchdog_->start();

    // ---------------- Safety button logic ----------------
   QObject::connect(estopBtn_, &QPushButton::clicked, this, [this]() {

    if (log_) log_->append(nowTs() + "E-STOP button pressed (request sent)");

});

   QObject::connect(resetBtn_, &QPushButton::clicked, this, [this]() {

    if (log_) log_->append(nowTs() + "RESET button pressed");

});

    // ---------------- UDP receiver (Simulink -> UI) ----------------
    // 1. Initialize the receiver on Port 5011
    receiver_ = std::make_unique<UdpReceiver>(5011);
    
	QObject::connect(receiver_.get(), &UdpReceiver::batteryReceived,
                 this, [this](uint8_t pct) {
    	if (batteryBar_) batteryBar_->setValue(pct);
});

    // 2. Connect the feedback to the animation widget
    QObject::connect(receiver_.get(), &UdpReceiver::joystickDataReceived, this,
    [this](const GamepadSample& s) {

    // ✅ Mark as connected whenever data arrives
    if (gpStatus_) {
        gpStatus_->setText("Gamepad: CONNECTED");
    }
    watchdog_->start();

    if (gamepadWidget_) {
        gamepadWidget_->setSample(s);
        gamepadWidget_->update();
    }

    if (ly_) ly_->setText(QString("Velocity: %1").arg(s.ly, 0, 'f', 2));
    if (rx_) rx_->setText(QString("Angle: %1").arg(s.rx, 0, 'f', 2));

    if (speedValue_) speedValue_->setText(QString::number(s.ly, 'f', 2));
    if (turnValue_)  turnValue_->setText(QString::number(s.rx, 'f', 2));
});
    
    QObject::connect(receiver_.get(),
    &UdpReceiver::stateReceived,
    this,
    [this](RoverState state) {

    static uint8_t lastState = 255;  // persists across calls
    uint8_t newState = static_cast<uint8_t>(state);

    if (modeValue_) {
        modeValue_->setText(stateToString(newState));
    }

    // Only log if state actually changed
    if (newState != lastState) {
        lastState = newState;

        if (log_) {
            log_->append(nowTs() +
                QString("State → %1").arg(stateToString(newState)));
        }
    }

});
}

QFrame* DashboardWindow::makeStateItem(const QString& title,
                                       const QString& value,
                                       QLabel** outValueLabel) 
{
    auto* box = new QFrame();
    box->setStyleSheet(
        "QFrame {"
        "  background: rgba(0,0,0,0.18);"
        "  border: 1px solid rgba(255,255,255,0.10);"
        "  border-radius: 14px;"
        "}"
    );

    auto* l = new QVBoxLayout(box);
    l->setContentsMargins(12, 10, 12, 10);
    l->setSpacing(6);

    auto* k = new QLabel(title);
    k->setStyleSheet("color: rgba(203,213,225,0.75); font-size: 11px; font-weight: 700;");

    auto* v = new QLabel(value);
    v->setStyleSheet("color: #e5e7eb; font-size: 13px; font-weight: 900;");

    l->addWidget(k);
    l->addWidget(v);

    if (outValueLabel) *outValueLabel = v;
    return box;
}


