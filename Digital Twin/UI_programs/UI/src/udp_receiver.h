#pragma once
#include <QObject>
#include <QUdpSocket>
#include "gamepad.h"

// Optional but recommended: shared enum
enum class RoverState : uint8_t {
    INITIAL = 0,
    IDLE = 1,
    FOLLOW = 2,
    MANUAL = 3,
    NAVIGATION = 4,
    ESTOP = 5
};

// --- UI packet (must match Simulink UDP send exactly) ---
#pragma pack(push, 1)
struct UiPacket {
    float lx;
    float ly;
    float rx;
    float ry;
    uint8_t state;
    uint16_t batteryPercent;  // new field
};
#pragma pack(pop)

class UdpReceiver : public QObject {
    Q_OBJECT
public:
    explicit UdpReceiver(uint16_t port, QObject* parent = nullptr);
    ~UdpReceiver() = default;

signals:
    void joystickDataReceived(const GamepadSample& sample);
    void stateReceived(RoverState state);
    void batteryReceived(uint8_t battery);   // new signal

private slots:
    void processIncomingData();

private:
    QUdpSocket* sock_ = nullptr;
};
