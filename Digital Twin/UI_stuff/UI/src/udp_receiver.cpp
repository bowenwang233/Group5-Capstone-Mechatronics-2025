#include "udp_receiver.h"
#include <QByteArray>

UdpReceiver::UdpReceiver(uint16_t port, QObject* parent)
    : QObject(parent)
{
    sock_ = new QUdpSocket(this);
    sock_->bind(QHostAddress::AnyIPv4, port);

    connect(sock_, &QUdpSocket::readyRead,
            this, &UdpReceiver::processIncomingData);
}

void UdpReceiver::processIncomingData() {
    while (sock_->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(sock_->pendingDatagramSize());
        sock_->readDatagram(datagram.data(), datagram.size());

        if (datagram.size() != sizeof(UiPacket)) continue;
        auto* data = reinterpret_cast<const UiPacket*>(datagram.data());

        GamepadSample s;
        s.lx = data->lx;
        s.ly = data->ly;
        s.rx = data->rx;
        s.ry = data->ry;

        RoverState state = static_cast<RoverState>(data->state);
        uint8_t battery = data->batteryPercent;  // <-- new
        float personDist = data->personDistance;

        emit joystickDataReceived(s);
        emit stateReceived(state);
        emit batteryReceived(battery);           // <-- emit signal
        emit personDistanceReceived(personDist);
    }
}
