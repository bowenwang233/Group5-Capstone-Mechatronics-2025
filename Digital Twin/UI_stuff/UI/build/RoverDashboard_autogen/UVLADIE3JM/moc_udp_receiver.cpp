/****************************************************************************
** Meta object code from reading C++ file 'udp_receiver.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../src/udp_receiver.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'udp_receiver.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_UdpReceiver_t {
    const uint offsetsAndSize[28];
    char stringdata0[166];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_UdpReceiver_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_UdpReceiver_t qt_meta_stringdata_UdpReceiver = {
    {
QT_MOC_LITERAL(0, 11), // "UdpReceiver"
QT_MOC_LITERAL(12, 20), // "joystickDataReceived"
QT_MOC_LITERAL(33, 0), // ""
QT_MOC_LITERAL(34, 13), // "GamepadSample"
QT_MOC_LITERAL(48, 6), // "sample"
QT_MOC_LITERAL(55, 13), // "stateReceived"
QT_MOC_LITERAL(69, 10), // "RoverState"
QT_MOC_LITERAL(80, 5), // "state"
QT_MOC_LITERAL(86, 15), // "batteryReceived"
QT_MOC_LITERAL(102, 7), // "uint8_t"
QT_MOC_LITERAL(110, 7), // "battery"
QT_MOC_LITERAL(118, 22), // "personDistanceReceived"
QT_MOC_LITERAL(141, 4), // "dist"
QT_MOC_LITERAL(146, 19) // "processIncomingData"

    },
    "UdpReceiver\0joystickDataReceived\0\0"
    "GamepadSample\0sample\0stateReceived\0"
    "RoverState\0state\0batteryReceived\0"
    "uint8_t\0battery\0personDistanceReceived\0"
    "dist\0processIncomingData"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_UdpReceiver[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   44,    2, 0x06,    1 /* Public */,
       5,    1,   47,    2, 0x06,    3 /* Public */,
       8,    1,   50,    2, 0x06,    5 /* Public */,
      11,    1,   53,    2, 0x06,    7 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      13,    0,   56,    2, 0x08,    9 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void, QMetaType::Float,   12,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void UdpReceiver::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<UdpReceiver *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->joystickDataReceived((*reinterpret_cast< std::add_pointer_t<GamepadSample>>(_a[1]))); break;
        case 1: _t->stateReceived((*reinterpret_cast< std::add_pointer_t<RoverState>>(_a[1]))); break;
        case 2: _t->batteryReceived((*reinterpret_cast< std::add_pointer_t<uint8_t>>(_a[1]))); break;
        case 3: _t->personDistanceReceived((*reinterpret_cast< std::add_pointer_t<float>>(_a[1]))); break;
        case 4: _t->processIncomingData(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (UdpReceiver::*)(const GamepadSample & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&UdpReceiver::joystickDataReceived)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (UdpReceiver::*)(RoverState );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&UdpReceiver::stateReceived)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (UdpReceiver::*)(uint8_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&UdpReceiver::batteryReceived)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (UdpReceiver::*)(float );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&UdpReceiver::personDistanceReceived)) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject UdpReceiver::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_UdpReceiver.offsetsAndSize,
    qt_meta_data_UdpReceiver,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_UdpReceiver_t
, QtPrivate::TypeAndForceComplete<UdpReceiver, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const GamepadSample &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<RoverState, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint8_t, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>


>,
    nullptr
} };


const QMetaObject *UdpReceiver::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *UdpReceiver::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_UdpReceiver.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int UdpReceiver::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void UdpReceiver::joystickDataReceived(const GamepadSample & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void UdpReceiver::stateReceived(RoverState _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void UdpReceiver::batteryReceived(uint8_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void UdpReceiver::personDistanceReceived(float _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
