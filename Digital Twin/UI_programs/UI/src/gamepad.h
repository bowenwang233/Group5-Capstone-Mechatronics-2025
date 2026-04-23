#pragma once
#include <cstdint>
#include <algorithm>
#include <cmath>

#ifdef _WIN32
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <Windows.h>
  #include <Xinput.h>
#endif

struct GamepadSample {
    bool connected = false;
    float lx = 0.0f; // [-1,1]
    float ly = 0.0f;
    float rx = 0.0f;
    float ry = 0.0f;
    uint16_t buttons = 0;
};

static inline float deadzoneNormalize(short v, short deadzone) {
    const int iv = static_cast<int>(v);
    const int ad = std::abs(iv);
    if (ad <= deadzone) return 0.0f;

    const float sign = (iv >= 0) ? 1.0f : -1.0f;
    const float mag = static_cast<float>(ad - deadzone);
    const float denom = static_cast<float>(32767 - deadzone);
    return sign * std::min(1.0f, mag / std::max(1.0f, denom));
}

static inline GamepadSample readGamepadXInput(int index = 0) {
    GamepadSample s;

#ifdef _WIN32
    XINPUT_STATE st;
    ZeroMemory(&st, sizeof(st));
    const DWORD res = XInputGetState(static_cast<DWORD>(index), &st);
    if (res != ERROR_SUCCESS) {
        s.connected = false;
        return s;
    }
    s.connected = true;

    const auto& g = st.Gamepad;
    s.buttons = g.wButtons;

    s.lx = deadzoneNormalize(g.sThumbLX, XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE);
    s.ly = deadzoneNormalize(g.sThumbLY, XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE);
    s.rx = deadzoneNormalize(g.sThumbRX, XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE);
    s.ry = deadzoneNormalize(g.sThumbRY, XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE);
#else
    (void)index;
#endif

    return s;
}

static inline bool isRBPressed(const GamepadSample& s) {
#ifdef _WIN32
    return (s.buttons & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0;
#else
    return false;
#endif
}