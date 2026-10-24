// Force feedback: the drive board's state (rt::DriveBoard, from the game's
// commands) on the host's wheel. A wheel with SDL haptics gets the board's
// native effects on its steering axis: a spring (centring, with or without a
// dead zone, or uncentring as a negative spring), friction (the board's
// resistance) or a constant force. Synthetic engine/road effects are additive
// periodic layers with their own safety cap. A gamepad without haptics gets
// rumble: the constant force on the large motor, uncentring on the small one;
// the springs and resistance, on all race, give none. A wheel whose haptics
// fail to open gets nothing, not rumble (on DirectInput, SDL's rumble is a
// shake through the wheel), and the open is tried again. The device is the
// one steering is bound to: the wheel/joystick binding first, else the
// gamepad.
#pragma once

#include "app/controls.h"
#include "runtime/drive_board.h"

#include <SDL3/SDL.h>

#include <cstdint>
#include <vector>

namespace app {

class ForceFeedback {
public:
    ~ForceFeedback() { close(); }
    // Each frame: the drive commands the game sent, the devices, the
    // settings (strength 0..1, 0 = off; invert the turning force; log: the
    // device, failed calls and each command with the game's frame, to stderr).
    // centring_override (Linux, for drivers without a spring effect): the
    // springs worked out here from the wheel's position (-1 left .. +1 right,
    // as the game sees it) and sent in the constant force, not as a spring.
    void update(const std::vector<uint8_t> &commands, const Devices &devices, const Controls &controls, float strength,
                bool invert, bool log = false, uint64_t frame = 0, float wheel = 0.0f, bool centring_override = false);
    void stop();  // the game paused: let the wheel go (the board's state is kept for resuming)
    void reset(); // a new game: stop, and the board as at power-on
    void close(); // release the device
    const rt::DriveBoard &state() const { return drive_; }
    const char *device_kind() const; // what force feedback plays on, for the launcher

private:
    rt::DriveBoard drive_;
    SDL_Joystick *joy_ = nullptr; // the device the effects are on
    SDL_Haptic *haptic_ = nullptr;
    bool wants_haptic_ = false; // the device has force feedback but it is not open (yet): no rumble
    uint64_t retry_at_ = 0;     // SDL_GetTicks() when the haptic open is tried again
    bool reported_ = false;     // the open failure was logged
    int spring_ = -1, friction_ = -1, constant_ = -1, periodic_ = -1, road_ = -1;
    float rpm_ = 0.0f;
    float last_load_ = 0.0f, engine_output_ = 0.0f, road_output_ = 0.0f;
    float engine_phase_ = 0.0f, road_phase_ = 0.0f;
    uint64_t last_update_ms_ = 0;
    float update_dt_ = 1.0f / 60.0f;
    bool telemetry_stale_ = false;
    int sent_[5] = {-1, -1, -1, -1, -1}; // only changed effects go to the device
    bool running_ = false, stopped_ = false, log_ = false;
    bool override_ = false; // centring worked out here, sent as constant force
    float wheel_ = 0.0f;    // the wheel's position, -1 left .. +1 right
    void open(SDL_Joystick *joy);
    void open_haptic();
    void apply(float strength, bool invert);
    void reset_synthetic();
};

} // namespace app
