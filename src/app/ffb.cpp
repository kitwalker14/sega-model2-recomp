#include "app/ffb.h"
#include "runtime/rom_import.h"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iterator>

namespace app {

namespace {

// The device steering is bound to: a wheel/joystick binding first, else the gamepad.
SDL_Joystick *steering_device(const Devices &d, const Controls &c) {
    for (Action a : {SteerRight, SteerLeft}) {
        const JoyInput &j = c.bind[a].joy;
        if (j.kind != JoyInput::None)
            if (SDL_Joystick *s = d.find(j.guid)) return s;
    }
    return d.pad ? SDL_GetGamepadJoystick(d.pad) : nullptr;
}

SDL_HapticDirection steering_axis() {
    SDL_HapticDirection dir{};
    dir.type = SDL_HAPTIC_STEERING_AXIS;
    return dir;
}

constexpr uint64_t kRetryMs = 3000; // between tries to open a wheel's haptics

struct VibrationProfile {
    float idle_gain, rpm_gain, rolling_gain, slip_gain, shift_gain, rough_gain, impact_gain;
    float engine_idle_hz, engine_full_hz, road_low_hz, road_high_hz;
};

// Gains are fractions of the separate 5% synthetic overlay, not of the wheel's
// full torque. Daytona has no RPM/speed telemetry, so load and board jolts are
// bounded fallbacks rather than measured vehicle data.
constexpr VibrationProfile kF1{0.04f, 0.14f, 0.06f, 0.12f, 0.18f, 0.20f, 0.25f,
                               28.0f, 65.0f, 8.0f, 14.0f};
constexpr VibrationProfile kNASCAR{0.08f, 0.18f, 0.12f, 0.12f, 0.16f, 0.14f, 0.25f,
                                   18.0f, 45.0f, 6.0f, 12.0f};
constexpr VibrationProfile kRally{0.05f, 0.13f, 0.08f, 0.15f, 0.18f, 0.28f, 0.30f,
                                  22.0f, 55.0f, 7.0f, 18.0f};
constexpr float kSyntheticCap = 0.05f;
constexpr float kSyntheticSlew = 20.0f;
constexpr uint64_t kSyntheticStaleMs = 250;

const VibrationProfile &vibration_profile(bool rally) {
    return rally ? kRally : kNASCAR;
}

void advance_phase(float &phase, float hz, float dt) {
    phase += hz * dt;
    phase -= std::floor(phase);
}

int periodic_key(Uint16 magnitude, Uint16 period, float phase) {
    const int phase_key = std::clamp(int(phase * 255.0f), 0, 255);
    return int(magnitude) | (int(period) << 16) | (phase_key << 24);
}

} // namespace

const char *ForceFeedback::device_kind() const {
    if (haptic_) return "wheel (force feedback)";
    if (wants_haptic_) return "wheel (force feedback did not open; retrying)";
    if (joy_) return "gamepad (rumble)";
    return "none";
}

void ForceFeedback::reset_synthetic() {
    rpm_ = 0.0f;
    last_load_ = 0.0f;
    engine_output_ = road_output_ = 0.0f;
    engine_phase_ = road_phase_ = 0.0f;
    last_update_ms_ = 0;
    update_dt_ = 1.0f / 60.0f;
    telemetry_stale_ = false;
}

void ForceFeedback::open(SDL_Joystick *joy) {
    joy_ = joy;
    reported_ = false;
    wants_haptic_ = SDL_IsJoystickHaptic(joy);
    if (wants_haptic_) {
        open_haptic();
        return;
    }
    // no haptics: rumble, if the device has it
    if (!SDL_GetBooleanProperty(SDL_GetJoystickProperties(joy), SDL_PROP_JOYSTICK_CAP_RUMBLE_BOOLEAN, false)) joy_ = nullptr;
}

void ForceFeedback::open_haptic() {
    if (!(haptic_ = SDL_OpenHapticFromJoystick(joy_))) {
        if (!reported_ || log_)
            std::fprintf(stderr, "daytona: force feedback on %s did not open (%s); trying again\n",
                         SDL_GetJoystickName(joy_), SDL_GetError());
        reported_ = true;
        retry_at_ = SDL_GetTicks() + kRetryMs;
        return;
    }
    const Uint32 features = SDL_GetHapticFeatures(haptic_);
    if (features & SDL_HAPTIC_AUTOCENTER) SDL_SetHapticAutocenter(haptic_, 0); // the game centres the wheel itself
    if (features & SDL_HAPTIC_GAIN) SDL_SetHapticGain(haptic_, 100);
    auto create = [&](Uint32 type) {
        if (!(features & type)) return -1;
        SDL_HapticEffect e{};
        if (type == SDL_HAPTIC_CONSTANT) {
            e.constant.type = SDL_HAPTIC_CONSTANT;
            e.constant.direction = steering_axis();
            e.constant.length = SDL_HAPTIC_INFINITY;
        } else {
            e.condition.type = Uint16(type);
            e.condition.direction = steering_axis();
            e.condition.length = SDL_HAPTIC_INFINITY;
        }
        const int id = SDL_CreateHapticEffect(haptic_, &e);
        if (id < 0 && log_) std::fprintf(stderr, "daytona: force feedback: effect not created (%s)\n", SDL_GetError());
        return id;
    };
    spring_ = create(SDL_HAPTIC_SPRING);
    friction_ = create(SDL_HAPTIC_FRICTION);
    constant_ = create(SDL_HAPTIC_CONSTANT);
    if (features & SDL_HAPTIC_SINE) {
        SDL_HapticEffect e{};
        e.periodic.type = SDL_HAPTIC_SINE;
        e.periodic.direction = steering_axis();
        e.periodic.length = SDL_HAPTIC_INFINITY;
        e.periodic.period = 36;
        periodic_ = SDL_CreateHapticEffect(haptic_, &e);
        road_ = SDL_CreateHapticEffect(haptic_, &e);
    }
    if (log_)
        std::fprintf(stderr, "daytona: force feedback on %s: features 0x%" PRIx32 ", spring %s, friction %s, constant %s, engine %s, road %s\n",
                     SDL_GetHapticName(haptic_), features, spring_ >= 0 ? "yes" : "no", friction_ >= 0 ? "yes" : "no",
                     constant_ >= 0 ? "yes" : "no", periodic_ >= 0 ? "yes" : "no", road_ >= 0 ? "yes" : "no");
    std::fill(std::begin(sent_), std::end(sent_), -1);
    running_ = false;
}

void ForceFeedback::close() {
    if (haptic_) {
        SDL_StopHapticEffects(haptic_);
        SDL_CloseHaptic(haptic_);
    } else if (joy_ && !wants_haptic_) {
        SDL_RumbleJoystick(joy_, 0, 0, 0);
    }
    haptic_ = nullptr;
    joy_ = nullptr;
    wants_haptic_ = false;
    spring_ = friction_ = constant_ = periodic_ = road_ = -1;
    running_ = false;
    reset_synthetic();
}

void ForceFeedback::update(const std::vector<uint8_t> &commands, const Devices &devices, const Controls &controls,
                           float strength, bool invert, bool log, uint64_t frame, float wheel, bool centring_override) {
    stopped_ = false;
    const uint64_t now = SDL_GetTicks();
    if (last_update_ms_) {
        const uint64_t gap = now - last_update_ms_;
        update_dt_ = std::clamp(float(gap) / 1000.0f, 0.001f, 0.1f);
        telemetry_stale_ = gap > kSyntheticStaleMs;
    } else {
        update_dt_ = 1.0f / 60.0f;
        telemetry_stale_ = false;
    }
    last_update_ms_ = now;
    drive_.rally = rt::is_rally();
    drive_.tick();
    log_ = log;
    wheel_ = std::clamp(wheel, -1.0f, 1.0f);
    if (centring_override != override_ && log_)
        std::fprintf(stderr, "daytona: force feedback: centring %s\n",
                     centring_override ? "worked out from the wheel's position, in the constant force (override)"
                                       : "through the device's spring");
    override_ = centring_override;
    for (uint8_t c : commands) {
        if (log_) std::fprintf(stderr, "daytona: force feedback: frame %" PRIu64 ": command %02x\n", frame, c);
        drive_.command(c);
    }
    SDL_Joystick *j = strength > 0 ? steering_device(devices, controls) : nullptr;
    if (j != joy_) {
        close();
        if (j) open(j);
    } else if (wants_haptic_ && !haptic_ && SDL_GetTicks() >= retry_at_) {
        open_haptic();
    }
    apply(strength, invert);
}

void ForceFeedback::stop() {
    if (stopped_) return;
    stopped_ = true;
    if (haptic_) {
        SDL_StopHapticEffects(haptic_);
        running_ = false;
        std::fill(std::begin(sent_), std::end(sent_), -1);
    } else if (joy_ && !wants_haptic_) {
        SDL_RumbleJoystick(joy_, 0, 0, 0);
    }
    reset_synthetic();
}

void ForceFeedback::reset() {
    stop();
    drive_ = rt::DriveBoard{};
    drive_.rally = rt::is_rally();
}

void ForceFeedback::apply(float strength, bool invert) {
    using Effect = rt::DriveBoard::Effect;
    if (!joy_) return;
    strength = std::clamp(strength, 0.0f, 1.0f);
    // With the centring override the springs come from the board's own loop on
    // the wheel's position and go out in the constant force; the spring effect
    // is left at nothing.
    const float push = override_ && haptic_ ? drive_.motor(wheel_) : drive_.push();
    const float force = std::clamp(push * (invert ? -1.0f : 1.0f), -1.0f, 1.0f) * strength;
    const float uncentre = drive_.level(Effect::Uncentre) * strength;
    if (!haptic_) {
        if (wants_haptic_) return; // a wheel whose haptics did not open: nothing until they do
        // rumble: the turning force's size on the large motor, uncentring on the small one
        const auto lo = Uint16(std::lround(std::fabs(force) * 0xffff));
        const auto hi = Uint16(std::lround(uncentre * 0xffff));
        SDL_RumbleJoystick(joy_, lo, hi, 200); // renewed every frame
        return;
    }
    // Only changes go to the device, and only once they are accepted: a failed
    // call is tried again next frame.
    auto send = [&](int slot, int id, int key, SDL_HapticEffect &e) {
        if (id < 0 || sent_[slot] == key) return;
        if (SDL_UpdateHapticEffect(haptic_, id, &e)) sent_[slot] = key;
        else if (log_) std::fprintf(stderr, "daytona: force feedback: effect update failed (%s)\n", SDL_GetError());
    };
    if (!rt::is_rally()) {   // (Sega Rally has no springs: only the constant force) the springs: towards the centre (the dead zone about 6% of the travel,
        // else 2%, as the board's), or away from it as a negative spring
        const float centre = override_ ? 0.0f : std::max(drive_.level(Effect::Centre), drive_.level(Effect::CentreDeadZone)) * strength;
        const auto sat = Uint16(std::lround((override_ ? 0.0f : std::max(centre, uncentre)) * 0xffff));
        const bool away = uncentre > 0;
        const Uint16 deadband = drive_.effect == Effect::CentreDeadZone ? 0x1000 : drive_.effect == Effect::Centre ? 0x0500 : 0;
        SDL_HapticEffect e{};
        e.condition.type = SDL_HAPTIC_SPRING;
        e.condition.direction = steering_axis();
        e.condition.length = SDL_HAPTIC_INFINITY;
        e.condition.right_sat[0] = e.condition.left_sat[0] = sat;
        e.condition.right_coeff[0] = e.condition.left_coeff[0] = Sint16(away ? -0x7fff : 0x7fff);
        e.condition.deadband[0] = deadband;
        send(0, spring_, sat ? int(sat) | int(away) << 16 | int(deadband) << 17 : 0, e);
    }
    {
        // Rally: road feel. The game's own steering torque is its cornering load, so
        // a damper that follows it firms up the wheel while the tyres grip and
        // lets go as the car slides (oversteer), with a small floor for the road.
        const float res = rt::is_rally() ? (std::fabs(drive_.force()) > 0.0f ? 0.18f + 0.6f * std::fabs(drive_.force()) : 0.0f)
                                         : std::max(drive_.level(Effect::Resistance),
                                                    drive_.motor_on ? 0.05f + 0.25f * drive_.load() : 0.0f);
        const auto level = Sint16(std::lround(res * strength * 0x7fff));
        SDL_HapticEffect e{};
        e.condition.type = SDL_HAPTIC_FRICTION;
        e.condition.direction = steering_axis();
        e.condition.length = SDL_HAPTIC_INFINITY;
        e.condition.right_sat[0] = e.condition.left_sat[0] = 0xffff;
        e.condition.right_coeff[0] = e.condition.left_coeff[0] = level;
        send(1, friction_, level, e);
    }
    {
        const auto level = Sint16(std::lround(force * 0x7fff));
        SDL_HapticEffect e{};
        e.constant.type = SDL_HAPTIC_CONSTANT;
        e.constant.direction = steering_axis();
        e.constant.length = SDL_HAPTIC_INFINITY;
        e.constant.level = level;
        send(2, constant_, level, e);
    }
    {
        const VibrationProfile &profile = vibration_profile(rt::is_rally());
        const bool emergency_disable = std::getenv("M1_FFB_DISABLE_SYNTHETIC") != nullptr;
        const float load = std::clamp(rt::is_rally() ? std::fabs(drive_.force()) : drive_.load(), 0.0f, 1.0f);
        const bool playing = drive_.motor_on;
        const float load_step = std::clamp(std::fabs(load - last_load_) * 8.0f, 0.0f, 1.0f);
        const float jolt = std::clamp(drive_.jolt, 0.0f, 1.0f);
        last_load_ = load;
        const float engine_rpm = playing && !telemetry_stale_ ? load : 0.0f;
        rpm_ += (engine_rpm - rpm_) * std::clamp(update_dt_ * 8.0f, 0.0f, 1.0f);
        const float r = std::clamp(rpm_, 0.0f, 1.0f);
        const float engine_fraction = playing && !telemetry_stale_
            ? profile.idle_gain + (profile.rpm_gain - profile.idle_gain) * std::pow(r, 1.4f) : 0.0f;
        const float road_fraction = playing && !telemetry_stale_
            ? profile.rolling_gain * load + profile.rough_gain * load_step + profile.impact_gain * jolt : 0.0f;
        const float total_fraction = engine_fraction + road_fraction;
        const float normalize = total_fraction > 1.0f ? 1.0f / total_fraction : 1.0f;
        const float target_engine = emergency_disable ? 0.0f : kSyntheticCap * strength * engine_fraction * normalize;
        const float target_road = emergency_disable ? 0.0f : kSyntheticCap * strength * road_fraction * normalize;
        const float step = kSyntheticSlew * update_dt_;
        engine_output_ += std::clamp(target_engine - engine_output_, -step, step);
        road_output_ += std::clamp(target_road - road_output_, -step, step);
        const float output_sum = engine_output_ + road_output_;
        if (output_sum > kSyntheticCap * strength && output_sum > 0.0f) {
            const float scale = kSyntheticCap * strength / output_sum;
            engine_output_ *= scale;
            road_output_ *= scale;
        }
        const float engine_hz = profile.engine_idle_hz + (profile.engine_full_hz - profile.engine_idle_hz) * r;
        const float road_r = std::clamp(0.35f * load + 0.65f * load_step + 0.5f * jolt, 0.0f, 1.0f);
        const float road_hz = profile.road_low_hz + (profile.road_high_hz - profile.road_low_hz) * road_r;
        if (playing && !telemetry_stale_) {
            advance_phase(engine_phase_, engine_hz, update_dt_);
            advance_phase(road_phase_, road_hz, update_dt_);
        } else {
            engine_phase_ = road_phase_ = 0.0f;
        }
        const Uint16 engine_period = Uint16(std::clamp(int(std::lround(1000.0f / engine_hz)), 1, 1000));
        const Uint16 road_period = Uint16(std::clamp(int(std::lround(1000.0f / road_hz)), 1, 1000));
        // SDL phases are hundredths of a degree, not a full Uint16 turn.
        const Uint16 engine_phase = Uint16(std::lround(engine_phase_ * 36000.0f));
        const Uint16 road_phase = Uint16(std::lround(road_phase_ * 36000.0f));
        if (road_ >= 0) {
            SDL_HapticEffect engine{};
            engine.periodic.type = SDL_HAPTIC_SINE;
            engine.periodic.direction = steering_axis();
            engine.periodic.length = SDL_HAPTIC_INFINITY;
            engine.periodic.period = engine_period;
            engine.periodic.phase = engine_phase;
            engine.periodic.magnitude = Sint16(std::lround(engine_output_ * 0x7fff));
            send(3, periodic_, periodic_key(Uint16(engine.periodic.magnitude), engine_period, engine_phase_), engine);

            SDL_HapticEffect road{};
            road.periodic.type = SDL_HAPTIC_SINE;
            road.periodic.direction = steering_axis();
            road.periodic.length = SDL_HAPTIC_INFINITY;
            road.periodic.period = road_period;
            road.periodic.phase = road_phase;
            road.periodic.magnitude = Sint16(std::lround(road_output_ * 0x7fff));
            send(4, road_, periodic_key(Uint16(road.periodic.magnitude), road_period, road_phase_), road);
        } else if (periodic_ >= 0) {
            const bool use_road = road_output_ > engine_output_;
            const Uint16 period = use_road ? road_period : engine_period;
            const float phase = use_road ? road_phase_ : engine_phase_;
            SDL_HapticEffect combined{};
            combined.periodic.type = SDL_HAPTIC_SINE;
            combined.periodic.direction = steering_axis();
            combined.periodic.length = SDL_HAPTIC_INFINITY;
            combined.periodic.period = period;
            combined.periodic.phase = Uint16(std::lround(phase * 36000.0f));
            combined.periodic.magnitude = Sint16(std::lround(std::min(kSyntheticCap * strength, output_sum) * 0x7fff));
            send(3, periodic_, periodic_key(Uint16(combined.periodic.magnitude), period, phase), combined);
        }
    }
    if (!running_) {
        running_ = true;
        for (int id : {spring_, friction_, constant_, periodic_, road_})
            if (id >= 0 && !(rt::is_rally() && id == spring_) && !SDL_RunHapticEffect(haptic_, id, 1)) {
                running_ = false; // all of them again next frame
                if (log_) std::fprintf(stderr, "daytona: force feedback: effect did not start (%s)\n", SDL_GetError());
            }
    }
}

} // namespace app
