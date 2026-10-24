// The force feedback drive board (838-10646 on Daytona USA), as the game
// sees it: the I/O board passes on each command byte the game writes to its
// dual-port RAM byte 0x11 (IoBoard), and the drive board's Z80 turns the
// wheel motor. Its program (EPR-16488A) is not run; its command handling is
// modelled instead, as read from the program and checked in MAME 0.289 by
// feeding the board commands and wheel positions and recording its motor
// output (docs/issues.md, #9).
//
// The board reads the wheel's position itself and runs one effect at a time:
// the last command sets it until the next. The motor runs only after an
// "on" command (0x0-). Strength n is the low 3 bits:
//   0x00-04, 08, 09, 0B, 0C  motor off       0x05-07, 0A, 0D-0F  motor on
//   0x10-1F                  no force        0x20-27  resistance (a brake)
//   0x30-37                  centring spring 0x38-3F  centring spring, dead zone
//   0x40-47                  uncentring: away from the centre
//   0x50-57 / 0x60-67        constant force, one way / the other
//   0x28-2F, 48-4F, 58-5F, 68-6F  no force
//   0x70-7F  a spring parameter; 0x80-FF  status queries and nothing: the
//   effect is unchanged.
// Daytona sends 0x0-, 0x2-, 0x3-, 0x4-, 0x5-, 0x6-, 0x7- and 0xF- in a race.
//
// The result is the effect for a host force feedback device, each strength
// 0..1 relative to the board's strongest command (at the factory DIP setting).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rt {

struct DriveBoard {
    enum class Effect : uint8_t { None, Resistance, Centre, CentreDeadZone, Uncentre, Force };
    bool motor_on = false; // off at power-on, until the game turns it on
    Effect effect = Effect::None;
    int strength = 0; // n, 0..7
    int direction = 1; // Force: +1 0x5- (right), -1 0x6- (left)

    // Sega Rally (Model 2A): its board program (EPR-17891) is a different,
    // smaller one, read from its disassembly. Each command sets one constant
    // motor torque until the next: 0x00 off; 0x8n / 0x9n torque one way and
    // 0xCn / 0xDn the other, strength (n + 1) for 0x8-/0xC- and (17 + n) for
    // 0x9-/0xD-, of 32 (the board's DIP adds 0..7 more); 0x10-0x17 set a
    // duty-cycle limit on the torque (0x10: none), other 0x1- and 0x4-/0x5-
    // are not sent by the game. The game itself works out the self-centring,
    // so the board has no springs.
    bool rally = false;
    float rally_level = 0.0f; // the torque, 0..1
    float jolt = 0.0f;        // the last strong torque, fading over frames
    int jolt_dir = 1;

    void rally_command(uint8_t cmd) {
        const int hi = cmd >> 4, n = cmd & 15;
        switch (hi) {
        case 0x8:
        case 0x9:
        case 0xC:
        case 0xD:
            motor_on = true;
            set(Effect::Force, 0);
            direction = hi < 0xC ? 1 : -1;
            rally_level = std::min(1.0f, float(n + 1 + (hi & 1) * 16) / 32.0f);
            // A host wheel is far stronger than the arcade's motor, so the weak
            // road and centring torques (0x8n/0xCn) are scaled down and the strong
            // ones (0x9n/0xDn: bumps, crashes) kept, and a jolt is held a few
            // frames so that a one-frame command is felt.
            rally_level = 0.1f + 0.9f * rally_level; // the first steps are lifted over the wheel's own dead zone
            if (hi & 1) { jolt = std::max(jolt, std::min(1.0f, float(n + 17) / 32.0f * 1.3f)); jolt_dir = direction; }
            return;
        case 0x0: set(Effect::None, 0); rally_level = 0.0f; return;
        default: return;
        }
    }
    void tick() {
        jolt *= 0.8f;
        if (jolt < 0.02f) jolt = 0.0f;
    }

    void command(uint8_t cmd) {
        if (rally) return rally_command(cmd);
        const int n = cmd & 7;
        switch (cmd >> 4) {
        case 0x0: motor_on = cmd == 0x0a || n >= 5; return;
        case 0x1: set(Effect::None, 0); return;
        case 0x2: set(cmd & 8 ? Effect::None : Effect::Resistance, n); return;
        case 0x3: set(cmd & 8 ? Effect::CentreDeadZone : Effect::Centre, n); return;
        case 0x4: set(cmd & 8 ? Effect::None : Effect::Uncentre, n); return;
        case 0x5:
        case 0x6:
        {
            // a sudden rise of the push (a wall, another car, a spin) is held as a jolt
            const float before = force();
            set(cmd & 8 ? Effect::None : Effect::Force, n);
            direction = (cmd >> 4) == 0x5 ? 1 : -1;
            const float after = level(Effect::Force) * float(direction);
            if (std::fabs(after - before) > 0.3f) { jolt = std::min(1.0f, std::fabs(after - before) * 1.3f); jolt_dir = after > before ? 1 : -1; }
            return;
        }
        default: return; // 0x7-: the spring's slope (not modelled); 0x8- and up: queries, nothing
        }
    }

    // The effect's strength, 0 when the motor is off or another effect runs.
    // The board's motor power is 0..63: about 7 + 2n for the springs and
    // resistance, 17 + 2n for uncentring and the constant force (its minimum
    // and the spring parameter the game sets included); 31 is the strongest.
    float level(Effect e) const {
        if (!motor_on || effect != e || e == Effect::None) return 0.0f;
        if (rally) return rally_level;
        const bool push = e == Effect::Uncentre || e == Effect::Force;
        return float((push ? 17 : 7) + 2 * strength) / 31.0f;
    }
    // The constant force, -1 (left) .. +1 (right).
    float force() const {
        if (rally) {
            const float base = level(Effect::Force);
            return jolt > base ? jolt * float(jolt_dir) : base * float(direction);
        }
        return level(Effect::Force) * float(direction);
    }
    // The constant force for the host: with a sudden rise (a collision) held a few frames.
    float push() const {
        if (rally) return force();
        const float base = force();
        return jolt > std::fabs(base) ? jolt * float(jolt_dir) : base;
    }
    // How hard the board is working, 0..1: stands in for the cornering load.
    float load() const {
        return std::max({level(Effect::Force), level(Effect::Uncentre), level(Effect::Centre),
                         level(Effect::CentreDeadZone), level(Effect::Resistance)});
    }

    // The force the board's own closed loop drives for a wheel at x (-1 full
    // left .. +1 full right; +1 pushes right), for a host device whose driver
    // has no spring effect (the launcher's centring override). The springs
    // push towards the centre, rising over 8% of the travel beyond their dead
    // zone (2%, or 6% for 0x38-0x3F) to their level: the board's ramp, about
    // 2 power steps per position step, reaches it within a few percent.
    // Uncentring pushes away; the constant force is as it is; resistance has
    // no direction and gives nothing here.
    float motor(float x) const {
        switch (effect) {
        case Effect::Centre:
        case Effect::CentreDeadZone: {
            const float dead = effect == Effect::CentreDeadZone ? 0.06f : 0.02f;
            const float ramp = std::clamp((std::fabs(x) - dead) / 0.08f, 0.0f, 1.0f);
            return -std::copysign(level(effect) * ramp, x);
        }
        case Effect::Uncentre: return std::copysign(level(effect), x); // at the centre: right, as the board's direction 1
        case Effect::Force: return force();
        default: return 0.0f;
        }
    }

private:
    void set(Effect e, int n) { effect = e, strength = n; }
};

} // namespace rt
