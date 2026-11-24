#include "runtime/drive_board.h"

#include <cmath>
#include <cstdio>

namespace {

int failures = 0;

void check(bool cond, const char *msg) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", msg);
        ++failures;
    }
}

void test_daytona_mode() {
    rt::DriveBoard db;
    check(!db.motor_on, "motor starts off");
    db.command(0x54);
    check(db.level(rt::DriveBoard::Effect::Force) == 0.0f, "force is zero while motor off");

    db.command(0x07);
    check(db.motor_on, "0x07 turns motor on");
    check(db.force() > 0.5f, "0x54 produces positive constant force");

    // Bit 3 clears Resistance/Uncentre/Force on Daytona's 838-10646 board,
    // while selecting CentreDeadZone on 0x3-.
    db.command(0x3a);
    check(db.effect == rt::DriveBoard::Effect::CentreDeadZone, "0x3a selects CentreDeadZone");
    check(db.motor(0.03f) == 0.0f, "CentreDeadZone has deadband around 0");
    check(db.motor(0.5f) < 0.0f, "CentreDeadZone pulls left when wheel is right");

    db.command(0x5f);
    check(db.effect == rt::DriveBoard::Effect::None, "0x5f is Effect::None on Daytona");

    // Sudden force jump triggers jolt, which decays via tick().
    db.command(0x67);
    check(db.jolt > 0.5f && db.jolt_dir == -1, "sudden 0x67 push triggers left jolt");
    const float j0 = db.jolt;
    db.tick();
    check(db.jolt < j0, "tick() decays jolt");
}

void test_rally_mode() {
    rt::DriveBoard db;
    db.rally = true;

    // Weak right torque (0x8n) and strong left torque + jolt (0xDn).
    db.command(0x84);
    check(db.motor_on, "rally 0x84 enables motor");
    check(db.effect == rt::DriveBoard::Effect::Force && db.direction == 1, "0x84 pushes right");
    check(db.jolt == 0.0f, "0x8n does not trigger jolt");

    db.command(0xd8);
    check(db.direction == -1, "0xd8 pushes left");
    check(db.jolt > 0.5f && db.jolt_dir == -1, "0xd8 triggers strong left jolt");
    check(db.force() < -0.5f, "0xd8 force is strong left");

    db.command(0x00);
    check(db.effect == rt::DriveBoard::Effect::None && db.rally_level == 0.0f, "0x00 clears rally torque");
}

} // namespace

int main() {
    test_daytona_mode();
    test_rally_mode();
    if (failures) {
        std::fprintf(stderr, "test_drive_board: %d failure(s)\n", failures);
        return 1;
    }
    return 0;
}
