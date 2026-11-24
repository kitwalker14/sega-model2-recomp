#include "runtime/scsp.h"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

int failures = 0;

void check(bool cond, const char *msg) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", msg);
        ++failures;
    }
}

void test_silence_and_midi() {
    std::vector<uint8_t> ram(512 * 1024, 0);
    snd::Scsp scsp(ram.data(), uint32_t(ram.size()));

    std::vector<uint8_t> sent;
    scsp.irq = [](int, bool) {};
    scsp.midi_out = [&](uint8_t b) { sent.push_back(b); };

    // Idle chip renders finite silence.
    std::vector<float> lr(128 * 2, 1.0f);
    scsp.generate(lr.data(), 128);
    bool all_zero = true;
    for (float s : lr) {
        if (s != 0.0f) { all_zero = false; break; }
    }
    check(all_zero, "idle SCSP renders exact silence");

    // MIDI output callback on register 0x406.
    scsp.write16(0x406, 0x00a5);
    check(sent.size() == 1 && sent[0] == 0xa5, "MIDI out register forwards byte");

    // MIDI input FIFO read back via register 0x404.
    scsp.midi_in(0x5a);
    const uint16_t midi_word = scsp.read16(0x404);
    check((midi_word & 0xff) == 0x5a, "MIDI in FIFO returns queued byte");
}

void test_timer_and_irq() {
    std::vector<uint8_t> ram(512 * 1024, 0);
    snd::Scsp scsp(ram.data(), uint32_t(ram.size()));

    int armed_which = -1;
    uint32_t armed_samples = 0;
    uint64_t armed_gen = 0;
    scsp.arm_timer = [&](int which, uint32_t samples, uint64_t gen) {
        armed_which = which;
        armed_samples = samples;
        armed_gen = gen;
    };

    int irq_level = 0;
    bool irq_asserted = false;
    scsp.irq = [&](int level, bool assert_line) {
        irq_level = level;
        irq_asserted = assert_line;
    };

    // Enable timer A interrupt (bit 6 in SCIEB @ 0x41e) and set SCILV0 bit 6 (level 1).
    scsp.write16(0x41e, 0x0040);
    scsp.write16(0x424, 0x0040);
    // Arm timer A (0x418): prescale 0, count 0xf0 -> 16 samples.
    scsp.write16(0x418, 0x00f0);
    check(armed_which == 0 && armed_samples > 0, "writing 0x418 arms timer A");

    scsp.timer_expired(0, armed_gen);
    check(irq_asserted && irq_level == 1, "timer A expiry asserts level-1 IRQ");

    // Acknowledge timer A via SCIRE @ 0x422.
    scsp.write16(0x422, 0x0040);
    check(!irq_asserted, "writing SCIRE clears timer A IRQ");
}

void test_pcm_slot_generates_audio() {
    std::vector<uint8_t> ram(512 * 1024, 0);
    // Write a 16-bit big-endian square wave at offset 0x1000 (64 samples).
    for (int i = 0; i < 64; ++i) {
        const int16_t v = (i & 8) ? 16000 : -16000;
        const uint32_t a = 0x1000 + uint32_t(i) * 2;
        ram[a] = uint8_t(uint16_t(v) >> 8);
        ram[a + 1] = uint8_t(uint16_t(v) & 0xff);
    }

    snd::Scsp scsp(ram.data(), uint32_t(ram.size()));
    scsp.irq = [](int, bool) {};
    scsp.write16(0x400, 0x000f); // master volume = 15 (max)

    // Slot 0: 16-bit PCM, normal loop, SA=0x1000, LSA=0, LEA=63, instant attack, full direct send.
    scsp.write16(0x002, 0x1000); // SA low
    scsp.write16(0x004, 0x0000); // LSA
    scsp.write16(0x006, 0x003f); // LEA
    scsp.write16(0x008, 0x001f); // AR = 31 (instant attack)
    scsp.write16(0x00a, 0x001f); // RR = 31
    scsp.write16(0x00c, 0x0000); // TL = 0 (loudest)
    scsp.write16(0x010, 0x0000); // OCT=0, FNS=0 (1x pitch)
    scsp.write16(0x016, 0xe000); // DISDL = 7 (max), DIPAN = 0 (center)
    scsp.write16(0x000, 0x1820); // KYONEX | KYONB | LPCTL=1 (normal loop)

    std::vector<float> lr(128 * 2, 0.0f);
    scsp.generate(lr.data(), 128);

    float peak = 0.0f;
    bool all_finite = true;
    for (float s : lr) {
        if (!std::isfinite(s)) all_finite = false;
        peak = std::max(peak, std::fabs(s));
    }
    check(all_finite, "active SCSP slot output is finite");
    check(peak > 0.01f, "active SCSP slot produces non-zero audio");
}

} // namespace

int main() {
    test_silence_and_midi();
    test_timer_and_irq();
    test_pcm_slot_generates_audio();
    if (failures) {
        std::fprintf(stderr, "test_scsp: %d failure(s)\n", failures);
        return 1;
    }
    return 0;
}
