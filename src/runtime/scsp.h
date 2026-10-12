// Yamaha YMF292-F "SCSP" (Sega Saturn Custom Sound Processor), the Model 2A
// sound board's sound chip, as native C++. Transplanted from MAME's scsp.cpp
// (BSD-3-Clause, ElSemi, R. Belmont, kingshriek; THIRD_PARTY.md) with the
// device framework removed: the sound board calls read16()/write16() on the
// 68000's register accesses and generate() to render samples at 44.1 kHz
// (clock / 512). Slot, envelope, LFO, panning, timer and interrupt
// arithmetic is MAME's. The effect DSP and the
// register-driven DMA are included; the external audio inputs are not.
//
// The chip reads its waveforms from the 68000's 512 KiB work RAM (the board's
// SCSP map), which the sound board hands in.
#pragma once

#include <cstdint>
#include <functional>
#include <vector>

namespace snd {

class Scsp {
public:
    static constexpr uint32_t kClock = 22579200; // 45.1584 MHz / 2
    static constexpr double kRate = kClock / 512.0;

    // ram: the chip's sample memory (the 68000's RAM at 0); size a power of two.
    Scsp(uint8_t *ram, uint32_t ram_size);

    // 68000 interrupt line: level (1..7), asserted or cleared.
    std::function<void(int level, bool assert_line)> irq;
    // Arm timer `which` (0 A, 1 B, 2 C) to expire after `samples` output
    // samples; the board calls timer_expired(which, generation) then.
    std::function<void(int which, uint32_t samples, uint64_t generation)> arm_timer;

    // A byte the 68000 wrote to the MIDI output (the line back to the i960's UART).
    std::function<void(uint8_t)> midi_out;

    uint16_t read16(uint32_t addr);
    void write16(uint32_t addr, uint16_t val);
    void timer_expired(int which, uint64_t generation);
    // A byte arrived on the MIDI input (the i960's command line).
    void midi_in(uint8_t byte);
    // n stereo samples, interleaved, MAME's put_int_clamp(v, 32768) scale.
    void generate(float *lr, int n);

private:
    enum State { Attack, Decay1, Decay2, Release };
    struct Lfo {
        uint16_t phase = 0;
        uint32_t phase_step = 0;
        const int *table = nullptr, *scale = nullptr;
    };
    struct Eg {
        int volume = 0, state = Release, AR = 0, D1R = 0, D2R = 0, RR = 0, DL = 0;
        uint8_t EGHOLD = 0;
    };
    struct Slot {
        uint16_t data[0x10] = {};
        uint8_t active = 0, backwards = 0;
        uint32_t cur_addr = 0, nxt_addr = 0, step = 0;
        Eg EG;
        Lfo PLFO, ALFO;
        int slot = 0;
    };

    uint8_t read_byte(uint32_t a) const { return a < ram_size_ ? ram_[a] : 0; }
    uint16_t read_word(uint32_t a) const {
        return a + 1 < ram_size_ ? uint16_t(ram_[a] << 8 | ram_[a + 1]) : 0;
    }

    int Get_AR(int base, int R) const;
    int Get_DR(int base, int R) const;
    void Compute_EG(Slot *slot);
    int EG_Update(Slot *slot);
    uint32_t Step(Slot *slot);
    void Compute_LFO(Slot *slot);
    void StartSlot(Slot *slot);
    void StopSlot(Slot *slot, int keyoff);
    void UpdateSlotReg(int s, int r);
    void UpdateReg(int reg);
    void UpdateRegR(int reg);
    uint8_t DecodeSCI(uint8_t irq) const;
    void CheckPendingIRQ();
    void ResetInterrupts();
    void arm(int which);
    int32_t UpdateSlot(Slot *slot);
    void LFO_Init();
    int32_t PLFO_Step(Lfo *lfo);
    int32_t ALFO_Step(Lfo *lfo);
    void LFO_ComputeStep(Lfo *lfo, uint32_t LFOF, uint32_t LFOWS, uint32_t LFOS, int alfo);
    uint32_t rand16();

    void write_word(uint32_t a, uint16_t v) {
        if (a + 1 < ram_size_) { ram_[a] = uint8_t(v >> 8); ram_[a + 1] = uint8_t(v); }
    }
    void exec_dma();
    void dsp_step();
    void dsp_start();
    uint16_t dsp_read(uint32_t addr) const;
    void dsp_write(uint32_t addr, uint16_t val);

    uint8_t *ram_;
    uint32_t ram_size_;

    struct Dma { uint32_t dmea = 0, drga = 0, dtlg = 0; int dgate = 0, ddir = 0; } dma_;
    uint8_t irq_dma_ = 0;
    // The effects DSP (MAME's SCSPDSP).
    struct Dsp {
        uint32_t RBP = 0, RBL = 8 * 1024;
        int16_t COEF[64] = {};
        uint16_t MADRS[32] = {};
        uint16_t MPRO[128 * 4] = {};
        int32_t TEMP[128] = {}, MEMS[32] = {};
        uint32_t DEC = 0;
        int32_t MIXS[16] = {};
        int16_t EXTS[2] = {};
        int16_t EFREG[16] = {};
        bool Stopped = true;
        int LastStep = 0;
    } dsp_;

    Slot slots_[32];
    uint16_t udata_[0x30 / 2] = {};
    int16_t ringbuf_[128] = {};
    uint8_t bufptr_ = 0;
    int16_t *rbufdst_ = nullptr;

    uint8_t latched_mslc_ = 0;
    uint16_t latched_mslc_data_ = 0;

    uint8_t irq_tima_ = 0, irq_timbc_ = 0, irq_midi_ = 0, irq_cpu_ = 0;
    uint8_t midi_stack_[32] = {}, midi_w_ = 0, midi_r_ = 0;
    uint32_t tim_pris_[3] = {}, tim_cnt_[3] = {};
    uint64_t tim_gen_[3] = {};

    std::vector<int32_t> eg_table_ = std::vector<int32_t>(0x400);
    std::vector<int> lpan_ = std::vector<int>(0x10000), rpan_ = std::vector<int>(0x10000);
    int artable_[64] = {}, drtable_[64] = {};
    int plfo_tri_[256], plfo_sqr_[256], plfo_saw_[256], plfo_noi_[256];
    int alfo_tri_[256], alfo_sqr_[256], alfo_saw_[256], alfo_noi_[256];
    int pscales_[8][256], ascales_[8][256];
    uint32_t rng_ = 0x2545f491;
};

} // namespace snd
