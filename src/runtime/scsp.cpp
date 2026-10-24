// Transplanted from MAME src/devices/sound/scsp.cpp (BSD-3-Clause,
// ElSemi, R. Belmont, kingshriek); see scsp.h.
#include "runtime/scsp.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace snd {

namespace {
constexpr int SHIFT = 12, LFO_SHIFT = 8, EG_SHIFT = 16;
using u32 = uint32_t; using u16 = uint16_t; using u8 = uint8_t; using s32 = int32_t; using s16 = int16_t;

const double ARTimes[64] = {100000, 100000, 8100.0, 6900.0, 6000.0, 4800.0, 4000.0, 3400.0, 3000.0, 2400.0, 2000.0, 1700.0, 1500.0,
    1200.0, 1000.0, 860.0, 760.0, 600.0, 500.0, 430.0, 380.0, 300.0, 250.0, 220.0, 190.0, 150.0, 130.0, 110.0, 95.0,
    76.0, 63.0, 55.0, 47.0, 38.0, 31.0, 27.0, 24.0, 19.0, 15.0, 13.0, 12.0, 9.4, 7.9, 6.8, 6.0, 4.7, 3.8, 3.4, 3.0, 2.4,
    2.0, 1.8, 1.6, 1.3, 1.1, 0.93, 0.85, 0.65, 0.53, 0.44, 0.40, 0.35, 0.0, 0.0};
const double DRTimes[64] = {100000, 100000, 118200.0, 101300.0, 88600.0, 70900.0, 59100.0, 50700.0, 44300.0, 35500.0, 29600.0, 25300.0, 22200.0, 17700.0,
    14800.0, 12700.0, 11100.0, 8900.0, 7400.0, 6300.0, 5500.0, 4400.0, 3700.0, 3200.0, 2800.0, 2200.0, 1800.0, 1600.0, 1400.0, 1100.0,
    920.0, 790.0, 690.0, 550.0, 460.0, 390.0, 340.0, 270.0, 230.0, 200.0, 170.0, 140.0, 110.0, 98.0, 85.0, 68.0, 57.0, 49.0, 43.0, 34.0,
    28.0, 25.0, 22.0, 18.0, 14.0, 12.0, 11.0, 8.5, 7.1, 6.1, 5.4, 4.3, 3.6, 3.1};
const float SDLT[8] = {-1000000.0f, -36.0f, -30.0f, -24.0f, -18.0f, -12.0f, -6.0f, 0.0f};
const float LFOFreq[32] = {
    0.17f, 0.19f, 0.23f, 0.27f, 0.34f, 0.39f, 0.45f, 0.55f, 0.68f, 0.78f, 0.92f, 1.10f, 1.39f, 1.60f, 1.87f, 2.27f,
    2.87f, 3.31f, 3.92f, 4.79f, 6.15f, 7.18f, 8.60f, 10.8f, 14.4f, 17.2f, 21.5f, 28.7f, 43.1f, 57.4f, 86.1f, 172.3f};
const float ASCALE[8] = {0.0f, 0.4f, 0.8f, 1.5f, 3.0f, 6.0f, 12.0f, 24.0f};
const float PSCALE[8] = {0.0f, 7.0f, 13.5f, 27.0f, 55.0f, 112.0f, 230.0f, 494.0f};

template <class S> inline u32 KEYONB(const S &s) { return s.data[0] & 0x0800; }
template <class S> inline u32 KEYONEX(const S &s) { return s.data[0] & 0x1000; }
template <class S> inline u32 SBCTL(const S &s) { return (s.data[0] >> 9) & 3; }
template <class S> inline u32 SSCTL(const S &s) { return (s.data[0] >> 7) & 3; }
template <class S> inline u32 LPCTL(const S &s) { return (s.data[0] >> 5) & 3; }
template <class S> inline u32 PCM8B(const S &s) { return s.data[0] & 0x10; }
template <class S> inline u32 SA(const S &s) { return ((s.data[0] & 0xF) << 16) | s.data[1]; }
template <class S> inline u32 LSA(const S &s) { return s.data[2]; }
template <class S> inline u32 LEA(const S &s) { return s.data[3]; }
template <class S> inline u32 D2R(const S &s) { return (s.data[4] >> 0xB) & 0x1F; }
template <class S> inline u32 D1R(const S &s) { return (s.data[4] >> 6) & 0x1F; }
template <class S> inline u32 EGHOLD(const S &s) { return s.data[4] & 0x20; }
template <class S> inline u32 AR(const S &s) { return s.data[4] & 0x1F; }
template <class S> inline u32 LPSLNK(const S &s) { return s.data[5] & 0x4000; }
template <class S> inline u32 KRS(const S &s) { return (s.data[5] >> 0xA) & 0xF; }
template <class S> inline u32 DL(const S &s) { return (s.data[5] >> 5) & 0x1F; }
template <class S> inline u32 RR(const S &s) { return s.data[5] & 0x1F; }
template <class S> inline u32 STWINH(const S &s) { return s.data[6] & 0x200; }
template <class S> inline u32 SDIR(const S &s) { return s.data[6] & 0x100; }
template <class S> inline u32 TL(const S &s) { return s.data[6] & 0xFF; }
template <class S> inline u32 MDL(const S &s) { return (s.data[7] >> 0xC) & 0xF; }
template <class S> inline u32 MDXSL(const S &s) { return (s.data[7] >> 6) & 0x3F; }
template <class S> inline u32 MDYSL(const S &s) { return s.data[7] & 0x3F; }
template <class S> inline u32 OCT(const S &s) { return (s.data[8] >> 0xB) & 0xF; }
template <class S> inline u32 FNS(const S &s) { return s.data[8] & 0x3FF; }
template <class S> inline u32 LFOF(const S &s) { return (s.data[9] >> 0xA) & 0x1F; }
template <class S> inline u32 PLFOWS(const S &s) { return (s.data[9] >> 8) & 3; }
template <class S> inline u32 PLFOS(const S &s) { return (s.data[9] >> 5) & 7; }
template <class S> inline u32 ALFOWS(const S &s) { return (s.data[9] >> 3) & 3; }
template <class S> inline u32 ALFOS(const S &s) { return s.data[9] & 7; }
template <class S> inline u32 IMXL(const S &s) { return s.data[0xA] & 7; }
template <class S> inline u32 DISDL(const S &s) { return (s.data[0xB] >> 0xD) & 7; }
template <class S> inline u32 ISEL(const S &s) { return (s.data[0xA] >> 3) & 0xF; }
template <class S> inline u32 EFSDL(const S &s) { return (s.data[0xB] >> 5) & 7; }
template <class S> inline u32 EFPAN(const S &s) { return s.data[0xB] & 0x1F; }
template <class S> inline u32 DIPAN(const S &s) { return (s.data[0xB] >> 8) & 0x1F; }

inline s32 sext(s32 v, int bits) { return s32(u32(v) << (32 - bits)) >> (32 - bits); }

u16 PACK(s32 val) {
    const int sign = (val >> 23) & 1;
    u32 temp = (val ^ (val << 1)) & 0xFFFFFF;
    int exponent = 0;
    for (int k = 0; k < 12; k++) {
        if (temp & 0x800000) break;
        temp <<= 1;
        exponent += 1;
    }
    if (exponent < 12) val = (val << exponent) & 0x3FFFFF;
    else val <<= 11;
    val >>= 11;
    val &= 0x7FF;
    val |= sign << 15;
    val |= exponent << 11;
    return u16(val);
}

s32 UNPACK(u16 val) {
    const int sign = (val >> 15) & 1;
    int exponent = (val >> 11) & 0xF;
    const int mantissa = val & 0x7FF;
    s32 uval = mantissa << 11;
    if (exponent > 11) {
        exponent = 11;
        uval |= sign << 22;
    } else {
        uval |= (sign ^ 1) << 22;
    }
    uval |= sign << 23;
    uval = sext(uval, 24);
    uval >>= exponent;
    return uval;
}
} // namespace

Scsp::Scsp(uint8_t *ram, uint32_t ram_size) : ram_(ram), ram_size_(ram_size) {
    std::fill(std::begin(eg_table_), std::end(eg_table_), 0);
    for (int i = 0; i < 0x400; ++i) {
        float envDB = float(3 * (i - 0x3ff)) / 32.0f;
        float scale = float(1 << SHIFT);
        eg_table_[i] = s32(powf(10.0f, envDB / 20.0f) * scale);
    }
    for (int i = 0; i < 0x10000; ++i) {
        int iTL = (i >> 0) & 0xff, iPAN = (i >> 8) & 0x1f, iSDL = (i >> 0xD) & 7;
        float SegaDB = 0.0f;
        if (iTL & 0x01) SegaDB -= 0.4f;
        if (iTL & 0x02) SegaDB -= 0.8f;
        if (iTL & 0x04) SegaDB -= 1.5f;
        if (iTL & 0x08) SegaDB -= 3.0f;
        if (iTL & 0x10) SegaDB -= 6.0f;
        if (iTL & 0x20) SegaDB -= 12.0f;
        if (iTL & 0x40) SegaDB -= 24.0f;
        if (iTL & 0x80) SegaDB -= 48.0f;
        const float TL = powf(10.0f, SegaDB / 20.0f);
        SegaDB = 0;
        if (iPAN & 0x1) SegaDB -= 3.0f;
        if (iPAN & 0x2) SegaDB -= 6.0f;
        if (iPAN & 0x4) SegaDB -= 12.0f;
        if (iPAN & 0x8) SegaDB -= 24.0f;
        float PAN, LPAN, RPAN;
        if ((iPAN & 0xf) == 0xf) PAN = 0.0f;
        else PAN = powf(10.0f, SegaDB / 20.0f);
        if (iPAN < 0x10) { LPAN = PAN; RPAN = 1.0f; }
        else { RPAN = PAN; LPAN = 1.0f; }
        const float fSDL = iSDL ? powf(10.0f, SDLT[iSDL] / 20.0f) : 0.0f;
        lpan_[i] = int(u32(float(1 << SHIFT) * (4.0f * LPAN * TL * fSDL)));
        rpan_[i] = int(u32(float(1 << SHIFT) * (4.0f * RPAN * TL * fSDL)));
    }
    artable_[0] = drtable_[0] = 0;
    artable_[1] = drtable_[1] = 0;
    for (int i = 2; i < 64; ++i) {
        double t = ARTimes[i], step, scale;
        if (t != 0.0) {
            step = (1023 * 1000.0) / (44100.0 * t);
            scale = double(1 << EG_SHIFT);
            artable_[i] = int(step * scale);
        } else {
            artable_[i] = 1024 << EG_SHIFT;
        }
        t = DRTimes[i];
        step = (1023 * 1000.0) / (44100.0 * t);
        scale = double(1 << EG_SHIFT);
        drtable_[i] = int(step * scale);
    }
    for (int i = 0; i < 32; ++i) {
        slots_[i].slot = i;
        slots_[i].active = 0;
        slots_[i].EG.state = Release;
    }
    LFO_Init();
    udata_[0x20 / 2] = 0;
    tim_cnt_[0] = tim_cnt_[1] = tim_cnt_[2] = 0xffff;
}

uint32_t Scsp::rand16() {
    rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
    return rng_ >> 8;
}

uint8_t Scsp::DecodeSCI(uint8_t irqn) const {
    const u32 lv0 = udata_[0x24 / 2] & 0xff, lv1 = udata_[0x26 / 2] & 0xff, lv2 = udata_[0x28 / 2] & 0xff;
    uint8_t sci = 0;
    sci |= (lv0 & (1 << irqn)) ? 1 : 0;
    sci |= ((lv1 & (1 << irqn)) ? 1 : 0) << 1;
    sci |= ((lv2 & (1 << irqn)) ? 1 : 0) << 2;
    return sci;
}

void Scsp::CheckPendingIRQ() {
    u32 pend = udata_[0x20 / 2];
    const u32 en = udata_[0x1e / 2];
    if (midi_w_ != midi_r_) {
        udata_[0x20 / 2] |= 8;
        pend |= 8;
    }
    if (!pend) return;
    if (pend & en & 0x20) { irq(irq_cpu_, true); return; }
    if ((pend & 0x40) && (en & 0x40)) { irq(irq_tima_, true); return; }
    if ((pend & 0x80) && (en & 0x80)) { irq(irq_timbc_, true); return; }
    if ((pend & 0x100) && (en & 0x100)) { irq(irq_timbc_, true); return; }
    if ((pend & 8) && (en & 8)) { irq(irq_midi_, true); return; }
    irq(0, false);
}

void Scsp::ResetInterrupts() {
    const u32 reset = udata_[0x22 / 2];
    if (reset & 0x40) irq(irq_tima_, false);
    if (reset & 0x180) irq(irq_timbc_, false);
    if (reset & 0x20) {
        udata_[0x20 / 2] &= ~0x20;
        irq(irq_cpu_, false);
    }
    if (reset & 0x8) irq(irq_midi_, false);
    CheckPendingIRQ();
}

void Scsp::arm(int which) {
    // MAME: one-shot after 512 / ((clock / prescale) / (255 - count)) seconds,
    // that is prescale * (255 - count) output samples.
    const int reg = 0x18 / 2 + which;
    tim_pris_[which] = 1u << ((udata_[reg] >> 8) & 7);
    tim_cnt_[which] = (udata_[reg] & 0xff) << 8;
    ++tim_gen_[which];
    if ((udata_[reg] & 0xff) != 255 && arm_timer)
        arm_timer(which, tim_pris_[which] * (255 - (udata_[reg] & 0xff)), tim_gen_[which]);
}

void Scsp::timer_expired(int which, uint64_t generation) {
    if (tim_gen_[which] != generation) return;
    tim_cnt_[which] = 0xFFFF;
    udata_[0x20 / 2] |= which == 0 ? 0x40 : which == 1 ? 0x80 : 0x100;
    udata_[0x18 / 2 + which] &= 0xff00;
    udata_[0x18 / 2 + which] |= tim_cnt_[which] >> 8;
    CheckPendingIRQ();
}

void Scsp::midi_in(uint8_t byte) {
    midi_stack_[midi_w_++] = byte;
    midi_w_ &= 31;
    CheckPendingIRQ();
}

int Scsp::Get_AR(int base, int R) const { return artable_[std::clamp(base + (R << 1), 0, 63)]; }
int Scsp::Get_DR(int base, int R) const { return drtable_[std::clamp(base + (R << 1), 0, 63)]; }

void Scsp::Compute_EG(Slot *slot) {
    const int octave = (int(OCT(*slot)) ^ 8) - 8;
    int rate;
    if (KRS(*slot) != 0xf) rate = octave + 2 * int(KRS(*slot)) + int((FNS(*slot) >> 9) & 1);
    else rate = 0;
    slot->EG.volume = 0x17F << EG_SHIFT;
    slot->EG.AR = Get_AR(rate, int(AR(*slot)));
    slot->EG.D1R = Get_DR(rate, int(D1R(*slot)));
    slot->EG.D2R = Get_DR(rate, int(D2R(*slot)));
    slot->EG.RR = Get_DR(rate, int(RR(*slot)));
    slot->EG.DL = 0x1f - int(DL(*slot));
    slot->EG.EGHOLD = EGHOLD(*slot) != 0;
}

int Scsp::EG_Update(Slot *slot) {
    switch (slot->EG.state) {
    case Attack:
        slot->EG.volume += slot->EG.AR;
        if (slot->EG.volume >= (0x3ff << EG_SHIFT)) {
            if (!LPSLNK(*slot)) {
                slot->EG.state = Decay1;
                if (slot->EG.D1R >= (1024 << EG_SHIFT)) slot->EG.state = Decay2;
            }
            slot->EG.volume = 0x3ff << EG_SHIFT;
        }
        if (slot->EG.EGHOLD) return 0x3ff << (SHIFT - 10);
        break;
    case Decay1:
        slot->EG.volume -= slot->EG.D1R;
        if (slot->EG.volume <= 0) slot->EG.volume = 0;
        if (slot->EG.volume >> (EG_SHIFT + 5) <= slot->EG.DL) slot->EG.state = Decay2;
        break;
    case Decay2:
        if (D2R(*slot) == 0) return (slot->EG.volume >> EG_SHIFT) << (SHIFT - 10);
        slot->EG.volume -= slot->EG.D2R;
        if (slot->EG.volume <= 0) slot->EG.volume = 0;
        break;
    case Release:
        slot->EG.volume -= slot->EG.RR;
        if (slot->EG.volume <= 0) {
            slot->EG.volume = 0;
            StopSlot(slot, 0);
        }
        break;
    default:
        return 1 << SHIFT;
    }
    return (slot->EG.volume >> EG_SHIFT) << (SHIFT - 10);
}

uint32_t Scsp::Step(Slot *slot) {
    int octave = (int(OCT(*slot)) ^ 8) - 8 + SHIFT - 10;
    u32 Fn = FNS(*slot) + (1 << 10);
    if (octave >= 0) Fn <<= octave;
    else Fn >>= -octave;
    return Fn;
}

void Scsp::Compute_LFO(Slot *slot) {
    if (PLFOS(*slot) != 0) LFO_ComputeStep(&slot->PLFO, LFOF(*slot), PLFOWS(*slot), PLFOS(*slot), 0);
    if (ALFOS(*slot) != 0) LFO_ComputeStep(&slot->ALFO, LFOF(*slot), ALFOWS(*slot), ALFOS(*slot), 1);
}

void Scsp::StartSlot(Slot *slot) {
    slot->active = 1;
    slot->cur_addr = 0;
    slot->nxt_addr = 1 << SHIFT;
    slot->step = Step(slot);
    Compute_EG(slot);
    slot->EG.state = Attack;
    slot->EG.volume = 0x17F << EG_SHIFT;
    slot->backwards = 0;
    Compute_LFO(slot);
}

void Scsp::StopSlot(Slot *slot, int keyoff) {
    if (keyoff) slot->EG.state = Release;
    else slot->active = 0;
    slot->data[0] &= ~0x800;
}

void Scsp::UpdateSlotReg(int s, int r) {
    Slot *slot = slots_ + s;
    switch (r & 0x3f) {
    case 0:
    case 1:
        if (KEYONEX(*slot)) {
            for (int sl = 0; sl < 32; ++sl) {
                Slot *s2 = slots_ + sl;
                if (KEYONB(*s2) && s2->EG.state == Release) StartSlot(s2);
                if (!KEYONB(*s2)) StopSlot(s2, 1);
            }
            slot->data[0] &= ~0x1000;
        }
        break;
    case 0x10:
    case 0x11:
        slot->step = Step(slot);
        break;
    case 0xA:
    case 0xB:
        slot->EG.RR = Get_DR(0, int(RR(*slot)));
        slot->EG.DL = 0x1f - int(DL(*slot));
        break;
    case 0x12:
    case 0x13:
        Compute_LFO(slot);
        break;
    }
}

void Scsp::UpdateReg(int reg) {
    switch (reg & 0x3f) {
    case 0x8:
    case 0x9:
        latched_mslc_ = (udata_[0x8 / 2] & 0xf800) >> 11;
        break;
    case 0x18: case 0x19: arm(0); break;
    case 0x1a: case 0x1b: arm(1); break;
    case 0x1c: case 0x1d: arm(2); break;
    case 0x1e: case 0x1f: CheckPendingIRQ(); break;
    case 0x20: case 0x21:
        if (udata_[0x1e / 2] & udata_[0x20 / 2] & 0x20) CheckPendingIRQ();
        break;
    case 0x22: case 0x23:
        udata_[0x20 / 2] &= ~udata_[0x22 / 2];
        ResetInterrupts();
        if (tim_cnt_[0] == 0xffff) udata_[0x20 / 2] |= 0x40;
        if (tim_cnt_[1] == 0xffff) udata_[0x20 / 2] |= 0x80;
        if (tim_cnt_[2] == 0xffff) udata_[0x20 / 2] |= 0x100;
        break;
    case 0x24: case 0x25: case 0x26: case 0x27: case 0x28: case 0x29:
        irq_tima_ = DecodeSCI(6);
        irq_timbc_ = DecodeSCI(7);
        irq_midi_ = DecodeSCI(3);
        irq_cpu_ = DecodeSCI(5);
        break;
    case 0x2: case 0x3:
        dsp_.RBL = (8 * 1024) << ((udata_[1] >> 7) & 3);
        dsp_.RBP = udata_[1] & 0x3f;
        break;
    case 0x12: case 0x13:
        dma_.dmea = (udata_[0x12 / 2] & 0xfffe) | (dma_.dmea & 0xf0000);
        break;
    case 0x14: case 0x15:
        dma_.dmea = ((udata_[0x14 / 2] & 0xf000) << 4) | (dma_.dmea & 0xfffe);
        dma_.drga = udata_[0x14 / 2] & 0x0ffe;
        break;
    case 0x16: case 0x17:
        dma_.dtlg = udata_[0x16 / 2] & 0x0ffe;
        dma_.ddir = (udata_[0x16 / 2] & 0x2000) >> 13;
        dma_.dgate = (udata_[0x16 / 2] & 0x4000) >> 14;
        if (udata_[0x16 / 2] & 0x1000) exec_dma();
        break;
    case 0x6: case 0x7:
        if (midi_out) midi_out(uint8_t(udata_[0x6 / 2]));
        break;
    default: break; // main-CPU interrupt registers: not modelled
    }
}

void Scsp::UpdateRegR(int reg) {
    switch (reg & 0x3f) {
    case 4:
    case 5: {
        u16 v = udata_[0x4 / 2];
        v &= 0xff00;
        v |= midi_stack_[midi_r_];
        if (midi_r_ != midi_w_) {
            ++midi_r_;
            midi_r_ &= 31;
        }
        if (midi_r_ == midi_w_) { // input FIFO empty: clear the IRQ
            irq(irq_midi_, false);
            udata_[0x20 / 2] &= ~8;
        }
        udata_[0x4 / 2] = v;
        break;
    }
    case 8:
    case 9:
        udata_[0x8 / 2] = latched_mslc_data_;
        break;
    default: break;
    }
}

void Scsp::write16(uint32_t addr, uint16_t val) {
    addr &= 0xffff;
    if (addr < 0x400) {
        const int slot = int(addr / 0x20);
        addr &= 0x1f;
        slots_[slot].data[addr / 2] = val;
        UpdateSlotReg(slot, int(addr & 0x1f));
    } else if (addr < 0x600) {
        if (addr < 0x430) {
            // SCIPD and MCIPD are r/o except for bit 5 CPU irqs
            if (addr == 0x420 || addr == 0x42e) udata_[(addr & 0x3f) / 2] |= val & 0x20;
            else udata_[(addr & 0x3f) / 2] = val;
            UpdateReg(int(addr & 0x3f));
        }
    } else if (addr < 0x700) {
        ringbuf_[(addr - 0x600) / 2] = int16_t(val);
    } else {
        dsp_write(addr, val);
    }
}

uint16_t Scsp::read16(uint32_t addr) {
    u16 v = 0;
    addr &= 0xffff;
    if (addr < 0x400) {
        v = slots_[addr / 0x20].data[(addr & 0x1f) / 2];
    } else if (addr < 0x600) {
        if (addr < 0x430) {
            UpdateRegR(int(addr & 0x3f));
            v = udata_[(addr & 0x3f) / 2];
        }
    } else if (addr < 0x700) {
        v = uint16_t(ringbuf_[(addr - 0x600) / 2]);
    } else {
        v = dsp_read(addr);
    }
    return v;
}

int32_t Scsp::UpdateSlot(Slot *slot) {
    if (SSCTL(*slot) == 3) return 0; // manual says cannot be used
    s32 sample = 0;
    int step = int(slot->step);
    u32 addr1, addr2, addr_select;
    u32 *addr[2] = {&addr1, &addr2};
    u32 *slot_addr[2] = {&slot->cur_addr, &slot->nxt_addr};

    if (PLFOS(*slot) != 0) {
        step = step * PLFO_Step(&slot->PLFO);
        step >>= SHIFT;
    }
    if (PCM8B(*slot)) {
        addr1 = slot->cur_addr >> SHIFT;
        addr2 = slot->nxt_addr >> SHIFT;
    } else {
        addr1 = (slot->cur_addr >> (SHIFT - 1)) & ~1u;
        addr2 = (slot->nxt_addr >> (SHIFT - 1)) & ~1u;
    }
    if (MDL(*slot) != 0 || MDXSL(*slot) != 0 || MDYSL(*slot) != 0) {
        s32 smp = (ringbuf_[(bufptr_ + MDXSL(*slot)) & 63] + ringbuf_[(bufptr_ + MDYSL(*slot)) & 63]) / 2;
        smp <<= 0xA;
        smp >>= 0x1A - MDL(*slot);
        if (!PCM8B(*slot)) smp <<= 1;
        addr1 += smp;
        addr2 += smp;
    }
    if (SSCTL(*slot) == 0) { // external DRAM data
        if (PCM8B(*slot)) {
            const int8_t p1 = int8_t(read_byte(SA(*slot) + addr1));
            const int8_t p2 = int8_t(read_byte(SA(*slot) + addr2));
            const s32 fpart = s32(slot->cur_addr & ((1 << SHIFT) - 1));
            const s32 s = int(p1 << 8) * ((1 << SHIFT) - fpart) + int(p2 << 8) * fpart;
            sample = s >> SHIFT;
        } else {
            const s16 p1 = s16(read_word(SA(*slot) + addr1));
            const s16 p2 = s16(read_word(SA(*slot) + addr2));
            const s32 fpart = s32(slot->cur_addr & ((1 << SHIFT) - 1));
            const s32 s = int(p1) * ((1 << SHIFT) - fpart) + int(p2) * fpart;
            sample = s >> SHIFT;
        }
    } else if (SSCTL(*slot) == 1) {
        sample = s16(rand16() & 0xffff); // internally generated noise
    } else {
        sample = 0;
    }
    if (SBCTL(*slot) & 0x1) sample ^= 0x7FFF;
    if (SBCTL(*slot) & 0x2) sample = s16(sample ^ 0x8000);

    if (slot->backwards) slot->cur_addr -= u32(step);
    else slot->cur_addr += u32(step);
    slot->nxt_addr = slot->cur_addr + (1 << SHIFT);
    addr1 = slot->cur_addr >> SHIFT;
    addr2 = slot->nxt_addr >> SHIFT;

    if (addr1 >= LSA(*slot) && !slot->backwards) {
        if (LPSLNK(*slot) && slot->EG.state == Attack) slot->EG.state = Decay1;
    }
    for (addr_select = 0; addr_select < 2; addr_select++) {
        s32 rem_addr;
        switch (LPCTL(*slot)) {
        case 0: // no loop
            if (*addr[addr_select] >= LSA(*slot) && *addr[addr_select] >= LEA(*slot)) StopSlot(slot, 0);
            break;
        case 1: // normal loop
            if (*addr[addr_select] >= LEA(*slot)) {
                rem_addr = s32(*slot_addr[addr_select] - (LEA(*slot) << SHIFT));
                *slot_addr[addr_select] = u32((LSA(*slot) << SHIFT) + rem_addr);
            }
            break;
        case 2: // reverse loop
            if ((*addr[addr_select] >= LSA(*slot)) && !slot->backwards) {
                rem_addr = s32(*slot_addr[addr_select] - (LSA(*slot) << SHIFT));
                *slot_addr[addr_select] = u32((LEA(*slot) << SHIFT) - rem_addr);
                slot->backwards = 1;
            } else if ((*addr[addr_select] < LSA(*slot) || (*slot_addr[addr_select] & 0x80000000)) && slot->backwards) {
                rem_addr = s32((LSA(*slot) << SHIFT) - *slot_addr[addr_select]);
                *slot_addr[addr_select] = u32((LEA(*slot) << SHIFT) - rem_addr);
            }
            break;
        case 3: // ping-pong
            if (*addr[addr_select] >= LEA(*slot)) {
                rem_addr = s32(*slot_addr[addr_select] - (LEA(*slot) << SHIFT));
                *slot_addr[addr_select] = u32((LEA(*slot) << SHIFT) - rem_addr);
                slot->backwards = 1;
            } else if ((*addr[addr_select] < LSA(*slot) || (*slot_addr[addr_select] & 0x80000000)) && slot->backwards) {
                rem_addr = s32((LSA(*slot) << SHIFT) - *slot_addr[addr_select]);
                *slot_addr[addr_select] = u32((LSA(*slot) << SHIFT) + rem_addr);
                slot->backwards = 0;
            }
            break;
        }
    }
    if (!SDIR(*slot)) {
        if (ALFOS(*slot) != 0) {
            sample = sample * ALFO_Step(&slot->ALFO);
            sample >>= SHIFT;
        }
        if (slot->EG.state == Attack) sample = (sample * EG_Update(slot)) >> SHIFT;
        else sample = (sample * eg_table_[EG_Update(slot) >> (SHIFT - 10)]) >> SHIFT;
    }
    if (!STWINH(*slot)) {
        const u16 Enc = u16((SDIR(*slot) ? 0 : TL(*slot)) | (0x7 << 0xd));
        *rbufdst_ = int16_t((sample * lpan_[Enc]) >> (SHIFT + 1));
    }
    return sample;
}

void Scsp::generate(float *lr, int n) {
    for (int s = 0; s < n; ++s) {
        s32 smpl = 0, smpr = 0;
        for (int sl = 0; sl < 32; ++sl) {
            rbufdst_ = ringbuf_ + bufptr_;
            if (slots_[sl].active) {
                Slot *slot = slots_ + sl;
                const s32 sample = UpdateSlot(slot);
                const u32 eff_tl = SDIR(*slot) ? 0 : TL(*slot);
                const u16 EncD = u16(eff_tl | (IMXL(*slot) << 13));
                dsp_.MIXS[ISEL(*slot)] += (sample * lpan_[EncD]) >> (SHIFT - 2);
                const u32 dir_tl = SDIR(*slot) ? 0 : TL(*slot);
                const u16 Enc = u16(dir_tl | (DIPAN(*slot) << 8) | (DISDL(*slot) << 13));
                smpl += (sample * lpan_[Enc]) >> SHIFT;
                smpr += (sample * rpan_[Enc]) >> SHIFT;
            }
            ++bufptr_;
            bufptr_ &= 63;
        }
        dsp_step();
        for (int i = 0; i < 16; ++i) {
            const Slot *slot = slots_ + i;
            if (EFSDL(*slot)) {
                const u16 Enc = u16((EFPAN(*slot) << 8) | (EFSDL(*slot) << 13));
                smpl += (dsp_.EFREG[i] * lpan_[Enc]) >> SHIFT;
                smpr += (dsp_.EFREG[i] * rpan_[Enc]) >> SHIFT;
            }
        }
        // latch the slot monitor (MSLC) as MAME does once per sample
        {
            const Slot *slot = slots_ + latched_mslc_;
            const u32 SGC = u32(slot->EG.state) & 3;
            const u32 CA = (slot->cur_addr >> (SHIFT + 12)) & 0xf;
            const u32 EG = (0x1f - (slot->EG.volume >> (EG_SHIFT + 5))) & 0x1f;
            latched_mslc_data_ = uint16_t((CA << 7) | (SGC << 5) | EG);
        }
        // The arcade's analog amp is louder than MAME's raw full-scale
        // scaling (Sega Rally's mix peaks near -24 dBFS), so apply makeup
        // gain, then tanh-limit so peaks never clip.
        constexpr float kMakeup = 6.0f;
        const float gain = kMakeup * float(udata_[0] & 0xf) / 15.0f;
        auto out = [gain](int v, int lim, float scale) {
            const float x = gain * float(std::clamp(v, -lim - 1, lim)) / scale;
            return std::fabs(x) < 0.7f ? x : std::copysign(0.7f + 0.3f * std::tanh((std::fabs(x) - 0.7f) / 0.3f), x);
        };
        if (udata_[0] & 0x100) {
            lr[2 * s] = out(smpl, 131071, 131072.0f);
            lr[2 * s + 1] = out(smpr, 131071, 131072.0f);
        } else {
            lr[2 * s] = out(smpl >> 2, 32767, 32768.0f);
            lr[2 * s + 1] = out(smpr >> 2, 32767, 32768.0f);
        }
    }
}

// The effects DSP (MAME scspdsp.cpp).

void Scsp::dsp_start() {
    dsp_.Stopped = false;
    int i;
    for (i = 127; i >= 0; --i) {
        const u16 *p = dsp_.MPRO + i * 4;
        if (p[0] || p[1] || p[2] || p[3]) break;
    }
    dsp_.LastStep = i + 1;
}

void Scsp::dsp_step() {
    Dsp &d = dsp_;
    if (d.Stopped) return;
    std::fill(std::begin(d.EFREG), std::end(d.EFREG), int16_t(0));
    s32 ACC = 0, MEMVAL = 0, FRC_REG = 0, Y_REG = 0;
    u32 ADRS_REG = 0;
    for (int step = 0; step < d.LastStep; ++step) {
        const u16 *IPtr = d.MPRO + step * 4;
        const u32 TRA = (IPtr[0] >> 8) & 0x7f, TWT = (IPtr[0] >> 7) & 1, TWA = IPtr[0] & 0x7f;
        const u32 XSEL = (IPtr[1] >> 15) & 1, YSEL = (IPtr[1] >> 13) & 3, IRA = (IPtr[1] >> 6) & 0x3f,
                  IWT = (IPtr[1] >> 5) & 1, IWA = IPtr[1] & 0x1f;
        const u32 TABLE = (IPtr[2] >> 15) & 1, MWT = (IPtr[2] >> 14) & 1, MRD = (IPtr[2] >> 13) & 1,
                  EWT = (IPtr[2] >> 12) & 1, EWA = (IPtr[2] >> 8) & 0xf, ADRL = (IPtr[2] >> 7) & 1,
                  FRCL = (IPtr[2] >> 6) & 1, SHIFTv = (IPtr[2] >> 4) & 3, YRL = (IPtr[2] >> 3) & 1,
                  NEGB = (IPtr[2] >> 2) & 1, ZERO = (IPtr[2] >> 1) & 1, BSEL = IPtr[2] & 1;
        const u32 NOFL = (IPtr[3] >> 15) & 1, COEFi = (IPtr[3] >> 9) & 0x3f, MASA = (IPtr[3] >> 2) & 0x1f,
                  ADREB = (IPtr[3] >> 1) & 1, NXADR = IPtr[3] & 1;

        s32 INPUTS;
        if (IRA <= 0x1f) INPUTS = d.MEMS[IRA];
        else if (IRA <= 0x2F) INPUTS = d.MIXS[IRA - 0x20] * 16;
        else if (IRA <= 0x31) INPUTS = d.EXTS[IRA - 0x30] * 256;
        else return;
        INPUTS = sext(INPUTS, 24);
        if (IWT) {
            d.MEMS[IWA] = MEMVAL;
            if (IRA == IWA) INPUTS = MEMVAL;
        }
        s32 B;
        if (!ZERO) {
            B = BSEL ? ACC : sext(d.TEMP[(TRA + d.DEC) & 0x7f], 24);
            if (NEGB) B = 0 - B;
        } else {
            B = 0;
        }
        s32 X = XSEL ? INPUTS : sext(d.TEMP[(TRA + d.DEC) & 0x7f], 24);
        s32 Y = 0;
        if (YSEL == 0) Y = FRC_REG;
        else if (YSEL == 1) Y = d.COEF[COEFi] >> 3;
        else if (YSEL == 2) Y = (Y_REG >> 11) & 0x1fff;
        else Y = (Y_REG >> 4) & 0x0fff;
        if (YRL) Y_REG = INPUTS;

        s32 SHIFTED = 0;
        if (SHIFTv == 0) SHIFTED = std::clamp<s32>(ACC, -0x00800000, 0x007fffff);
        else if (SHIFTv == 1) SHIFTED = std::clamp<s32>(ACC * 2, -0x00800000, 0x007fffff);
        else if (SHIFTv == 2) SHIFTED = sext(ACC * 2, 24);
        else SHIFTED = sext(ACC, 24);

        Y = sext(Y, 13);
        const int64_t v = (int64_t(X) * int64_t(Y)) >> 12;
        ACC = int(v + B);

        if (TWT) d.TEMP[(TWA + d.DEC) & 0x7f] = SHIFTED;
        if (FRCL) FRC_REG = SHIFTv == 3 ? (SHIFTED & 0x0fff) : ((SHIFTED >> 11) & 0x1fff);

        if (MRD || MWT) {
            u32 ADDR = d.MADRS[MASA];
            if (!TABLE) ADDR += d.DEC;
            if (ADREB) ADDR += ADRS_REG & 0x0FFF;
            if (NXADR) ADDR++;
            if (!TABLE) ADDR &= d.RBL - 1;
            else ADDR &= 0xffff;
            ADDR += d.RBP << 12;
            ADDR <<= 1;
            if (MRD && (step & 1)) {
                if (NOFL) MEMVAL = s32(u32(read_word(ADDR)) << 8);
                else MEMVAL = UNPACK(read_word(ADDR));
            }
            if (MWT && (step & 1)) {
                if (NOFL) write_word(ADDR, u16(SHIFTED >> 8));
                else write_word(ADDR, PACK(SHIFTED));
            }
        }
        if (ADRL) ADRS_REG = SHIFTv == 3 ? u32((SHIFTED >> 12) & 0xfff) : u32(INPUTS >> 16);
        if (EWT) d.EFREG[EWA] = int16_t(d.EFREG[EWA] + (SHIFTED >> 8));
    }
    --d.DEC;
    std::fill(std::begin(d.MIXS), std::end(d.MIXS), 0);
}

uint16_t Scsp::dsp_read(uint32_t addr) const {
    const Dsp &d = dsp_;
    if (addr < 0x780) return uint16_t(d.COEF[(addr - 0x700) / 2]);
    if (addr < 0x7c0) return d.MADRS[(addr - 0x780) / 2];
    if (addr < 0x800) return d.MADRS[(addr - 0x7c0) / 2];
    if (addr < 0xC00) return d.MPRO[(addr - 0x800) / 2];
    if (addr < 0xE00) return (addr & 2) ? uint16_t(d.TEMP[(addr >> 2) & 0x7f] & 0xffff) : uint16_t(d.TEMP[(addr >> 2) & 0x7f] >> 16);
    if (addr < 0xE80) return (addr & 2) ? uint16_t(d.MEMS[(addr >> 2) & 0x1f] & 0xffff) : uint16_t(d.MEMS[(addr >> 2) & 0x1f] >> 16);
    if (addr < 0xEC0) return (addr & 2) ? uint16_t(d.MIXS[(addr >> 2) & 0xf] & 0xffff) : uint16_t(d.MIXS[(addr >> 2) & 0xf] >> 16);
    if (addr < 0xEE0) return uint16_t(d.EFREG[(addr - 0xec0) / 2]);
    if (addr < 0xEE4) return uint16_t(d.EXTS[(addr - 0xee0) / 2]);
    return 0;
}

void Scsp::dsp_write(uint32_t addr, uint16_t val) {
    Dsp &d = dsp_;
    if (addr < 0x780) d.COEF[(addr - 0x700) / 2] = int16_t(val);
    else if (addr < 0x7c0) d.MADRS[(addr - 0x780) / 2] = val;
    else if (addr < 0x800) d.MADRS[(addr - 0x7c0) / 2] = val;
    else if (addr < 0xC00) {
        d.MPRO[(addr - 0x800) / 2] = val;
        if (addr == 0xBF0) dsp_start();
    }
}

// Register-driven DMA between the chip's registers and sound RAM (MAME exec_dma).
void Scsp::exec_dma() {
    u16 tmp_dma[3];
    if (!dma_.ddir)
        for (int i = 0; i < 3; i++) tmp_dma[i] = udata_[(0x12 + i * 2) / 2];
    if (dma_.ddir) {
        if (dma_.dgate) {
            for (u32 i = 0; i < dma_.dtlg; i += 2) { write_word(dma_.dmea, 0); dma_.dmea += 2; }
        } else {
            for (u32 i = 0; i < dma_.dtlg; i += 2) {
                write_word(dma_.dmea, read16(dma_.drga));
                dma_.dmea += 2;
                dma_.drga += 2;
            }
        }
    } else {
        if (dma_.dgate) {
            for (u32 i = 0; i < dma_.dtlg; i += 2) { write16(dma_.drga, 0); dma_.drga += 2; }
        } else {
            for (u32 i = 0; i < dma_.dtlg; i += 2) {
                write16(dma_.drga, read_word(dma_.dmea));
                dma_.dmea += 2;
                dma_.drga += 2;
            }
        }
    }
    if (!dma_.ddir)
        for (int i = 0; i < 3; i++) udata_[(0x12 + i * 2) / 2] = tmp_dma[i];
    udata_[0x16 / 2] &= ~0x1000;
    if ((udata_[0x1e / 2] & 0x10) && irq) irq(irq_dma_, true);
}

// LFO handling

void Scsp::LFO_Init() {
    auto LFIX = [](float v) { return u32(float(1 << LFO_SHIFT) * v); };
    for (int i = 0; i < 256; ++i) {
        int a, p;
        a = 255 - i;
        p = i < 128 ? i : i - 256;
        alfo_saw_[i] = a;
        plfo_saw_[i] = p;
        if (i < 128) { a = 255; p = 127; }
        else { a = 0; p = -128; }
        alfo_sqr_[i] = a;
        plfo_sqr_[i] = p;
        a = i < 128 ? 255 - (i * 2) : (i * 2) - 256;
        if (i < 64) p = i * 2;
        else if (i < 128) p = 255 - i * 2;
        else if (i < 192) p = 256 - i * 2;
        else p = i * 2 - 511;
        alfo_tri_[i] = a;
        plfo_tri_[i] = p;
        a = int(rand16() & 0xff);
        p = 128 - a;
        alfo_noi_[i] = a;
        plfo_noi_[i] = p;
    }
    for (int s = 0; s < 8; ++s) {
        float limit = PSCALE[s];
        for (int i = -128; i < 128; ++i)
            pscales_[s][i + 128] = int(LFIX(powf(2.0f, ((limit * float(i)) / 128.0f) / 1200.0f)));
        limit = -ASCALE[s];
        for (int i = 0; i < 256; ++i)
            ascales_[s][i] = int(LFIX(powf(10.0f, ((limit * float(i)) / 256.0f) / 20.0f)));
    }
}

int32_t Scsp::PLFO_Step(Lfo *lfo) {
    lfo->phase = uint16_t(lfo->phase + lfo->phase_step);
    int p = lfo->table[lfo->phase >> LFO_SHIFT];
    p = lfo->scale[p + 128];
    return p << (SHIFT - LFO_SHIFT);
}

int32_t Scsp::ALFO_Step(Lfo *lfo) {
    lfo->phase = uint16_t(lfo->phase + lfo->phase_step);
    int p = lfo->table[lfo->phase >> LFO_SHIFT];
    p = lfo->scale[p];
    return p << (SHIFT - LFO_SHIFT);
}

void Scsp::LFO_ComputeStep(Lfo *lfo, uint32_t LFOF, uint32_t LFOWS, uint32_t LFOS, int alfo) {
    const float step = LFOFreq[LFOF] * 256.0f / 44100.0f;
    lfo->phase_step = u32(float(1 << LFO_SHIFT) * step);
    if (alfo) {
        switch (LFOWS) {
        case 0: lfo->table = alfo_saw_; break;
        case 1: lfo->table = alfo_sqr_; break;
        case 2: lfo->table = alfo_tri_; break;
        case 3: lfo->table = alfo_noi_; break;
        }
        lfo->scale = ascales_[LFOS];
    } else {
        switch (LFOWS) {
        case 0: lfo->table = plfo_saw_; break;
        case 1: lfo->table = plfo_sqr_; break;
        case 2: lfo->table = plfo_tri_; break;
        case 3: lfo->table = plfo_noi_; break;
        }
        lfo->scale = pscales_[LFOS];
    }
}

} // namespace snd
