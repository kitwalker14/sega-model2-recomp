#include "runtime/sound_board.h"

#include "runtime/native_sound_sequencer.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

namespace snd {

SoundBoard::SoundBoard(const std::vector<uint8_t> &program, const std::vector<uint8_t> &pcm1, const std::vector<uint8_t> &pcm2, bool scsp)
    : scsp_(scsp), program_(program), pcm1_rom_(pcm1), pcm2_rom_(pcm2),
      pcm1_(kPcmClock, pcm1_rom_.data(), uint32_t(pcm1_rom_.size())),
      pcm2_(kPcmClock, pcm2_rom_.data(), uint32_t(pcm2_rom_.size())) {
    if (program_.size() < 0x40000) throw rt::Fatal("sound program image missing or short (re-run the importer)");
    cpu_.rom = program_.data();
    if (scsp_) {
        samples_ = pcm1;
        samples_.insert(samples_.end(), pcm2.begin(), pcm2.end());
        cpu_.scsp = true;
        cpu_.samples = samples_.data();
        cpu_.samples_size = uint32_t(samples_.size());
        ips_ = kIps * 11289600 / 10000000;
    }
    cpu_.dev = this;
    cpu_.reset();
    if (scsp_) {
        scsp_dev_ = std::make_unique<Scsp>(cpu_.sram.data(), uint32_t(cpu_.sram.size()));
        scsp_dev_->irq = [this](int level, bool on) {
            cpu_.irq_line = on ? level : 0;
            sched_.poke();
        };
        scsp_dev_->midi_out = [this](uint8_t b) { std::lock_guard<std::mutex> g(reply_mu_); reply_.push_back(b); ++reply_total_; };
        scsp_dev_->arm_timer = [this](int which, uint32_t samples, uint64_t gen) {
            const uint64_t at = sched_.count + (uint64_t(samples) * ips_ + 44099) / 44100;
            sched_.at(at, [this, which, gen] {
                render_scsp_to(sched_.count);
                scsp_dev_->timer_expired(which, gen);
            });
        };
    }
    ym_.reset();
}

void SoundBoard::send(const uint8_t *bytes, size_t n) {
    const uint64_t kByteTime = ips_ * kByteBits / kBaud; // instructions per byte on the line
    for (size_t i = 0; i < n; ++i) {
        const uint64_t at = std::max(line_free_, sched_.count) + kByteTime;
        line_free_ = at;
        const uint8_t b = bytes[i];
        sched_.at(at, [this, b] { rx_arrive(b); });
    }
}

void SoundBoard::rx_arrive(uint8_t byte) {
    if (scsp_) {
        render_scsp_to(sched_.count);
        if (std::getenv("SCSP_TRACE")) std::fprintf(stderr, "midi in %02x n=%llu\n", byte, (unsigned long long)sched_.count);
        scsp_dev_->midi_in(byte);
        ++received_;
        return;
    }
    if (!rx_enable_) return; // receiver off: the byte is lost, as on the chip
    if (rxrdy_) errors_ |= 0x10; // overrun: the new byte replaces the unread one
    rx_hold_ = byte;
    rxrdy_ = true;
    cpu_.irq_line = 2; // RxRDY on IPL level 2
    ++received_;
}

void SoundBoard::advance(double seconds) {
    const double want = seconds * double(ips_) + carry_;
    const uint64_t n = uint64_t(want);
    carry_ = want - double(n);
    run_to(sched_.count + n);
}

void SoundBoard::run_to(uint64_t count) {
    while (sched_.count < count) {
        sched_.end_count = count;
        if (!sndgen::has_code(cpu_.pc)) {
            char b[96];
            std::snprintf(b, sizeof b, "sound 68000: no recompiled code at %06x", cpu_.pc);
            throw rt::Fatal(b);
        }
        sndgen::run(env_);
    }
    if (scsp_) {
        render_scsp_to(count);
        return;
    }
    render_fm_to(count);
    render_pcm_to(count);
}

void SoundBoard::render_scsp_to(uint64_t count) {
    const uint64_t due = count * 44100 / ips_;
    constexpr int kChunk = 256;
    float buf[kChunk * 2];
    while (scsp_done_ < due) {
        const int n = int(std::min<uint64_t>(kChunk, due - scsp_done_));
        scsp_dev_->generate(buf, n);
        scsp_out_.insert(scsp_out_.end(), buf, buf + n * 2);
        scsp_done_ += uint64_t(n);
    }
}

void SoundBoard::render_fm_to(uint64_t count) {
    const uint64_t due = count * kYmClock / (144 * ips_);
    constexpr int kChunk = 256;
    ymfm::ym3438::output_data out[kChunk];
    while (fm_done_ < due) {
        const int n = int(std::min<uint64_t>(kChunk, due - fm_done_));
        ym_.generate(out, uint32_t(n));
        for (int i = 0; i < n; ++i) {
            fm_out_.push_back(0.30f * float(out[i].data[0]) / 32768.0f);
            fm_out_.push_back(0.30f * float(out[i].data[1]) / 32768.0f);
        }
        fm_done_ += uint64_t(n);
    }
}

void SoundBoard::render_pcm_to(uint64_t count) {
    const uint64_t due = count * kPcmClock / (224 * ips_);
    constexpr int kChunk = 256;
    float l1[kChunk], r1[kChunk], l2[kChunk], r2[kChunk];
    while (pcm_done_ < due) {
        const int n = int(std::min<uint64_t>(kChunk, due - pcm_done_));
        pcm1_.generate(l1, r1, n);
        pcm2_.generate(l2, r2, n);
        for (int i = 0; i < n; ++i) {
            pcm_out_.push_back(0.5f * l1[i] + 0.5f * l2[i]);
            pcm_out_.push_back(0.5f * r1[i] + 0.5f * r2[i]);
        }
        pcm_done_ += uint64_t(n);
    }
}

uint16_t SoundBoard::read16(uint32_t addr) {
    if (scsp_) {
        if (addr < 0x100000 || addr >= 0x101000) return 0;
        render_scsp_to(sched_.count);
        const uint16_t rv = scsp_dev_->read16((addr - 0x100000) & ~1u);
        if (std::getenv("SCSP_TRACE") && ((addr & 0xffe) == 0x404 || (addr & 0xffe) == 0x420)) std::fprintf(stderr, "scsp r %03x=%04x pc=%06x n=%llu\n", addr - 0x100000, rv, cpu_.pc, (unsigned long long)sched_.count);
        return rv;
    }
    return uint16_t(read(addr) << 8 | read(addr + 1));
}

uint8_t SoundBoard::read(uint32_t addr) {
    if (scsp_) {
        const uint16_t v = read16(addr & ~1u);
        return uint8_t(addr & 1 ? v : v >> 8);
    }
    switch (addr) {
    case 0xc20001: // UART data
        rxrdy_ = false;
        cpu_.irq_line = 0;
        return rx_hold_;
    case 0xc20003: // UART status: DSR, TxEMPTY, TxRDY always; RxRDY; PE/OE/FE
        return uint8_t(0x85 | (rxrdy_ ? 2 : 0) | errors_);
    case 0xd00001: case 0xd00003: case 0xd00005: case 0xd00007:
        return ym_.read((addr - 0xd00001) / 2);
    default:
        return 0; // MultiPCM status reads 0 (as MAME)
    }
}

// The driver's voices: two pools of 28, ten bytes each, in its RAM at
// 0xf01500 and 0xf01618. Byte 0 is zero when free, bit 3 the MultiPCM;
// byte 1 the slot code it writes to the chip, byte 3 the owning channel
// (the command's low nibble), byte 6 0xff for the engine layers' reserved
// records, which never take a voice (the allocator wants 0) and carry chip
// 0's bit with chip 1's fixed slot codes. Read from the driver's code: the
// allocator at 0x1688, the slot select at 0x1652, a note's start at
// 0xc22-0xd26.
int SoundBoard::voice_channel(unsigned chip, uint8_t slot) const {
    for (uint32_t pool : {0x1500u, 0x1618u})
        for (uint32_t v = pool; v < pool + 28 * 10; v += 10) {
            const uint8_t *voice = &cpu_.ram[v];
            if (voice[0] && voice[6] != 0xff && (voice[0] >> 3 & 1) == chip && voice[1] == slot) return voice[3] & 15;
        }
    return -1;
}

void SoundBoard::write(uint32_t addr, uint16_t data) {
    if (scsp_) {
        if (addr >= 0x100000 && addr < 0x101000) {
            if (std::getenv("SCSP_TRACE")) std::fprintf(stderr, "scsp w %03x=%04x pc=%06x n=%llu\n", addr - 0x100000, data, cpu_.pc, (unsigned long long)sched_.count);
            render_scsp_to(sched_.count);
            scsp_dev_->write16((addr - 0x100000) & ~1u, data);
        }
        return;
    }
    const uint8_t d = uint8_t(data);
    switch (addr) {
    case 0xc20001: break; // UART transmit: nothing listens on the i960 side
    case 0xc20003:        // UART mode / command
        if (expect_mode_) {
            expect_mode_ = false;
        } else {
            if (d & 0x10) errors_ = 0; // error reset
            rx_enable_ = d & 0x04;
            if (d & 0x40) expect_mode_ = true; // internal reset: next write is the mode
        }
        break;
    case 0xc40001: case 0xc40003: case 0xc40005: case 0xc40007:
    case 0xc60001: case 0xc60003: case 0xc60005: case 0xc60007: {
        render_pcm_to(sched_.count);
        const unsigned chip = addr >= 0xc60001, offset = (addr & 7) >> 1;
        MultiPcm &pcm = chip ? pcm2_ : pcm1_;
        if (offset == 1) pcm_slot_[chip] = d;
        else if (offset == 2) pcm_register_[chip] = d;
        else if (offset == 0 && pcm_register_[chip] == 4 && (d & 0x80)) { // key on: music's voice or an effect's
            const int channel = voice_channel(chip, pcm_slot_[chip]);
            pcm.set_effect(channel < 0 || !NativeSoundSequencer::music_channel(unsigned(channel)));
        }
        pcm.write(offset, d);
        break;
    }
    case 0xc50000: render_pcm_to(sched_.count); pcm1_.set_bank(data & 3); break;
    case 0xc70000: render_pcm_to(sched_.count); pcm2_.set_bank(data & 3); break;
    case 0xd00001: case 0xd00003: case 0xd00005: case 0xd00007:
        render_fm_to(sched_.count);
        ym_.write((addr - 0xd00001) / 2, d);
        break;
    default: break; // 0xc40012: effect DSP (not fitted)
    }
}

// ymfm asks for a timer in YM clocks. Expiries are kept in exact YM clocks
// (a periodic timer re-armed from its own expiry does not drift) and mapped
// to the instruction count at which the 68000 would see them.
void SoundBoard::ymfm_set_timer(uint32_t tnum, int32_t duration_in_clocks) {
    const uint64_t gen = ++timer_gen_[tnum];
    if (duration_in_clocks < 0) return;
    const uint64_t base = in_timer_ == int(tnum) ? timer_due_clk_[tnum] : ym_clock_at(sched_.count);
    const uint64_t due = base + uint64_t(duration_in_clocks);
    timer_due_clk_[tnum] = due;
    const uint64_t at = (due * ips_ + kYmClock - 1) / kYmClock;
    sched_.at(at, [this, tnum, gen] {
        if (timer_gen_[tnum] != gen) return; // re-armed or stopped since
        render_fm_to(sched_.count);
        in_timer_ = int(tnum);
        m_engine->engine_timer_expired(tnum);
        in_timer_ = -1;
    });
}

} // namespace snd
