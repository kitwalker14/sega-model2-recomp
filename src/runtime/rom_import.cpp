#include "runtime/rom_import.h"

#include "runtime/archive.h"
#include "runtime/zip.h"

#include <cstdio>
#include <cstring>
#include <iterator>
#include <span>

namespace rt {

namespace {

enum Region { Program, MainData, CoproData, Polygons, Textures, CoproTables, SoundProgram, Pcm1, Pcm2 };

struct Load {
    const char *file;
    uint32_t crc;
    Region region;
    uint32_t offset, size; // ROM_LOAD32_WORD: 16-bit words into bytes 0-1 (offset 0) or 2-3 (offset 2) of each dword;
                           // the sound regions load whole files (see import_rom_set)
};

// MAME model2.cpp ROM_START(daytona93), ROM_START(daytona) and
// MODEL2_CPU_BOARD at dddd7368. The parts both sets share come first.
#define SHARED_LOADS                                                         \
    {"mpr-16528.10", 0x9CE591F6, MainData, 0x000000, 0x200000},              \
    {"mpr-16529.11", 0xF7095EAF, MainData, 0x000002, 0x200000},              \
    {"mpr-16537.ic28", 0x36B7C35A, CoproData, 0x000000, 0x200000},           \
    {"mpr-16536.ic29", 0x6D6AFED9, CoproData, 0x000002, 0x200000},           \
    {"mpr-16523.ic16", 0x2F484D42, Polygons, 0x000000, 0x200000},            \
    {"mpr-16518.ic20", 0xDF683BF7, Polygons, 0x000002, 0x200000},            \
    {"mpr-16524.ic17", 0x34658BD7, Polygons, 0x400000, 0x200000},            \
    {"mpr-16519.ic21", 0xFACD1C81, Polygons, 0x400002, 0x200000},            \
    {"mpr-16525.ic18", 0xFB517521, Polygons, 0x800000, 0x200000},            \
    {"mpr-16520.ic22", 0xD66BD9BD, Polygons, 0x800002, 0x200000},            \
    {"mpr-16522.25", 0x55D39A57, Textures, 0x000000, 0x200000},              \
    {"mpr-16521.24", 0xAF1934FB, Textures, 0x000002, 0x200000},              \
    {"opr-14742a.45", 0x90C6B117, CoproTables, 0x000000, 0x020000},          \
    {"opr-14743a.46", 0xAE7F446B, CoproTables, 0x000002, 0x020000},          \
    /* Model 1 sound board (segam1audio): the MultiPCMs' sample ROMs */      \
    {"mpr-16491.32", 0x89920903, Pcm1, 0x000000, 0x200000},                  \
    {"mpr-16492.33", 0x459E701B, Pcm1, 0x200000, 0x200000},                  \
    {"mpr-16493.4", 0x9990DB15, Pcm2, 0x000000, 0x200000},                   \
    {"mpr-16494.5", 0x600E1D6C, Pcm2, 0x200000, 0x200000}

// Daytona USA, Deluxe '93 (daytona93).
const Load kDaytona93[] = {
    {"epr-16530a.12", 0x39E962B5, Program, 0x000000, 0x020000},
    {"epr-16531a.13", 0x693126EB, Program, 0x000002, 0x020000},
    {"mpr-16526.8", 0x5273B8B5, MainData, 0x400000, 0x200000},
    {"mpr-16527.9", 0xFC4CB0EF, MainData, 0x400002, 0x200000},
    {"epr-16534a.6", 0x1BB0D72D, MainData, 0x800000, 0x100000},
    {"epr-16535a.7", 0x459A8BFB, MainData, 0x800002, 0x100000},
    {"epr-16646.ic19", 0x7BA9FD6B, Polygons, 0xC00000, 0x080000},
    {"epr-16645.ic23", 0x78FE0B8A, Polygons, 0xC00002, 0x080000},
    {"mpr-16517.27", 0x4705D3DD, Textures, 0x800000, 0x200000},
    {"mpr-16516.26", 0xA260D45D, Textures, 0x800002, 0x200000},
    // the sound board's 68000 program (ROM_LOAD16_WORD_SWAP)
    {"epr-16489.7", 0xC20E543E, SoundProgram, 0x000000, 0x020000},
    {"epr-16490.8", 0xC24EDAAB, SoundProgram, 0x020000, 0x020000},
    SHARED_LOADS,
};

// Daytona USA, Revision A, 1994 (daytona).
const Load kDaytona[] = {
    {"epr-16722a.12", 0x48B94318, Program, 0x000000, 0x020000},
    {"epr-16723a.13", 0x8AF8B32D, Program, 0x000002, 0x020000},
    {"mpr-16808.8", 0x44F1F5A0, MainData, 0x400000, 0x200000},
    {"mpr-16809.9", 0x37A2DD12, MainData, 0x400002, 0x200000},
    {"epr-16724a.6", 0x469F10FD, MainData, 0x800000, 0x080000},
    {"epr-16725a.7", 0xBA0DF8DB, MainData, 0x800002, 0x080000},
    {"mpr-16772.ic19", 0x770ED912, Polygons, 0xC00000, 0x200000},
    {"mpr-16771.ic23", 0xA2205124, Polygons, 0xC00002, 0x200000},
    {"mpr-16770.27", 0xF9FA7BFB, Textures, 0x800000, 0x200000},
    {"mpr-16769.26", 0xE57429E9, Textures, 0x800002, 0x200000},
    {"epr-16720.7", 0x8E73CFFD, SoundProgram, 0x000000, 0x020000},
    {"epr-16721.8", 0x1BB3B7B7, SoundProgram, 0x020000, 0x020000},
    SHARED_LOADS,
};
#undef SHARED_LOADS

// Sega Rally Championship, Revision B (srallycb): a Model 2A board, which
// has no TGP and so no TGP program to upload. MAME model2.cpp
// ROM_START(srallycb). The program and data ROMs are shared with srallyc
// (Revision C); the copro tables are the shared MODEL2_CPU_BOARD chips.
const Load kSrallycb[] = {
    {"epr-17888b.12", 0x95BCE0B9, Program, 0x000000, 0x080000},
    {"epr-17889b.13", 0x395C425E, Program, 0x000002, 0x080000},
    {"mpr-17746.10", 0x8FE311F4, MainData, 0x000000, 0x200000},
    {"mpr-17747.11", 0x543593FD, MainData, 0x000002, 0x200000},
    {"mpr-17744.8", 0x71FED098, MainData, 0x400000, 0x200000},
    {"mpr-17745.9", 0x8ECCA705, MainData, 0x400002, 0x200000},
    {"mpr-17884.6", 0x4CFC95E1, MainData, 0x800000, 0x200000},
    {"mpr-17885.7", 0xA08D2467, MainData, 0x800002, 0x200000},
    {"mpr-17754.29", 0x81A84F67, CoproData, 0x000000, 0x200000},
    {"mpr-17755.28", 0x2A6E7DA4, CoproData, 0x000002, 0x200000},
    {"mpr-17748.16", 0x3148A2B2, Polygons, 0x000000, 0x200000},
    {"mpr-17750.20", 0x232AEC29, Polygons, 0x000002, 0x200000},
    {"mpr-17749.17", 0x0838D184, Polygons, 0x400000, 0x200000},
    {"mpr-17751.21", 0xED87AC62, Polygons, 0x400002, 0x200000},
    {"mpr-17753.25", 0x6DB0EB36, Textures, 0x000000, 0x200000},
    {"mpr-17752.24", 0xD6AA86CE, Textures, 0x000002, 0x200000},
    {"opr-14742a.45", 0x90C6B117, CoproTables, 0x000000, 0x020000},
    {"opr-14743a.46", 0xAE7F446B, CoproTables, 0x000002, 0x020000},
    {"epr-17890.30", 0x5BAC3FA1, SoundProgram, 0x000000, 0x040000},
    {"mpr-17756.31", 0x7725F111, Pcm1, 0x000000, 0x200000},
    {"mpr-17757.32", 0x1616E649, Pcm1, 0x200000, 0x200000},
    {"mpr-17886.36", 0x54A72923, Pcm2, 0x000000, 0x200000},
    {"mpr-17887.37", 0x38C31FDD, Pcm2, 0x200000, 0x200000},
};

// Per set: its files; main_data's ROM_COPY source (copied to every 1 MB
// from there up to 0xF00000); where the TGP program the i960 uploads at
// boot sits in main_data (found by matching MAME's upload; the same 2,024
// words in both sets, checked by CRC); and whether the board has a TGP at
// all (Model 2A sets, such as Sega Rally, do not).
struct Set {
    const char *name;
    const Load *loads;
    size_t count;
    uint32_t mirror_from, tgp_offset;
    bool has_tgp = true;
};
constexpr uint32_t kTgpWords = 2024, kTgpCrc = 0xD6D611DDu;
const Set kSets[] = {
    {"daytona93", kDaytona93, std::size(kDaytona93), 0x900000, 0x860020, true},
    {"daytona", kDaytona, std::size(kDaytona), 0x800000, 0x800020, true},
    {"srallyc", kSrallycb, std::size(kSrallycb), 0xF00000, 0x000000, false},
};

const Set &this_set() {
    for (const Set &s : kSets)
        if (!std::strcmp(s.name, M2_ROMSET)) return s;
    throw ZipError(std::string("unknown ROM set ") + M2_ROMSET);
}

std::string hex8(uint32_t v) {
    char b[16];
    std::snprintf(b, sizeof b, "%08x", v);
    return b;
}

} // namespace

std::vector<RomCheck> check_rom_set(const std::string &zip_path) {
    std::vector<RomCheck> out;
    const auto z = open_archive(zip_path);
    const Set &set = this_set();
    for (const Load &l : std::span(set.loads, set.count)) {
        RomCheck c;
        c.file = l.file;
        const auto it = z->entries().find(l.file);
        if (it == z->entries().end()) c.problem = "missing";
        else if (it->second.size != l.size) c.problem = "wrong size";
        else if (it->second.has_crc && it->second.crc != l.crc)
            c.problem = "wrong CRC " + hex8(it->second.crc) + " (expected " + hex8(l.crc) + ")";
        else c.ok = true;
        out.push_back(c);
    }
    return out;
}

M2Board::Images import_rom_set(const std::string &zip_path) {
    const auto z = open_archive(zip_path);
    M2Board::Images img;
    img.program.assign(0x200000, 0);
    img.main_data.assign(0x2000000, 0);
    img.copro_data.assign(0x800000, 0);
    img.polygons.assign(0x1000000, 0);
    img.textures.assign(0x1000000, 0);
    img.copro_tables.assign(0x40000, 0);
    img.sound_program.assign(0x40000, 0);
    img.pcm1.assign(0x400000, 0);
    img.pcm2.assign(0x400000, 0);
    const Set &set = this_set();
    for (const Load &l : std::span(set.loads, set.count)) {
        const std::vector<uint8_t> data = z->read(l.file); // CRC against the archive's own
        if (data.size() != l.size || crc32(data.data(), data.size()) != l.crc)
            throw ZipError(std::string(l.file) + ": not the " + set.name + " ROM this build was recompiled from");
        std::vector<uint8_t> *r = nullptr;
        switch (l.region) {
        case Program: r = &img.program; break;
        case MainData: r = &img.main_data; break;
        case CoproData: r = &img.copro_data; break;
        case Polygons: r = &img.polygons; break;
        case Textures: r = &img.textures; break;
        case CoproTables: r = &img.copro_tables; break;
        case SoundProgram: // 68000, big-endian: the files hold byte-swapped words
            for (uint32_t i = 0; i < l.size; i += 2) {
                img.sound_program[l.offset + i] = data[i + 1];
                img.sound_program[l.offset + i + 1] = data[i];
            }
            continue;
        case Pcm1: std::copy(data.begin(), data.end(), img.pcm1.begin() + l.offset); continue;
        case Pcm2: std::copy(data.begin(), data.end(), img.pcm2.begin() + l.offset); continue;
        }
        for (uint32_t w = 0; w < l.size / 2; w++) {
            (*r)[l.offset + w * 4] = data[w * 2];
            (*r)[l.offset + w * 4 + 1] = data[w * 2 + 1];
        }
    }
    for (uint32_t dst = set.mirror_from + 0x100000; dst <= 0xF00000; dst += 0x100000) // ROM_COPY mirrors
        std::copy_n(img.main_data.begin() + set.mirror_from, 0x100000, img.main_data.begin() + dst);
    return img;
}

const char *rom_set_name() { return M2_ROMSET; }

std::vector<uint8_t> tgp_program(const M2Board::Images &img) {
    const Set &set = this_set();
    if (!set.has_tgp) { // Model 2A (Sega Rally): the 1,665-word program the i960 uploads sits in its own program ROM
        constexpr uint32_t kRallyOffset = 372372, kRallyWords = 1665, kRallyCrc = 0x072484B1u;
        const uint8_t *q = img.program.data() + kRallyOffset;
        if (crc32(q, kRallyWords * 4) != kRallyCrc) throw ZipError("TGP program not where expected");
        return {q, q + kRallyWords * 4};
    }
    const uint8_t *p = img.main_data.data() + set.tgp_offset;
    if (crc32(p, kTgpWords * 4) != kTgpCrc) throw ZipError("TGP program not where expected");
    return {p, p + kTgpWords * 4};
}

} // namespace rt
