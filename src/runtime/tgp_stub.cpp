// Stub for the generated TGP program on Model 2A boards that have no TGP
// (Sega Rally, srallyc). CMake uses this in place of the generated
// <set>_tgp/tgp_gen.cpp. The i960 on these boards never uploads or boots a
// TGP, so tgpgen::run is never called; if it somehow were, the coproctl boot
// path (m2_tgp_board.cpp) would reject the upload first because program_words
// is 0. Keeps the tgpgen:: symbols the runtime references.
#include "runtime/tgp.h"

namespace rt {
namespace tgpgen {

void run(Tgp &, uint64_t) {}

const uint32_t program_crc32 = 0;
const uint32_t program_words = 0;

} // namespace tgpgen
} // namespace rt
