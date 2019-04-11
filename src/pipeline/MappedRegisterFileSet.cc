#include "MappedRegisterFileSet.hh"

namespace simeng {
namespace pipeline {

MappedRegisterFileSet::MappedRegisterFileSet(
    RegisterFileSet& physicalRegisterFileSet, const RegisterAliasTable& rat,
    uint8_t thread)
    : ArchitecturalRegisterFileSet(physicalRegisterFileSet),
      rat_(rat),
      thread_(thread) {}

RegisterValue MappedRegisterFileSet::get(Register reg) const {
  return ArchitecturalRegisterFileSet::get(rat_.getMapping(reg, thread_));
}

void MappedRegisterFileSet::set(Register reg, const RegisterValue& value) {
  return ArchitecturalRegisterFileSet::set(rat_.getMapping(reg, thread_),
                                           value);
}

}  // namespace pipeline
}  // namespace simeng
