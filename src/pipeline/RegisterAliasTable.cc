#include "RegisterAliasTable.hh"

#include <cassert>

namespace simeng {
namespace pipeline {

RegisterAliasTable::RegisterAliasTable(
    std::vector<RegisterFileStructure> architecturalStructure,
    std::vector<uint16_t> physicalRegisterCounts, uint8_t threads)
    : mappingTables_(threads,
                     RegisterMappingTable(architecturalStructure.size())),
      historyTable_(architecturalStructure.size()),
      destinationTable_(architecturalStructure.size()),
      freeQueues_(architecturalStructure.size()) {
  assert(architecturalStructure.size() == physicalRegisterCounts.size() &&
         "The number of physical register types does not match the number of "
         "architectural register types");
  for (size_t type = 0; type < architecturalStructure.size(); type++) {
    auto archCount = architecturalStructure[type].quantity;
    auto physCount = physicalRegisterCounts[type];
    assert(archCount < physCount &&
           "Must have more physical registers than architectural registers");
    assert(archCount * threads < physCount &&
           "Must have enough physical registers to cover the architectural "
           "register state for each thread");

    // Add physical registers to free queue
    for (size_t tag = 0; tag < physCount; tag++) {
      freeQueues_[type].push(tag);
    }

    // Set up history/destination tables
    historyTable_[type].resize(physCount);
    destinationTable_[type].resize(physCount);

    // For each thread, set up the initial mapping table state for this register
    // type
    for (size_t thread = 0; thread < threads; thread++) {
      mappingTables_[thread][type].resize(archCount);

      for (size_t tag = 0; tag < archCount; tag++) {
        // Pre-assign a physical register to each architectural register
        mappingTables_[thread][type][tag] = freeQueues_[type].front();
        freeQueues_[type].pop();
      }
    }
  }
};

Register RegisterAliasTable::getMapping(Register architectural,
                                        uint8_t threadId) const {
  auto tag = mappingTables_[threadId][architectural.type][architectural.tag];
  return {architectural.type, tag};
}

bool RegisterAliasTable::canAllocate(uint8_t type,
                                     unsigned int quantity) const {
  return (freeQueues_[type].size() >= quantity);
}

unsigned int RegisterAliasTable::freeRegistersAvailable(uint8_t type) const {
  return freeQueues_[type].size();
}

Register RegisterAliasTable::allocate(Register architectural,
                                      uint8_t threadId) {
  std::queue<uint16_t>& freeQueue = freeQueues_[architectural.type];
  assert(freeQueue.size() > 0 &&
         "Attempted to allocate free register when none were available");

  auto tag = freeQueue.front();
  freeQueue.pop();

  // Keep the old physical register in the history table
  historyTable_[architectural.type][tag] =
      mappingTables_[threadId][architectural.type][architectural.tag];

  // Update the mapping table with the new tag, and mark the architectural
  // register it replaces in the destination table
  mappingTables_[threadId][architectural.type][architectural.tag] = tag;
  destinationTable_[architectural.type][tag] = architectural.tag;

  return {architectural.type, tag};
}

void RegisterAliasTable::commit(Register physical, uint8_t threadId) {
  // Find the register previously mapped to the same architectural register and
  // free it
  auto oldTag = historyTable_[physical.type][physical.tag];
  freeQueues_[physical.type].push(oldTag);
}
void RegisterAliasTable::rewind(Register physical, uint8_t threadId) {
  // Find which architectural tag this referred to
  auto destinationTag = destinationTable_[physical.type][physical.tag];
  // Rewind the mapping table to the old physical tag
  mappingTables_[threadId][physical.type][destinationTag] =
      historyTable_[physical.type][physical.tag];
  // Add the rewound physical tag back to the free queue
  freeQueues_[physical.type].push(physical.tag);
}
void RegisterAliasTable::free(Register physical) {
  freeQueues_[physical.type].push(physical.tag);
}

}  // namespace pipeline
}  // namespace simeng
