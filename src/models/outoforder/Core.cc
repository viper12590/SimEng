#include "Core.hh"

#include <algorithm>
#include <iostream>
#include <string>

// Temporary; until config options are available
#include "../../A64Instruction.hh"

namespace simeng {
namespace models {
namespace outoforder {

// TODO: Replace with config options
const std::initializer_list<uint16_t> physicalRegisterQuantities = {128, 128,
                                                                    128};
const std::initializer_list<RegisterFileStructure> physicalRegisterStructures =
    {{8, 128}, {16, 128}, {1, 128}};
const unsigned int robSize = 16;
const unsigned int rsSize = 16;
const unsigned int loadQueueSize = 16;
const unsigned int storeQueueSize = 8;
const unsigned int frontendWidth = 2;
const unsigned int commitWidth = 2;
const std::vector<std::vector<uint16_t>> portArrangement = {
    {A64InstructionGroups::LOAD, A64InstructionGroups::STORE},
    {A64InstructionGroups::ARITHMETIC},
    {A64InstructionGroups::BRANCH}};
const unsigned int executionUnitCount = portArrangement.size();
const unsigned int threads = 1;

// TODO: Replace simple process memory space with memory hierarchy interface.
Core::Core(const span<char> processMemory, uint64_t entryPoint,
           const Architecture& isa, BranchPredictor& branchPredictor,
           pipeline::PortAllocator& portAllocator)
    : isa_(isa),
      registerFileSet_(physicalRegisterStructures),
      registerAliasTable_(isa.getRegisterFileStructures(),
                          physicalRegisterQuantities, threads),
      processMemory_(processMemory),
      loadStoreQueue_(loadQueueSize, storeQueueSize, processMemory.data()),
      reorderBuffer_(
          robSize, registerAliasTable_, loadStoreQueue_,
          [this](auto instruction) { raiseException(instruction); },
          [this](auto afterSeqId, auto address, auto threadId) {
            raiseFlush(afterSeqId, address, threadId, true);
          }),
      fetchToDecodeBuffer_(frontendWidth, {}),
      decodeToRenameBuffer_(frontendWidth, nullptr),
      renameToDispatchBuffer_(frontendWidth, nullptr),
      issuePorts_(executionUnitCount, {1, nullptr}),
      completionSlots_(executionUnitCount, {1, nullptr}),
      fetchUnit_(fetchToDecodeBuffer_, processMemory.data(),
                 processMemory.size(), {entryPoint}, isa, branchPredictor),
      decodeUnit_(fetchToDecodeBuffer_, decodeToRenameBuffer_, branchPredictor,
                  [this](auto address, auto threadId) {
                    raiseFlush(0, address, threadId, false);
                  }),
      renameUnit_(decodeToRenameBuffer_, renameToDispatchBuffer_,
                  reorderBuffer_, registerAliasTable_, loadStoreQueue_,
                  physicalRegisterStructures.size()),
      dispatchIssueUnit_(renameToDispatchBuffer_, issuePorts_, registerFileSet_,
                         portAllocator, physicalRegisterQuantities, rsSize),
      writebackUnit_(completionSlots_, registerFileSet_),
      flushConditions_(threads) {
  // Construct a mapped register file set for each thread
  for (uint8_t i = 0; i < threads; i++) {
    mappedRegisterFileSets_.emplace_back(registerFileSet_, registerAliasTable_,
                                         i);
  }
  for (size_t i = 0; i < executionUnitCount; i++) {
    executionUnits_.emplace_back(
        issuePorts_[i], completionSlots_[i],
        [this](auto regs, auto values) {
          dispatchIssueUnit_.forwardOperands(regs, values);
        },
        [this](auto uop) { loadStoreQueue_.startLoad(uop); },
        [this](auto uop) {}, [this](auto uop) { uop->setCommitReady(); },
        [this](auto afterSeqId, auto address, auto threadId) {
          raiseFlush(afterSeqId, address, threadId, true);
        },
        branchPredictor);
  }
  // Query and apply initial state
  auto state = isa.getInitialState(processMemory);
  for (uint8_t i = 0; i < threads; i++) {
    applyStateChange(state, i);
  }
};

void Core::tick() {
  ticks_++;

  // Writeback must be ticked at start of cycle, to ensure decode reads the
  // correct values
  writebackUnit_.tick();

  // Tick units
  fetchUnit_.tick();
  decodeUnit_.tick();
  renameUnit_.tick();
  dispatchIssueUnit_.tick();
  for (auto& eu : executionUnits_) {
    // Tick each execution unit
    eu.tick();
  }

  // Late tick for the dispatch/issue unit to issue newly ready uops
  dispatchIssueUnit_.issue();

  // Tick buffers
  // Each unit must have wiped the entries at the head of the buffer after use,
  // as these will now loop around and become the tail.
  fetchToDecodeBuffer_.tick();
  decodeToRenameBuffer_.tick();
  renameToDispatchBuffer_.tick();
  for (auto& issuePort : issuePorts_) {
    issuePort.tick();
  }
  for (auto& completionSlot : completionSlots_) {
    completionSlot.tick();
  }

  // Commit instructions from ROB
  reorderBuffer_.commit(commitWidth);

  if (exceptionGenerated_) {
    handleException();
  }

  flushIfNeeded();
}

void Core::flushIfNeeded() {
  if (!flushPending_) {
    return;
  }

  bool outOfOrderFlushed = false;

  for (size_t thread = 0; thread < flushConditions_.size(); thread++) {
    auto& conditions = flushConditions_[thread];
    if (!conditions.shouldFlush) {
      continue;
    }

    if (conditions.outOfOrder) {
      outOfOrderFlushed = true;
      // Flush out-of-order instructions
      reorderBuffer_.flush(conditions.afterSeqId, thread);
      // In-order flush can only happen during decode; flush everything
      // after that.
      decodeToRenameBuffer_.replaceIf(
          [&thread](auto uop) {
            return (uop != nullptr && uop->getThreadId() == thread);
          },
          nullptr);
      renameToDispatchBuffer_.replaceIf(
          [&thread](auto uop) {
            return (uop != nullptr && uop->getThreadId() == thread);
          },
          nullptr);
    }
    fetchUnit_.updatePC(conditions.address, thread);

    fetchToDecodeBuffer_.replaceIf(
        [&thread](auto macroOp) {
          return (macroOp.size() > 0 && macroOp[0]->getThreadId() == thread);
        },
        {});
    fetchToDecodeBuffer_.stall(false);

    conditions.shouldFlush = false;
  }

  if (outOfOrderFlushed) {
    dispatchIssueUnit_.purgeFlushed();
    loadStoreQueue_.purgeFlushed();
  }
  flushes_++;

  flushPending_ = false;
}

bool Core::hasHalted() const {
  if (hasHalted_) {
    return true;
  }

  // Core is considered to have halted when the fetch unit has halted, and there
  // are no uops at the head of any buffer.
  if (!fetchUnit_.hasHalted()) {
    return false;
  }

  if (reorderBuffer_.size() > 0) {
    return false;
  }

  auto decodeSlots = fetchToDecodeBuffer_.getHeadSlots();
  for (size_t slot = 0; slot < fetchToDecodeBuffer_.getWidth(); slot++) {
    if (decodeSlots[slot].size() > 0) {
      return false;
    }
  }

  auto renameSlots = decodeToRenameBuffer_.getHeadSlots();
  for (size_t slot = 0; slot < decodeToRenameBuffer_.getWidth(); slot++) {
    if (renameSlots[slot] != nullptr) {
      return false;
    }
  }

  return true;
}

void Core::raiseException(const std::shared_ptr<Instruction>& instruction) {
  exceptionGenerated_ = true;
  exceptionGeneratingInstruction_ = instruction;
}

void Core::raiseFlush(uint64_t afterSeqId, uint64_t address, uint8_t threadId,
                      bool outOfOrder) {
  auto& conditions = flushConditions_[threadId];
  if (conditions.shouldFlush) {
    if (!outOfOrder) {
      // There's already a flush pending; as this didn't occur in an
      // out-of-order section, the flush must be for an older instruction
      return;
    }
    if (conditions.outOfOrder && conditions.afterSeqId < afterSeqId) {
      // There's already a flush pending for an older instruction
      return;
    }
  }

  conditions.shouldFlush = true;
  conditions.afterSeqId = afterSeqId;
  conditions.outOfOrder = outOfOrder;
  conditions.address = address;

  flushPending_ = true;
}

void Core::handleException() {
  fetchToDecodeBuffer_.fill({});
  fetchToDecodeBuffer_.stall(false);

  decodeToRenameBuffer_.fill(nullptr);
  decodeToRenameBuffer_.stall(false);

  renameToDispatchBuffer_.fill(nullptr);
  renameToDispatchBuffer_.stall(false);

  auto threadId = exceptionGeneratingInstruction_->getThreadId();

  // Flush everything younger than the exception-generating instruction.
  // This must happen prior to handling the exception to ensure the commit state
  // is up-to-date with the register mapping table
  reorderBuffer_.flush(exceptionGeneratingInstruction_->getSequenceId(),
                       threadId);
  dispatchIssueUnit_.purgeFlushed();
  loadStoreQueue_.purgeFlushed();

  exceptionGenerated_ = false;

  auto result = isa_.handleException(exceptionGeneratingInstruction_,
                                     mappedRegisterFileSets_[threadId],
                                     processMemory_.data());

  if (result.fatal) {
    hasHalted_ = true;
    std::cout << "Halting due to fatal exception" << std::endl;
    return;
  }

  fetchUnit_.updatePC(result.instructionAddress, threadId);
  applyStateChange(result.stateChange, threadId);
}

void Core::applyStateChange(const ProcessStateChange& change, uint8_t thread) {
  // Update registers
  for (size_t i = 0; i < change.modifiedRegisters.size(); i++) {
    mappedRegisterFileSets_[thread].set(change.modifiedRegisters[i],
                                        change.modifiedRegisterValues[i]);
  }

  // Update memory
  for (size_t i = 0; i < change.memoryAddresses.size(); i++) {
    const auto& request = change.memoryAddresses[i];
    const auto& data = change.memoryAddressValues[i];

    auto address = processMemory_.data() + request.first;
    assert(request.first + request.second <= processMemory_.size() &&
           "Attempted to store outside memory limit");
    memcpy(address, data.getAsVector<char>(), request.second);
  }
}

std::map<std::string, std::string> Core::getStats() const {
  auto retired = writebackUnit_.getInstructionsWrittenCount();
  auto ipc = retired / static_cast<float>(ticks_);

  auto branchStalls = fetchUnit_.getBranchStalls();

  auto earlyFlushes = decodeUnit_.getEarlyFlushes();

  auto allocationStalls = renameUnit_.getAllocationStalls();
  auto robStalls = renameUnit_.getROBStalls();
  auto lqStalls = renameUnit_.getLoadQueueStalls();
  auto sqStalls = renameUnit_.getStoreQueueStalls();

  auto rsStalls = dispatchIssueUnit_.getRSStalls();
  auto frontendStalls = dispatchIssueUnit_.getFrontendStalls();
  auto backendStalls = dispatchIssueUnit_.getBackendStalls();
  auto outOfOrderIssues = dispatchIssueUnit_.getOutOfOrderIssueCount();
  auto portBusyStalls = dispatchIssueUnit_.getPortBusyStalls();

  return {{"cycles", std::to_string(ticks_)},
          {"retired", std::to_string(retired)},
          {"ipc", std::to_string(ipc)},
          {"flushes", std::to_string(flushes_)},
          {"fetch.branchStalls", std::to_string(branchStalls)},
          {"decode.earlyFlushes", std::to_string(earlyFlushes)},
          {"rename.allocationStalls", std::to_string(allocationStalls)},
          {"rename.robStalls", std::to_string(robStalls)},
          {"rename.lqStalls", std::to_string(lqStalls)},
          {"rename.sqStalls", std::to_string(sqStalls)},
          {"dispatch.rsStalls", std::to_string(rsStalls)},
          {"issue.frontendStalls", std::to_string(frontendStalls)},
          {"issue.backendStalls", std::to_string(backendStalls)},
          {"issue.outOfOrderIssues", std::to_string(outOfOrderIssues)},
          {"issue.portBusyStalls", std::to_string(portBusyStalls)}};
}

}  // namespace outoforder
}  // namespace models
}  // namespace simeng
