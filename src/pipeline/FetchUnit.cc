#include "FetchUnit.hh"

namespace simeng {
namespace pipeline {

FetchUnit::FetchUnit(PipelineBuffer<MacroOp>& output, const char* insnPtr,
                     unsigned int programByteLength,
                     std::vector<uint64_t> entryPoints, const Architecture& isa,
                     BranchPredictor& branchPredictor)
    : output_(output),
      programCounters_(entryPoints),
      insnPtr_(insnPtr),
      programByteLength_(programByteLength),
      isa_(isa),
      branchPredictor_(branchPredictor){};

void FetchUnit::tick() {
  if (output_.isStalled()) {
    return;
  }

  const uint8_t threadId = 0;

  auto outputSlots = output_.getTailSlots();
  for (size_t slot = 0; slot < output_.getWidth(); slot++) {
    if (hasHalted_) {
      // PC is outside instruction memory region; do nothing
      break;
    }

    auto& macroOp = outputSlots[slot];

    auto& pc = programCounters_[threadId];
    auto prediction = branchPredictor_.predict(pc);
    auto bytesRead =
        isa_.predecode(insnPtr_ + pc, 4, pc, threadId, prediction, macroOp);

    if (!prediction.taken) {
      // Predicted as not taken; increment PC to next instruction
      pc += bytesRead;
    } else {
      // Predicted as taken; set PC to predicted target address
      pc = prediction.target;
    }

    if (pc >= programByteLength_) {
      hasHalted_ = true;
      break;
    }

    if (prediction.taken) {
      if (slot + 1 < output_.getWidth()) {
        branchStalls_++;
      }
      // Can't continue fetch immediately after a branch
      break;
    }
  }
};

bool FetchUnit::hasHalted() const { return hasHalted_; }

void FetchUnit::updatePC(uint64_t address, uint8_t threadId) {
  auto& pc = programCounters_[threadId];
  pc = address;
  hasHalted_ = (pc >= programByteLength_);
}

uint64_t FetchUnit::getBranchStalls() const { return branchStalls_; }

}  // namespace pipeline
}  // namespace simeng
