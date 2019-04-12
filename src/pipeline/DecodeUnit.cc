#include "DecodeUnit.hh"

#include <cassert>

namespace simeng {
namespace pipeline {

DecodeUnit::DecodeUnit(
    PipelineBuffer<MacroOp>& input,
    PipelineBuffer<std::shared_ptr<Instruction>>& output,
    BranchPredictor& predictor,
    std::function<void(uint64_t address, uint8_t threadId)> raiseFlush)
    : input_(input),
      output_(output),
      predictor_(predictor),
      raiseFlush_(raiseFlush){};

void DecodeUnit::tick() {
  if (output_.isStalled()) {
    input_.stall(true);
    return;
  }

  input_.stall(false);

  for (size_t slot = 0; slot < input_.getWidth(); slot++) {
    auto& macroOp = input_.getHeadSlots()[slot];

    // Assume single uop per macro op for this version
    // TODO: Stall on multiple uops and siphon one per cycle, recording progress
    assert(macroOp.size() <= 1 &&
           "Multiple uops per macro-op not yet supported");

    if (macroOp.size() == 0) {
      // Nothing to process for this macro-op
      continue;
    }

    auto& uop = macroOp[0];

    output_.getTailSlots()[slot] = uop;
    input_.getHeadSlots()[slot].clear();

    // Check preliminary branch prediction results now that the instruction is
    // decoded. Identifies:
    // - Non-branch instructions mistakenly predicted as branches
    // - Incorrect targets for immediate branches
    auto [misprediction, correctAddress] = uop->checkEarlyBranchMisprediction();
    if (misprediction) {
      earlyFlushes_++;
      raiseFlush_(correctAddress, uop->getThreadId());

      if (!uop->isBranch()) {
        // Non-branch incorrectly predicted as a branch; let the predictor know
        predictor_.update(uop->getInstructionAddress(), false, correctAddress);
      }

      // Skip processing remaining macro-ops, as they need to be flushed
      break;
    }
  }
}

uint64_t DecodeUnit::getEarlyFlushes() const { return earlyFlushes_; };

}  // namespace pipeline
}  // namespace simeng
