#include "ExecuteUnit.hh"

#include <cstring>

namespace simeng {
namespace pipeline {

ExecuteUnit::ExecuteUnit(
    PipelineBuffer<std::shared_ptr<Instruction>>& input,
    PipelineBuffer<std::shared_ptr<Instruction>>& output,
    std::function<void(span<Register>, span<RegisterValue>)> forwardOperands,
    std::function<void(const std::shared_ptr<Instruction>&)> handleLoad,
    std::function<void(const std::shared_ptr<Instruction>&)> handleStore,
    std::function<void(const std::shared_ptr<Instruction>&)> raiseException,
    std::function<void(uint64_t afterSeqId, uint64_t address, uint8_t threadId)>
        raiseFlush,
    BranchPredictor& predictor)
    : input_(input),
      output_(output),
      forwardOperands_(forwardOperands),
      handleLoad_(handleLoad),
      handleStore_(handleStore),
      raiseException_(raiseException),
      raiseFlush_(raiseFlush),
      predictor_(predictor) {}

void ExecuteUnit::tick() {
  tickCounter_++;

  auto& uop = input_.getHeadSlots()[0];
  if (uop != nullptr) {
    if (!uop->isFlushed()) {
      // TODO: Retrieve latency from the instruction
      const unsigned int latency = 1;

      if (latency == 1 && pipeline_.size() == 0) {
        // Pipeline is empty and insn will execute this cycle; bypass
        execute(uop);
      } else {
        // Add insn to pipeline
        pipeline_.push({uop, tickCounter_ + latency - 1});
      }
    }
    input_.getHeadSlots()[0] = nullptr;
  }

  if (pipeline_.size() == 0) {
    return;
  }

  // Pop flushed instructions from the pipeline until a non-flushed instruction
  // is found. If the pipeline ends up empty, return early.
  while (pipeline_.front().insn->isFlushed()) {
    pipeline_.pop();
    if (pipeline_.size() == 0) {
      return;
    }
  }

  auto& head = pipeline_.front();
  if (head.readyAt <= tickCounter_) {
    execute(head.insn);
    pipeline_.pop();
  }
}

void ExecuteUnit::execute(std::shared_ptr<Instruction>& uop) {
  assert(uop->canExecute() &&
         "Attempted to execute an instruction before it was ready");

  if (uop->exceptionEncountered()) {
    // Exception encountered prior to execution
    // TODO: Identify whether this can be removed; executing an
    // exception-encountered uop would have to be guaranteed to be safe
    raiseException_(uop);
    return;
  }

  if (uop->isLoad()) {
    uop->generateAddresses();
    handleLoad_(uop);
  } else if (uop->isStore()) {
    uop->generateAddresses();
  }

  uop->execute();
  if (uop->exceptionEncountered()) {
    // Exception; don't forward results, don't pass uop forward
    raiseException_(uop);
    return;
  }

  if (uop->isStore()) {
    handleStore_(uop);
  } else if (uop->isBranch()) {
    // Update branch predictor with branch results
    predictor_.update(uop->getInstructionAddress(), uop->wasBranchTaken(),
                      uop->getBranchAddress());

    if (uop->wasBranchMispredicted()) {
      // Misprediction; flush the pipeline
      raiseFlush_(uop->getSequenceId(), uop->getBranchAddress(),
                  uop->getThreadId());
    }
  }

  // Operand forwarding; allows a dependent uop to execute next cycle
  forwardOperands_(uop->getDestinationRegisters(), uop->getResults());

  output_.getTailSlots()[0] = uop;
}

}  // namespace pipeline
}  // namespace simeng
