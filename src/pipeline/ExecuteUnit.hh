#pragma once

#include <functional>
#include <queue>

#include "../BranchPredictor.hh"
#include "../Instruction.hh"
#include "PipelineBuffer.hh"

namespace simeng {
namespace pipeline {

/** An execution unit pipeline entry, containing an instruction, and an
 * indication of when it's reached the front of the execution pipeline. */
struct ExecutionUnitPipelineEntry {
  /** The instruction queued for execution. */
  std::shared_ptr<Instruction> insn;
  /** The tick number this instruction will reach the front of the queue at. */
  uint64_t readyAt;
};

/** An execute unit for a pipelined processor. Executes instructions and
 * forwards results. */
class ExecuteUnit {
 public:
  /** Constructs an execute unit with references to an input and output buffer,
   * the currently used branch predictor, and handlers for forwarding operands,
   * loads/stores, and exceptions. */
  ExecuteUnit(
      PipelineBuffer<std::shared_ptr<Instruction>>& input,
      PipelineBuffer<std::shared_ptr<Instruction>>& output,
      std::function<void(span<Register>, span<RegisterValue>)> forwardOperands,
      std::function<void(const std::shared_ptr<Instruction>&)> handleLoad,
      std::function<void(const std::shared_ptr<Instruction>&)> handleStore,
      std::function<void(const std::shared_ptr<Instruction>&)> raiseException,
      std::function<void(uint64_t afterSeqId, uint64_t address,
                         uint8_t threadId)>
          raiseFlush,
      BranchPredictor& predictor);

  /** Tick the execute unit. Places incoming instructions into the pipeline and
   * executes an instruction that has reached the head of the pipeline, if
   * present. */
  void tick();

 private:
  /** Execute the supplied uop, write it into the output buffer, and forward
   * results back to dispatch/issue. */
  void execute(std::shared_ptr<Instruction>& uop);

  /** A buffer of instructions to execute. */
  PipelineBuffer<std::shared_ptr<Instruction>>& input_;

  /** A buffer for writing executed instructions into. */
  PipelineBuffer<std::shared_ptr<Instruction>>& output_;

  /** A function handle called when forwarding operands. */
  std::function<void(span<Register>, span<RegisterValue>)> forwardOperands_;

  /** A function handle called after generating the addresses for a load. */
  std::function<void(const std::shared_ptr<Instruction>&)> handleLoad_;
  /** A function handle called after aquiring the data for a store. */
  std::function<void(const std::shared_ptr<Instruction>&)> handleStore_;

  /** A function handle called upon exception generation. */
  std::function<void(const std::shared_ptr<Instruction>&)> raiseException_;

  /** A function handle called upon flush discovery. */
  std::function<void(uint64_t afterSeqId, uint64_t address, uint8_t threadId)>
      raiseFlush_;

  /** A reference to the branch predictor, for updating with prediction results.
   */
  BranchPredictor& predictor_;

  /** The execution unit's internal pipeline, holding instructions until their
   * execution latency has expired and they are ready for their final results to
   * be calculated and forwarded. */
  std::queue<ExecutionUnitPipelineEntry> pipeline_;

  /** The number of times this unit has been ticked. */
  uint64_t tickCounter_ = 0;
};

}  // namespace pipeline
}  // namespace simeng
