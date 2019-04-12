#pragma once

#include <functional>

#include "../Architecture.hh"
#include "PipelineBuffer.hh"

namespace simeng {
namespace pipeline {

/** A decode unit for a pipelined processor. Splits pre-decoded macro-ops into
 * uops. */
class DecodeUnit {
 public:
  /** Constructs a decode unit with references to input/output buffers, the
   * current branch predictor, and a function handle to raise encountered
   * flushes. */
  DecodeUnit(
      PipelineBuffer<MacroOp>& input,
      PipelineBuffer<std::shared_ptr<Instruction>>& output,
      BranchPredictor& predictor,
      std::function<void(uint64_t address, uint8_t threadId)> raiseFlush);

  /** Ticks the decode unit. Breaks macro-ops into uops, and performs early
   * branch misprediction checks. */
  void tick();

  /** Retrieve the number of times that the decode unit requested a flush due to
   * discovering a branch misprediction early. */
  uint64_t getEarlyFlushes() const;

 private:
  /** A buffer of macro-ops to split into uops. */
  PipelineBuffer<MacroOp>& input_;
  /** A buffer for writing decoded uops into. */
  PipelineBuffer<std::shared_ptr<Instruction>>& output_;

  /** A reference to the current branch predictor. */
  BranchPredictor& predictor_;

  /** A function handle to call to raise an encountered flush. */
  std::function<void(uint64_t address, uint8_t threadId)> raiseFlush_;

  /** The number of times that the decode unit requested a flush due to
   * discovering a branch misprediction early. */
  uint64_t earlyFlushes_ = 0;
};

}  // namespace pipeline
}  // namespace simeng
