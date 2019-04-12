#pragma once

#include <deque>
#include <functional>

#include "../Instruction.hh"
#include "LoadStoreQueue.hh"
#include "RegisterAliasTable.hh"

namespace simeng {
namespace pipeline {

/** A Reorder Buffer (ROB) implementation. Contains an in-order queue of
 * in-flight instructions. */
class ReorderBuffer {
 public:
  /** Constructs a reorder buffer of maximum size `maxSize`, supplying a
   * reference to the register alias table. */
  ReorderBuffer(
      unsigned int maxSize, RegisterAliasTable& rat, LoadStoreQueue& lsq,
      std::function<void(const std::shared_ptr<Instruction>&)> raiseException,
      std::function<void(uint64_t afterSeqId, uint64_t address,
                         uint8_t threadId)>
          raiseFlush);

  /** Add the provided instruction to the ROB. */
  void reserve(const std::shared_ptr<Instruction>& insn);

  /** Commit and remove up to `maxCommitSize` instructions. */
  unsigned int commit(unsigned int maxCommitSize);

  /** Flush all instructions with a sequence ID greater than `afterSeqId`. */
  void flush(uint64_t afterSeqId, uint8_t threadId);

  /** Retrieve the current size of the ROB. */
  unsigned int size() const;

  /** Retrieve the current amount of free space in the ROB. */
  unsigned int getFreeSpace() const;

 private:
  /** A reference to the register alias table. */
  RegisterAliasTable& rat_;

  /** A reference to the load/store queue. */
  LoadStoreQueue& lsq_;

  /** The maximum size of the ROB. */
  unsigned int maxSize_;

  /** A function to call upon exception generation. */
  std::function<void(std::shared_ptr<Instruction>)> raiseException_;

  /** A function to call upon flush discovery. */
  std::function<void(uint64_t afterSeqId, uint64_t address, uint8_t threadId)>
      raiseFlush_;

  /** The buffer containing in-flight instructions. */
  std::deque<std::shared_ptr<Instruction>> buffer_;

  /** Whether the core should be flushed after the most recent commit. */
  bool shouldFlush_ = false;

  /** The target instruction address the PC should be reset to after the most
   * recent commit.
   */
  uint64_t pc_;

  /** The sequence ID of the youngest instruction that should remain after the
   * current flush. */
  uint64_t flushAfter_;

  /** The next available sequence ID. */
  uint64_t seqId_ = 0;
};

}  // namespace pipeline
}  // namespace simeng
