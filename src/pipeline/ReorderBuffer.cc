#include "ReorderBuffer.hh"

#include <algorithm>
#include <cassert>

namespace simeng {
namespace pipeline {

ReorderBuffer::ReorderBuffer(
    unsigned int maxSize, RegisterAliasTable& rat, LoadStoreQueue& lsq,
    std::function<void(const std::shared_ptr<Instruction>&)> raiseException,
    std::function<void(uint64_t afterSeqId, uint64_t address, uint8_t threadId)>
        raiseFlush)
    : rat_(rat),
      lsq_(lsq),
      maxSize_(maxSize),
      raiseException_(raiseException),
      raiseFlush_(raiseFlush) {}

void ReorderBuffer::reserve(const std::shared_ptr<Instruction>& insn) {
  assert(buffer_.size() < maxSize_ &&
         "Attempted to reserve entry in reorder buffer when already full");
  insn->setSequenceId(seqId_);
  seqId_++;
  buffer_.push_back(insn);
}

unsigned int ReorderBuffer::commit(unsigned int maxCommitSize) {
  shouldFlush_ = false;
  size_t maxCommits =
      std::min(static_cast<size_t>(maxCommitSize), buffer_.size());

  unsigned int n;
  for (n = 0; n < maxCommits; n++) {
    auto& uop = buffer_[0];
    if (!uop->canCommit()) {
      break;
    }

    if (uop->exceptionEncountered()) {
      raiseException_(uop);
      buffer_.pop_front();
      return n + 1;
    }

    const auto& threadId = uop->getThreadId();

    const auto& destinations = uop->getDestinationRegisters();
    for (const auto& reg : destinations) {
      rat_.commit(reg, threadId);
    }

    // If it's a memory op, commit the entry at the head of the respective queue
    if (uop->isStore()) {
      bool violationFound = lsq_.commitStore(uop);
      if (violationFound) {
        // Memory order violation found; aborting commits and flushing
        auto load = lsq_.getViolatingLoad();

        raiseFlush_(load->getSequenceId() - 1, load->getInstructionAddress(),
                    load->getThreadId());

        buffer_.pop_front();
        return n + 1;
      }
    } else if (uop->isLoad()) {
      lsq_.commitLoad(uop);
    }
    buffer_.pop_front();
  }

  return n;
}

void ReorderBuffer::flush(uint64_t afterSeqId, uint8_t threadId) {
  // Iterate backwards from the tail of the queue to find and remove ops newer
  // than `afterSeqId`
  while (!buffer_.empty()) {
    auto& uop = buffer_.back();
    if (uop->getSequenceId() <= afterSeqId) {
      break;
    }
    if (uop->getThreadId() != threadId) {
      continue;
    }

    for (const auto& reg : uop->getDestinationRegisters()) {
      rat_.rewind(reg, threadId);
    }
    uop->setFlushed();
    buffer_.pop_back();
  }
}

unsigned int ReorderBuffer::size() const { return buffer_.size(); }

unsigned int ReorderBuffer::getFreeSpace() const {
  return maxSize_ - buffer_.size();
}

}  // namespace pipeline
}  // namespace simeng
