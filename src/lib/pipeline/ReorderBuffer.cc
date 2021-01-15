#include "simeng/pipeline/ReorderBuffer.hh"

#include <algorithm>
#include <cassert>

namespace simeng {
namespace pipeline {

ReorderBuffer::ReorderBuffer(
    unsigned int maxSize, RegisterAliasTable& rat, LoadStoreQueue& lsq,
    std::function<void(const std::shared_ptr<Instruction>&)> raiseException)
    : rat_(rat),
      lsq_(lsq),
      maxSize_(maxSize),
      raiseException_(raiseException) {}

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

    bool tracePrintable = false;
    std::map<uint64_t, Trace*>::iterator it;
    cycleTrace tr;
    if (uop->getTraceId() != 0) {
      it = traceMap.find(uop->getTraceId());
      if (it != traceMap.end()) {
        tracePrintable = true;
        tr = it->second->getCycleTraces();
      }
    }

    instructionsCommitted_++;

    if (tracePrintable) {
      if (tr.finished != 1) {
        tr.retire = trace_cycle;
        tr.finished = 1;
        it->second->setCycleTraces(tr);
      }
    }

    if (uop->exceptionEncountered()) {
      if (tracePrintable) {
        tr.finished = 1;
        it->second->setCycleTraces(tr);
      }
      // Exception.rob.robCommit
      probeTrace newProbe = {17, trace_cycle, uop->getTraceId()};
      Trace* newTrace = new Trace;
      newTrace->setProbeTraces(newProbe);
      probeList.push_back(newTrace);

      raiseException_(uop);
      buffer_.pop_front();
      return n + 1;
    }

    const auto& destinations = uop->getDestinationRegisters();
    for (const auto& reg : destinations) {
      rat_.commit(reg);
    }

    // If it's a memory op, commit the entry at the head of the respective queue
    if (uop->isStore()) {
      bool violationFound = lsq_.commitStore(uop);
      if (violationFound) {
        // Memory order violation found; aborting commits and flushing
        auto load = lsq_.getViolatingLoad();
        shouldFlush_ = true;
        flushAfter_ = load->getSequenceId() - 1;
        pc_ = load->getInstructionAddress();

        if (tracePrintable) {
          tr.finished = 1;
          it->second->setCycleTraces(tr);
        }
        // Flush.rob.storeViolation
        probeTrace newProbe = {15, trace_cycle, uop->getTraceId()};
        Trace* newTrace = new Trace;
        newTrace->setProbeTraces(newProbe);
        probeList.push_back(newTrace);

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

void ReorderBuffer::flush(uint64_t afterSeqId) {
  // Iterate backwards from the tail of the queue to find and remove ops newer
  // than `afterSeqId`
  while (!buffer_.empty()) {
    auto& uop = buffer_.back();
    if (uop->getSequenceId() <= afterSeqId) {
      break;
    }

    for (const auto& reg : uop->getDestinationRegisters()) {
      rat_.rewind(reg);
    }
    uop->setFlushed();

    if (uop->getTraceId() != 0) {
      std::map<uint64_t, Trace*>::iterator it =
          traceMap.find(uop->getTraceId());
      if (it != traceMap.end()) {
        cycleTrace tr = it->second->getCycleTraces();
        tr.finished = 1;
        it->second->setCycleTraces(tr);
      }
    }

    buffer_.pop_back();
  }
}

unsigned int ReorderBuffer::size() const { return buffer_.size(); }

unsigned int ReorderBuffer::getFreeSpace() const {
  return maxSize_ - buffer_.size();
}

bool ReorderBuffer::shouldFlush() const { return shouldFlush_; }
uint64_t ReorderBuffer::getFlushAddress() const { return pc_; }
uint64_t ReorderBuffer::getFlushSeqId() const { return flushAfter_; }

uint64_t ReorderBuffer::getInstructionsCommittedCount() const {
  return instructionsCommitted_;
}

}  // namespace pipeline
}  // namespace simeng
