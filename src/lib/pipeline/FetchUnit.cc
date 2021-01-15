#include "simeng/pipeline/FetchUnit.hh"

namespace simeng {
namespace pipeline {

FetchUnit::FetchUnit(PipelineBuffer<MacroOp>& output,
                     MemoryInterface& instructionMemory,
                     uint64_t programByteLength, uint64_t entryPoint,
                     uint8_t blockAlignmentBits, const arch::Architecture& isa,
                     BranchPredictor& branchPredictor)
    : output_(output),
      pc_(entryPoint),
      instructionMemory_(instructionMemory),
      programByteLength_(programByteLength),
      isa_(isa),
      branchPredictor_(branchPredictor),
      blockSize_(1 << blockAlignmentBits),
      blockMask_(~(blockSize_ - 1)) {
  assert(blockSize_ >= isa_.getMaxInstructionSize() &&
         "fetch block size must be larger than the largest instruction");
  fetchBuffer_ = new uint8_t[2 * blockSize_];
  requestFromPC();
}

FetchUnit::~FetchUnit() { delete[] fetchBuffer_; }

void FetchUnit::tick() {
  if (output_.isStalled()) {
    return;
  }

  if (hasHalted_) {
    return;
  }

  // Pointer to the instruction data to decode from
  const uint8_t* buffer;
  uint8_t bufferOffset;

  // Check if more instruction data is required
  if (bufferedBytes_ < isa_.getMaxInstructionSize()) {
    // Calculate the address of the next fetch block
    uint64_t blockAddress;
    if (bufferedBytes_ > 0) {
      // There is already some data in the buffer, so check for the next block
      bufferOffset = 0;
      blockAddress = pc_ + bufferedBytes_;
      assert((blockAddress & ~blockMask_) == 0 && "misaligned fetch buffer");
    } else {
      // Fetch buffer is empty, so start from the PC
      blockAddress = pc_ & blockMask_;
      bufferOffset = pc_ - blockAddress;
    }

    // Find fetched memory that matches the desired block
    const auto& fetched = instructionMemory_.getCompletedReads();

    size_t fetchIndex;
    for (fetchIndex = 0; fetchIndex < fetched.size(); fetchIndex++) {
      if (fetched[fetchIndex].target.address == blockAddress) {
        break;
      }
    }
    if (fetchIndex == fetched.size()) {
      // Need to wait for fetched instructions
      // Stalled.fetch.instructionFetch
      probeTrace newProbe = {0, trace_cycle, 0};
      Trace* newTrace = new Trace;
      newTrace->setProbeTraces(newProbe);
      probeList.push_back(newTrace);
      return;
    }

    // TODO: Handle memory faults
    assert(fetched[fetchIndex].data && "Memory read failed");
    const uint8_t* fetchData = fetched[fetchIndex].data.getAsVector<uint8_t>();

    if (bufferedBytes_ > 0) {
      // Copy fetched data to fetch buffer after existing data
      std::memcpy(fetchBuffer_ + bufferedBytes_, fetchData + bufferOffset,
                  blockSize_ - bufferOffset);
      bufferedBytes_ += blockSize_ - bufferOffset;
      buffer = fetchBuffer_;
    } else {
      // Use the incoming fetch data directly to avoid a copy
      buffer = fetchData;
      bufferedBytes_ = blockSize_ - bufferOffset;
    }
  } else {
    // There is already enough data in the fetch buffer, so use that
    buffer = fetchBuffer_;
    bufferOffset = 0;
  }

  // Check we have enough data to begin decoding
  if (bufferedBytes_ < isa_.getMaxInstructionSize()) {
    // Stalled.fetch.instructionDecode
    probeTrace newProbe = {1, trace_cycle, 0};
    Trace* newTrace = new Trace;
    newTrace->setProbeTraces(newProbe);
    probeList.push_back(newTrace);
    return;
  }

  auto outputSlots = output_.getTailSlots();
  for (size_t slot = 0; slot < output_.getWidth(); slot++) {
    auto& macroOp = outputSlots[slot];

    BranchPrediction prediction = {false, 0};
    std::string disasm;
    auto bytesRead = isa_.predecode(buffer + bufferOffset, bufferedBytes_, pc_,
                                    prediction, macroOp, disasm);

    // If predecode fails, bail and wait for more data
    if (bytesRead == 0) {
      assert(bufferedBytes_ < isa_.getMaxInstructionSize() &&
             "unexpected predecode failure");
      break;
    }

    assert(bytesRead <= bufferedBytes_ &&
           "Predecode consumed more bytes than were available");

    // Create map element for new fetch
    if (disasm != "") {
      macroOp[0]->setTraceId(traceId);
      const uint32_t insn =
          *static_cast<const uint32_t*>((void*)(buffer + bufferOffset));
      fetchTrace newFetch = {trace_cycle, insn, pc_, 0, disasm};
      cycleTrace newCycleTrace = {newFetch, 0, 0, 0, 0, 0, 0};
      Trace* newTrace = new Trace;
      newTrace->setCycleTraces(newCycleTrace);
      traceMap.insert({macroOp[0]->getTraceId(), newTrace});
      // Denote id has been assigned/used
      traceId++;
    } else {
      macroOp[0]->setTraceId(traceId);
      // Denote id has been assigned/used
      traceId++;
    }

    // Increment the offset, decrement available bytes
    bufferOffset += bytesRead;
    bufferedBytes_ -= bytesRead;

    // Create brnach prediction after identifing instruction type
    // (e.g. RET, BL, etc).
    prediction = branchPredictor_.predict(macroOp[0]);
    macroOp[0]->setBranchPrediction(prediction);

    if (!prediction.taken) {
      // Predicted as not taken; increment PC to next instruction
      pc_ += bytesRead;
    } else {
      // Predicted as taken; set PC to predicted target address
      pc_ = prediction.target;
    }

    if (pc_ >= programByteLength_) {
      // Halt.fetch.programMemoryExceeded
      probeTrace newProbe = {16, trace_cycle, 0};
      Trace* newTrace = new Trace;
      newTrace->setProbeTraces(newProbe);
      probeList.push_back(newTrace);

      hasHalted_ = true;
      break;
    }

    if (prediction.taken) {
      if (slot + 1 < output_.getWidth()) {
        // Branch.fetch.stalled
        probeTrace newProbe = {12, trace_cycle, macroOp[0]->getTraceId()};
        Trace* newTrace = new Trace;
        newTrace->setProbeTraces(newProbe);
        probeList.push_back(newTrace);

        branchStalls_++;
      }
      // Can't continue fetch immediately after a branch
      bufferedBytes_ = 0;
      break;
    }

    // Too few bytes remaining in buffer to continue
    if (bufferedBytes_ == 0) {
      break;
    }
  }

  if (bufferedBytes_ > 0) {
    // Move start of fetched data to beginning of fetch buffer
    std::memmove(fetchBuffer_, buffer + bufferOffset, bufferedBytes_);
  }

  instructionMemory_.clearCompletedReads();
}

bool FetchUnit::hasHalted() const { return hasHalted_; }

void FetchUnit::updatePC(uint64_t address) {
  pc_ = address;
  bufferedBytes_ = 0;
  hasHalted_ = (pc_ >= programByteLength_);
}

void FetchUnit::requestFromPC() {
  // Do nothing if buffer already contains enough data
  if (bufferedBytes_ >= isa_.getMaxInstructionSize()) return;

  uint64_t blockAddress;
  if (bufferedBytes_ > 0) {
    // There's already some data in the buffer, so fetch the next block
    blockAddress = pc_ + bufferedBytes_;
    assert((blockAddress & ~blockMask_) == 0 && "misaligned fetch buffer");
  } else {
    // Fetch buffer is empty, so fetch from the PC
    blockAddress = pc_ & blockMask_;
  }

  instructionMemory_.requestRead({blockAddress, blockSize_});
}

uint64_t FetchUnit::getBranchStalls() const { return branchStalls_; }

}  // namespace pipeline
}  // namespace simeng
