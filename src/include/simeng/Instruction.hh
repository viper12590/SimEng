#pragma once

#include <vector>

#include "simeng/MemoryInterface.hh"
#include "simeng/RegisterFileSet.hh"
#include "simeng/RegisterValue.hh"
#include "simeng/span.hh"

using InstructionException = short;

namespace simeng {

/** A branch result prediction for an instruction. */
struct BranchPrediction {
  /** Whether the branch will be taken. */
  bool taken;

  /** The branch instruction's target address. If `taken = false`, the value
   * will be ignored. */
  uint64_t target;
};

/** An abstract instruction definition.
 * Each supported ISA should provide a derived implementation of this class. */
class Instruction {
 public:
  virtual ~Instruction(){};

  /** Check whether an exception has been encountered while processing this
   * instruction. */
  bool exceptionEncountered() const;

  /** Retrieve the source registers this instruction reads. */
  virtual const span<Register> getOperandRegisters() const = 0;

  /** Retrieve the destination registers this instruction will write to.
   * A register value of -1 signifies a Zero Register read, and should not be
   * renamed. */
  virtual const span<Register> getDestinationRegisters() const = 0;

  /** Override the specified source register with a renamed physical register.
   */
  virtual void renameSource(uint8_t i, Register renamed) = 0;

  /** Override the specified destination register with a renamed physical
   * register. */
  virtual void renameDestination(uint8_t i, Register renamed) = 0;

  /** Provide a value for the operand at the specified index. */
  virtual void supplyOperand(uint8_t i, const RegisterValue& value) = 0;

  /** Check whether the operand at index `i` has had a value supplied. */
  virtual bool isOperandReady(int i) const = 0;

  /** Check whether all operand values have been supplied, and the instruction
   * is ready to execute. */
  virtual bool canExecute() const = 0;

  /** Execute the instruction. */
  virtual void execute() = 0;

  /** Check whether the instruction has executed and has results ready to
   * write back. */
  bool hasExecuted() const;

  /** Mark the instruction as ready to commit. */
  void setCommitReady();

  /** Check whether the instruction has written its values back and is ready to
   * commit. */
  bool canCommit() const;

  /** Retrieve register results. */
  virtual const span<RegisterValue> getResults() const = 0;

  /** Generate memory addresses this instruction wishes to access. */
  virtual span<const MemoryAccessTarget> generateAddresses() = 0;

  /** Provide data from a requested memory address. */
  virtual void supplyData(uint64_t address, const RegisterValue& data) = 0;

  /** Retrieve previously generated memory addresses. */
  virtual span<const MemoryAccessTarget> getGeneratedAddresses() const = 0;

  /** Retrieve supplied memory data. */
  virtual span<const RegisterValue> getData() const = 0;

  /** Check whether all required data has been supplied. */
  bool hasAllData() const;

  /** Early misprediction check; see if it's possible to determine whether the
   * next instruction address was mispredicted without executing the
   * instruction. Returns a {mispredicted, target} tuple representing whether
   * the instruction was mispredicted, and the correct target address. */
  virtual std::tuple<bool, uint64_t> checkEarlyBranchMisprediction() const = 0;

  /** Check for misprediction. */
  bool wasBranchMispredicted() const;

  /** Retrieve branch address. */
  uint64_t getBranchAddress() const;

  /** Was the branch taken? */
  bool wasBranchTaken() const;

  /** Is this a store operation? */
  virtual bool isStore() const = 0;

  /** Is this a load operation? */
  virtual bool isLoad() const = 0;

  /** Is this a branch operation? */
  virtual bool isBranch() const = 0;

  /** Is this a return instruction? */
  virtual bool isRET() const = 0;

  /** Is this a branch and link instruction? */
  virtual bool isBL() const = 0;

  /** Is this a SVE instruction? */
  /** Maybe remove as aarch64 exclusive? */
  virtual bool isSVE() const = 0;

  /** Set this instruction's instruction memory address. */
  void setInstructionAddress(uint64_t address);

  /** Get this instruction's instruction memory address. */
  uint64_t getInstructionAddress() const;

  /** Supply a branch prediction. */
  void setBranchPrediction(BranchPrediction prediction);

  /** Set this instruction's sequence ID. */
  void setSequenceId(uint64_t seqId);

  /** Retrieve this instruction's sequence ID. */
  uint64_t getSequenceId() const;

  /** Mark this instruction as flushed. */
  void setFlushed();

  /** Check whether this instruction has been flushed. */
  bool isFlushed() const;

  /** Retrieve the instruction group this instruction belongs to. */
  virtual uint16_t getGroup() const = 0;

  /** Retrieve the number of cycles this instruction will take to execute. */
  uint16_t getLatency() const;

  /** Retrieve the number of cycles this instruction will take to be prcoessed
   * by the LSQ. */
  uint16_t getLSQLatency() const;

  /** Retrieve the number of cycles this instruction will block the unit
   * executing it. */
  uint16_t getStallCycles() const;

  /** Get this instruction's supported set of ports. */
  virtual const std::vector<uint8_t>& getSupportedPorts() = 0;

  bool shouldSplitRequests() const;

 protected:
  /** Whether an exception has been encountered. */
  bool exceptionEncountered_ = false;

  /** The location in memory of this instruction was decoded at. */
  uint64_t instructionAddress_;

  /** Whether or not this instruction has been executed. */
  bool executed_ = false;

  /** Whether or not this instruction is ready to commit. */
  bool canCommit_ = false;

  // Memory
  /** The number of data items that still need to be supplied. */
  uint8_t dataPending_ = 0;

  // Branches
  /** The predicted branching result. */
  BranchPrediction prediction_;

  /** A branching address calculated by this instruction during execution. */
  uint64_t branchAddress_;

  /** Was the branch taken? */
  bool branchTaken_ = false;

  // Flushing
  /** This instruction's sequence ID; a higher ID represents a chronologically
   * newer instruction. */
  uint64_t sequenceId_;

  /** Has this instruction been flushed? */
  bool flushed_ = false;

  /** The number of cycles this instruction takes to execute. */
  uint16_t latency_ = 1;

  /** The number of cycles a load or store instruction takes to execute within
   * the load/store queue. */
  uint16_t lsqExecutionLatency_ = 1;

  /** The number of cycles this instruction will stall the unit executing it
   * for. */
  uint16_t stallCycles_ = 1;

  /** The execution ports that this instruction can be issued to. */
  std::vector<uint8_t> supportedPorts_ = {};

  /** Whether this instructions' memory accesses should be treated as many
   * independent requests. **/
  bool splitMemoryRequests_ = false;
};

}  // namespace simeng