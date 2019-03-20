#include "../MockInstruction.hh"
#include "Instruction.hh"
#include "PipelineBuffer.hh"
#include "RegisterFileSet.hh"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "inorder/WritebackUnit.hh"

using ::testing::_;
using ::testing::DoAll;
using ::testing::Field;
using ::testing::Return;
using ::testing::SetArgReferee;

namespace simeng {
namespace inorder {

class InOrderWritebackUnitTest : public testing::Test {
 public:
  InOrderWritebackUnitTest()
      : input(1, nullptr),
        registerFileSet({{8, 2}}),
        uop(new MockInstruction),
        uopPtr(uop),
        writebackUnit(input, registerFileSet) {}

 protected:
  PipelineBuffer<std::shared_ptr<Instruction>> input;
  RegisterFileSet registerFileSet;

  MockInstruction* uop;
  std::shared_ptr<Instruction> uopPtr;
  WritebackUnit writebackUnit;
};

// Tests that a value is correctly written back, and the uop is cleared from the
// buffer
TEST_F(InOrderWritebackUnitTest, Tick) {
  input.getHeadSlots()[0] = uopPtr;
  uint32_t result = 1;
  std::vector<RegisterValue> results = {RegisterValue(result)};
  std::vector<Register> destinations = {{0, 1}};

  EXPECT_CALL(*uop, getResults())
      .WillOnce(Return(span<RegisterValue>(results.data(), results.size())));
  EXPECT_CALL(*uop, getDestinationRegisters())
      .WillOnce(
          Return(span<Register>(destinations.data(), destinations.size())));

  writebackUnit.tick();

  EXPECT_EQ(registerFileSet.get(destinations[0]).get<uint32_t>(), result);
  EXPECT_EQ(input.getHeadSlots()[0], nullptr);
}

}  // namespace inorder
}  // namespace simeng