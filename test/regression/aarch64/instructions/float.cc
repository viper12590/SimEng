#include "AArch64RegressionTest.hh"

#include <cmath>

namespace {

using InstFloat = AArch64RegressionTest;

TEST_P(InstFloat, fabs) {
  RUN_AARCH64(R"(
    fmov s0, 2.0
    fmov s1, -0.125
    fmov s2, 12.5
    fabs s3, s0
    fabs s4, s1
    fabs s5, s2
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)), 2.f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(4)), 0.125f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(5)), 12.5f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(5)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(5)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(5)), 0.f);

  RUN_AARCH64(R"(
    fmov d0, 2.0
    fmov d1, -0.125
    fmov d2, 12.5
    fabs d3, d0
    fabs d4, d1
    fabs d5, d2
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)), 2.0);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(4)), 0.125);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(4)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(5)), 12.5);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(5)), 0.0);
}

TEST_P(InstFloat, fadd) {
  // 32-bit
  RUN_AARCH64(R"(
    fmov s0, 1.0
    fmov s1, -0.125
    fmov s2, 7.5
    fadd s3, s0, s1
    fadd s4, s0, s2
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)), 0.875f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(4)), 8.5f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(4)), 0.f);

  // 64-bit
  RUN_AARCH64(R"(
    fmov d0, 1.0
    fmov d1, -0.125
    fmov d2, 7.5
    fadd d3, d0, d1
    fadd d4, d0, d2
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)), 0.875);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(4)), 8.5);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(4)), 0.0);
}

TEST_P(InstFloat, fcmp32) {
  // 1.25 == 1.25
  RUN_AARCH64(R"(
    fmov s0, 1.25
    fmov s1, 1.25
    fcmp s0, s1
  )");
  EXPECT_EQ(getNZCV(), 0b0110);

  // 1.25 > -1.25
  RUN_AARCH64(R"(
    fmov s0, 1.25
    fmov s1, -1.25
    fcmp s0, s1
  )");
  EXPECT_EQ(getNZCV(), 0b0010);

  // 1.25 < 10.5
  RUN_AARCH64(R"(
    fmov s0, 1.25
    fmov s1, 10.5
    fcmp s0, s1
  )");
  EXPECT_EQ(getNZCV(), 0b1000);

  // 1.25 > 0.0 (immediate)
  RUN_AARCH64(R"(
    fmov s0, 1.25
    fcmp s0, 0.0
  )");
  EXPECT_EQ(getNZCV(), 0b0010);

  // 1.0 vs NaN
  initialHeapData_.resize(8);
  reinterpret_cast<float*>(initialHeapData_.data())[0] = std::nan("");
  RUN_AARCH64(R"(
    # Get heap address
    mov x0, 0
    mov x8, 214
    svc #0

    fmov s0, 1.0
    ldr s1, [x0]
    fcmp s0, s1
  )");
  EXPECT_EQ(getNZCV(), 0b0011);
}

TEST_P(InstFloat, fcmp64) {
  // 1.25 == 1.25
  RUN_AARCH64(R"(
    fmov d0, 1.25
    fmov d1, 1.25
    fcmp d0, d1
  )");
  EXPECT_EQ(getNZCV(), 0b0110);

  // 1.25 > -1.25
  RUN_AARCH64(R"(
    fmov d0, 1.25
    fmov d1, -1.25
    fcmp d0, d1
  )");
  EXPECT_EQ(getNZCV(), 0b0010);

  // 1.25 < 10.5
  RUN_AARCH64(R"(
    fmov d0, 1.25
    fmov d1, 10.5
    fcmp d0, d1
  )");
  EXPECT_EQ(getNZCV(), 0b1000);

  // 1.25 > 0.0 (immediate)
  RUN_AARCH64(R"(
    fmov d0, 1.25
    fcmp d0, 0.0
  )");
  EXPECT_EQ(getNZCV(), 0b0010);

  // 1.0 vs NaN
  initialHeapData_.resize(8);
  reinterpret_cast<double*>(initialHeapData_.data())[0] = std::nan("");
  RUN_AARCH64(R"(
    # Get heap address
    mov x0, 0
    mov x8, 214
    svc #0

    fmov d0, 1.0
    ldr d1, [x0]
    fcmp d0, d1
  )");
  EXPECT_EQ(getNZCV(), 0b0011);
}

TEST_P(InstFloat, fcsel32) {
  // 1.25 == 1.25
  RUN_AARCH64(R"(
    fmov s0, 1.25
    fmov s1, 1.25
    fmov s2, 5.0
    fcmp s0, s1
    fcsel s3, s2, s1, eq
    fcsel s4, s2, s1, lo
    fcsel s5, s2, s1, gt
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)), 5.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(4)), 1.25f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(5)), 1.25f);

  // 1.25 > -1.25
  RUN_AARCH64(R"(
    fmov s0, 1.25
    fmov s1, -1.25
    fmov s2, 5.0
    fcmp s0, s1
    fcsel s3, s2, s1, eq
    fcsel s4, s2, s1, lo
    fcsel s5, s2, s1, gt
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)), -1.25f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(4)), -1.25f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(5)), 5.f);

  // 1.25 < 10.5
  RUN_AARCH64(R"(
    fmov s0, 1.25
    fmov s1, 10.5
    fmov s2, 5.0
    fcmp s0, s1
    fcsel s3, s2, s1, eq
    fcsel s4, s2, s1, lo
    fcsel s5, s2, s1, gt
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)), 10.5f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(4)), 5.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(5)), 10.5f);

  // 1.0 vs NaN
  initialHeapData_.resize(8);
  reinterpret_cast<float*>(initialHeapData_.data())[0] = std::nan("");
  RUN_AARCH64(R"(
    # Get heap address
    mov x0, 0
    mov x8, 214
    svc #0

    fmov s0, 1.0
    ldr s1, [x0]
    fmov s2, 5.0
    fcmp s0, s1
    fcsel s3, s2, s0, eq
    fcsel s4, s2, s0, lo
    fcsel s5, s2, s0, gt
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)), 1.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(4)), 1.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(5)), 1.f);
}

TEST_P(InstFloat, fcsel64) {
  // 1.25 == 1.25
  RUN_AARCH64(R"(
    fmov d0, 1.25
    fmov d1, 1.25
    fmov d2, 5.0
    fcmp d0, d1
    fcsel d3, d2, d1, eq
    fcsel d4, d2, d1, lo
    fcsel d5, d2, d1, gt
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)), 5.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(4)), 1.25);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(5)), 1.25);

  // 1.25 > -1.25
  RUN_AARCH64(R"(
    fmov d0, 1.25
    fmov d1, -1.25
    fmov d2, 5.0
    fcmp d0, d1
    fcsel d3, d2, d1, eq
    fcsel d4, d2, d1, lo
    fcsel d5, d2, d1, gt
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)), -1.25);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(4)), -1.25);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(5)), 5.0);

  // 1.25 < 10.5
  RUN_AARCH64(R"(
    fmov d0, 1.25
    fmov d1, 10.5
    fmov d2, 5.0
    fcmp d0, d1
    fcsel d3, d2, d1, eq
    fcsel d4, d2, d1, lo
    fcsel d5, d2, d1, gt
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)), 10.5);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(4)), 5.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(5)), 10.5);

  // 1.0 vs NaN
  initialHeapData_.resize(8);
  reinterpret_cast<double*>(initialHeapData_.data())[0] = std::nan("");
  RUN_AARCH64(R"(
    # Get heap address
    mov x0, 0
    mov x8, 214
    svc #0

    fmov d0, 1.0
    ldr d1, [x0]
    fmov d2, 5.0
    fcmp d0, d1
    fcsel d3, d2, d0, eq
    fcsel d4, d2, d0, lo
    fcsel d5, d2, d0, gt
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)), 1.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(4)), 1.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(5)), 1.0);
}

TEST_P(InstFloat, fcvt) {
  initialHeapData_.resize(32);
  double* heap = reinterpret_cast<double*>(initialHeapData_.data());
  heap[0] = 1.0;
  heap[1] = -42.76;
  heap[2] = -0.125;
  heap[3] = 321.5;

  // 32-bit to 64-bit
  RUN_AARCH64(R"(
    fmov s0, 1.25
    fmov s1, -10.5
    fcvt d0, s0
    fcvt d1, s1
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(0)), 1.25);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(1)), -10.5);

  // 64-bit to 32-bit
  RUN_AARCH64(R"(
    fmov d0, 1.25
    fmov d1, -10.5
    fcvt s0, d0
    fcvt s1, d1
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(0)), 1.25f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(1)), -10.5f);

  // Signed, round to zero
  RUN_AARCH64(R"(
    # Get heap address
    mov x0, 0
    mov x8, 214
    svc #0

    ldp d0, d1, [x0]
    ldp d2, d3, [x0, #16]
    fcvtzs w0, d0
    fcvtzs w1, d1
    fcvtzs w2, d2
    fcvtzs w3, d3
  )");
  EXPECT_EQ((getGeneralRegister<int32_t>(0)), 1);
  EXPECT_EQ((getGeneralRegister<int32_t>(1)), -42);
  EXPECT_EQ((getGeneralRegister<int32_t>(2)), 0);
  EXPECT_EQ((getGeneralRegister<int32_t>(3)), 321);
}

TEST_P(InstFloat, fdiv) {
  // FP32
  RUN_AARCH64(R"(
    fmov s0, 2.0
    fmov s1, -0.125
    fmov s2, 16
    fdiv s3, s0, s1
    fdiv s4, s0, s2
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)), -16);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(4)), 0.125);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(4)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(4)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(4)), 0.0);

  // FP64
  RUN_AARCH64(R"(
    fmov d0, 2.0
    fmov d1, -0.125
    fmov d2, 12.5
    fdiv d3, d0, d1
    fdiv d4, d0, d2
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)), -16);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(4)), 0.16);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(4)), 0.0);
}

TEST_P(InstFloat, fmadd) {
  // 32-bit
  RUN_AARCH64(R"(
    fmov s0, 2.0
    fmov s1, -0.125
    fmov s2, 7.5
    fmadd s3, s0, s1, s2
    fmadd s4, s1, s2, s0
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)), 7.25f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(4)), 1.0625f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(4)), 0.f);

  // 64-bit
  RUN_AARCH64(R"(
    fmov d0, 2.0
    fmov d1, -0.125
    fmov d2, 7.5
    fmadd d3, d0, d1, d2
    fmadd d4, d1, d2, d0
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)), 7.25);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(4)), 1.0625);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(4)), 0.0);
}

TEST_P(InstFloat, fmov) {
  // FP32 scalar from immediate
  RUN_AARCH64(R"(
    fmov s0, 1.0
    fmov s1, -0.125
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(0)), 1.0);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(0)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(0)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(0)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(1)), -0.125);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(1)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(1)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(1)), 0.0);

  // FP32 scalar from register
  RUN_AARCH64(R"(
    fmov s0, 1.0
    fmov s1, -0.125
    fmov s2, s1
    fmov s3, s0
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(2)), -0.125);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(2)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(2)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(2)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)), 1.0);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(3)), 0.0);

  // FP32 general from scalar
  initialHeapData_.resize(8);
  reinterpret_cast<float*>(initialHeapData_.data())[0] = 128.5;
  reinterpret_cast<float*>(initialHeapData_.data())[1] = -0.0625;
  RUN_AARCH64(R"(
    # Get heap address
    mov x0, 0
    mov x8, 214
    svc #0

    ldr s1, [x0]
    ldr s2, [x0, #4]
    fmov w1, s1
    fmov w2, s2
  )");
  EXPECT_EQ((getGeneralRegister<float>(1)), 128.5);
  EXPECT_EQ((getGeneralRegister<float>(2)), -0.0625);

  // FP32 scalar from general
  initialHeapData_.resize(8);
  reinterpret_cast<float*>(initialHeapData_.data())[0] = 128.5;
  reinterpret_cast<float*>(initialHeapData_.data())[1] = -0.0625;
  RUN_AARCH64(R"(
    # Get heap address
    mov x0, 0
    mov x8, 214
    svc #0

    ldr w1, [x0]
    ldr w2, [x0, #4]
    fmov s0, w1
    fmov s1, w2
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(0)), 128.5);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(0)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(0)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(0)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(1)), -0.0625);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(1)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(1)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(1)), 0.0);

  // FP64 scalar from immediate
  RUN_AARCH64(R"(
    fmov d0, 1.0
    fmov d1, -0.125
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(0)), 1.0);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(0)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(1)), -0.125);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(1)), 0.0);

  // FP64 scalar from register
  RUN_AARCH64(R"(
    fmov d0, 1.0
    fmov d1, -0.125
    fmov d2, d1
    fmov d3, d0
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(2)), -0.125);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(2)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)), 1.0);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(3)), 0.0);

  // FP64 general from scalar
  initialHeapData_.resize(16);
  reinterpret_cast<double*>(initialHeapData_.data())[0] = 123.456;
  reinterpret_cast<double*>(initialHeapData_.data())[1] = -0.00032;
  RUN_AARCH64(R"(
    # Get heap address
    mov x0, 0
    mov x8, 214
    svc #0

    ldr d1, [x0]
    ldr d2, [x0, #8]
    fmov x1, d1
    fmov x2, d2
  )");
  EXPECT_EQ((getGeneralRegister<double>(1)), 123.456);
  EXPECT_EQ((getGeneralRegister<double>(2)), -0.00032);

  // FP64 scalar from general
  initialHeapData_.resize(16);
  reinterpret_cast<double*>(initialHeapData_.data())[0] = 123.456;
  reinterpret_cast<double*>(initialHeapData_.data())[1] = -0.00032;
  RUN_AARCH64(R"(
    # Get heap address
    mov x0, 0
    mov x8, 214
    svc #0

    ldr x1, [x0]
    ldr x2, [x0, #8]
    fmov d0, x1
    fmov d1, x2
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(0)), 123.456);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(0)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(1)), -0.00032);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(1)), 0.0);

  // FP64 top half to general
  initialHeapData_.resize(32);
  reinterpret_cast<double*>(initialHeapData_.data())[0] = 111.111;
  reinterpret_cast<double*>(initialHeapData_.data())[1] = 123.456;
  reinterpret_cast<double*>(initialHeapData_.data())[2] = 111.111;
  reinterpret_cast<double*>(initialHeapData_.data())[3] = -0.00032;
  RUN_AARCH64(R"(
    # Get heap address
    mov x0, 0
    mov x8, 214
    svc #0

    ldr q1, [x0]
    ldr q2, [x0, #16]
    fmov x1, v1.d[1]
    fmov x2, v2.d[1]
  )");
  EXPECT_EQ((getGeneralRegister<double>(1)), 123.456);
  EXPECT_EQ((getGeneralRegister<double>(2)), -0.00032);

  // FP64 top half from general
  initialHeapData_.resize(32);
  reinterpret_cast<double*>(initialHeapData_.data())[0] = 123.456;
  reinterpret_cast<double*>(initialHeapData_.data())[1] = -0.00032;
  RUN_AARCH64(R"(
    # Get heap address
    mov x0, 0
    mov x8, 214
    svc #0

    ldr d1, [x0]
    ldr x2, [x0, #8]
    fmov v1.d[1], x2
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(1)), 123.456);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(1)), -0.00032);
}

TEST_P(InstFloat, fmsub) {
  // 32-bit
  RUN_AARCH64(R"(
    fmov s0, 2.0
    fmov s1, -0.125
    fmov s2, 7.5
    fmsub s3, s0, s1, s2
    fmsub s4, s1, s2, s0
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)), 7.75f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(4)), 2.9375f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(4)), 0.f);

  // 64-bit
  RUN_AARCH64(R"(
    fmov d0, 2.0
    fmov d1, -0.125
    fmov d2, 7.5
    fmsub d3, d0, d1, d2
    fmsub d4, d1, d2, d0
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)), 7.75);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(4)), 2.9375);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(4)), 0.0);
}

TEST_P(InstFloat, fmul) {
  // 32-bit
  RUN_AARCH64(R"(
    fmov s0, 2.0
    fmov s1, -0.125
    fmov s2, 7.5
    fmul s3, s0, s1
    fmul s4, s0, s2
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)), -0.25f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(4)), 15.f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(4)), 0.f);

  // 64-bit
  RUN_AARCH64(R"(
    fmov d0, 2.0
    fmov d1, -0.125
    fmov d2, 7.5
    fmul d3, d0, d1
    fmul d4, d0, d2
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)), -0.25);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(4)), 15.0);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(4)), 0.0);
}

TEST_P(InstFloat, fneg) {
  // 32-bit
  RUN_AARCH64(R"(
    fmov s0, 2.0
    fmov s1, -0.125
    fmov s2, 12.5
    fneg s3, s0
    fneg s4, s1
    fneg s5, s2
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)), -2.f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(4)), 0.125f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(5)), -12.5f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(5)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(5)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(5)), 0.f);

  // 64-bit
  RUN_AARCH64(R"(
    fmov d0, 2.0
    fmov d1, -0.125
    fmov d2, 12.5
    fneg d3, d0
    fneg d4, d1
    fneg d5, d2
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)), -2.0);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(4)), 0.125);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(4)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(5)), -12.5);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(5)), 0.0);
}

TEST_P(InstFloat, fnmsub) {
  // 32-bit
  RUN_AARCH64(R"(
    fmov s0, 2.0
    fmov s1, -0.125
    fmov s2, 7.5
    fnmsub s3, s0, s1, s2
    fnmsub s4, s1, s2, s0
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)), -7.75f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(4)), -2.9375f);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(4)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(4)), 0.f);

  // 64-bit
  RUN_AARCH64(R"(
    fmov d0, 2.0
    fmov d1, -0.125
    fmov d2, 7.5
    fnmsub d3, d0, d1, d2
    fnmsub d4, d1, d2, d0
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)), -7.75);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(4)), -2.9375);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(4)), 0.0);
}

TEST_P(InstFloat, fsqrt) {
  // 32-bit
  RUN_AARCH64(R"(
    fmov s0, 2.0
    fmov s1, -0.125
    fsqrt s2, s0
    fsqrt s3, s1
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(2)), ::sqrtf(2.f));
  EXPECT_EQ((getVectorRegisterElement<float, 1>(2)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(2)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(2)), 0.f);
  EXPECT_TRUE(std::isnan(getVectorRegisterElement<float, 0>(3)));
  EXPECT_EQ((getVectorRegisterElement<float, 1>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(3)), 0.f);

  // 64-bit
  RUN_AARCH64(R"(
    fmov d0, 2.0
    fmov d1, -0.125
    fsqrt d2, d0
    fsqrt d3, d1
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(2)), ::sqrt(2.0));
  EXPECT_EQ((getVectorRegisterElement<double, 1>(2)), 0.0);
  EXPECT_TRUE(std::isnan(getVectorRegisterElement<double, 0>(3)));
  EXPECT_EQ((getVectorRegisterElement<double, 1>(3)), 0.0);
}

TEST_P(InstFloat, fsub) {
  // FP32
  RUN_AARCH64(R"(
    fmov s0, 1.0
    fmov s1, -0.125
    fmov s2, 7.5
    fsub s3, s0, s1
    fsub s4, s0, s2
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)), 1.125);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(4)), -6.5);
  EXPECT_EQ((getVectorRegisterElement<float, 1>(4)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 2>(4)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 3>(4)), 0.0);

  // FP64
  RUN_AARCH64(R"(
    fmov d0, 1.0
    fmov d1, -0.125
    fmov d2, 7.5
    fsub d3, d0, d1
    fsub d4, d0, d2
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)), 1.125);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(4)), -6.5);
  EXPECT_EQ((getVectorRegisterElement<double, 1>(4)), 0.0);
}

TEST_P(InstFloat, scvtf) {
  // 32-bit integer
  initialHeapData_.resize(16);
  int32_t* heap32 = reinterpret_cast<int32_t*>(initialHeapData_.data());
  heap32[0] = 1;
  heap32[1] = -1;
  heap32[2] = INT32_MAX;
  heap32[3] = INT32_MIN;
  RUN_AARCH64(R"(
    # Get heap address
    mov x0, 0
    mov x8, 214
    svc #0

    # Load and convert integer values
    ldp s0, s1, [x0]
    scvtf s0, s0
    scvtf s1, s1
    ldp s2, s3, [x0, #8]
    scvtf s2, s2
    scvtf s3, s3

    # Load and convert integer values (via general)
    ldp w1, w2, [x0]
    scvtf s4, w1
    scvtf s5, w2
    ldp w3, w4, [x0, #8]
    scvtf s6, w3
    scvtf s7, w4

    # Load and convert integer values to double precision (via general)
    ldp w1, w2, [x0]
    scvtf d8, w1
    scvtf d9, w2
    ldp w3, w4, [x0, #8]
    scvtf d10, w3
    scvtf d11, w4
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(0)), 1.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(1)), -1.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(2)),
            static_cast<float>(INT32_MAX));
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)),
            static_cast<float>(INT32_MIN));
  EXPECT_EQ((getVectorRegisterElement<float, 0>(4)), 1.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(5)), -1.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(6)),
            static_cast<float>(INT32_MAX));
  EXPECT_EQ((getVectorRegisterElement<float, 0>(7)),
            static_cast<float>(INT32_MIN));
  EXPECT_EQ((getVectorRegisterElement<double, 0>(8)), 1.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(9)), -1.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(10)),
            static_cast<double>(INT32_MAX));
  EXPECT_EQ((getVectorRegisterElement<double, 0>(11)),
            static_cast<double>(INT32_MIN));

  // 64-bit integer
  initialHeapData_.resize(32);
  int64_t* heap64 = reinterpret_cast<int64_t*>(initialHeapData_.data());
  heap64[0] = 1;
  heap64[1] = -1;
  heap64[2] = INT64_MAX;
  heap64[3] = INT64_MIN;
  RUN_AARCH64(R"(
    # Get heap address
    mov x0, 0
    mov x8, 214
    svc #0

    # Load and convert integer values
    ldp d0, d1, [x0]
    scvtf d0, d0
    scvtf d1, d1
    ldp d2, d3, [x0, #16]
    scvtf d2, d2
    scvtf d3, d3

    # Load and convert integer values (via general)
    ldp x1, x2, [x0]
    scvtf d4, x1
    scvtf d5, x2
    ldp x3, x4, [x0, #16]
    scvtf d6, x3
    scvtf d7, x4

    # Load and convert integer values to single precision (via general)
    ldp x1, x2, [x0]
    scvtf s8, x1
    scvtf s9, x2
    ldp x3, x4, [x0, #16]
    scvtf s10, x3
    scvtf s11, x4
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(0)), 1.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(1)), -1.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(2)),
            static_cast<double>(INT64_MAX));
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)),
            static_cast<double>(INT64_MIN));
  EXPECT_EQ((getVectorRegisterElement<double, 0>(4)), 1.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(5)), -1.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(6)),
            static_cast<double>(INT64_MAX));
  EXPECT_EQ((getVectorRegisterElement<double, 0>(7)),
            static_cast<double>(INT64_MIN));
  EXPECT_EQ((getVectorRegisterElement<float, 0>(8)), 1.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(9)), -1.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(10)),
            static_cast<float>(INT64_MAX));
  EXPECT_EQ((getVectorRegisterElement<float, 0>(11)),
            static_cast<float>(INT64_MIN));
}

TEST_P(InstFloat, ucvtf) {
  // 32-bit integer
  initialHeapData_.resize(16);
  uint32_t* heap32 = reinterpret_cast<uint32_t*>(initialHeapData_.data());
  heap32[0] = 1;
  heap32[1] = 65537;
  heap32[2] = UINT32_MAX;
  heap32[3] = 0;
  RUN_AARCH64(R"(
    # Get heap address
    mov x0, 0
    mov x8, 214
    svc #0

    # Load and convert integer values
    ldp s0, s1, [x0]
    ucvtf s0, s0
    ucvtf s1, s1
    ldp s2, s3, [x0, #8]
    ucvtf s2, s2
    ucvtf s3, s3

    # Load and convert integer values (via general)
    ldp w1, w2, [x0]
    ucvtf s4, w1
    ucvtf s5, w2
    ldp w3, w4, [x0, #8]
    ucvtf s6, w3
    ucvtf s7, w4

    # Load and convert integer values to double precision (via general)
    ldp w1, w2, [x0]
    ucvtf d8, w1
    ucvtf d9, w2
    ldp w3, w4, [x0, #8]
    ucvtf d10, w3
    ucvtf d11, w4
  )");
  EXPECT_EQ((getVectorRegisterElement<float, 0>(0)), 1.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(1)), 65537.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(2)),
            static_cast<float>(UINT32_MAX));
  EXPECT_EQ((getVectorRegisterElement<float, 0>(3)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(4)), 1.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(5)), 65537.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(6)),
            static_cast<float>(UINT32_MAX));
  EXPECT_EQ((getVectorRegisterElement<float, 0>(7)), 0.f);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(8)), 1.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(9)), 65537.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(10)),
            static_cast<double>(UINT32_MAX));
  EXPECT_EQ((getVectorRegisterElement<double, 0>(11)), 0.0);

  // 64-bit integer
  initialHeapData_.resize(32);
  uint64_t* heap64 = reinterpret_cast<uint64_t*>(initialHeapData_.data());
  heap64[0] = 1;
  heap64[1] = (UINT64_C(1) << 48);
  heap64[2] = UINT64_MAX;
  heap64[3] = 0;
  RUN_AARCH64(R"(
    # Get heap address
    mov x0, 0
    mov x8, 214
    svc #0

    # Load and convert integer values
    ldp d0, d1, [x0]
    ucvtf d0, d0
    ucvtf d1, d1
    ldp d2, d3, [x0, #16]
    ucvtf d2, d2
    ucvtf d3, d3

    # Load and convert integer values (via general)
    ldp x1, x2, [x0]
    ucvtf d4, x1
    ucvtf d5, x2
    ldp x3, x4, [x0, #16]
    ucvtf d6, x3
    ucvtf d7, x4

    # Load and convert integer values to single precision (via general)
    ldp x1, x2, [x0]
    ucvtf s8, x1
    ucvtf s9, x2
    ldp x3, x4, [x0, #16]
    ucvtf s10, x3
    ucvtf s11, x4
  )");
  EXPECT_EQ((getVectorRegisterElement<double, 0>(0)), 1.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(1)),
            static_cast<float>(UINT64_C(1) << 48));
  EXPECT_EQ((getVectorRegisterElement<double, 0>(2)),
            static_cast<double>(UINT64_MAX));
  EXPECT_EQ((getVectorRegisterElement<double, 0>(3)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(4)), 1.0);
  EXPECT_EQ((getVectorRegisterElement<double, 0>(5)),
            static_cast<float>(UINT64_C(1) << 48));
  EXPECT_EQ((getVectorRegisterElement<double, 0>(6)),
            static_cast<double>(UINT64_MAX));
  EXPECT_EQ((getVectorRegisterElement<double, 0>(7)), 0.0);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(8)), 1.f);
  EXPECT_EQ((getVectorRegisterElement<float, 0>(9)),
            static_cast<float>(UINT64_C(1) << 48));
  EXPECT_EQ((getVectorRegisterElement<float, 0>(10)),
            static_cast<float>(UINT64_MAX));
  EXPECT_EQ((getVectorRegisterElement<float, 0>(11)), 0.f);
}

INSTANTIATE_TEST_SUITE_P(AArch64, InstFloat, ::testing::Values(EMULATION),
                         coreTypeToString);

}  // namespace
