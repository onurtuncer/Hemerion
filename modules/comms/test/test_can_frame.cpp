// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_can_frame.cpp
//
// Native unit test for the CAN framing layer. Run by CTest under the
// test-native preset. Unity (referenced in modules/README.md as the project's
// test framework) is not yet vendored, so this uses plain asserts and an exit
// code; switch to Unity test cases once vendor/Unity exists.
// ------------------------------------------------------------------------------
#include <array>
#include <cassert>
#include <cstdio>

#include "hemerion/comms/can_fifo.h"
#include "hemerion/comms/can_frame.h"

using hemerion::comms::CanFifo;
using hemerion::comms::CanFrame;
using hemerion::comms::CanFrameError;
using hemerion::comms::kCanExtendedIdMax;
using hemerion::comms::kCanStandardIdMax;
using hemerion::comms::make_frame;
using hemerion::comms::validate_frame;

namespace
{

void test_standard_frame_roundtrip()
{
  const std::array<std::uint8_t, 3> payload = { 0x01, 0x02, 0x03 };
  CanFrame frame;
  const CanFrameError error = make_frame(0x123, false, payload.data(), 3, false, frame);

  assert(error == CanFrameError::kNone);
  assert(frame.id == 0x123);
  assert(frame.dlc == 3);
  assert(!frame.extended);
  assert(frame.data[0] == 0x01 && frame.data[1] == 0x02 && frame.data[2] == 0x03);
}

void test_standard_id_out_of_range()
{
  CanFrame frame;
  const CanFrameError error = make_frame(kCanStandardIdMax + 1, false, nullptr, 0, false, frame);
  assert(error == CanFrameError::kIdOutOfRange);
}

void test_extended_id_accepted()
{
  CanFrame frame;
  const CanFrameError error = make_frame(kCanExtendedIdMax, true, nullptr, 0, false, frame);
  assert(error == CanFrameError::kNone);
  assert(frame.id == kCanExtendedIdMax);
}

void test_dlc_out_of_range()
{
  const std::array<std::uint8_t, 9> payload = {};
  CanFrame frame;
  const CanFrameError error = make_frame(0x1, false, payload.data(), 9, false, frame);
  assert(error == CanFrameError::kDlcOutOfRange);
}

void test_remote_request_frame_has_no_payload_bytes_copied()
{
  CanFrame frame;
  const CanFrameError error = make_frame(0x7FF, false, nullptr, 0, true, frame);
  assert(error == CanFrameError::kNone);
  assert(frame.remote_request);
  assert(frame.dlc == 0);
}

void test_validate_frame_rejects_corrupted_dlc()
{
  CanFrame frame;
  frame.id = 0x10;
  frame.dlc = 200;  // Not reachable via make_frame(); simulates a decoded/corrupted frame.
  assert(validate_frame(frame) == CanFrameError::kDlcOutOfRange);
}

void test_fifo_push_pop_order_and_capacity()
{
  CanFifo<2> fifo;
  CanFrame a;
  CanFrame b;
  CanFrame c;
  assert(make_frame(0x1, false, nullptr, 0, false, a) == CanFrameError::kNone);
  assert(make_frame(0x2, false, nullptr, 0, false, b) == CanFrameError::kNone);
  assert(make_frame(0x3, false, nullptr, 0, false, c) == CanFrameError::kNone);

  assert(fifo.push(a));
  assert(fifo.push(b));
  assert(!fifo.push(c));  // Capacity 2, already full.
  assert(fifo.full());

  CanFrame out;
  assert(fifo.pop(out) && out.id == 0x1);
  assert(fifo.pop(out) && out.id == 0x2);
  assert(!fifo.pop(out));  // Empty.
  assert(fifo.empty());
}

}  // namespace

int main()
{
  test_standard_frame_roundtrip();
  test_standard_id_out_of_range();
  test_extended_id_accepted();
  test_dlc_out_of_range();
  test_remote_request_frame_has_no_payload_bytes_copied();
  test_validate_frame_rejects_corrupted_dlc();
  test_fifo_push_pop_order_and_capacity();

  std::puts("test_can_frame: all checks passed");
  return 0;
}
