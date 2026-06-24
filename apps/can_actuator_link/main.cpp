// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// apps/can_actuator_link/main.cpp
//
// Minimal consumer of modules/comms: packs actuator setpoints into CAN
// frames and stages them in a CanFifo. Transmission is stubbed as printing
// the frame, since cmake/hemerion_hal/can.h and a BSP don't exist yet --
// replace drain_to_stdout's print with a hal_can_transmit() call once they do.
// ------------------------------------------------------------------------------
#include <array>
#include <cstdint>
#include <cstring>
#include <print>

#include "hemerion/comms/can_fifo.h"
#include "hemerion/comms/can_frame.h"

namespace {

constexpr std::size_t kTxFifoCapacity = 8;

struct Setpoint {
  std::uint16_t actuator_id;
  float value;
};

[[nodiscard]] bool stage_setpoint_frame(const Setpoint& setpoint,
                                         hemerion::comms::CanFifo<kTxFifoCapacity>& tx_fifo) {
  std::array<std::uint8_t, 4> payload = {};
  std::memcpy(payload.data(), &setpoint.value, sizeof(setpoint.value));

  hemerion::comms::CanFrame frame;
  const hemerion::comms::CanFrameError error = hemerion::comms::make_frame(
      setpoint.actuator_id, false, payload.data(), payload.size(), false, frame);
  if (error != hemerion::comms::CanFrameError::kNone) {
    return false;
  }
  return tx_fifo.push(frame);
}

void drain_to_stdout(hemerion::comms::CanFifo<kTxFifoCapacity>& tx_fifo) {
  hemerion::comms::CanFrame frame;
  while (tx_fifo.pop(frame)) {
    float value = 0.0F;
    std::memcpy(&value, frame.data.data(), sizeof(value));
    std::println("CAN TX id=0x{:03X} dlc={} value={}", frame.id, static_cast<unsigned>(frame.dlc),
                 value);
  }
}

}  // namespace

int main() {
  hemerion::comms::CanFifo<kTxFifoCapacity> tx_fifo;

  constexpr std::array<Setpoint, 2> kSetpoints = {
      Setpoint{.actuator_id = 0x100, .value = 0.25F},
      Setpoint{.actuator_id = 0x101, .value = -0.10F},
  };

  for (const Setpoint& setpoint : kSetpoints) {
    if (!stage_setpoint_frame(setpoint, tx_fifo)) {
      std::println(stderr, "failed to stage setpoint for actuator 0x{:03X}", setpoint.actuator_id);
    }
  }

  drain_to_stdout(tx_fifo);
  return 0;
}
