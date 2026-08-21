// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file mmc5983ma_driver.cpp
/// @brief Implements the register-level MMC5983MA driver declared in
/// mmc5983ma_driver.h.

#include "Hemerion/mag/mmc5983ma/mmc5983ma_driver.h"

#include <array>

#include "Hemerion/mag/mag_conversion.h"

namespace hemerion::sensors::mag::mmc5983ma
{

namespace
{

[[nodiscard]] constexpr std::uint8_t reg_address(Mmc5983maRegister reg) { return static_cast<std::uint8_t>(reg); }

// The two continuous rates the datasheet's CM_Freq table conditions on a
// bandwidth. (Its separate "max output data rate" table is a
// characterization result rather than a mode constraint, so it is documented
// against Mmc5983maBandwidth instead of enforced here -- rejecting a
// configuration the part will actually run is worse than letting a caller
// ask for one the part merely will not sustain.)
[[nodiscard]] constexpr bool rate_reachable(Mmc5983maContinuousRate rate, Mmc5983maBandwidth bandwidth)
{
  if (rate == Mmc5983maContinuousRate::k200Hz)
  {
    return bandwidth == Mmc5983maBandwidth::k200Hz;
  }
  if (rate == Mmc5983maContinuousRate::k1000Hz)
  {
    return bandwidth == Mmc5983maBandwidth::k800Hz;
  }
  return true;
}

// Midpoint of a SET/RESET measurement pair, relative to null field: the
// bridge offset both measurements carry with the same sign, while the field
// carries opposite signs and cancels. Both inputs are 18-bit, so the sum
// cannot overflow.
[[nodiscard]] constexpr std::int32_t pair_offset(std::uint32_t after_set, std::uint32_t after_reset)
{
  const std::int32_t sum = static_cast<std::int32_t>(after_set) + static_cast<std::int32_t>(after_reset);
  return (sum / 2) - static_cast<std::int32_t>(kMmc5983maNullFieldOutput);
}

}  // namespace

Mmc5983maError Mmc5983maDriver::probe(const Mmc5983maConfig& config)
{
  if (!rate_reachable(config.continuous_rate, config.bandwidth))
  {
    return Mmc5983maError::kInvalidConfiguration;
  }
  config_ = config;

  // Identify before touching anything: a software reset sent to whatever
  // part is actually at this address would be an unfriendly way to discover
  // a wiring mistake.
  std::uint8_t product_id = 0;
  if (!read_register(Mmc5983maRegister::kProductId, product_id))
  {
    return Mmc5983maError::kTransferFailed;
  }
  if (product_id != kMmc5983maProductId)
  {
    return Mmc5983maError::kIdentityMismatch;
  }

  // Known state: the software reset clears every register and re-reads OTP,
  // dropping any configuration a previous run (or bootloader probe) left
  // behind.
  if (!write_register(Mmc5983maRegister::kInternalControl1, kMmc5983maControl1SoftReset))
  {
    return Mmc5983maError::kTransferFailed;
  }
  bus_.delay_ms(kMmc5983maSoftResetDelayMs);

  // The part re-answering its identity is the sign the reset is over; a
  // wrong answer here means the settle time was not respected or the bus
  // glitched.
  if (!read_register(Mmc5983maRegister::kProductId, product_id))
  {
    return Mmc5983maError::kTransferFailed;
  }
  if (product_id != kMmc5983maProductId)
  {
    return Mmc5983maError::kIdentityMismatch;
  }

  if (!write_register(
          Mmc5983maRegister::kInternalControl1,
          static_cast<std::uint8_t>(static_cast<std::uint8_t>(config_.bandwidth) & kMmc5983maControl1BandwidthMask)))
  {
    return Mmc5983maError::kTransferFailed;
  }

  // Internal control 0 is write-only, so its two persistent bits have to be
  // shadowed: every later write of a self-clearing trigger has to re-assert
  // them or it would silently turn them off.
  control0_shadow_ = 0;
  if (config_.interrupt_enable)
  {
    control0_shadow_ |= kMmc5983maControl0InterruptEnable;
  }
  if (config_.automatic_set_reset)
  {
    control0_shadow_ |= kMmc5983maControl0AutoSetResetEnable;
  }
  if (!write_register(Mmc5983maRegister::kInternalControl0, control0_shadow_))
  {
    return Mmc5983maError::kTransferFailed;
  }

  // Condition the bridges before measuring anything. The datasheet's
  // bring-up ends here ("the MEMSIC AMR sensors have been conditioned for
  // optimum performance and data measurements can commence"); without it the
  // sensing polarity is whatever the last disturbing field left behind, and
  // a part that came up RESET reads the whole field negated.
  if (!write_register(Mmc5983maRegister::kInternalControl0,
                      static_cast<std::uint8_t>(control0_shadow_ | kMmc5983maControl0Set)))
  {
    return Mmc5983maError::kTransferFailed;
  }

  if (config_.automatic_set_reset)
  {
    // The part cancels the offset itself, so there is nothing for the
    // software offset to hold -- and a stale one from set_bridge_offset() or
    // an earlier probe() would now be subtracted from already-corrected
    // readings.
    bridge_offset_ = Mmc5983maBridgeOffset{};
  }
  else if (config_.calibrate_offset_on_probe)
  {
    const Mmc5983maError error = calibrate_offset();
    if (error != Mmc5983maError::kNone)
    {
      return error;
    }
  }

  // Mode last, so the part starts free-running under the configuration it
  // will measure with. (calibrate_offset() already restored this; writing it
  // again costs one transaction and keeps the ordering rule true on both
  // paths.)
  return apply_continuous_mode();
}

Mmc5983maError Mmc5983maDriver::calibrate_offset()
{
  // Refused rather than run under Auto_SR_en, and not merely because it
  // would be redundant. With that bit set the part runs its own SET/RESET
  // pair inside every measurement and outputs the positive-polarity
  // difference, so the manual pair below reads the *same* value twice
  // instead of a sign-flipped pair: the field no longer cancels, and the
  // "offset" this would store is the standing field. Installing that would
  // null out the magnetometer.
  if (config_.automatic_set_reset)
  {
    return Mmc5983maError::kInvalidConfiguration;
  }

  // Out of continuous mode first: the free-running engine would otherwise
  // latch measurements between the SET and its read, and the pair would no
  // longer bracket one field.
  if (!write_register(Mmc5983maRegister::kInternalControl2,
                      mmc5983ma_control2(Mmc5983maContinuousRate::kOff, false, config_.periodic_set, false)))
  {
    return Mmc5983maError::kTransferFailed;
  }

  Mmc5983maFieldCounts after_set;
  Mmc5983maFieldCounts after_reset;

  // SET: sensing polarity positive, so this measurement is +H + offset.
  if (!write_register(Mmc5983maRegister::kInternalControl0,
                      static_cast<std::uint8_t>(control0_shadow_ | kMmc5983maControl0Set)))
  {
    return Mmc5983maError::kTransferFailed;
  }
  Mmc5983maError error = trigger_and_wait(kMmc5983maControl0TakeMeasurement, kMmc5983maStatusMeasurementDone);
  if (error != Mmc5983maError::kNone)
  {
    return error;
  }
  if (!read_field_counts(after_set))
  {
    return Mmc5983maError::kTransferFailed;
  }

  // RESET: polarity flipped, so this one is -H + offset. The offset does not
  // flip -- it is the bridge's, not the field's -- which is the whole trick.
  if (!write_register(Mmc5983maRegister::kInternalControl0,
                      static_cast<std::uint8_t>(control0_shadow_ | kMmc5983maControl0Reset)))
  {
    return Mmc5983maError::kTransferFailed;
  }
  error = trigger_and_wait(kMmc5983maControl0TakeMeasurement, kMmc5983maStatusMeasurementDone);
  if (error != Mmc5983maError::kNone)
  {
    return error;
  }
  if (!read_field_counts(after_reset))
  {
    return Mmc5983maError::kTransferFailed;
  }

  bridge_offset_.x = pair_offset(after_set.x, after_reset.x);
  bridge_offset_.y = pair_offset(after_set.y, after_reset.y);
  bridge_offset_.z = pair_offset(after_set.z, after_reset.z);

  // Leave the part SET. Skipping this is the subtle way to get a
  // sign-flipped magnetometer: every subsequent measurement would still be
  // offset-corrected, and still be the negative of the field.
  if (!write_register(Mmc5983maRegister::kInternalControl0,
                      static_cast<std::uint8_t>(control0_shadow_ | kMmc5983maControl0Set)))
  {
    return Mmc5983maError::kTransferFailed;
  }

  return apply_continuous_mode();
}

Mmc5983maError Mmc5983maDriver::measure_once(MagSample& out)
{
  const Mmc5983maError error = trigger_and_wait(kMmc5983maControl0TakeMeasurement, kMmc5983maStatusMeasurementDone);
  if (error != Mmc5983maError::kNone)
  {
    return error;
  }

  Mmc5983maFieldCounts counts;
  if (!read_field_counts(counts))
  {
    return Mmc5983maError::kTransferFailed;
  }
  if (convert(counts, out) != MagConversionError::kNone)
  {
    return Mmc5983maError::kInvalidConfiguration;
  }
  return Mmc5983maError::kNone;
}

Mmc5983maReadResult Mmc5983maDriver::read_sample(MagSample& out)
{
  std::uint8_t status = 0;
  if (!read_register(Mmc5983maRegister::kStatus, status))
  {
    return Mmc5983maReadResult::kTransferFailed;
  }
  if ((status & kMmc5983maStatusMeasurementDone) == 0U)
  {
    return Mmc5983maReadResult::kNoNewData;
  }

  Mmc5983maFieldCounts counts;
  if (!read_field_counts(counts))
  {
    return Mmc5983maReadResult::kTransferFailed;
  }

  // Acknowledge after the burst, not before: a failed burst then leaves the
  // flag standing and the next poll retries the same measurement. The cost
  // is the converse race -- a measurement completing between the burst and
  // this write is acknowledged unread -- which continuous mode corrects one
  // period later.
  if (!write_register(Mmc5983maRegister::kStatus, kMmc5983maStatusMeasurementDone))
  {
    return Mmc5983maReadResult::kTransferFailed;
  }

  if (convert(counts, out) != MagConversionError::kNone)
  {
    return Mmc5983maReadResult::kTransferFailed;
  }
  return Mmc5983maReadResult::kSample;
}

Mmc5983maError Mmc5983maDriver::read_temperature(float& temperature_c)
{
  const Mmc5983maError error = trigger_and_wait(kMmc5983maControl0TakeTemperature, kMmc5983maStatusTemperatureDone);
  if (error != Mmc5983maError::kNone)
  {
    return error;
  }

  std::uint8_t tout = 0;
  if (!read_register(Mmc5983maRegister::kTout, tout))
  {
    return Mmc5983maError::kTransferFailed;
  }
  temperature_c = mmc5983ma_temperature_c(tout);
  return Mmc5983maError::kNone;
}

Mmc5983maError Mmc5983maDriver::apply_continuous_mode()
{
  // Periodic set only goes on when the part will honour it: the datasheet
  // conditions En_prd_set on both Auto_SR_en and Cmm_en.
  const bool continuous = config_.continuous_rate != Mmc5983maContinuousRate::kOff;
  const bool periodic_set_enable = continuous && config_.automatic_set_reset;
  if (!write_register(
          Mmc5983maRegister::kInternalControl2,
          mmc5983ma_control2(config_.continuous_rate, continuous, config_.periodic_set, periodic_set_enable)))
  {
    return Mmc5983maError::kTransferFailed;
  }
  return Mmc5983maError::kNone;
}

Mmc5983maError Mmc5983maDriver::trigger_and_wait(std::uint8_t trigger_bit, std::uint8_t done_bit)
{
  // The shadow rides along: the trigger bits are self-clearing, the
  // interrupt and automatic-set/reset bits in the same register are not.
  if (!write_register(Mmc5983maRegister::kInternalControl0, static_cast<std::uint8_t>(control0_shadow_ | trigger_bit)))
  {
    return Mmc5983maError::kTransferFailed;
  }

  for (std::uint32_t attempt = 0; attempt < kMeasurementPollAttempts; ++attempt)
  {
    std::uint8_t status = 0;
    if (!read_register(Mmc5983maRegister::kStatus, status))
    {
      return Mmc5983maError::kTransferFailed;
    }
    if ((status & done_bit) != 0U)
    {
      if (!write_register(Mmc5983maRegister::kStatus, done_bit))
      {
        return Mmc5983maError::kTransferFailed;
      }
      return Mmc5983maError::kNone;
    }
    bus_.delay_ms(kMeasurementPollIntervalMs);
  }
  return Mmc5983maError::kMeasurementTimeout;
}

bool Mmc5983maDriver::read_field_counts(Mmc5983maFieldCounts& out)
{
  // One burst from Xout0 through XYZout2: the low two bits of all three axes
  // live in that last byte, so reading fewer registers would truncate every
  // axis to 16 bits -- which is exactly the part's documented 16-bit mode,
  // and not what this driver claims to be doing.
  std::array<std::uint8_t, kMmc5983maDataBurstLength> burst{};
  if (!bus_.read_registers(reg_address(Mmc5983maRegister::kXout0), burst.data(), burst.size()))
  {
    return false;
  }
  out = decode_mmc5983ma_field(burst);
  return true;
}

MagConversionError Mmc5983maDriver::convert(const Mmc5983maFieldCounts& counts, MagSample& out) const
{
  const auto axis = [](std::uint32_t raw, std::int32_t offset) {
    return static_cast<std::int32_t>(raw) - static_cast<std::int32_t>(kMmc5983maNullFieldOutput) - offset;
  };

  MagRawSample raw;
  raw.mag_x = axis(counts.x, bridge_offset_.x);
  raw.mag_y = axis(counts.y, bridge_offset_.y);
  raw.mag_z = axis(counts.z, bridge_offset_.z);
  // The part has no clock of its own -- no SENSORTIME counterpart -- so the
  // stamp can only come from the controller, which is what MagRawSample
  // documents its timestamp to be.
  raw.timestamp_us = bus_.now_us();

  return convert_raw_to_si(raw, kMmc5983maScale, out);
}

bool Mmc5983maDriver::read_register(Mmc5983maRegister reg, std::uint8_t& out)
{
  return bus_.read_registers(reg_address(reg), &out, 1);
}

bool Mmc5983maDriver::write_register(Mmc5983maRegister reg, std::uint8_t value)
{
  return bus_.write_register(reg_address(reg), value);
}

}  // namespace hemerion::sensors::mag::mmc5983ma
