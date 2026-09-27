// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file environment.hpp
/// @brief Shared option plumbing for the co-simulation hosts: the GPS error
/// model presets, the MIL-F-8785C turbulence rules, and the comma-separated
/// option parser.
///
/// The three hosts in `examples/` are deliberately readable end to end and
/// duplicate a great deal on purpose -- each example is meant to be followed
/// without chasing shared code. These three pieces are the exception, for one
/// reason each:
///
/// * the **GPS presets** are a claim about a receiver, quoted in
///   `doc/sensor_models.rst`, and three copies drifting apart would make the
///   documentation wrong about at least one of them;
/// * the **Dryden rules** are a formula out of a standard, which is exactly
///   the kind of thing that is silently wrong in a copy;
/// * the **parser** has a grammar worth testing once.
///
/// All three are pure: no FMI, no Ecos, no sockets. That is what lets
/// `tests/unit` cover them in CI, where no Aetherion install exists and no
/// co-simulation can run.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

/// @namespace hemerion::examples
/// @brief Host-side helpers shared by the co-simulation examples.
namespace hemerion::examples
{

// ---------------------------------------------------------------------------
// Option parsing
// ---------------------------------------------------------------------------

/// @brief Parses "a,b" or "a,b,c" into `out`, throwing if the shape is wrong.
///
/// std::sscanf would be shorter, but MSVC's CRT deprecates it and the
/// diagnostic it gives ("expected 3 fields") is worse than naming the option's
/// grammar. std::stod's own exception carries the offending text.
///
/// @param text    The option's value.
/// @param out     Destination; its size sets how many fields are required.
/// @param option  Option name, for the message.
/// @param grammar What the option accepts, for the message.
inline void parse_csv_doubles(const char* text, std::span<double> out, const char* option, const char* grammar)
{
  const auto bad = [option, grammar] { throw std::invalid_argument(std::string(option) + " takes " + grammar); };

  std::string_view rest(text);
  for (std::size_t i = 0; i < out.size(); ++i)
  {
    const std::size_t comma = rest.find(',');
    const std::string_view field = rest.substr(0, comma);
    const bool last = (i + 1 == out.size());
    // A missing separator before the last field, or a separator after it, is
    // the wrong arity rather than a bad number.
    if (field.empty() || (comma == std::string_view::npos && !last))
    {
      bad();
    }
    std::size_t consumed = 0;
    try
    {
      out[i] = std::stod(std::string(field), &consumed);
    }
    catch (const std::exception&)
    {
      bad();
    }
    if (consumed != field.size())  // trailing rubbish: "1.0abc"
    {
      bad();
    }
    rest = (comma == std::string_view::npos) ? std::string_view{} : rest.substr(comma + 1);
  }
  if (!rest.empty())
  {
    bad();
  }
}

// ---------------------------------------------------------------------------
// Turbulence
// ---------------------------------------------------------------------------

/// Feet to metres, the unit MIL-F-8785C is written in.
inline constexpr double kFeetToMetres = 0.3048;

/// Dryden intensities and scale lengths at one altitude, in SI.
struct DrydenAtAltitude
{
  double sigma_u_mps;  ///< Longitudinal gust intensity, 1-sigma [m/s].
  double sigma_v_mps;  ///< Lateral gust intensity, 1-sigma [m/s].
  double sigma_w_mps;  ///< Vertical gust intensity, 1-sigma [m/s].
  double L_u_m;        ///< Longitudinal scale length [m].
  double L_v_m;        ///< Lateral scale length [m].
  double L_w_m;        ///< Vertical scale length [m].
};

/// @brief MIL-F-8785C low-altitude turbulence, evaluated at one altitude.
///
/// The standard's low-altitude rules are formulae rather than a chart: the
/// vertical intensity is 0.1 W20, the horizontal ones scale it by
/// (h / 1000 ft)^0.4, the vertical scale length is the altitude itself and the
/// horizontal ones h / (0.177 + 0.000823 h)^1.2, all in feet.
///
/// **Above 2000 ft this is an extrapolation, not the standard.** There
/// MIL-F-8785C switches to a probability-of-exceedance chart, which is why
/// Aetherion exposes the six Dryden numbers rather than a preset (see its
/// TODO-wind-turbulence-atmosphere.md). Continuing the low-altitude form
/// upward gives a defensible, monotone, reproducible intensity for a
/// demonstration; it is not a certification atmosphere, and a run that needs
/// one should write turb.sigma_* and turb.L_* directly.
///
/// @param altitude_m Altitude above ground [m]; floored at 10 ft, below which
///                   the horizontal scale length loses meaning.
/// @param w20_mps    Wind speed at 20 ft, the standard's intensity parameter
///                   [m/s]: about 5 light, 15 moderate, 30 severe.
[[nodiscard]] inline DrydenAtAltitude dryden_low_altitude(double altitude_m, double w20_mps)
{
  const double h_ft = std::max(10.0, altitude_m / kFeetToMetres);
  const double sigma_w = 0.1 * w20_mps;
  const double sigma_uv = sigma_w / std::pow(h_ft / 1000.0, 0.4);

  DrydenAtAltitude dryden;
  dryden.sigma_u_mps = sigma_uv;
  dryden.sigma_v_mps = sigma_uv;
  dryden.sigma_w_mps = sigma_w;
  dryden.L_u_m = (h_ft / std::pow(0.177 + 0.000823 * h_ft, 1.2)) * kFeetToMetres;
  dryden.L_v_m = dryden.L_u_m;
  dryden.L_w_m = h_ft * kFeetToMetres;
  return dryden;
}

// ---------------------------------------------------------------------------
// GPS error model
// ---------------------------------------------------------------------------

/// The GPS FMU's error-model parameters as one record, so the Ecos parameter
/// set and the run's .config sidecar cannot disagree about what the receiver
/// was -- and plot_results.py can draw the autocorrelation a run *should*
/// show from the sidecar alone.
struct GpsErrorModel
{
  const char* name;
  double horizontal_pos_noise_m;
  double vertical_pos_noise_m;
  double speed_noise_mps;
  double course_noise_deg;
  double horizontal_pos_correlated_m;
  double vertical_pos_correlated_m;
  double position_correlation_time_s;
  double speed_correlated_mps;
  double course_correlated_deg;
  double velocity_correlation_time_s;
  double accuracy_scale;
};

/// The FMU's own defaults: white per epoch, an honest hAcc/vAcc. Written to
/// the parameter set explicitly even though the FMU would default to them, so
/// the sidecar is always a complete record.
inline constexpr GpsErrorModel kWhiteReceiver{ "white", 1.5, 3.0, 0.1, 1.0, 0.0, 0.0, 100.0, 0.0, 0.0, 10.0, 1.0 };

/// The realistic receiver. Each position channel's total 1-sigma stays within
/// 2 % of the default's -- nothing on a published figure moves by magnitude,
/// and position is what they all plot -- while most of it moves into a slow
/// Gauss-Markov term (tau 100 s on position, 10 s on velocity), and hAcc/vAcc
/// report 70 % of the truth, as a receiver's own estimate tends to. The
/// velocity channels are looser: speed totals 0.112 m/s against a 0.1 m/s
/// default and course 1.044 deg against 1.0, because rounding those to hit a
/// total exactly would have meant quoting receiver characteristics chosen to
/// flatter an arithmetic claim rather than to be plausible. The values are tabulated in doc/sensor_models.rst.
inline constexpr GpsErrorModel kCorrelatedReceiver{ "correlated", 0.3,   0.6, 0.05, 0.3,  1.5,
                                                    3.0,          100.0, 0.1, 1.0,  10.0, 0.7 };

/// @brief Total 1-sigma of a channel whose error is white plus correlated.
[[nodiscard]] inline double total_sigma(double white, double correlated) { return std::hypot(white, correlated); }

}  // namespace hemerion::examples
