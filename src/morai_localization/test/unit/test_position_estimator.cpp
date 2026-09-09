#include "morai_localization/estimation/position_estimator.hpp"
#include <cassert>
#include <iostream>
#include <limits>

int main() {
  using morai_localization::PositionEstimator;
  PositionEstimator estimator;
  double raw_squared = 0.0, estimated_squared = 0.0;
  // Known truth, quantized GPS, same 30 Hz motion input; no map/offset input.
  for (int i = 0; i < 1800; ++i) {
    const double t = i / 30.0;
    const double angle = 0.5 * t;
    const double x = 20.0 * std::sin(angle);
    const double y = 20.0 * (1.0 - std::cos(angle));
    const double gps_x = std::round(x / .148) * .148;
    const double gps_y = std::round(y / .185) * .185;
    const auto p = estimator.update(gps_x, gps_y, t, 10., angle, true, .3);
    if (i > 30) {
      raw_squared += std::pow(gps_x-x,2) + std::pow(gps_y-y,2);
      estimated_squared += std::pow(p.x-x,2) + std::pow(p.y-y,2);
    }
  }
  assert(estimated_squared < .4 * raw_squared);
  auto p = estimator.update(1000., 2000., 61., 10., 0., true, .3);
  assert(p.x == 1000. && p.y == 2000. && !estimator.corrected());
  p = estimator.update(1000.1, 2000., 61.03, 10., 0., false, .3);
  assert(p.x == 1000.1 && !estimator.corrected());
  p = estimator.update(1000.2, 2000., 61.06, 10., 0., true, .3);
  assert(p.x == 1000.2 && !estimator.corrected());
  p = estimator.update(1000.3, 2000., 61.06, 10., 0., true, .3);
  assert(p.x == 1000.3 && !estimator.corrected());
  p = estimator.update(1000.4, 2000., 61.09, 10., 0., true, 0.);
  assert(p.x == 1000.4 && !estimator.corrected());
  p = estimator.update(1000.5, 2000., 61.12, 0., 0., true, .3);
  assert(p.x == 1000.5 && !estimator.corrected());
  p = estimator.update(-1000., -2000., 61.15, 10., 0., true, .3);
  assert(p.x == -1000. && p.y == -2000. && !estimator.corrected());
  p = estimator.update(-1001., -2000., 60., 10., 0., true, .3);
  assert(p.x == -1001. && !estimator.corrected());
  bool threw = false;
  try { estimator.update(std::numeric_limits<double>::quiet_NaN(),0,1,0,0,true,.3); }
  catch (const std::invalid_argument&) { threw = true; }
  assert(threw);
  PositionEstimator constant_speed;
  for (int i=0; i<90; ++i) {
    const double t=i/30.;
    p=constant_speed.update(10.*t,0,t,10.,0.,true,.3);
    assert(std::abs(p.x-10.*t)<1e-10 && p.y==0.);
  }
  std::cout << "PASS: quantized circle RMSE ratio=" << std::sqrt(estimated_squared/raw_squared)
            << "; no constant-speed lag; stale/gap/teleport/disabled/stop/invalid guards\n";
}
