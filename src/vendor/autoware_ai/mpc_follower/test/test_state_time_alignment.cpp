#include "mpc_follower/state_time_alignment.h"
#include <cassert>
#include <iostream>
#include <limits>

int main() {
  using namespace state_time;
  auto p=predictPose({1.,2.,0.},10.,0.,.1);
  assert(std::abs(p.x-2.)<1e-12 && p.y==2. && p.yaw==0.);
  p=predictPose({0.,0.,0.},10.,1.,.1);
  assert(std::abs(p.x-10*std::sin(.1))<1e-12);
  assert(std::abs(p.y-10*(1-std::cos(.1)))<1e-12);
  assert(std::abs(p.yaw-.1)<1e-12);
  auto reversed=predictPose(p,-10.,-1.,.1);
  assert(std::hypot(reversed.x,reversed.y)<1e-12);
  std::deque<Command> h{{0.,0.},{.1,.3},{.2,-.3}};
  assert(commandAt(h,.15)==.3);
  assert(predictSteering(0.,.1,.2,h,.1,.05,.77)==0.);
  const double step=predictSteering(0.,.2,.25,h,.1,.05,.77);
  assert(std::abs(step-.3*.77*(1-std::exp(-1.)))<1e-12);
  const double split=predictSteering(step,.25,.35,h,.1,.05,.77);
  assert(std::abs(split-predictSteering(0.,.2,.35,h,.1,.05,.77))<1e-12);
  for (double age:{-.001,.151,std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected=false;try { predictPose({0.,0.,0.},1.,0.,age); }
    catch(const std::invalid_argument&) { rejected=true; } assert(rejected);
  }
  bool rejected=false;
  try { predictSteering(0.,.05,.1,h,.1,.05,.77); }
  catch(const std::invalid_argument&) { rejected=true; } assert(rejected);
  std::cout<<"PASS: straight/arc/reverse; delayed step/reversal/split-time consistency; age and missing-history guards\n";
}
