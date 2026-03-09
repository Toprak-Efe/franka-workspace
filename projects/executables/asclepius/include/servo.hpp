#pragma once

#include <array>
#include <realtime_loop/control/firstohold.hpp>
#include <realtime_loop/control/triplebuffer.hpp>

namespace Asclepius {

struct Velocities {
  std::array<double, 6> data;
  Control::Timestamp timestamp;
}; // struct Velocities 
using VelocityBuffer = Control::TripleBuffer<Velocities>;

struct Forces {
  std::array<double, 6> data;
  Control::Timestamp timestamp;
}; // struct Forces 
using ForceBuffer = Control::TripleBuffer<Forces>;

struct ForceVelocityBuffer {
    VelocityBuffer velocity;
    ForceBuffer force;
}; // ForceVelocityBuffer

}; // namespace Asclepius
