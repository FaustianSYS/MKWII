#pragma once

#include "flightsim/fdm/state.hpp"
#include "flightsim/sim/scheduler.hpp"

namespace flightsim {
namespace sim {

// Non-safety I/O boundary: adapters (ROS2, UDP HITL, etc.) implement this
// interface without modifying the safety-critical scheduler.
class IoAdapter {
public:
    virtual ~IoAdapter() = default;

    virtual void on_initialize(const SimOutputs& outputs) = 0;
    virtual fdm::ControlInputs read_controls() = 0;
    virtual void write_outputs(const SimOutputs& outputs) = 0;
};

}  // namespace sim
}  // namespace flightsim
