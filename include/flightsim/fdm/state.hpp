#pragma once

#include "flightsim/core/constants.hpp"
#include "flightsim/core/types.hpp"

namespace flightsim {
namespace fdm {

struct ControlInputs {
    float elevator{0.0F};
    float aileron{0.0F};
    float rudder{0.0F};
    float throttle{0.0F};
};

struct MassProperties {
    float mass_kg{1000.0F};
    core::Vec3 cg_body_m{};
    core::Mat3 inertia_kgm2{core::Mat3::identity()};
};

struct ForceMoment {
    core::Vec3 force_body_n{};
    core::Vec3 moment_body_nm{};
};

struct Environment {
    float density_kgm3{core::kSeaLevelDensity};
    float wind_ned_mps{};
};

// @req LLR-FDM-001
struct AircraftState {
    core::Vec3 position_ned_m{};
    core::Vec3 velocity_ned_mps{};
    core::Quaternion attitude{};
    core::Vec3 angular_rate_body_rps{};
    ForceMoment body_wrench{};
    Environment environment{};
    MassProperties mass_props{};
    ControlInputs controls{};
};

}  // namespace fdm
}  // namespace flightsim
