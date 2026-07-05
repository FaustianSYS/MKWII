#include "flightsim/fdm/init.hpp"

#include "flightsim/fdm/environment.hpp"

namespace flightsim {
namespace fdm {

void initialize_state(AircraftState& state, const AircraftConfig& config) noexcept {
    (void)config;
    state.position_ned_m = core::Vec3{0.0F, 0.0F, -1000.0F};
    state.velocity_ned_mps = core::Vec3{80.0F, 0.0F, 0.0F};
    state.attitude = core::Quaternion::identity();
    state.angular_rate_body_rps = core::Vec3{};
    state.body_wrench = ForceMoment{};
    state.controls = ControlInputs{};
    state.mass_props.mass_kg = 1000.0F;
    state.mass_props.cg_body_m = core::Vec3{};
    state.mass_props.inertia_kgm2 = core::Mat3::identity();
    state.mass_props.inertia_kgm2.row0[0] = 1200.0F;
    state.mass_props.inertia_kgm2.row1[1] = 2000.0F;
    state.mass_props.inertia_kgm2.row2[2] = 1800.0F;
    update_environment(state);
}

}  // namespace fdm
}  // namespace flightsim
