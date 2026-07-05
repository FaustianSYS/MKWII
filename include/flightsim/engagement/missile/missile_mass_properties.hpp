#pragma once

#include "flightsim/core/types.hpp"
#include "flightsim/engagement/missile/missile_eom.hpp"

namespace flightsim {
namespace engagement {

struct MissileObject;

struct MissilePropulsionProperties {
    float dry_mass_kg{90.0F};
    float propellant_mass_kg{62.0F};
    float burn_time_sec{8.0F};
    core::Vec3 cg_dry_body_m{};
    core::Vec3 cg_full_body_m{};
    MissileInertia inertia_dry{};
    MissileInertia inertia_full{};
};

struct MissileMassProperties {
    float mass_kg{152.0F};
    core::Vec3 cg_body_m{};
    MissileInertia inertia{};
};

void initialize_mass_properties(MissileObject& missile) noexcept;

void update_mass_properties_from_burn(MissileObject& missile, float flight_time_sec) noexcept;

}  // namespace engagement
}  // namespace flightsim
