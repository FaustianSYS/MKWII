#include "flightsim/engagement/missile/missile_mass_properties.hpp"

#include "flightsim/engagement/missile/missile_object.hpp"

namespace flightsim {
namespace engagement {

namespace {

float lerp(float a, float b, float t) noexcept {
    return a + ((b - a) * t);
}

}  // namespace

void initialize_mass_properties(MissileObject& missile) noexcept {
    const MissilePropulsionProperties& propulsion = missile.attributes.propulsion;
    MissileMassProperties& mass_properties = missile.mass_properties;

    mass_properties.mass_kg = propulsion.dry_mass_kg + propulsion.propellant_mass_kg;
    mass_properties.cg_body_m = propulsion.cg_full_body_m;
    mass_properties.inertia = propulsion.inertia_full;
}

void update_mass_properties_from_burn(MissileObject& missile, float flight_time_sec) noexcept {
    const MissilePropulsionProperties& propulsion = missile.attributes.propulsion;
    MissileMassProperties& mass_properties = missile.mass_properties;

    const float burn_time = propulsion.burn_time_sec > 0.0F ? propulsion.burn_time_sec : 1.0e-6F;
    const float burn_fraction = core::clamp(flight_time_sec / burn_time, 0.0F, 1.0F);
    const float propellant_remaining = propulsion.propellant_mass_kg * (1.0F - burn_fraction);

    mass_properties.mass_kg = propulsion.dry_mass_kg + propellant_remaining;
    mass_properties.cg_body_m = propulsion.cg_full_body_m +
                              ((propulsion.cg_dry_body_m - propulsion.cg_full_body_m) * burn_fraction);
    mass_properties.inertia.ixx_kgm2 =
        lerp(propulsion.inertia_full.ixx_kgm2, propulsion.inertia_dry.ixx_kgm2, burn_fraction);
    mass_properties.inertia.iyy_kgm2 =
        lerp(propulsion.inertia_full.iyy_kgm2, propulsion.inertia_dry.iyy_kgm2, burn_fraction);
    mass_properties.inertia.izz_kgm2 =
        lerp(propulsion.inertia_full.izz_kgm2, propulsion.inertia_dry.izz_kgm2, burn_fraction);
}

}  // namespace engagement
}  // namespace flightsim
