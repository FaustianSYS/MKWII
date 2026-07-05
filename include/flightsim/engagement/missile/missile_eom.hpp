#pragma once

#include "flightsim/core/types.hpp"

namespace flightsim {
namespace engagement {

struct MissileWrench {
    core::Vec3 force_body_n{};
    core::Vec3 moment_body_nm{};
};

struct MissileInertia {
    float ixx_kgm2{5.0F};
    float iyy_kgm2{170.0F};
    float izz_kgm2{170.0F};
};

// Semi-implicit Euler integration of 6-DOF rigid-body equations in NED.
void integrate_missile_eom(core::Vec3& position_ned_m, core::Vec3& velocity_ned_mps, core::Quaternion& attitude,
                           core::Vec3& angular_rate_body_rps, const MissileWrench& wrench, float mass_kg,
                           const MissileInertia& inertia, float dt) noexcept;

core::Quaternion quaternion_from_body_x_ned(const core::Vec3& body_x_ned) noexcept;

core::Vec3 body_x_ned(const core::Quaternion& attitude) noexcept;

core::Vec3 ned_to_body(const core::Quaternion& attitude, const core::Vec3& vector_ned) noexcept;

}  // namespace engagement
}  // namespace flightsim
