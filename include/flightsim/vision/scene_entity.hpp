#pragma once

#include <cstdint>

#include "flightsim/core/types.hpp"
#include "flightsim/engagement/missile/missile_object.hpp"
#include "flightsim/engagement/target/target.hpp"

namespace flightsim {
namespace vision {

enum class SceneEntityType : std::uint8_t {
    Missile = 0U,
    Shahed = 1U,
    Depot = 2U,
};

struct SceneEntity {
    const char* id{"entity"};
    const char* model{"generic"};
    SceneEntityType type{SceneEntityType::Shahed};
    core::Vec3 position_ned_m{};
    core::Vec3 velocity_ned_mps{};
    core::Quaternion attitude{core::Quaternion::identity()};
};

struct SceneState {
    std::uint64_t sim_step{0U};
    SceneEntity missile{};
    SceneEntity shahed{};
    SceneEntity depot{};
};

SceneState build_scene_state(const engagement::MissileObject& missile, const engagement::TargetState& shahed,
                             const core::Vec3& depot_position_ned_m, std::uint64_t sim_step) noexcept;

}  // namespace vision
}  // namespace flightsim
