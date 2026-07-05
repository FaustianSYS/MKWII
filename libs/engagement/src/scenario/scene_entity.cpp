#include "flightsim/vision/scene_entity.hpp"

#include "flightsim/engagement/missile/missile_eom.hpp"

namespace flightsim {
namespace vision {

namespace {

core::Vec3 normalize_or_zero(const core::Vec3& v) noexcept {
    const float mag = v.magnitude();
    if (mag < 1.0e-6F) {
        return core::Vec3{};
    }
    return v * (1.0F / mag);
}

}  // namespace

SceneState build_scene_state(const engagement::MissileObject& missile, const engagement::TargetState& shahed,
                             const core::Vec3& depot_position_ned_m, const std::uint64_t sim_step) noexcept {
    SceneState scene{};
    scene.sim_step = sim_step;

    scene.missile.id = "missile";
    scene.missile.model = "generic_missile";
    scene.missile.type = SceneEntityType::Missile;
    scene.missile.position_ned_m = missile.position_ned_m;
    scene.missile.velocity_ned_mps = missile.velocity_ned_mps;
    scene.missile.attitude = missile.attitude;

    scene.shahed.id = "shahed";
    scene.shahed.model = "shahed_136";
    scene.shahed.type = SceneEntityType::Shahed;
    scene.shahed.position_ned_m = shahed.position_ned_m;
    scene.shahed.velocity_ned_mps = shahed.velocity_ned_mps;
    const core::Vec3 shahed_forward = normalize_or_zero(shahed.velocity_ned_mps);
    if (shahed_forward.magnitude() > 1.0e-6F) {
        scene.shahed.attitude = engagement::quaternion_from_body_x_ned(shahed_forward);
    }

    scene.depot.id = "depot";
    scene.depot.model = "depot";
    scene.depot.type = SceneEntityType::Depot;
    scene.depot.position_ned_m = depot_position_ned_m;
    scene.depot.velocity_ned_mps = core::Vec3{};
    scene.depot.attitude = core::Quaternion::identity();

    return scene;
}

}  // namespace vision
}  // namespace flightsim
