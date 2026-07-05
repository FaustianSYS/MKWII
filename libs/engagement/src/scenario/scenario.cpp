#include "flightsim/engagement/scenario/scenario.hpp"

#include "flightsim/engagement/missile/missile.hpp"
#include "flightsim/engagement/target/target.hpp"
#include "flightsim/vision/scene_entity.hpp"

namespace flightsim {
namespace engagement {

namespace {

core::Vec3 normalize_vec(const core::Vec3& v) noexcept {
    const float mag = v.magnitude();
    if (mag < 1.0e-6F) {
        return core::Vec3{};
    }
    return v * (1.0F / mag);
}

}  // namespace

EngagementScenario::EngagementScenario(ScenarioConfig config) noexcept : config_(config) {}

void EngagementScenario::initialize() noexcept {
    state_ = ScenarioState{};

    config_.target.inbound = config_.target_inbound;
    config_.target.outbound = config_.target_outbound;
    if (config_.target.inbound) {
        config_.target.inbound_reference_ned_m = config_.depot_position_ned_m;
    } else if (config_.target.outbound) {
        config_.target.outbound_reference_ned_m = config_.missile_launch_position_ned_m;
    }

    const float initial_heading = config_.target.inbound
                                      ? inbound_heading_rad(config_.depot_position_ned_m, config_.target_start_position_ned_m)
                                      : config_.target_initial_heading_rad;
    initialize_target(state_.target, config_.target, state_.target_runtime, config_.target_start_position_ned_m,
                      initial_heading);

    const core::Vec3 to_shahed = config_.target_start_position_ned_m - config_.missile_launch_position_ned_m;
    const core::Vec3 launch_dir = normalize_vec(to_shahed);
    const core::Vec3 launch_velocity = launch_dir * config_.missile_launch_speed_mps;
    initialize_missile(state_.missile, config_.missile, config_.missile_launch_position_ned_m, launch_velocity);

    state_.step_count = 0U;
    state_.miss_distance_m = range_to_target(state_.missile, state_.target.position_ned_m);
    state_.intercept = false;
    state_.complete = false;
    state_.scene =
        vision::build_scene_state(state_.missile, state_.target, config_.depot_position_ned_m, state_.step_count);
}

void EngagementScenario::step() noexcept {
    if (state_.complete) {
        return;
    }

    MissileThreat threat{};
    if (state_.missile.active) {
        threat.active = true;
        threat.position_ned_m = state_.missile.position_ned_m;
        threat.velocity_ned_mps = state_.missile.velocity_ned_mps;
    }
    step_target(state_.target, config_.target, state_.target_runtime, config_.dt_sec, &threat);

    const vision::SeekerTrack* guidance_track = nullptr;
    vision::SeekerTrack external_track{};
    if (config_.use_vision_seeker && seeker_source_ != nullptr && seeker_source_->poll(external_track) &&
        external_track.valid) {
        guidance_track = &external_track;
        state_.last_seeker_track = external_track;
    } else if (config_.use_vision_seeker) {
        state_.last_seeker_track = {};
    }

    step_missile(state_.missile, state_.target.position_ned_m, state_.target.velocity_ned_mps, config_.dt_sec,
                 guidance_track);

    state_.miss_distance_m = range_to_target(state_.missile, state_.target.position_ned_m);
    state_.intercept = state_.missile.hit;
    state_.complete = state_.intercept || !state_.missile.active;
    state_.scene =
        vision::build_scene_state(state_.missile, state_.target, config_.depot_position_ned_m, state_.step_count);
    ++state_.step_count;
}

void EngagementScenario::run(std::uint64_t max_steps) noexcept {
    for (std::uint64_t i = 0U; i < max_steps && !state_.complete; ++i) {
        step();
    }
}

}  // namespace engagement
}  // namespace flightsim
