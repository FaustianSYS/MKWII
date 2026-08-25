#include "flightsim/engagement/scenario/scenario.hpp"

#include <chrono>
#include <cmath>

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

void randomize_initial_spawns(ScenarioConfig& config) noexcept {
    std::uint32_t seed = config.spawn_rng_seed;
    if (seed == 0U) {
        const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
        seed = static_cast<std::uint32_t>(ticks) ^ static_cast<std::uint32_t>(ticks >> 32);
        if (seed == 0U) {
            seed = 1U;
        }
    }

    auto next_u = [&seed]() noexcept {
        seed = seed * 1664525U + 1013904223U;
        return seed;
    };
    auto next_f = [&next_u](float lo, float hi) noexcept {
        const float u = static_cast<float>(next_u() & 0x00FFFFFFU) / static_cast<float>(0x00FFFFFFU);
        return lo + (hi - lo) * u;
    };

    const float half = config.play_area_half_m > 1.0F ? config.play_area_half_m : 500.0F;
    const float margin = 50.0F;
    const float lo = -half + margin;
    const float hi = half - margin;

    config.missile_launch_position_ned_m = core::Vec3{next_f(lo, hi), next_f(lo, hi), 0.0F};

    config.depot_position_ned_m = core::Vec3{next_f(lo, hi), next_f(lo, hi), 0.0F};
    for (int i = 0; i < 24; ++i) {
        if ((config.depot_position_ned_m - config.missile_launch_position_ned_m).magnitude() >= 200.0F) {
            break;
        }
        config.depot_position_ned_m = core::Vec3{next_f(lo, hi), next_f(lo, hi), 0.0F};
    }

    config.target_start_position_ned_m =
        core::Vec3{next_f(lo, hi), next_f(lo, hi), next_f(-110.0F, -55.0F)};
    for (int i = 0; i < 32; ++i) {
        const float range_m =
            (config.target_start_position_ned_m - config.missile_launch_position_ned_m).magnitude();
        if (range_m >= 150.0F && range_m <= 480.0F) {
            break;
        }
        config.target_start_position_ned_m =
            core::Vec3{next_f(lo, hi), next_f(lo, hi), next_f(-110.0F, -55.0F)};
    }

    config.target.rng_seed = next_u();
    config.spawn_rng_seed = seed;
}

}  // namespace

EngagementScenario::EngagementScenario(ScenarioConfig config) noexcept : config_(config) {}

void EngagementScenario::initialize() noexcept {
    state_ = ScenarioState{};

    if (config_.randomize_initial_positions) {
        randomize_initial_spawns(config_);
    }

    config_.target.inbound = config_.target_inbound;
    config_.target.outbound = config_.target_outbound;
    if (config_.target.constrain_play_area) {
        config_.target.play_area_half_m = config_.play_area_half_m;
    }
    if (config_.target.inbound) {
        config_.target.inbound_reference_ned_m = config_.depot_position_ned_m;
    } else if (config_.target.outbound) {
        config_.target.outbound_reference_ned_m = config_.missile_launch_position_ned_m;
    }

    // Free spline track: randomize initial heading when not bound to depot/missile.
    float initial_heading = config_.target_initial_heading_rad;
    if (config_.target.inbound) {
        initial_heading = inbound_heading_rad(config_.depot_position_ned_m, config_.target_start_position_ned_m);
    } else if (config_.target.outbound) {
        initial_heading = config_.target_initial_heading_rad;
    } else {
        // DeterministicRng not available here yet — use spawn seed LCG nibble via re-seeded target rng later.
        // Temporary: derive from spawn seed bits so each run starts on a different spline heading.
        const std::uint32_t bits = config_.spawn_rng_seed ^ config_.target.rng_seed;
        initial_heading = (static_cast<float>(bits & 0xFFFFU) / 65535.0F) * 6.2831853F - 3.14159265F;
    }
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

void EngagementScenario::reinitialize() noexcept {
    if (config_.randomize_initial_positions) {
        config_.spawn_rng_seed = 0U;  // fresh clock seed each rerun
    }
    initialize();
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
