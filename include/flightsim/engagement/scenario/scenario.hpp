#pragma once

#include <cstdint>

#include "flightsim/core/types.hpp"
#include "flightsim/engagement/missile/missile_object.hpp"
#include "flightsim/engagement/target/target.hpp"
#include "flightsim/vision/seeker_source.hpp"
#include "flightsim/vision/seeker_track.hpp"
#include "flightsim/vision/scene_entity.hpp"

namespace flightsim {
namespace engagement {

struct ScenarioConfig {
    float dt_sec{0.01F};
    MissileAttributes missile{default_missile_attributes()};
    TargetConfig target{};
    core::Vec3 missile_launch_position_ned_m{0.0F, 0.0F, 0.0F};
    core::Vec3 depot_position_ned_m{8000.0F, 2000.0F, 0.0F};
    core::Vec3 target_start_position_ned_m{8000.0F, 2000.0F, -500.0F};
    float target_initial_heading_rad{0.0F};
    bool target_outbound{false};
    bool target_inbound{true};
    float missile_launch_speed_mps{80.0F};
    bool use_vision_seeker{false};
    // When true, initialize() picks new missile/target/depot poses inside the play square.
    bool randomize_initial_positions{false};
    float play_area_half_m{500.0F};  // 1 km × 1 km centered on origin
    std::uint32_t spawn_rng_seed{0U};  // 0 = seed from clock each initialize()
};

struct ScenarioState {
    MissileObject missile{};
    TargetState target{};
    TargetRuntime target_runtime{};
    std::uint64_t step_count{0U};
    float miss_distance_m{0.0F};
    bool intercept{false};
    bool complete{false};
    vision::SceneState scene{};
    vision::SeekerTrack last_seeker_track{};
};

class EngagementScenario {
public:
    explicit EngagementScenario(ScenarioConfig config) noexcept;

    void initialize() noexcept;
    void reinitialize() noexcept;
    void step() noexcept;
    void run(std::uint64_t max_steps) noexcept;

    void set_seeker_source(vision::ISeekerTrackSource* source) noexcept { seeker_source_ = source; }

    const ScenarioState& state() const noexcept { return state_; }
    const ScenarioConfig& config() const noexcept { return config_; }

private:
    ScenarioConfig config_;
    ScenarioState state_{};
    vision::ISeekerTrackSource* seeker_source_{nullptr};
};

}  // namespace engagement
}  // namespace flightsim
