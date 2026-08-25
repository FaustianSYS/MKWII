#include "flightsim/engagement/target/target.hpp"

#include <cmath>

namespace flightsim {
namespace engagement {

namespace {

constexpr float kPi = 3.14159265F;
constexpr float kTwoPi = kPi * 2.0F;

float horizontal_magnitude(const core::Vec3& v) noexcept {
    return std::sqrt((v.x * v.x) + (v.y * v.y));
}

float max_yaw_rate_for_config(const TargetConfig& config) noexcept {
    float yaw = config.max_yaw_rate_rps;
    if (config.min_turn_radius_m > 1.0F && config.speed_mps > 0.1F) {
        const float from_radius = config.speed_mps / config.min_turn_radius_m;
        yaw = (yaw > 0.0F) ? std::min(yaw, from_radius) : from_radius;
    }
    return std::max(yaw, 0.01F);
}

float wrap_angle_rad(float angle_rad) noexcept {
    while (angle_rad > kPi) {
        angle_rad -= kTwoPi;
    }
    while (angle_rad < -kPi) {
        angle_rad += kTwoPi;
    }
    return angle_rad;
}

float shortest_heading_delta(float from_rad, float to_rad) noexcept {
    return wrap_angle_rad(to_rad - from_rad);
}

float eval_cubic_hermite(float p0, float p1, float v0, float v1, float u, float duration) noexcept {
    if (duration <= 1.0e-6F) {
        return p1;
    }

    const float s = core::clamp(u / duration, 0.0F, 1.0F);
    const float s2 = s * s;
    const float s3 = s2 * s;
    const float h00 = (2.0F * s3) - (3.0F * s2) + 1.0F;
    const float h10 = s3 - (2.0F * s2) + s;
    const float h01 = (-2.0F * s3) + (3.0F * s2);
    const float h11 = s3 - s2;
    return (h00 * p0) + (h10 * duration * v0) + (h01 * p1) + (h11 * duration * v1);
}

float eval_cubic_hermite_derivative(float p0, float p1, float v0, float v1, float u, float duration) noexcept {
    if (duration <= 1.0e-6F) {
        return v1;
    }

    const float s = core::clamp(u / duration, 0.0F, 1.0F);
    const float s2 = s * s;
    const float dh00 = (6.0F * s2) - (6.0F * s);
    const float dh10 = (3.0F * s2) - (4.0F * s) + 1.0F;
    const float dh01 = (-6.0F * s2) + (6.0F * s);
    const float dh11 = (3.0F * s2) - (2.0F * s);
    return ((dh00 * p0) + (dh10 * duration * v0) + (dh01 * p1) + (dh11 * duration * v1)) / duration;
}

void apply_inbound_soft_correction(const TargetConfig& config, const TargetState& state, float& heading_rad,
                                     float& heading_rate_rps) noexcept {
    if (!config.inbound) {
        return;
    }

    const core::Vec3 toward{
        config.inbound_reference_ned_m.x - state.position_ned_m.x,
        config.inbound_reference_ned_m.y - state.position_ned_m.y,
        0.0F};
    const float horiz = horizontal_magnitude(toward);
    if (horiz < 1.0e-3F) {
        return;
    }

    const float inbound_heading = std::atan2(toward.y, toward.x);
    const core::Vec3 toward_dir{toward.x / horiz, toward.y / horiz, 0.0F};
    const core::Vec3 vel_horiz{
        std::cos(heading_rad) * config.speed_mps,
        std::sin(heading_rad) * config.speed_mps,
        0.0F};

    if (vel_horiz.dot(toward_dir) < 0.0F) {
        const float correction = shortest_heading_delta(heading_rad, inbound_heading);
        heading_rad = wrap_angle_rad(heading_rad + (correction * 0.12F));
        heading_rate_rps = core::clamp(heading_rate_rps + (correction * 0.05F), -max_yaw_rate_for_config(config),
                                       max_yaw_rate_for_config(config));
    }
}

void apply_missile_evasion(const TargetConfig& config, const TargetState& state, TargetRuntime& runtime,
                           const MissileThreat* missile_threat, float& heading_rad, float& heading_rate_rps,
                           float& climb_mps) noexcept {
    if (!config.evade_missile || missile_threat == nullptr || !missile_threat->active) {
        return;
    }

    const core::Vec3 rel{
        state.position_ned_m.x - missile_threat->position_ned_m.x,
        state.position_ned_m.y - missile_threat->position_ned_m.y,
        state.position_ned_m.z - missile_threat->position_ned_m.z};
    const float range = rel.magnitude();
    if (range > config.evasion_range_m || range < 8.0F) {
        return;
    }

    const core::Vec3 los = rel * (1.0F / range);
    const core::Vec3 rel_vel{
        state.velocity_ned_mps.x - missile_threat->velocity_ned_mps.x,
        state.velocity_ned_mps.y - missile_threat->velocity_ned_mps.y,
        state.velocity_ned_mps.z - missile_threat->velocity_ned_mps.z};
    const float closing =
        -((rel_vel.x * los.x) + (rel_vel.y * los.y) + (rel_vel.z * los.z));
    if (closing < 4.0F) {
        return;
    }

    float los_h_x = los.x;
    float los_h_y = los.y;
    const float horiz = horizontal_magnitude(core::Vec3{los_h_x, los_h_y, 0.0F});
    if (horiz < 1.0e-3F) {
        return;
    }
    los_h_x /= horiz;
    los_h_y /= horiz;

    const float weave =
        std::sin((runtime.evasion_phase_sec / config.evasion_weave_period_sec) * kTwoPi);
    const float sign = weave >= 0.0F ? 1.0F : -1.0F;
    const float beam_x = -sign * los_h_y;
    const float beam_y = sign * los_h_x;
    const float evasion_heading = std::atan2(beam_y, beam_x);

    const float range_factor = core::clamp(1.0F - (range / config.evasion_range_m), 0.0F, 1.0F);
    const float smooth_range = range_factor * range_factor;
    const float closing_factor = core::clamp(closing / 80.0F, 0.0F, 1.0F);
    const float threat = smooth_range * closing_factor;
    const float evasion_weight = threat * config.evasion_gain;

    const float evasion_delta = shortest_heading_delta(heading_rad, evasion_heading);
    heading_rad = wrap_angle_rad(heading_rad + (evasion_delta * evasion_weight));
    heading_rate_rps = core::clamp(heading_rate_rps + (sign * evasion_weight * max_yaw_rate_for_config(config) * 0.20F),
                                   -max_yaw_rate_for_config(config), max_yaw_rate_for_config(config));

    if (config.inbound) {
        const float mission_heading = inbound_heading_rad(config.inbound_reference_ned_m, state.position_ned_m);
        const float mission_delta = shortest_heading_delta(heading_rad, mission_heading);
        heading_rad = wrap_angle_rad(heading_rad + (mission_delta * (1.0F - evasion_weight) * 0.22F));
    }

    const float vertical_weave =
        std::sin((runtime.evasion_phase_sec / (config.evasion_weave_period_sec * 0.85F)) * kTwoPi + 0.7F);
    climb_mps = core::clamp(climb_mps + (vertical_weave * evasion_weight * config.max_climb_rate_mps * 0.28F),
                            -config.max_climb_rate_mps, config.max_climb_rate_mps);
}

void apply_outbound_soft_correction(const TargetConfig& config, const TargetState& state,
                                    float& heading_rad, float& heading_rate_rps) noexcept {
    if (!config.outbound) {
        return;
    }

    const core::Vec3 away = state.position_ned_m - config.outbound_reference_ned_m;
    const float horiz = horizontal_magnitude(away);
    if (horiz < 1.0e-3F) {
        return;
    }

    const float outbound_heading = std::atan2(away.y, away.x);
    const core::Vec3 away_dir{away.x / horiz, away.y / horiz, 0.0F};
    const core::Vec3 vel_horiz{
        std::cos(heading_rad) * config.speed_mps,
        std::sin(heading_rad) * config.speed_mps,
        0.0F};

    if (vel_horiz.dot(away_dir) < 0.0F) {
        const float correction = shortest_heading_delta(heading_rad, outbound_heading);
        heading_rad = wrap_angle_rad(heading_rad + (correction * 0.12F));
        heading_rate_rps = core::clamp(heading_rate_rps + (correction * 0.05F), -max_yaw_rate_for_config(config),
                                       max_yaw_rate_for_config(config));
    }
}

void apply_play_area_soft_correction(const TargetConfig& config, const TargetState& state, float& heading_rad,
                                     float& heading_rate_rps) noexcept {
    if (!config.constrain_play_area || config.play_area_half_m <= 1.0F) {
        return;
    }

    const float half = config.play_area_half_m;
    const float margin = core::clamp(config.play_area_margin_m, 10.0F, half * 0.8F);
    const float soft = half - margin;
    const float x = state.position_ned_m.x;
    const float y = state.position_ned_m.y;

    float push_n = 0.0F;
    float push_e = 0.0F;
    if (x > soft) {
        push_n -= (x - soft) / margin;
    } else if (x < -soft) {
        push_n += (-soft - x) / margin;
    }
    if (y > soft) {
        push_e -= (y - soft) / margin;
    } else if (y < -soft) {
        push_e += (-soft - y) / margin;
    }

    const float push_mag = std::sqrt((push_n * push_n) + (push_e * push_e));
    if (push_mag < 1.0e-4F) {
        return;
    }

    const float toward_heading = std::atan2(push_e, push_n);
    const float weight = core::clamp(push_mag, 0.0F, 1.0F);
    const float correction = shortest_heading_delta(heading_rad, toward_heading);
    heading_rad = wrap_angle_rad(heading_rad + (correction * weight * 0.18F));
    heading_rate_rps = core::clamp(heading_rate_rps + (correction * weight * 0.05F), -max_yaw_rate_for_config(config),
                                   max_yaw_rate_for_config(config));
}

void splice_initial_spline_segment(TargetRuntime& runtime, const TargetConfig& config,
                                   const core::Vec3& start_position_ned_m) noexcept {
    TargetSplineSegment next{};
    next.duration_sec =
        core::clamp(config.spline_segment_sec * config.initial_spline_duration_scale, 0.5F, 30.0F);

    float mission_heading = runtime.heading_rad;
    if (config.inbound) {
        mission_heading = inbound_heading_rad(config.inbound_reference_ned_m, start_position_ned_m);
    } else if (config.outbound) {
        mission_heading = outbound_heading_rad(config.outbound_reference_ned_m, start_position_ned_m);
    }

    const float yaw_lim = max_yaw_rate_for_config(config);
    const float max_delta = std::min(config.max_heading_change_rad, yaw_lim * next.duration_sec * 0.90F);
    const float curve_sign = runtime.rng.uniform(0.0F, 1.0F) >= 0.5F ? 1.0F : -1.0F;
    const float curve_mag =
        core::clamp(config.initial_spline_curve_rad, 0.05F, std::max(max_delta, 0.05F));
    const float heading_delta = -curve_sign * curve_mag;  // ease toward mission heading

    next.start_heading_rad = wrap_angle_rad(mission_heading + (curve_sign * curve_mag));
    next.end_heading_rad = wrap_angle_rad(next.start_heading_rad + heading_delta);

    if (config.smooth_path_only) {
        const float omega = heading_delta / std::max(next.duration_sec, 1.0e-3F);
        next.start_heading_rate_rps = core::clamp(omega, -yaw_lim, yaw_lim);
        next.end_heading_rate_rps = next.start_heading_rate_rps;
        next.start_climb_accel_mps2 = 0.0F;
        next.end_climb_accel_mps2 = 0.0F;
        next.end_climb_mps = runtime.climb_rate_mps;
    } else {
        next.start_heading_rate_rps = curve_sign * yaw_lim * 0.20F;
        next.end_heading_rate_rps = -curve_sign * yaw_lim * 0.08F;
        next.start_climb_accel_mps2 =
            runtime.rng.uniform(-config.max_climb_accel_mps2 * 0.25F, config.max_climb_accel_mps2 * 0.25F);
        next.end_climb_accel_mps2 = 0.0F;
        next.end_climb_mps =
            runtime.climb_rate_mps +
            (curve_sign * runtime.rng.uniform(0.08F, 0.28F) * config.max_climb_rate_mps);
    }
    next.start_climb_mps = runtime.climb_rate_mps;

    runtime.heading_rad = next.start_heading_rad;
    runtime.heading_rate_rps = next.start_heading_rate_rps;
    runtime.climb_rate_mps = next.start_climb_mps;
    runtime.climb_accel_mps2 = next.start_climb_accel_mps2;
    runtime.segment = next;
    runtime.segment_elapsed_sec = 0.0F;
}

void splice_spline_segment(TargetRuntime& runtime, const TargetConfig& config, const TargetState& state) noexcept {
    TargetSplineSegment next{};
    const float jitter = core::clamp(config.spline_segment_jitter, 0.0F, 0.9F);
    const float dur_lo = config.spline_segment_sec * (1.0F - jitter);
    const float dur_hi = config.spline_segment_sec * (1.0F + jitter);
    next.duration_sec = core::clamp(runtime.rng.uniform(dur_lo, dur_hi), 0.5F, 30.0F);
    next.start_heading_rad = runtime.heading_rad;
    next.start_climb_mps = runtime.climb_rate_mps;
    next.start_heading_rate_rps = runtime.heading_rate_rps;
    next.start_climb_accel_mps2 = runtime.climb_accel_mps2;

    const float yaw_lim = max_yaw_rate_for_config(config);
    // Arc length constraint: Δψ ≤ (V/R) * Δt  (= yaw_lim * duration)
    const float max_by_radius = yaw_lim * next.duration_sec * 0.95F;
    const float max_delta = std::min(core::clamp(config.max_heading_change_rad, 0.05F, kPi), max_by_radius);

    float heading_delta = runtime.rng.uniform(-max_delta, max_delta);
    const float mission_blend = core::clamp(config.spline_mission_blend, 0.0F, 1.0F);
    if (mission_blend > 0.0F && config.inbound) {
        const float inbound_heading = inbound_heading_rad(config.inbound_reference_ned_m, state.position_ned_m);
        const float inbound_pull = shortest_heading_delta(runtime.heading_rad, inbound_heading);
        heading_delta = (heading_delta * (1.0F - mission_blend)) + (inbound_pull * mission_blend);
    } else if (mission_blend > 0.0F && config.outbound) {
        const float outbound_heading = outbound_heading_rad(config.outbound_reference_ned_m, state.position_ned_m);
        const float outbound_pull = shortest_heading_delta(runtime.heading_rad, outbound_heading);
        heading_delta = (heading_delta * (1.0F - mission_blend)) + (outbound_pull * mission_blend);
    }

    // Soft wander bias toward play-area center so the track stays inside the box.
    if (config.constrain_play_area && config.play_area_half_m > 1.0F) {
        const float half = config.play_area_half_m;
        const float r = horizontal_magnitude(state.position_ned_m);
        if (r > half * 0.40F) {
            const float toward_center =
                std::atan2(-state.position_ned_m.y, -state.position_ned_m.x);
            const float pull = shortest_heading_delta(runtime.heading_rad, toward_center);
            const float weight = core::clamp((r / half) * 0.22F, 0.0F, 0.28F);
            heading_delta = (heading_delta * (1.0F - weight)) + (pull * weight);
        }
    }

    heading_delta = core::clamp(heading_delta, -max_delta, max_delta);
    next.end_heading_rad = wrap_angle_rad(runtime.heading_rad + heading_delta);

    if (config.smooth_path_only) {
        // Constant turn-rate arc (C1-smooth match of ω across the segment).
        const float omega = heading_delta / std::max(next.duration_sec, 1.0e-3F);
        next.start_heading_rate_rps = core::clamp(omega, -yaw_lim, yaw_lim);
        next.end_heading_rate_rps = next.start_heading_rate_rps;
        next.end_climb_mps =
            core::clamp(runtime.climb_rate_mps + runtime.rng.uniform(-0.35F, 0.35F) * config.max_climb_rate_mps,
                        -config.max_climb_rate_mps, config.max_climb_rate_mps);
        next.start_climb_accel_mps2 = 0.0F;
        next.end_climb_accel_mps2 = 0.0F;
    } else {
        next.end_climb_mps = runtime.rng.uniform(-config.max_climb_rate_mps, config.max_climb_rate_mps);
        next.end_heading_rate_rps = runtime.rng.uniform(-yaw_lim, yaw_lim);
        next.end_climb_accel_mps2 =
            runtime.rng.uniform(-config.max_climb_accel_mps2, config.max_climb_accel_mps2);
    }

    runtime.segment = next;
    runtime.segment_elapsed_sec = 0.0F;
}

void sample_spline_segment(const TargetSplineSegment& segment, float elapsed_sec, float& heading_rad,
                           float& heading_rate_rps, float& climb_mps, float& climb_accel_mps2) noexcept {
    const float u = core::clamp(elapsed_sec, 0.0F, segment.duration_sec);
    const float start_heading = segment.start_heading_rad;
    const float end_heading = wrap_angle_rad(start_heading + shortest_heading_delta(start_heading, segment.end_heading_rad));

    heading_rad = eval_cubic_hermite(start_heading, end_heading, segment.start_heading_rate_rps,
                                     segment.end_heading_rate_rps, u, segment.duration_sec);
    heading_rate_rps = eval_cubic_hermite_derivative(start_heading, end_heading, segment.start_heading_rate_rps,
                                                     segment.end_heading_rate_rps, u, segment.duration_sec);

    climb_mps = eval_cubic_hermite(segment.start_climb_mps, segment.end_climb_mps, segment.start_climb_accel_mps2,
                                   segment.end_climb_accel_mps2, u, segment.duration_sec);
    climb_accel_mps2 =
        eval_cubic_hermite_derivative(segment.start_climb_mps, segment.end_climb_mps, segment.start_climb_accel_mps2,
                                      segment.end_climb_accel_mps2, u, segment.duration_sec);
}

}  // namespace

float inbound_heading_rad(const core::Vec3& aim_ned_m, const core::Vec3& position_ned_m) noexcept {
    const core::Vec3 toward{
        aim_ned_m.x - position_ned_m.x,
        aim_ned_m.y - position_ned_m.y,
        0.0F};
    const float horiz = horizontal_magnitude(toward);
    if (horiz < 1.0e-6F) {
        return 0.0F;
    }
    return std::atan2(toward.y, toward.x);
}

float outbound_heading_rad(const core::Vec3& reference_ned_m, const core::Vec3& position_ned_m) noexcept {
    const core::Vec3 away{
        position_ned_m.x - reference_ned_m.x,
        position_ned_m.y - reference_ned_m.y,
        0.0F};
    const float horiz = horizontal_magnitude(away);
    if (horiz < 1.0e-6F) {
        return 0.0F;
    }
    return std::atan2(away.y, away.x);
}

void initialize_target(TargetState& state, TargetConfig& config, TargetRuntime& runtime,
                       const core::Vec3& start_position_ned_m, float initial_heading_rad) noexcept {
    state.position_ned_m = start_position_ned_m;
    runtime.rng = DeterministicRng(config.rng_seed);
    runtime.heading_rad = initial_heading_rad;
    runtime.heading_rate_rps = 0.0F;
    runtime.climb_rate_mps = 0.0F;
    runtime.climb_accel_mps2 = 0.0F;
    runtime.segment_elapsed_sec = 0.0F;
    runtime.evasion_phase_sec = 0.0F;

    if (config.inbound) {
        runtime.heading_rad = inbound_heading_rad(config.inbound_reference_ned_m, start_position_ned_m);
        const core::Vec3 toward = config.inbound_reference_ned_m - start_position_ned_m;
        const float horiz = horizontal_magnitude(toward);
        if (horiz > 1.0e-3F) {
            const float climb_scale = core::clamp((-toward.z / horiz) * 0.25F, -1.0F, 1.0F);
            runtime.climb_rate_mps =
                core::clamp(climb_scale * config.speed_mps, -config.max_climb_rate_mps, config.max_climb_rate_mps);
        }
    } else if (config.outbound) {
        runtime.heading_rad = outbound_heading_rad(config.outbound_reference_ned_m, start_position_ned_m);
        const core::Vec3 away = start_position_ned_m - config.outbound_reference_ned_m;
        const float horiz = horizontal_magnitude(away);
        if (horiz > 1.0e-3F) {
            const float climb_scale = core::clamp((away.z / horiz) * 0.35F, -1.0F, 1.0F);
            runtime.climb_rate_mps =
                core::clamp(climb_scale * config.speed_mps, -config.max_climb_rate_mps, config.max_climb_rate_mps);
        }
    }

    if (config.curved_initial_spline) {
        splice_initial_spline_segment(runtime, config, start_position_ned_m);
    } else {
        splice_spline_segment(runtime, config, state);
    }

    const float horiz_speed = config.speed_mps;
    state.velocity_ned_mps = core::Vec3{
        horiz_speed * std::cos(runtime.heading_rad),
        horiz_speed * std::sin(runtime.heading_rad),
        runtime.climb_rate_mps};
}

void step_target(TargetState& state, const TargetConfig& config, TargetRuntime& runtime, const float dt,
                 const MissileThreat* missile_threat) noexcept {
    if (dt <= 0.0F) {
        return;
    }

    runtime.evasion_phase_sec += dt;

    runtime.segment_elapsed_sec += dt;
    while (runtime.segment_elapsed_sec >= runtime.segment.duration_sec) {
        const float excess = runtime.segment_elapsed_sec - runtime.segment.duration_sec;
        sample_spline_segment(runtime.segment, runtime.segment.duration_sec, runtime.heading_rad,
                              runtime.heading_rate_rps, runtime.climb_rate_mps, runtime.climb_accel_mps2);
        splice_spline_segment(runtime, config, state);
        runtime.segment_elapsed_sec = excess;
    }

    float heading_rad = runtime.heading_rad;
    float heading_rate_rps = runtime.heading_rate_rps;
    float climb_mps = runtime.climb_rate_mps;
    float climb_accel_mps2 = runtime.climb_accel_mps2;
    sample_spline_segment(runtime.segment, runtime.segment_elapsed_sec, heading_rad, heading_rate_rps, climb_mps,
                          climb_accel_mps2);

    heading_rad = wrap_angle_rad(heading_rad);
    const float yaw_lim = max_yaw_rate_for_config(config);
    heading_rate_rps = core::clamp(heading_rate_rps, -yaw_lim, yaw_lim);
    climb_mps = core::clamp(climb_mps, -config.max_climb_rate_mps, config.max_climb_rate_mps);
    climb_accel_mps2 =
        core::clamp(climb_accel_mps2, -config.max_climb_accel_mps2, config.max_climb_accel_mps2);

    apply_inbound_soft_correction(config, state, heading_rad, heading_rate_rps);
    apply_outbound_soft_correction(config, state, heading_rad, heading_rate_rps);
    apply_play_area_soft_correction(config, state, heading_rad, heading_rate_rps);
    if (!config.smooth_path_only) {
        apply_missile_evasion(config, state, runtime, missile_threat, heading_rad, heading_rate_rps, climb_mps);
    } else if (config.evade_missile) {
        // Radius-limited soft evasion only (no weave snaps).
        apply_missile_evasion(config, state, runtime, missile_threat, heading_rad, heading_rate_rps, climb_mps);
        heading_rate_rps = core::clamp(heading_rate_rps, -yaw_lim, yaw_lim);
    }

    runtime.heading_rad = heading_rad;
    runtime.heading_rate_rps = heading_rate_rps;
    runtime.climb_rate_mps = climb_mps;
    runtime.climb_accel_mps2 = climb_accel_mps2;

    const float horiz_speed = config.speed_mps;
    state.velocity_ned_mps = core::Vec3{
        horiz_speed * std::cos(heading_rad),
        horiz_speed * std::sin(heading_rad),
        climb_mps};
    state.position_ned_m = state.position_ned_m + (state.velocity_ned_mps * dt);

    // Keep altitude in a flyable band over the play area (NED Down positive toward ground).
    if (config.constrain_play_area) {
        constexpr float kMinDown = -160.0F;  // ~160 m AGL
        constexpr float kMaxDown = -25.0F;   // ~25 m AGL
        if (state.position_ned_m.z < kMinDown) {
            state.position_ned_m.z = kMinDown;
            runtime.climb_rate_mps = std::max(runtime.climb_rate_mps, 0.0F);
        } else if (state.position_ned_m.z > kMaxDown) {
            state.position_ned_m.z = kMaxDown;
            runtime.climb_rate_mps = std::min(runtime.climb_rate_mps, 0.0F);
        }
    }
}

}  // namespace engagement
}  // namespace flightsim
