#include "flightsim/engagement/missile/missile.hpp"

#include "flightsim/core/constants.hpp"
#include "flightsim/engagement/missile/missile_eom.hpp"
#include "flightsim/engagement/missile/geometric_seeker_source.hpp"

#include <cmath>

namespace flightsim {
namespace engagement {

namespace {

core::Vec3 normalize(const core::Vec3& v) noexcept {
    const float mag = v.magnitude();
    if (mag < 1.0e-6F) {
        return core::Vec3{};
    }
    return v * (1.0F / mag);
}

core::Vec3 clamp_magnitude(const core::Vec3& v, float max_mag) noexcept {
    const float mag = v.magnitude();
    if (mag <= max_mag || mag < 1.0e-6F) {
        return v;
    }
    return v * (max_mag / mag);
}

void apply_surface_rate_limit(ControlSurfaces& current, const ControlSurfaces& commanded,
                              const ControlSurfaceLimits& limits, float dt) noexcept {
    const float max_delta = limits.max_rate_rps * dt;
    current.fin_pitch_rad += core::clamp(commanded.fin_pitch_rad - current.fin_pitch_rad, -max_delta, max_delta);
    current.fin_yaw_rad += core::clamp(commanded.fin_yaw_rad - current.fin_yaw_rad, -max_delta, max_delta);
    current.fin_roll_rad += core::clamp(commanded.fin_roll_rad - current.fin_roll_rad, -max_delta, max_delta);

    current.fin_pitch_rad =
        core::clamp(current.fin_pitch_rad, -limits.max_deflection_rad, limits.max_deflection_rad);
    current.fin_yaw_rad = core::clamp(current.fin_yaw_rad, -limits.max_deflection_rad, limits.max_deflection_rad);
    current.fin_roll_rad = core::clamp(current.fin_roll_rad, -limits.max_deflection_rad, limits.max_deflection_rad);
}

ControlSurfaces accel_to_surface_command(const core::Vec3& commanded_accel_ned, const core::Quaternion& attitude,
                                         const MissileAttributes& attrs) noexcept {
    ControlSurfaces cmd{};
    const core::Vec3 accel_body = ned_to_body(attitude, commanded_accel_ned);
    const float force_per_rad = attrs.aero.fin_force_per_rad_n;
    if (force_per_rad < 1.0e-3F) {
        return cmd;
    }

    const float desired_fy = attrs.mass_kg * accel_body.y;
    const float desired_fz = attrs.mass_kg * accel_body.z;

    cmd.fin_yaw_rad = core::clamp(desired_fy / force_per_rad, -attrs.surface_limits.max_deflection_rad,
                                  attrs.surface_limits.max_deflection_rad);
    cmd.fin_pitch_rad = core::clamp(desired_fz / force_per_rad, -attrs.surface_limits.max_deflection_rad,
                                    attrs.surface_limits.max_deflection_rad);
    return cmd;
}

MissileWrench compute_wrench(const MissileObject& missile, const core::Vec3& guidance_accel_ned,
                             const core::Vec3& attitude_reference_ned, float alignment_gain_scale) noexcept {
    const MissileAero& aero = missile.attributes.aero;
    const float mass = missile.attributes.mass_kg;
    MissileWrench wrench{};

    if (missile.thrust_n > 0.0F) {
        wrench.force_body_n.x += missile.thrust_n;
    }

    const core::Vec3 guidance_force_ned = guidance_accel_ned * mass;
    wrench.force_body_n = wrench.force_body_n + ned_to_body(missile.attitude, guidance_force_ned);

    const float speed = missile.velocity_ned_mps.magnitude();
    if (speed > 1.0F) {
        const float dynamic_pressure = 0.5F * core::kSeaLevelDensity * speed * speed;
        const core::Vec3 vel_body = ned_to_body(missile.attitude, missile.velocity_ned_mps);
        const float speed_body_x = std::fabs(vel_body.x);
        wrench.force_body_n.x -= dynamic_pressure * aero.reference_area_m2 * aero.drag_coefficient *
                                 core::clamp(speed_body_x / speed, 0.0F, 1.0F);

        if (speed_body_x > 5.0F) {
            const float alpha = std::atan2(-vel_body.z, vel_body.x);
            const float beta = std::atan2(vel_body.y, vel_body.x);
            wrench.moment_body_nm.y += -aero.static_pitch_moment_per_rad_nm * alpha;
            wrench.moment_body_nm.z += -aero.static_yaw_moment_per_rad_nm * beta;
        }

        const core::Vec3 boresight_ned = body_x_ned(missile.attitude);
        core::Vec3 reference_dir = attitude_reference_ned;
        if (reference_dir.magnitude() < 1.0e-6F) {
            reference_dir = missile.velocity_ned_mps * (1.0F / speed);
        } else {
            reference_dir = normalize(reference_dir);
        }

        const core::Vec3 misalign_ned = boresight_ned.cross(reference_dir);
        const core::Vec3 misalign_body = ned_to_body(missile.attitude, misalign_ned);
        const float align_gain = aero.velocity_alignment_moment_nm * alignment_gain_scale;
        wrench.moment_body_nm.y += misalign_body.z * align_gain;
        wrench.moment_body_nm.z -= misalign_body.y * align_gain;
    }

    wrench.moment_body_nm.x += missile.surfaces.fin_roll_rad * aero.fin_roll_moment_per_rad_nm;
    wrench.moment_body_nm.y += missile.surfaces.fin_pitch_rad * aero.fin_pitch_moment_per_rad_nm;
    wrench.moment_body_nm.z -= missile.surfaces.fin_yaw_rad * aero.fin_yaw_moment_per_rad_nm;

    const float damp = aero.damping_moment_per_rps_nm;
    wrench.moment_body_nm = wrench.moment_body_nm - (missile.angular_rate_body_rps * damp);

    return wrench;
}

}  // namespace

void initialize_missile(MissileObject& missile, const MissileAttributes& attributes,
                        const core::Vec3& launch_position_ned_m,
                        const core::Vec3& launch_velocity_ned_mps) noexcept {
    missile.attributes = attributes;
    missile.position_ned_m = launch_position_ned_m;
    missile.velocity_ned_mps = launch_velocity_ned_mps;
    missile.attitude = quaternion_from_body_x_ned(normalize(launch_velocity_ned_mps));
    missile.angular_rate_body_rps = core::Vec3{};
    missile.surfaces = ControlSurfaces{};
    missile.commanded_surfaces = ControlSurfaces{};
    missile.wrench = MissileWrench{};
    missile.thrust_n = attributes.max_thrust_n;
    missile.flight_time_sec = 0.0F;
    missile.active = true;
    missile.hit = false;
    missile.seeker_locked = false;
    missile.seeker_los_angle_rad = 0.0F;
    missile.seeker_range_m = 0.0F;
}

core::Vec3 missile_nose_position(const MissileObject& missile) noexcept {
    const core::Vec3 nose_dir = body_x_ned(missile.attitude);
    const float nose_offset = missile.attributes.length_m * 0.5F;
    return missile.position_ned_m + (nose_dir * nose_offset);
}

float range_to_target(const MissileObject& missile, const core::Vec3& target_position_ned_m) noexcept {
    return (target_position_ned_m - missile_nose_position(missile)).magnitude();
}

void step_missile(MissileObject& missile, const core::Vec3& target_position_ned_m,
                  const core::Vec3& target_velocity_ned_mps, float dt,
                  const vision::SeekerTrack* seeker_track) noexcept {
    if (!missile.active || missile.hit || dt <= 0.0F) {
        return;
    }

    missile.flight_time_sec += dt;

    const float range = range_to_target(missile, target_position_ned_m);
    if (range <= missile.attributes.kill_radius_m) {
        missile.hit = true;
        missile.active = false;
        missile.thrust_n = 0.0F;
        missile.wrench = MissileWrench{};
        return;
    }

    vision::SeekerTrack geometric_track =
        compute_geometric_seeker_track(missile, target_position_ned_m);
    const vision::SeekerTrack* active_track = seeker_track;
    if (active_track == nullptr || !active_track->valid) {
        active_track = &geometric_track;
    }
    apply_seeker_track_to_missile(missile, *active_track);

    const bool motor_burning = missile.flight_time_sec <= missile.attributes.burn_time_sec;
    missile.thrust_n = motor_burning ? missile.attributes.max_thrust_n : 0.0F;

    core::Vec3 line_of_sight = active_track->los_unit_ned.magnitude() > 1.0e-6F
                                   ? normalize(active_track->los_unit_ned)
                                   : normalize(target_position_ned_m - missile.position_ned_m);

    const core::Vec3 relative_position = target_position_ned_m - missile.position_ned_m;
    const core::Vec3 relative_velocity = target_velocity_ned_mps - missile.velocity_ned_mps;
    const float closing_speed =
        core::clamp(-relative_velocity.dot(line_of_sight), 0.0F, missile.attributes.max_speed_mps);

    const float nav_gain =
        missile.seeker_locked ? missile.attributes.navigation_gain : missile.attributes.navigation_gain * 0.25F;

    core::Vec3 line_of_sight_rate = active_track->los_rate_ned;
    if (line_of_sight_rate.magnitude() < 1.0e-6F) {
        const float range_sq = relative_position.dot(relative_position);
        line_of_sight_rate =
            range_sq > 1.0e-6F ? relative_position.cross(relative_velocity) * (1.0F / range_sq) : core::Vec3{};
    }

    const core::Vec3 velocity_direction = normalize(missile.velocity_ned_mps);
    core::Vec3 commanded_accel = core::Vec3{};
    if (velocity_direction.magnitude() > 1.0e-6F) {
        commanded_accel = line_of_sight_rate.cross(velocity_direction) * (nav_gain * closing_speed);
    }

    commanded_accel = clamp_magnitude(commanded_accel, missile.attributes.max_lateral_accel_mps2);
    commanded_accel.z -= core::kGravity;

    if (motor_burning) {
        const float speed = missile.velocity_ned_mps.magnitude();
        const float motor_accel = missile.thrust_n / missile.attributes.mass_kg;
        const float speed_deficit = core::clamp(missile.attributes.max_speed_mps - speed, 0.0F, 500.0F);
        const float boost_accel = core::clamp(motor_accel * 0.65F, 0.0F, speed_deficit * 4.0F);
        if (line_of_sight.magnitude() > 1.0e-6F) {
            commanded_accel = commanded_accel + (line_of_sight * boost_accel);
        }
    }

    const float max_commanded_accel =
        missile.attributes.max_lateral_accel_mps2 + core::kGravity + (missile.thrust_n / missile.attributes.mass_kg);
    commanded_accel = clamp_magnitude(commanded_accel, max_commanded_accel);

    missile.commanded_surfaces =
        accel_to_surface_command(commanded_accel, missile.attitude, missile.attributes);
    apply_surface_rate_limit(missile.surfaces, missile.commanded_surfaces, missile.attributes.surface_limits, dt);

    const core::Vec3 attitude_reference =
        motor_burning ? line_of_sight : normalize(missile.velocity_ned_mps);
    const float alignment_gain_scale = motor_burning ? 3.0F : 1.0F;
    missile.wrench = compute_wrench(missile, commanded_accel, attitude_reference, alignment_gain_scale);

    integrate_missile_eom(missile.position_ned_m, missile.velocity_ned_mps, missile.attitude,
                          missile.angular_rate_body_rps, missile.wrench, missile.attributes.mass_kg,
                          missile.attributes.inertia, dt);

    missile.velocity_ned_mps = clamp_magnitude(missile.velocity_ned_mps, missile.attributes.max_speed_mps);
}

}  // namespace engagement
}  // namespace flightsim
