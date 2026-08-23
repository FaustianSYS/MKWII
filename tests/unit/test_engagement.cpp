#include "minimal_test.hpp"

#include "flightsim/engagement/missile/missile.hpp"
#include "flightsim/engagement/missile/missile_autopilot.hpp"
#include "flightsim/engagement/missile/missile_control_allocation.hpp"
#include "flightsim/engagement/missile/missile_mass_properties.hpp"
#include "flightsim/engagement/missile/missile_object.hpp"
#include "flightsim/engagement/common/rng.hpp"
#include "flightsim/engagement/scenario/air_defense_scenario.hpp"
#include "flightsim/engagement/scenario/scenario.hpp"
#include "flightsim/engagement/target/target.hpp"

int run_engagement_tests() {
    using flightsim::core::Vec3;
    using flightsim::engagement::ControlAllocationConfig;
    using flightsim::engagement::DeterministicRng;
    using flightsim::engagement::EngagementScenario;
    using flightsim::engagement::IndividualFins;
    using flightsim::engagement::MissileAutopilotConfig;
    using flightsim::engagement::MissileAutopilotState;
    using flightsim::engagement::MissileAttributes;
    using flightsim::engagement::MissileObject;
    using flightsim::engagement::ScenarioConfig;
    using flightsim::engagement::TargetConfig;
    using flightsim::engagement::TargetRuntime;
    using flightsim::engagement::TargetState;
    using flightsim::engagement::VirtualAxisCommand;
    using flightsim::engagement::accel_command_to_rate_setpoint;
    using flightsim::engagement::allocate_virtual_axes_to_fin_commands;
    using flightsim::engagement::default_missile_attributes;
    using flightsim::engagement::initialize_missile;
    using flightsim::engagement::initialize_target;
    using flightsim::engagement::range_to_target;
    using flightsim::engagement::reset_missile_autopilot;
    using flightsim::engagement::step_fin_servos;
    using flightsim::engagement::step_missile;
    using flightsim::engagement::step_target;
    using flightsim::engagement::update_mass_properties_from_burn;
    using flightsim::engagement::update_missile_rate_autopilot;

    {
        DeterministicRng rng{99U};
        const float a = rng.uniform(0.0F, 1.0F);
        const float b = rng.uniform(0.0F, 1.0F);
        REQUIRE(a >= 0.0F);
        REQUIRE(a <= 1.0F);
        REQUIRE(b >= 0.0F);
        REQUIRE(b <= 1.0F);

        DeterministicRng rng2{99U};
        REQUIRE_APPROX(rng2.uniform(0.0F, 1.0F), a, 0.0001F);
    }

    {
        const MissileAttributes attrs = default_missile_attributes();
        REQUIRE_APPROX(attrs.length_m, 3.66F, 0.01F);
        REQUIRE(attrs.max_thrust_n > 0.0F);
        REQUIRE(attrs.optical_window.fov_azimuth_rad > 0.0F);
        REQUIRE(attrs.inertia.iyy_kgm2 > 0.0F);
        REQUIRE(attrs.aero.fin_force_per_rad_n > 0.0F);
        REQUIRE_APPROX(attrs.propulsion.dry_mass_kg + attrs.propulsion.propellant_mass_kg, attrs.mass_kg, 0.01F);
        REQUIRE(attrs.allocation.servo_limits.max_rate_rps > 5.0F);
    }

    {
        ControlAllocationConfig config{};
        config.servo_limits.max_deflection_rad = 0.35F;
        config.servo_limits.max_rate_rps = 8.7F;
        config.servo_limits.time_constant_sec = 0.03F;

        VirtualAxisCommand command{};
        command.pitch_rad = 0.35F;
        command.yaw_rad = 0.20F;

        IndividualFins commanded = allocate_virtual_axes_to_fin_commands(command, config);
        IndividualFins fins{};
        for (int i = 0; i < 200; ++i) {
            step_fin_servos(fins, commanded, config.servo_limits, 0.01F);
            for (int f = 0; f < IndividualFins::kCount; ++f) {
                REQUIRE(std::fabs(fins.fin[f].position_rad) <= config.servo_limits.max_deflection_rad + 0.001F);
                REQUIRE(std::fabs(fins.fin[f].rate_rps) <= config.servo_limits.max_rate_rps + 0.001F);
            }
        }
    }

    {
        MissileObject missile{};
        MissileAttributes attrs = default_missile_attributes();
        initialize_missile(missile, attrs, Vec3{}, Vec3{300.0F, 0.0F, 0.0F});
        REQUIRE_APPROX(missile.mass_properties.mass_kg, attrs.mass_kg, 0.01F);
        REQUIRE_APPROX(missile.mass_properties.cg_body_m.x, attrs.propulsion.cg_full_body_m.x, 0.01F);

        update_mass_properties_from_burn(missile, attrs.propulsion.burn_time_sec);
        REQUIRE_APPROX(missile.mass_properties.mass_kg, attrs.propulsion.dry_mass_kg, 0.01F);
        REQUIRE_APPROX(missile.mass_properties.cg_body_m.x, attrs.propulsion.cg_dry_body_m.x, 0.01F);
        REQUIRE(missile.mass_properties.inertia.iyy_kgm2 < attrs.propulsion.inertia_full.iyy_kgm2);
    }

    {
        MissileAutopilotConfig config{};
        config.enabled = true;
        config.max_rate_rps = 5.0F;
        config.accel_to_rate_gain = 1.0F;
        config.min_speed_mps = 20.0F;
        config.rate_loop_blend = 0.35F;
        config.max_deflection_rad = 0.35F;
        config.pitch = {0.12F, 0.03F, 0.0F, 0.02F, 0.08F};
        config.yaw = {0.12F, 0.03F, 0.0F, 0.02F, 0.08F};
        config.roll = {0.08F, 0.02F, 0.0F, 0.01F, 0.05F};

        const Vec3 rate_sp =
            accel_command_to_rate_setpoint(Vec3{0.0F, 40.0F, -20.0F}, Vec3{200.0F, 0.0F, 0.0F}, config);
        REQUIRE(rate_sp.y > 0.0F);
        REQUIRE(rate_sp.z > 0.0F);
        REQUIRE(std::fabs(rate_sp.x) < 1.0e-5F);

        MissileAutopilotState state{};
        reset_missile_autopilot(state);
        VirtualAxisCommand cmd{};
        for (int i = 0; i < 50; ++i) {
            cmd = update_missile_rate_autopilot(state, Vec3{}, rate_sp, 0.01F, config);
        }
        REQUIRE(std::fabs(cmd.pitch_rad) > 0.0F || std::fabs(cmd.yaw_rad) > 0.0F);
        REQUIRE(std::fabs(cmd.pitch_rad) <= config.max_deflection_rad + 0.001F);
        REQUIRE(std::fabs(cmd.yaw_rad) <= config.max_deflection_rad + 0.001F);
        REQUIRE(std::fabs(state.rate_integral.y) <= config.pitch.lim_int + 0.001F);
    }

    {
        MissileObject missile{};
        MissileAttributes attrs = default_missile_attributes();
        initialize_missile(missile, attrs, Vec3{}, Vec3{300.0F, 0.0F, 0.0F});
        REQUIRE(missile.attitude.is_valid());
        REQUIRE_APPROX(missile.attitude.w, 1.0F, 0.01F);
    }

    {
        TargetState target{};
        TargetConfig config{};
        TargetRuntime runtime{};
        config.speed_mps = 30.0F;
        config.spline_segment_sec = 2.0F;
        config.rng_seed = 7U;
        config.outbound = true;
        config.outbound_reference_ned_m = Vec3{};
        initialize_target(target, config, runtime, Vec3{1000.0F, 0.0F, -200.0F}, 0.0F);

        const float start_range = (target.position_ned_m - config.outbound_reference_ned_m).magnitude();
        float max_heading_jump = 0.0F;
        float prev_heading = runtime.heading_rad;
        for (int i = 0; i < 400; ++i) {
            step_target(target, config, runtime, 0.01F);
            const float jump = std::fabs(runtime.heading_rad - prev_heading);
            if (jump > max_heading_jump) {
                max_heading_jump = jump;
            }
            prev_heading = runtime.heading_rad;
        }
        const float end_range = (target.position_ned_m - config.outbound_reference_ned_m).magnitude();
        REQUIRE(end_range > start_range);
        REQUIRE(max_heading_jump < 0.08F);
    }

    {
        MissileObject missile{};
        MissileAttributes attrs = default_missile_attributes();
        attrs.burn_time_sec = 20.0F;
        initialize_missile(missile, attrs, Vec3{}, Vec3{80.0F, 0.0F, 0.0F});
        const Vec3 target_pos{1000.0F, 0.0F, 0.0F};

        const float launch_speed = missile.velocity_ned_mps.magnitude();
        step_missile(missile, target_pos, Vec3{}, 0.01F);
        REQUIRE(launch_speed < 120.0F);
        REQUIRE(missile.velocity_ned_mps.magnitude() >= launch_speed);
        REQUIRE_APPROX(missile.thrust_n, attrs.max_thrust_n, 1.0F);
        REQUIRE_APPROX(missile.attributes.length_m, 3.66F, 0.01F);
        REQUIRE(missile.attitude.is_valid());

        for (int i = 0; i < 5000 && missile.active; ++i) {
            step_missile(missile, target_pos, Vec3{}, 0.01F);
        }
        REQUIRE(missile.hit);
        REQUIRE(missile.angular_rate_body_rps.is_valid());
    }

    {
        MissileObject missile{};
        MissileAttributes attrs = default_missile_attributes();
        initialize_missile(missile, attrs, Vec3{}, Vec3{300.0F, 0.0F, 0.0F});
        const Vec3 target_pos{1000.0F, 200.0F, -100.0F};
        for (int i = 0; i < 200; ++i) {
            step_missile(missile, target_pos, Vec3{0.0F, 40.0F, 0.0F}, 0.01F);
        }
        REQUIRE(missile.seeker_locked || missile.surfaces.fin_yaw_rad != 0.0F ||
                missile.angular_rate_body_rps.magnitude() > 0.01F);
    }

    {
        ScenarioConfig config = flightsim::engagement::default_air_defense_config();
        EngagementScenario scenario(config);
        scenario.initialize();
        scenario.run(35000U);

        REQUIRE(scenario.state().intercept);
        REQUIRE(scenario.state().miss_distance_m <= config.missile.kill_radius_m);
    }

    {
        // Longer outbound engagement exercises rate-loop damping over burn + coast.
        ScenarioConfig config{};
        config.dt_sec = 0.01F;
        config.missile = default_missile_attributes();
        config.missile.navigation_gain = 6.5F;
        config.missile.max_speed_mps = 700.0F;
        config.missile.max_lateral_accel_mps2 = 60.0F;
        config.target.speed_mps = 40.0F;
        config.target.rng_seed = 2024U;
        config.target_outbound = true;
        config.missile_launch_speed_mps = 80.0F;

        EngagementScenario scenario(config);
        scenario.initialize();
        scenario.run(35000U);

        REQUIRE(scenario.state().intercept);
        REQUIRE(scenario.state().miss_distance_m <= config.missile.kill_radius_m);
    }

    return 0;
}
