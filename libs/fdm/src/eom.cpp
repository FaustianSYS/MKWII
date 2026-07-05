#include "flightsim/fdm/eom.hpp"

#include <cmath>

namespace flightsim {
namespace fdm {

namespace {

struct StateDerivative {
    core::Vec3 velocity_ned_mps{};
    core::Vec3 acceleration_ned_mps2{};
    core::Quaternion attitude_rate{};
    core::Vec3 angular_accel_body_rps2{};
};

core::Vec3 body_to_ned(const core::Quaternion& attitude, const core::Vec3& body) noexcept {
    return attitude.rotate(body);
}

StateDerivative compute_derivative(const AircraftState& state) noexcept {
    StateDerivative deriv{};

    const float mass = state.mass_props.mass_kg;
    if (mass <= 0.0F) {
        return deriv;
    }

    const core::Vec3 gravity_ned{0.0F, 0.0F, core::kGravity * mass};
    const core::Vec3 force_ned = body_to_ned(state.attitude, state.body_wrench.force_body_n) + gravity_ned;
    deriv.acceleration_ned_mps2 = force_ned * (1.0F / mass);
    deriv.velocity_ned_mps = state.velocity_ned_mps;

    const core::Vec3 pqr = state.angular_rate_body_rps;
    const core::Vec3 inertia_diag{
        state.mass_props.inertia_kgm2.row0[0],
        state.mass_props.inertia_kgm2.row1[1],
        state.mass_props.inertia_kgm2.row2[2]};

    const core::Vec3 angular_momentum{
        inertia_diag.x * pqr.x,
        inertia_diag.y * pqr.y,
        inertia_diag.z * pqr.z};

    const core::Vec3 gyro = pqr.cross(angular_momentum);
    const core::Vec3 net_moment = state.body_wrench.moment_body_nm - gyro;

    deriv.angular_accel_body_rps2 = core::Vec3{
        net_moment.x / inertia_diag.x,
        net_moment.y / inertia_diag.y,
        net_moment.z / inertia_diag.z};

    const core::Quaternion q = state.attitude;
    deriv.attitude_rate = core::Quaternion{
        0.5F * (((-q.x) * pqr.x) - (q.y * pqr.y) - (q.z * pqr.z)),
        0.5F * ((q.w * pqr.x) + (q.y * pqr.z) - (q.z * pqr.y)),
        0.5F * ((q.w * pqr.y) + (q.z * pqr.x) - (q.x * pqr.z)),
        0.5F * ((q.w * pqr.z) + (q.x * pqr.y) - (q.y * pqr.x))};

    return deriv;
}

AircraftState apply_derivative(const AircraftState& base, const StateDerivative& deriv, float scale) noexcept {
    AircraftState next = base;
    next.velocity_ned_mps = base.velocity_ned_mps + (deriv.velocity_ned_mps * scale);
    next.position_ned_m = base.position_ned_m + (deriv.acceleration_ned_mps2 * (0.5F * scale * scale)) +
                        (base.velocity_ned_mps * scale);
    next.angular_rate_body_rps =
        base.angular_rate_body_rps + (deriv.angular_accel_body_rps2 * scale);
    next.attitude = core::Quaternion{
        base.attitude.w + (deriv.attitude_rate.w * scale),
        base.attitude.x + (deriv.attitude_rate.x * scale),
        base.attitude.y + (deriv.attitude_rate.y * scale),
        base.attitude.z + (deriv.attitude_rate.z * scale)};
    next.attitude.normalize();
    return next;
}

void integrate_semi_implicit_euler(AircraftState& state, float dt) noexcept {
    const StateDerivative deriv = compute_derivative(state);

    state.velocity_ned_mps = state.velocity_ned_mps + (deriv.acceleration_ned_mps2 * dt);
    state.position_ned_m = state.position_ned_m + (state.velocity_ned_mps * dt);
    state.angular_rate_body_rps = state.angular_rate_body_rps + (deriv.angular_accel_body_rps2 * dt);

    state.attitude = core::Quaternion{
        state.attitude.w + (deriv.attitude_rate.w * dt),
        state.attitude.x + (deriv.attitude_rate.x * dt),
        state.attitude.y + (deriv.attitude_rate.y * dt),
        state.attitude.z + (deriv.attitude_rate.z * dt)};
    state.attitude.normalize();
}

void integrate_rk4(AircraftState& state, float dt) noexcept {
    const StateDerivative k1 = compute_derivative(state);
    const AircraftState s2 = apply_derivative(state, k1, dt * 0.5F);
    const StateDerivative k2 = compute_derivative(s2);
    const AircraftState s3 = apply_derivative(state, k2, dt * 0.5F);
    const StateDerivative k3 = compute_derivative(s3);
    const AircraftState s4 = apply_derivative(state, k3, dt);
    const StateDerivative k4 = compute_derivative(s4);

    const core::Vec3 accel = (k1.acceleration_ned_mps2 + (k2.acceleration_ned_mps2 * 2.0F) +
                              (k3.acceleration_ned_mps2 * 2.0F) + k4.acceleration_ned_mps2) *
                             (1.0F / 6.0F);
    const core::Vec3 ang_accel = (k1.angular_accel_body_rps2 + (k2.angular_accel_body_rps2 * 2.0F) +
                                  (k3.angular_accel_body_rps2 * 2.0F) + k4.angular_accel_body_rps2) *
                                 (1.0F / 6.0F);
    const core::Quaternion q_rate = core::Quaternion{
        k1.attitude_rate.w + (2.0F * k2.attitude_rate.w) + (2.0F * k3.attitude_rate.w) + k4.attitude_rate.w,
        k1.attitude_rate.x + (2.0F * k2.attitude_rate.x) + (2.0F * k3.attitude_rate.x) + k4.attitude_rate.x,
        k1.attitude_rate.y + (2.0F * k2.attitude_rate.y) + (2.0F * k3.attitude_rate.y) + k4.attitude_rate.y,
        k1.attitude_rate.z + (2.0F * k2.attitude_rate.z) + (2.0F * k3.attitude_rate.z) + k4.attitude_rate.z};

    state.velocity_ned_mps = state.velocity_ned_mps + (accel * dt);
    state.position_ned_m = state.position_ned_m + (state.velocity_ned_mps * dt);
    state.angular_rate_body_rps = state.angular_rate_body_rps + (ang_accel * dt);
    state.attitude = core::Quaternion{
        state.attitude.w + (q_rate.w * dt * (1.0F / 6.0F)),
        state.attitude.x + (q_rate.x * dt * (1.0F / 6.0F)),
        state.attitude.y + (q_rate.y * dt * (1.0F / 6.0F)),
        state.attitude.z + (q_rate.z * dt * (1.0F / 6.0F))};
    state.attitude.normalize();
}

}  // namespace

void integrate_eom(AircraftState& state, float dt, IntegratorMethod method) noexcept {
    if (dt <= 0.0F) {
        return;
    }

    if (method == IntegratorMethod::Rk4) {
        integrate_rk4(state, dt);
    } else {
        integrate_semi_implicit_euler(state, dt);
    }
}

}  // namespace fdm
}  // namespace flightsim
