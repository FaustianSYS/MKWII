#include "flightsim/engagement/missile/missile_eom.hpp"

#include "flightsim/core/constants.hpp"

#include <cmath>

namespace flightsim {
namespace engagement {

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

StateDerivative compute_derivative(const core::Vec3& velocity_ned_mps, const core::Quaternion& attitude,
                                   const core::Vec3& angular_rate_body_rps, const MissileWrench& wrench,
                                   float mass_kg, const MissileInertia& inertia) noexcept {
    StateDerivative deriv{};

    if (mass_kg <= 0.0F) {
        return deriv;
    }

    const core::Vec3 gravity_ned{0.0F, 0.0F, core::kGravity * mass_kg};
    const core::Vec3 force_ned = body_to_ned(attitude, wrench.force_body_n) + gravity_ned;
    deriv.acceleration_ned_mps2 = force_ned * (1.0F / mass_kg);
    deriv.velocity_ned_mps = velocity_ned_mps;

    const core::Vec3 pqr = angular_rate_body_rps;
    const core::Vec3 inertia_diag{inertia.ixx_kgm2, inertia.iyy_kgm2, inertia.izz_kgm2};
    const core::Vec3 angular_momentum{
        inertia_diag.x * pqr.x,
        inertia_diag.y * pqr.y,
        inertia_diag.z * pqr.z};

    const core::Vec3 gyro = pqr.cross(angular_momentum);
    const core::Vec3 net_moment = wrench.moment_body_nm - gyro;

    deriv.angular_accel_body_rps2 = core::Vec3{
        net_moment.x / inertia_diag.x,
        net_moment.y / inertia_diag.y,
        net_moment.z / inertia_diag.z};

    const core::Quaternion q = attitude;
    deriv.attitude_rate = core::Quaternion{
        0.5F * (((-q.x) * pqr.x) - (q.y * pqr.y) - (q.z * pqr.z)),
        0.5F * ((q.w * pqr.x) + (q.y * pqr.z) - (q.z * pqr.y)),
        0.5F * ((q.w * pqr.y) + (q.z * pqr.x) - (q.x * pqr.z)),
        0.5F * ((q.w * pqr.z) + (q.x * pqr.y) - (q.y * pqr.x))};

    return deriv;
}

}  // namespace

core::Quaternion quaternion_from_body_x_ned(const core::Vec3& body_x_ned) noexcept {
    const float mag = body_x_ned.magnitude();
    if (mag < 1.0e-6F) {
        return core::Quaternion::identity();
    }

    const core::Vec3 body_x = body_x_ned * (1.0F / mag);
    const core::Vec3 ned_down{0.0F, 0.0F, 1.0F};
    core::Vec3 body_y = ned_down.cross(body_x);
    if (body_y.magnitude() < 1.0e-6F) {
        body_y = core::Vec3{0.0F, 1.0F, 0.0F};
    } else {
        const float y_mag = body_y.magnitude();
        body_y = body_y * (1.0F / y_mag);
    }

    const core::Vec3 body_z = body_x.cross(body_y);

    const core::Mat3 rotation = [&]() {
        core::Mat3 m{};
        m.row0[0] = body_x.x;
        m.row0[1] = body_y.x;
        m.row0[2] = body_z.x;
        m.row1[0] = body_x.y;
        m.row1[1] = body_y.y;
        m.row1[2] = body_z.y;
        m.row2[0] = body_x.z;
        m.row2[1] = body_y.z;
        m.row2[2] = body_z.z;
        return m;
    }();

    const float trace = rotation.row0[0] + rotation.row1[1] + rotation.row2[2];
    core::Quaternion q{};
    if (trace > 0.0F) {
        const float s = std::sqrt(trace + 1.0F) * 2.0F;
        q.w = 0.25F * s;
        q.x = (rotation.row2[1] - rotation.row1[2]) / s;
        q.y = (rotation.row0[2] - rotation.row2[0]) / s;
        q.z = (rotation.row1[0] - rotation.row0[1]) / s;
    } else if (rotation.row0[0] > rotation.row1[1] && rotation.row0[0] > rotation.row2[2]) {
        const float s = std::sqrt(1.0F + rotation.row0[0] - rotation.row1[1] - rotation.row2[2]) * 2.0F;
        q.w = (rotation.row2[1] - rotation.row1[2]) / s;
        q.x = 0.25F * s;
        q.y = (rotation.row0[1] + rotation.row1[0]) / s;
        q.z = (rotation.row0[2] + rotation.row2[0]) / s;
    } else if (rotation.row1[1] > rotation.row2[2]) {
        const float s = std::sqrt(1.0F + rotation.row1[1] - rotation.row0[0] - rotation.row2[2]) * 2.0F;
        q.w = (rotation.row0[2] - rotation.row2[0]) / s;
        q.x = (rotation.row0[1] + rotation.row1[0]) / s;
        q.y = 0.25F * s;
        q.z = (rotation.row1[2] + rotation.row2[1]) / s;
    } else {
        const float s = std::sqrt(1.0F + rotation.row2[2] - rotation.row0[0] - rotation.row1[1]) * 2.0F;
        q.w = (rotation.row1[0] - rotation.row0[1]) / s;
        q.x = (rotation.row0[2] + rotation.row2[0]) / s;
        q.y = (rotation.row1[2] + rotation.row2[1]) / s;
        q.z = 0.25F * s;
    }

    q.normalize();
    return q;
}

core::Vec3 body_x_ned(const core::Quaternion& attitude) noexcept {
    return attitude.rotate(core::Vec3{1.0F, 0.0F, 0.0F});
}

core::Vec3 ned_to_body(const core::Quaternion& attitude, const core::Vec3& vector_ned) noexcept {
    const core::Mat3 rotation = attitude.to_rotation_matrix();
    return core::Vec3{
        (rotation.row0[0] * vector_ned.x) + (rotation.row1[0] * vector_ned.y) +
            (rotation.row2[0] * vector_ned.z),
        (rotation.row0[1] * vector_ned.x) + (rotation.row1[1] * vector_ned.y) +
            (rotation.row2[1] * vector_ned.z),
        (rotation.row0[2] * vector_ned.x) + (rotation.row1[2] * vector_ned.y) +
            (rotation.row2[2] * vector_ned.z)};
}

void integrate_missile_eom(core::Vec3& position_ned_m, core::Vec3& velocity_ned_mps, core::Quaternion& attitude,
                           core::Vec3& angular_rate_body_rps, const MissileWrench& wrench, float mass_kg,
                           const MissileInertia& inertia, float dt) noexcept {
    if (dt <= 0.0F) {
        return;
    }

    const StateDerivative deriv =
        compute_derivative(velocity_ned_mps, attitude, angular_rate_body_rps, wrench, mass_kg, inertia);

    velocity_ned_mps = velocity_ned_mps + (deriv.acceleration_ned_mps2 * dt);
    position_ned_m = position_ned_m + (velocity_ned_mps * dt);
    angular_rate_body_rps = angular_rate_body_rps + (deriv.angular_accel_body_rps2 * dt);

    attitude = core::Quaternion{
        attitude.w + (deriv.attitude_rate.w * dt),
        attitude.x + (deriv.attitude_rate.x * dt),
        attitude.y + (deriv.attitude_rate.y * dt),
        attitude.z + (deriv.attitude_rate.z * dt)};
    attitude.normalize();
}

}  // namespace engagement
}  // namespace flightsim
