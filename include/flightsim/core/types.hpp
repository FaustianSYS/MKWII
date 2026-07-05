#pragma once

#include <cmath>
#include <cstdint>
#include <type_traits>

namespace flightsim {
namespace core {

// @req LLR-CORE-001
inline bool is_finite(float value) noexcept {
    return std::isfinite(value);
}

// @req LLR-CORE-002
inline float clamp(float value, float min_val, float max_val) noexcept {
    if (value < min_val) {
        return min_val;
    }
    if (value > max_val) {
        return max_val;
    }
    return value;
}

template <typename T, std::size_t N>
class FixedArray {
public:
    static_assert(N > 0U, "FixedArray size must be positive");

    using value_type = T;
    static constexpr std::size_t size = N;

    constexpr T& operator[](std::size_t index) noexcept { return data_[index]; }
    constexpr const T& operator[](std::size_t index) const noexcept { return data_[index]; }

    constexpr T* data() noexcept { return data_; }
    constexpr const T* data() const noexcept { return data_; }

private:
    T data_[N]{};
};

struct Vec3 {
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};

    constexpr Vec3() noexcept = default;
    constexpr Vec3(float x_val, float y_val, float z_val) noexcept
        : x(x_val), y(y_val), z(z_val) {}

    // @req LLR-CORE-003
    Vec3 operator+(const Vec3& other) const noexcept {
        return Vec3{x + other.x, y + other.y, z + other.z};
    }

    // @req LLR-CORE-004
    Vec3 operator-(const Vec3& other) const noexcept {
        return Vec3{x - other.x, y - other.y, z - other.z};
    }

    // @req LLR-CORE-005
    Vec3 operator*(float scalar) const noexcept {
        return Vec3{x * scalar, y * scalar, z * scalar};
    }

    // @req LLR-CORE-006
    float dot(const Vec3& other) const noexcept {
        return (x * other.x) + (y * other.y) + (z * other.z);
    }

    // @req LLR-CORE-007
    Vec3 cross(const Vec3& other) const noexcept {
        return Vec3{
            (y * other.z) - (z * other.y),
            (z * other.x) - (x * other.z),
            (x * other.y) - (y * other.x)};
    }

    // @req LLR-CORE-008
    float magnitude() const noexcept {
        return std::sqrt(dot(*this));
    }

    // @req LLR-CORE-009
    bool is_valid() const noexcept {
        return is_finite(x) && is_finite(y) && is_finite(z);
    }
};

struct Mat3 {
    FixedArray<float, 3> row0{};
    FixedArray<float, 3> row1{};
    FixedArray<float, 3> row2{};

    // @req LLR-CORE-010
    Vec3 multiply(const Vec3& v) const noexcept {
        return Vec3{
            (row0[0] * v.x) + (row0[1] * v.y) + (row0[2] * v.z),
            (row1[0] * v.x) + (row1[1] * v.y) + (row1[2] * v.z),
            (row2[0] * v.x) + (row2[1] * v.y) + (row2[2] * v.z)};
    }

    // @req LLR-CORE-011
    static Mat3 identity() noexcept {
        Mat3 m{};
        m.row0[0] = 1.0F;
        m.row1[1] = 1.0F;
        m.row2[2] = 1.0F;
        return m;
    }
};

struct Quaternion {
    float w{1.0F};
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};

    // @req LLR-CORE-012
    static Quaternion identity() noexcept {
        return Quaternion{};
    }

    // @req LLR-CORE-013
    void normalize() noexcept {
        const float mag = std::sqrt((w * w) + (x * x) + (y * y) + (z * z));
        if (mag > 1.0e-12F) {
            const float inv = 1.0F / mag;
            w *= inv;
            x *= inv;
            y *= inv;
            z *= inv;
        } else {
            w = 1.0F;
            x = 0.0F;
            y = 0.0F;
            z = 0.0F;
        }
    }

    // @req LLR-CORE-014
    Mat3 to_rotation_matrix() const noexcept {
        const float ww = w * w;
        const float xx = x * x;
        const float yy = y * y;
        const float zz = z * z;
        const float wx = w * x;
        const float wy = w * y;
        const float wz = w * z;
        const float xy = x * y;
        const float xz = x * z;
        const float yz = y * z;

        Mat3 m{};
        m.row0[0] = ww + xx - yy - zz;
        m.row0[1] = 2.0F * (xy - wz);
        m.row0[2] = 2.0F * (xz + wy);
        m.row1[0] = 2.0F * (xy + wz);
        m.row1[1] = ww - xx + yy - zz;
        m.row1[2] = 2.0F * (yz - wx);
        m.row2[0] = 2.0F * (xz - wy);
        m.row2[1] = 2.0F * (yz + wx);
        m.row2[2] = ww - xx - yy + zz;
        return m;
    }

    // @req LLR-CORE-015
    Vec3 rotate(const Vec3& v) const noexcept {
        return to_rotation_matrix().multiply(v);
    }

    // @req LLR-CORE-016
    bool is_valid() const noexcept {
        return is_finite(w) && is_finite(x) && is_finite(y) && is_finite(z);
    }
};

enum class ResultCode : std::uint8_t {
    Ok = 0U,
    InvalidInput = 1U,
    NumericalFault = 2U,
    OutOfBounds = 3U,
};

template <typename T>
struct Result {
    T value{};
    ResultCode code{ResultCode::Ok};

    static Result ok(T val) noexcept {
        Result r{};
        r.value = val;
        r.code = ResultCode::Ok;
        return r;
    }

    static Result error(ResultCode err) noexcept {
        Result r{};
        r.code = err;
        return r;
    }

    bool ok() const noexcept { return code == ResultCode::Ok; }
};

}  // namespace core
}  // namespace flightsim
