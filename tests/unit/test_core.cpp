#include "minimal_test.hpp"

#include "flightsim/core/atmosphere.hpp"
#include "flightsim/core/types.hpp"

int run_core_tests() {
    using flightsim::core::isa_density;
    using flightsim::core::kSeaLevelDensity;
    using flightsim::core::Quaternion;
    using flightsim::core::Vec3;

    {
        const Vec3 a{1.0F, 2.0F, 3.0F};
        const Vec3 b{4.0F, 5.0F, 6.0F};
        const Vec3 sum = a + b;
        REQUIRE(sum.x == 5.0F);
        REQUIRE(sum.dot(b) == 109.0F);
        REQUIRE(a.is_valid());
    }

    {
        Quaternion q{2.0F, 0.0F, 0.0F, 0.0F};
        q.normalize();
        REQUIRE_APPROX(q.w, 1.0F, 0.001F);
    }

    {
        REQUIRE_APPROX(isa_density(0.0F), kSeaLevelDensity, 0.01F);
    }

    return 0;
}
