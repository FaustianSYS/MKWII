#pragma once

#include <cstdio>
#include <cstdlib>
#include <cmath>

namespace flightsim_test {

inline int& failures() {
    static int count = 0;
    return count;
}

inline void require(bool condition, const char* expr, const char* file, int line) {
    if (!condition) {
        std::fprintf(stderr, "FAIL %s:%d: %s\n", file, line, expr);
        ++failures();
    }
}

inline void require_approx(float actual, float expected, float margin, const char* expr, const char* file, int line) {
    if (std::fabs(actual - expected) > margin) {
        std::fprintf(stderr, "FAIL %s:%d: %s (got %f expected %f +/- %f)\n",
                     file, line, expr, actual, expected, margin);
        ++failures();
    }
}

inline int run_tests(int (*suite)()) {
    failures() = 0;
    const int rc = suite();
    if (failures() > 0) {
        std::fprintf(stderr, "%d assertion(s) failed\n", failures());
        return rc != 0 ? rc : 1;
    }
    return rc;
}

}  // namespace flightsim_test

#define REQUIRE(expr) flightsim_test::require((expr), #expr, __FILE__, __LINE__)
#define REQUIRE_FALSE(expr) flightsim_test::require(!(expr), "!(" #expr ")", __FILE__, __LINE__)
#define REQUIRE_APPROX(actual, expected, margin) \
    flightsim_test::require_approx((actual), (expected), (margin), #actual, __FILE__, __LINE__)
