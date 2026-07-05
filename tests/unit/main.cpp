#include "minimal_test.hpp"

int run_core_tests();
int run_safety_tests();
int run_fdm_tests();
int run_engagement_tests();
int run_vision_tests();

int main() {
    int rc = 0;
    rc |= flightsim_test::run_tests(run_core_tests);
    rc |= flightsim_test::run_tests(run_safety_tests);
    rc |= flightsim_test::run_tests(run_fdm_tests);
    rc |= flightsim_test::run_tests(run_engagement_tests);
#ifdef FLIGHTSIM_HAS_VISION
    rc |= flightsim_test::run_tests(run_vision_tests);
#endif
    return rc;
}
