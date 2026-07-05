#include "minimal_test.hpp"

int run_integration_tests();

int main() {
    return flightsim_test::run_tests(run_integration_tests);
}
