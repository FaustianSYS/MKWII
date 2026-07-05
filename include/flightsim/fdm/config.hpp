#pragma once

namespace flightsim {
namespace fdm {

struct AircraftConfig {
    float wing_area_m2{16.0F};
    float wing_span_m{10.0F};
    float max_thrust_n{8000.0F};
    float reference_chord_m{1.5F};
};

}  // namespace fdm
}  // namespace flightsim
