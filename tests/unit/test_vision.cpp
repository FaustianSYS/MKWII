#include "minimal_test.hpp"

#include <vector>

#include "flightsim/engagement/missile/geometric_seeker_source.hpp"
#include "flightsim/engagement/missile/missile.hpp"
#include "flightsim/engagement/missile/missile_object.hpp"
#include "flightsim/engagement/scenario/air_defense_scenario.hpp"
#include "flightsim/vision/camera_model.hpp"
#include "flightsim/vision/seeker_image_processor.hpp"

int run_vision_tests() {
    using flightsim::core::Quaternion;
    using flightsim::core::Vec3;
    using flightsim::engagement::MissileAttributes;
    using flightsim::engagement::MissileObject;
    using flightsim::engagement::apply_seeker_track_to_missile;
    using flightsim::engagement::compute_geometric_seeker_track;
    using flightsim::engagement::default_missile_attributes;
    using flightsim::engagement::initialize_missile;
    using flightsim::engagement::step_missile;
    using flightsim::vision::CameraModel;
    using flightsim::vision::SeekerImageProcessor;
    using flightsim::vision::render_synthetic_drone_blob;

    {
        CameraModel camera{};
        camera.width_px = 640;
        camera.height_px = 480;
        camera.fov_azimuth_rad = 0.52F;
        camera.fov_elevation_rad = 0.52F;

        float az = 0.0F;
        float el = 0.0F;
        camera.pixel_to_bearing_rad(320.0F, 240.0F, az, el);
        REQUIRE_APPROX(az, 0.0F, 0.001F);
        REQUIRE_APPROX(el, 0.0F, 0.001F);

        camera.pixel_to_bearing_rad(640.0F, 240.0F, az, el);
        REQUIRE(az > 0.0F);

        float px = 0.0F;
        float py = 0.0F;
        REQUIRE(camera.bearing_rad_to_pixel(az, el, px, py));
        REQUIRE_APPROX(px, 640.0F, 1.0F);
        REQUIRE_APPROX(py, 240.0F, 1.0F);
    }

    {
        CameraModel camera{};
        camera.width_px = 640;
        camera.height_px = 480;
        camera.fov_azimuth_rad = 0.52F;
        camera.fov_elevation_rad = 0.52F;

        const Vec3 missile_pos{};
        const Vec3 target_pos{5000.0F, 0.0F, -100.0F};
        float px = 0.0F;
        float py = 0.0F;
        REQUIRE(camera.target_to_pixel(missile_pos, Quaternion::identity(), target_pos, px, py));
        REQUIRE(px > 300.0F);
        REQUIRE(px < 340.0F);
        REQUIRE(py > 220.0F);
        REQUIRE(py < 260.0F);
    }

    {
        std::vector<std::uint8_t> image(640U * 480U, 0U);
        render_synthetic_drone_blob(image.data(), 640, 480, 640, 360.0F, 220.0F, 8, 255U);

        CameraModel camera{};
        SeekerImageProcessor processor(camera);
        const auto detection = processor.detect(image.data(), 640, 480, 640);
        REQUIRE(detection.detected);
        REQUIRE(detection.centroid_x_px > 300.0F);
        REQUIRE(detection.centroid_y_px > 200.0F);

        const auto track = processor.track_from_detection(detection, Quaternion::identity(), 5000.0F, 1.0);
        REQUIRE(track.valid);
        REQUIRE(track.los_unit_ned.magnitude() > 0.9F);
    }

    {
        MissileObject missile{};
        MissileAttributes attrs = default_missile_attributes();
        initialize_missile(missile, attrs, Vec3{}, Vec3{200.0F, 0.0F, 0.0F});
        const Vec3 target_pos{5000.0F, 0.0F, -100.0F};

        const auto track = compute_geometric_seeker_track(missile, target_pos);
        REQUIRE(track.valid);
        REQUIRE(track.locked);
        apply_seeker_track_to_missile(missile, track);
        REQUIRE(missile.seeker_locked);
    }

    {
        MissileObject missile{};
        MissileAttributes attrs = default_missile_attributes();
        initialize_missile(missile, attrs, Vec3{}, Vec3{200.0F, 0.0F, 0.0F});
        const Vec3 target_pos{5000.0F, 500.0F, 0.0F};

        flightsim::vision::SeekerTrack biased_track = compute_geometric_seeker_track(missile, target_pos);
        biased_track.los_unit_ned = Vec3{1.0F, 0.0F, 0.0F};
        biased_track.valid = true;
        biased_track.locked = true;

        for (int i = 0; i < 200 && missile.active; ++i) {
            step_missile(missile, target_pos, Vec3{}, 0.01F, &biased_track);
        }
        REQUIRE(missile.velocity_ned_mps.y >= 0.0F);
    }

    {
        const auto config = flightsim::engagement::default_air_defense_config();
        REQUIRE_APPROX(config.missile_launch_position_ned_m.z, 0.0F, 0.01F);
        REQUIRE(config.target_start_position_ned_m.z < -50.0F);
        REQUIRE(config.target.speed_mps >= 15.0F);
    }

    return 0;
}
