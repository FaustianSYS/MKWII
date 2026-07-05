/**
 * fsros2_bridge.cpp
 * Compiled with the system toolchain (g++ / clang system libc++, full RTTI).
 * Wraps rclcpp and exposes a plain-C API so UE5's stripped libc++ never sees
 * rclcpp templates, typeid, dynamic_cast, or std::get_deleter.
 */

#include "fsros2_bridge.h"

#include <atomic>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <flightsim_msgs/msg/scene_state.hpp>
#include <flightsim_msgs/msg/missile_state.hpp>
#include <flightsim_msgs/msg/engagement_status.hpp>

// ---------------------------------------------------------------------------
// Internal bridge object
// ---------------------------------------------------------------------------

struct Bridge
{
    std::shared_ptr<rclcpp::Node> node;

    rclcpp::Subscription<flightsim_msgs::msg::SceneState>::SharedPtr scene_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr             image_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr        cam_info_pub;

    FsRos2SceneStateCb    scene_cb      = nullptr;
    void*                 scene_ud      = nullptr;

    FsRos2MissileStateCb  missile_cb    = nullptr;
    void*                 missile_ud    = nullptr;

    FsRos2EngagementStatusCb engagement_cb = nullptr;
    void*                    engagement_ud = nullptr;

    rclcpp::Subscription<flightsim_msgs::msg::MissileState>::SharedPtr     missile_sub;
    rclcpp::Subscription<flightsim_msgs::msg::EngagementStatus>::SharedPtr engagement_sub;

    std::atomic<bool>  stop_spin{false};
    std::thread        spin_thread;

    explicit Bridge(const char* node_name)
        : node(std::make_shared<rclcpp::Node>(node_name))
    {
        image_pub = node->create_publisher<sensor_msgs::msg::Image>(
            "/flightsim/seeker_camera/image",
            rclcpp::QoS(1).best_effort());

        cam_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/flightsim/seeker_camera/camera_info",
            rclcpp::QoS(1).best_effort());

        // Spin thread: 1 ms poll, keeps CPU usage low.
        spin_thread = std::thread([this]()
        {
            while (!stop_spin.load(std::memory_order_relaxed) && rclcpp::ok())
            {
                rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
    }

    ~Bridge()
    {
        stop_spin.store(true, std::memory_order_relaxed);
        if (spin_thread.joinable()) spin_thread.join();
    }

    void subscribe_missile_state()
    {
        missile_sub = node->create_subscription<flightsim_msgs::msg::MissileState>(
            "/flightsim/missile_state",
            rclcpp::QoS(10),
            [this](flightsim_msgs::msg::MissileState::SharedPtr msg)
            {
                if (!missile_cb) return;
                FsRos2MissileState s{};
                for (int i = 0; i < 3; ++i) {
                    s.position_ned_m[i]  = msg->position_ned_m[i];
                    s.velocity_ned_mps[i] = msg->velocity_ned_mps[i];
                }
                s.attitude_wxyz[0] = msg->attitude_w;
                s.attitude_wxyz[1] = msg->attitude_x;
                s.attitude_wxyz[2] = msg->attitude_y;
                s.attitude_wxyz[3] = msg->attitude_z;
                s.thrust_n        = msg->thrust_n;
                s.length_m        = msg->length_m;
                s.fin_pitch_rad   = msg->fin_pitch_rad;
                s.fin_yaw_rad     = msg->fin_yaw_rad;
                s.seeker_range_m  = msg->seeker_range_m;
                s.active          = msg->active  ? 1u : 0u;
                s.hit             = msg->hit      ? 1u : 0u;
                s.seeker_locked   = msg->seeker_locked ? 1u : 0u;
                missile_cb(&s, missile_ud);
            });
    }

    void subscribe_engagement_status()
    {
        engagement_sub = node->create_subscription<flightsim_msgs::msg::EngagementStatus>(
            "/flightsim/engagement_status",
            rclcpp::QoS(10),
            [this](flightsim_msgs::msg::EngagementStatus::SharedPtr msg)
            {
                if (!engagement_cb) return;
                FsRos2EngagementStatus s{};
                s.range_m        = msg->range_m;
                s.miss_distance_m = msg->miss_distance_m;
                s.step_count     = msg->step_count;
                s.intercept      = msg->intercept ? 1u : 0u;
                s.complete       = msg->complete  ? 1u : 0u;
                engagement_cb(&s, engagement_ud);
            });
    }

    void subscribe_scene_state()
    {
        scene_sub = node->create_subscription<flightsim_msgs::msg::SceneState>(
            "/flightsim/scene_state",
            rclcpp::QoS(10),
            [this](flightsim_msgs::msg::SceneState::SharedPtr msg)
            {
                if (!scene_cb) return;

                // Convert to flat C structs on the stack; avoid heap allocation.
                const int n = static_cast<int>(msg->entities.size());
                std::vector<FsRos2Entity> ents(static_cast<std::size_t>(n));

                for (int i = 0; i < n; ++i)
                {
                    const auto& e = msg->entities[static_cast<std::size_t>(i)];
                    FsRos2Entity& out = ents[static_cast<std::size_t>(i)];

                    std::strncpy(out.id,   e.id.c_str(),    sizeof(out.id)   - 1);
                    std::strncpy(out.type, e.type.c_str(),  sizeof(out.type) - 1);
                    out.id  [sizeof(out.id)   - 1] = '\0';
                    out.type[sizeof(out.type) - 1] = '\0';

                    out.position_ned_m[0] = e.position_ned_m[0];
                    out.position_ned_m[1] = e.position_ned_m[1];
                    out.position_ned_m[2] = e.position_ned_m[2];

                    out.attitude_wxyz[0] = e.attitude_wxyz[0];
                    out.attitude_wxyz[1] = e.attitude_wxyz[1];
                    out.attitude_wxyz[2] = e.attitude_wxyz[2];
                    out.attitude_wxyz[3] = e.attitude_wxyz[3];
                }

                scene_cb(ents.data(), n, msg->sim_step, scene_ud);
            });
    }
};

// ---------------------------------------------------------------------------
// C API implementation
// ---------------------------------------------------------------------------

void fsros2_global_init(void)
{
    if (!rclcpp::ok())
    {
        int    argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
    }
}

void fsros2_global_shutdown(void)
{
    if (rclcpp::ok()) rclcpp::shutdown();
}

void* fsros2_create_bridge(const char* node_name)
{
    try { return new Bridge(node_name ? node_name : "flightsim_ue5_node"); }
    catch (...) { return nullptr; }
}

void fsros2_destroy_bridge(void* handle)
{
    delete static_cast<Bridge*>(handle);
}

void fsros2_set_scene_state_cb(void* handle, FsRos2SceneStateCb cb, void* userdata)
{
    Bridge* b = static_cast<Bridge*>(handle);
    if (!b) return;
    b->scene_cb = cb;
    b->scene_ud = userdata;
    b->subscribe_scene_state();
}

void fsros2_set_missile_state_cb(void* handle, FsRos2MissileStateCb cb, void* userdata)
{
    Bridge* b = static_cast<Bridge*>(handle);
    if (!b) return;
    b->missile_cb = cb;
    b->missile_ud = userdata;
    b->subscribe_missile_state();
}

void fsros2_set_engagement_status_cb(void* handle, FsRos2EngagementStatusCb cb, void* userdata)
{
    Bridge* b = static_cast<Bridge*>(handle);
    if (!b) return;
    b->engagement_cb = cb;
    b->engagement_ud = userdata;
    b->subscribe_engagement_status();
}

int fsros2_publish_image(void* handle, int width, int height, const uint8_t* data)
{
    Bridge* b = static_cast<Bridge*>(handle);
    if (!b || !data) return -1;

    sensor_msgs::msg::Image msg;
    msg.header.stamp    = b->node->now();
    msg.header.frame_id = "seeker_camera";
    msg.width           = static_cast<uint32_t>(width);
    msg.height          = static_cast<uint32_t>(height);
    msg.encoding        = "mono8";
    msg.is_bigendian    = 0;
    msg.step            = static_cast<uint32_t>(width);
    msg.data.assign(data, data + static_cast<std::size_t>(width) * height);

    b->image_pub->publish(msg);
    return 0;
}

int fsros2_publish_camera_info(void* handle, int width, int height, float hfov_deg)
{
    Bridge* b = static_cast<Bridge*>(handle);
    if (!b) return -1;

    const double W   = static_cast<double>(width);
    const double H   = static_cast<double>(height);
    const double hfov_rad = static_cast<double>(hfov_deg) * (M_PI / 180.0);
    const double fx  = (W * 0.5) / std::tan(hfov_rad * 0.5);

    sensor_msgs::msg::CameraInfo info;
    info.header.stamp    = b->node->now();
    info.header.frame_id = "seeker_camera";
    info.width           = static_cast<uint32_t>(W);
    info.height          = static_cast<uint32_t>(H);
    info.distortion_model = "plumb_bob";
    info.d  = {0.0, 0.0, 0.0, 0.0, 0.0};
    info.k  = {fx,  0.0, W*0.5,
               0.0, fx,  H*0.5,
               0.0, 0.0, 1.0};
    info.r  = {1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0};
    info.p  = {fx,  0.0, W*0.5, 0.0,
               0.0, fx,  H*0.5, 0.0,
               0.0, 0.0, 1.0,   0.0};

    b->cam_info_pub->publish(info);
    return 0;
}
