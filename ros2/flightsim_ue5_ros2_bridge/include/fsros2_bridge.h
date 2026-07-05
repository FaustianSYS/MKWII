/**
 * fsros2_bridge.h — plain C interface to rclcpp for the UE5 plugin.
 *
 * Compiled by the system toolchain (full RTTI, libstdc++).
 * UE5 links against libflightsim_ue5_ros2_bridge.so and sees ONLY this header —
 * no rclcpp, no STL templates, no RTTI.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* --------------------------------------------------------------------------
 * Entity snapshot delivered via callback from the spin thread.
 * All strings are null-terminated, buffers are fixed-size.
 * -------------------------------------------------------------------------- */
typedef struct
{
    char    id[64];
    char    type[32];           /* "missile" | "target" | "depot" */
    float   position_ned_m[3];
    float   attitude_wxyz[4];   /* w, x, y, z */
} FsRos2Entity;

typedef void (*FsRos2SceneStateCb)(const FsRos2Entity* entities,
                                   int                 count,
                                   uint64_t            sim_step,
                                   void*               userdata);

/* --------------------------------------------------------------------------
 * Missile telemetry snapshot (from /flightsim/missile_state).
 * -------------------------------------------------------------------------- */
typedef struct
{
    float    position_ned_m[3];
    float    velocity_ned_mps[3];
    float    attitude_wxyz[4];
    float    thrust_n;
    float    length_m;
    float    fin_pitch_rad;
    float    fin_yaw_rad;
    float    seeker_range_m;
    uint8_t  active;
    uint8_t  hit;
    uint8_t  seeker_locked;
} FsRos2MissileState;

typedef void (*FsRos2MissileStateCb)(const FsRos2MissileState* state, void* userdata);

/* --------------------------------------------------------------------------
 * Engagement status snapshot (from /flightsim/engagement_status).
 * -------------------------------------------------------------------------- */
typedef struct
{
    float    range_m;
    float    miss_distance_m;
    uint64_t step_count;
    uint8_t  intercept;
    uint8_t  complete;
} FsRos2EngagementStatus;

typedef void (*FsRos2EngagementStatusCb)(const FsRos2EngagementStatus* status, void* userdata);

/* --------------------------------------------------------------------------
 * Lifecycle
 * -------------------------------------------------------------------------- */

/** Must be called once before any other function (calls rclcpp::init). */
void fsros2_global_init(void);

/** Calls rclcpp::shutdown.  Call after all bridges are destroyed. */
void fsros2_global_shutdown(void);

/* --------------------------------------------------------------------------
 * Bridge handle — one per UE5 world.
 * Creates a rclcpp node and starts its own 1 ms spin thread internally.
 * -------------------------------------------------------------------------- */
void* fsros2_create_bridge(const char* node_name);
void  fsros2_destroy_bridge(void* handle);

/** Register the callback that fires on every incoming SceneState message. */
void fsros2_set_scene_state_cb(void*              handle,
                                FsRos2SceneStateCb cb,
                                void*              userdata);

/** Register the callback for /flightsim/missile_state. */
void fsros2_set_missile_state_cb(void*                handle,
                                  FsRos2MissileStateCb cb,
                                  void*                userdata);

/** Register the callback for /flightsim/engagement_status. */
void fsros2_set_engagement_status_cb(void*                    handle,
                                      FsRos2EngagementStatusCb cb,
                                      void*                    userdata);

/* --------------------------------------------------------------------------
 * Publishing
 * -------------------------------------------------------------------------- */

/**
 * Publish a grayscale (mono8) seeker camera frame.
 * @param data  row-major, width*height bytes.
 * @returns 0 on success, -1 if bridge is invalid.
 */
int fsros2_publish_image(void*          handle,
                          int            width,
                          int            height,
                          const uint8_t* mono8_data);

/**
 * Publish a CameraInfo message (pinhole, no distortion).
 * @param hfov_deg  horizontal field-of-view in degrees.
 */
int fsros2_publish_camera_info(void*  handle,
                                int    width,
                                int    height,
                                float  hfov_deg);

#ifdef __cplusplus
}
#endif
