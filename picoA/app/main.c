// PicoA robot firmware, the brain. Talks to PicoB over the link, maps the
// surroundings with the ToF sensor and watches for movement, sends snapshots of what
// it knows to the robot server. It does nothing by itself: the server's commands
// over WiFi (link_pc.h), or the same typed on USB, start each action
// (commands.h, doc/ARCHITECTURE.md); each action is a move (move.h). Both data
// pipelines are here: the ToF's (poll -> stamp -> pack and unpack per use -> the map,
// watch; packed for the PC during a move) and the IMU's (PicoB's imu records, on
// PicoA's clock -> batched for the PC during a move).
#include <stdio.h>
#include "pico/stdlib.h"
#include "clock_start.h"
#include "pico/stdio_usb.h"
#include "pico/rand.h"
#include "body.h"
#include "pose.h"
#include "tof.h"
#include "tof_rays.h"
#include "tof_format.h"
#include "cell_map.h"
#include "move.h"
#include "commands.h"
#include "link_pc.h"
#include "camera.h"
#include "watch.h"
#include "loop_stats.h"
#include "snapshot.h"
#include "opt4048.h"
#include "vision.h"
#include "robot_mounting.h" // ROBOT_MOUNTING, from common/mounting.json

#define USB_CHECK_US 3000000 // after power-up: time for a computer to enumerate the USB
#define COLOR_US     100000  // the colour sensor is asked this often (a reading comes ~2.5 times a second)
#define BUILD        __DATE__ " " __TIME__ // when the firmware was built (main.c's compile time), in hello and snapshots

static uint32_t boot_id; // random at power-up: names this boot on the PC

// Connects to WiFi once the USB had time to enumerate, with or without a computer
// on USB: both take the same commands. Then the robot waits, motors off.
static void start_up(absolute_time_t usb_check) {
    static bool started;
    if (!started && time_reached(usb_check)) {
        started = true;
        link_pc_start(boot_id, BUILD, commands_status);
    }
}

static void stage_done(const char *stage) { loop_stats_stage(stage, time_us_32()); }

// A posed frame as a use gets it: packed in its format and unpacked again, the same
// data the PC gets (doc/DATA_FLOW_PLAN.md). NULL if that failed (a format table bug):
// the use skips the frame.
static const posed_frame_t *as_used(tof_use_t use, const posed_frame_t *f) {
    static uint8_t bytes[TOF_FORMAT_MAX];
    static posed_frame_t unpacked;
    uint32_t n_bytes = tof_pack(use, f, bytes);
    return tof_unpack(tof_format_of(use), bytes, n_bytes, &unpacked) ? &unpacked : NULL;
}

// The ToF pipeline: each new frame, stamped with where the robot was, to the PC
// during a move, into the snapshot, the map and watch. Static: ~1.6 KB, off the
// main loop's stack.
static void tof_pipeline(void) {
    static tof_frame_t frame;
    static posed_frame_t posed;
    if (!tof_poll(&frame)) return;
    pose_stamp(&frame, &posed);
    move_tof(&posed);      // during a move: to the PC (bulk lane, the map use's format)
    snapshot_tof(&posed);  // the snapshot use
    const posed_frame_t *used = as_used(TOF_USE_MAP, &posed);
    if (used) cell_map_add(used);
    used = as_used(TOF_USE_WATCH, &posed);
    if (used) watch_add(used);
}

// The IMU pipeline: each imu record from PicoB (body turned its time into PicoA's)
// to the move, which batches them for the PC while one runs.
static void imu_pipeline(void) {
    imu_record_t r;
    while (body_imu_take(&r)) move_imu(&r);
}

int main(void) {
    clock_start();
    stdio_init_all();
    boot_id = get_rand_32();
    snapshot_init(boot_id, BUILD);
    absolute_time_t usb_check = make_timeout_time_us(USB_CHECK_US);
    body_init();
    tof_rays_init(&ROBOT_MOUNTING); // before anything that uses the ToF's rays
    bool camera_watch_ok = watch_init(&ROBOT_MOUNTING);
    cell_map_clear();
    bool tof_ok = tof_init(&ROBOT_MOUNTING.tof, TOF_DEFAULT_HZ); // ~2 s: uploads the ToF sensor's firmware
    bool camera_ok = camera_init();
    bool color_ok = opt4048_init();
    absolute_time_t next_color = get_absolute_time();
    for (;;) {
        loop_stats_begin(time_us_32());
        start_up(usb_check);
        body_update();
        pose_update();
        imu_pipeline();
        stage_done("body");
        tof_pipeline();
        stage_done("tof");
        camera_update();
        stage_done("camera");
        watch_update();
        stage_done("watch");
        vision_update();
        stage_done("vision");
        move_update();
        stage_done("move");
        if (color_ok && time_reached(next_color)) {
            next_color = make_timeout_time_us(COLOR_US);
            opt4048_poll();
        }
        snapshot_update();
        stage_done("snapshot");
        link_pc_update();
        stage_done("wifi");
        commands_update();
        stage_done("commands");
        static bool warned;
        if (!warned && (stdio_usb_connected() || link_pc_connected())) {
            warned = true;
            if (!tof_ok) printf("ToF sensor not working: no map\n");
            if (!camera_ok) printf("Camera not working\n");
            if (!camera_watch_ok) printf("Camera's mounting wrong (common/mounting.json): no camera movement\n");
            if (!color_ok) printf("Colour sensor not working\n");
        }
    }
}
