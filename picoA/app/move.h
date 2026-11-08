#pragma once
// A move (doc/ARCHITECTURE.md, moves; doc/DATA_FLOW_PLAN.md): one action with its
// still time around it, the unit the PC stores. Its phases:
//   start  the motors switch on; the robot must stand still (at most 3 s, else
//          no_start). The PC gets "move start" (control lane) and, from now until
//          the end, every tof frame (bulk lane, the map use's format) and every
//          imu record from PicoB, 10 to an imu batch (bulk lane).
//   pre    it stands still pre_ms (default 200 ms)
//   act    the action (behaviour.h: drive, turn, go to, watch) until it says done
//   post   it stands still post_ms (default 200 ms), and until PicoB says still
//          (at most 3 s); this is the settling after an action, not on top of it
//   end    the motors go off; the last imu batch goes, however few records it has;
//          the PC gets "move end": how it ended, the frames and imu batches sent
//          and dropped (the bulk lane full, or no server). Each frame and batch
//          carries its move's number.
// A stop (or PicoB lost, or PicoB's safety stop) ends it at once, in any phase; a
// new move replaces the running one (the motors stay on). A move that can't start
// (blocked, already there) ends at once, with no phases and no frames, so its
// command is still answered. Nothing is sent while no move runs.
// An imu batch holds up to 10 records measured one after the other; a gap (a record
// lost on the way from PicoB) starts a new one, so a batch's records are evenly
// spaced (wire.h).
// One command is one move; a go to (turn, then drive) is one move; `robot steer
// r30 d50` is two commands, so two moves, each with its own still times.
// Only this module switches the motors for an action and turns the frames on and off.
#include <stdbool.h>
#include <stdint.h>
#include "link_msgs.h"
#include "tof_frame.h"
#include "wire.h"

#define MOVE_PRE_MS_DEFAULT  200
#define MOVE_POST_MS_DEFAULT 200

typedef enum { MOVE_DRIVE = WIRE_MOVE_DRIVE, MOVE_ROTATE = WIRE_MOVE_ROTATE, MOVE_GOTO = WIRE_MOVE_GOTO,
               MOVE_WATCH = WIRE_MOVE_WATCH } move_kind_t;

// How a move ended.
typedef enum {
    MOVE_END_DONE,         // the action finished
    MOVE_END_OBSTACLE,     // a drive or go to stopped before an obstacle on the map (or didn't start for one)
    MOVE_END_STOPPED,      // stop
    MOVE_END_REPLACED,     // another move started
    MOVE_END_SAFETY_STOP,  // PicoB switched the motors off (stop_reason says why)
    MOVE_END_PICOB_LOST,
    MOVE_END_NO_START,     // the motors didn't switch on, or the robot didn't stand still
} move_end_t;

// What the act phase does: the action's own code (behaviour.c gives it).
typedef enum { MOVE_ACTING, MOVE_ACT_DONE, MOVE_ACT_OBSTACLE } move_act_t;
typedef struct {
    void (*begin)(const odom_report_t *start); // the act phase starts (the robot stands still, motors on)
    move_act_t (*step)(const odom_report_t *o); // every loop while acting; it drives (body_drive)
    void (*end)(const odom_report_t *o, move_end_t how); // the move has ended, however; once
} move_actor_t;

// Starts a move: param is its parameter (drive: m, + = forward; rotate: rad, + = left;
// go to: m to the goal; watch: s); command, the number of its command. A running
// move ends first (replaced).
void move_start(move_kind_t kind, float param, uint32_t command, const move_actor_t *actor);
// A move that ends before it starts (how: obstacle, or done for a goal already
// reached): numbered, its start and end sent, nothing else; a running move goes on.
void move_refused(move_kind_t kind, float param, uint32_t command, move_end_t how);
void move_stop(void);   // ends the running move: the robot stands still, motors off
void move_update(void); // call every loop iteration, after the ToF pipeline
// Every posed frame (main.c): during a move it goes to the PC, counted.
void move_tof(const posed_frame_t *f);
// Every imu record from PicoB, oldest first, its time on PicoA's clock (main.c,
// body_imu_take()): during a move it goes to the PC in an imu batch, counted.
void move_imu(const imu_record_t *r);

// The still times (ms, each 0-MOVE_STILL_MAX_MS): from the next move on.
#define MOVE_STILL_MAX_MS 2000
bool move_set_still(int pre_ms, int post_ms); // false (nothing changed) if one is out of range
int move_pre_ms(void);
int move_post_ms(void);

// How a move ended.
typedef struct {
    uint32_t move;        // its number (from 1 at power-up)
    uint32_t command;     // the number of the command that started it
    move_kind_t kind;
    move_end_t how;
    uint8_t stop_reason;  // MOVE_END_SAFETY_STOP: PicoB's STOP_* (link_msgs.h)
    float moved_m;        // from where it began acting to where it ended, straight
    float rotated_rad;    // how far it turned, + = left (not wrapped)
    float goal_m;         // go to: how far the goal was when it started; 0 for the others
    uint32_t frames_sent, frames_dropped;
    uint32_t imu_sent, imu_dropped; // imu batches
} move_record_t;
// Counts the moves that ended; the last one's record (all zero before the first).
uint32_t move_endings(move_record_t *last);
// The oldest ended move not yet taken, oldest first (a refused move and the running
// one can end in the same loop: both are kept); false if none waits. The last
// MOVE_ENDED_KEPT are kept; older ones not taken are lost (counted in the console).
#define MOVE_ENDED_KEPT 8
bool move_next_ended(move_record_t *r);
// How a move ended as the PC names it (wire_end_t; the safety stops by their reason).
uint8_t move_wire_end(const move_record_t *r);
const char *move_kind_name(move_kind_t kind); // "drive", "rotate", "goto", "watch"
const char *move_running(void);               // the running move's kind name, "" if none
