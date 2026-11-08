#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "clock_start.h"
#include "body.h"
#include "link_pc.h"
#include "move.h"

#define START_TIMEOUT_US 3000000 // the motors switching on and the robot standing still
#define POST_MAX_US      3000000 // post: PicoB not saying "still" after this long: it ends anyway

static const char *const kind_name[] = {"drive", "rotate", "goto", "watch"};  // in the data: status, snapshots, the PC
static const char *const print_name[] = {"Drive", "Rotate", "Go to", "Watch"}; // in the console

typedef struct {
    uint32_t number, command;
    move_kind_t kind;
    float param;
    bool acted;                 // the act phase began (start is set)
    bool obstacle;              // the act phase ended before an obstacle
    odom_report_t start;        // the robot when the act phase began
    uint32_t sent, dropped;     // its frames
    uint32_t imu_sent, imu_dropped;      // its imu batches
} move_t;

static enum { IDLE, STARTING, PRE, ACT, POST } phase;
static int pre_ms = MOVE_PRE_MS_DEFAULT, post_ms = MOVE_POST_MS_DEFAULT;
static move_t cur;              // the running move (phase != IDLE)
static wire_imu_batch_t batch;  // the running move's imu records not sent yet
static const move_actor_t *running_actor;
static absolute_time_t deadline, still_until;
static uint32_t numbered;       // moves numbered since power-up
static uint32_t endings;        // moves that ended, and the last one's record
static move_record_t last;
static move_record_t ended_queue[MOVE_ENDED_KEPT]; // ended, not yet taken: endings - taken of them
static uint32_t taken;

const char *move_kind_name(move_kind_t k) { return (unsigned)k < WIRE_MOVE_KINDS ? kind_name[k] : "?"; }
const char *move_running(void) { return phase == IDLE ? "" : kind_name[cur.kind]; }
int move_pre_ms(void) { return pre_ms; }
int move_post_ms(void) { return post_ms; }

bool move_set_still(int pre, int post) {
    if (pre < 0 || pre > MOVE_STILL_MAX_MS || post < 0 || post > MOVE_STILL_MAX_MS) return false;
    pre_ms = pre;
    post_ms = post;
    return true;
}

uint32_t move_endings(move_record_t *out) {
    if (out) *out = last;
    return endings;
}

bool move_next_ended(move_record_t *r) {
    if (endings - taken > MOVE_ENDED_KEPT) {
        printf("Moves: %lu ended moves not answered (more than %d at once)\n", (unsigned long)(endings - taken - MOVE_ENDED_KEPT),
               MOVE_ENDED_KEPT);
        taken = endings - MOVE_ENDED_KEPT;
    }
    if (taken == endings) return false;
    *r = ended_queue[taken++ % MOVE_ENDED_KEPT];
    return true;
}

static wire_end_t wire_end_of(move_end_t how, uint8_t stop_reason) {
    switch (how) {
    case MOVE_END_DONE: return WIRE_END_OK;
    case MOVE_END_OBSTACLE: return WIRE_END_OBSTACLE;
    case MOVE_END_STOPPED: return WIRE_END_STOPPED;
    case MOVE_END_REPLACED: return WIRE_END_REPLACED;
    case MOVE_END_PICOB_LOST: return WIRE_END_PICOB_LOST;
    case MOVE_END_NO_START: return WIRE_END_NO_START;
    case MOVE_END_SAFETY_STOP:
        switch (stop_reason) {
        case STOP_TILT: return WIRE_END_TILT;
        case STOP_LEFT_WHEELS: case STOP_RIGHT_WHEELS: return WIRE_END_WHEELS;
        case STOP_NO_DRIVE: return WIRE_END_NO_DRIVE;
        default: return WIRE_END_SAFETY;
        }
    }
    return WIRE_END_ENDED;
}

uint8_t move_wire_end(const move_record_t *r) { return (uint8_t)wire_end_of(r->how, r->stop_reason); }

// A new move, numbered; the PC told (refused: it ends at once, a running move goes on).
static move_t numbered_move(move_kind_t k, float p, uint32_t c, bool refused) {
    move_t m = {.number = ++numbered, .command = c, .kind = k, .param = p};
    wire_move_start_t s = {.move = m.number, .command = c, .kind = (uint8_t)k, .param = p, .t_us = clock_since_start_us(),
                           .pre_ms = (uint16_t)pre_ms, .post_ms = (uint16_t)post_ms, .flags = refused ? WIRE_MOVE_REFUSED : 0};
    uint8_t msg[WIRE_HEADER + WIRE_MOVE_START_SIZE];
    uint32_t n = wire_encode_move_start(&s, msg);
    link_send_control(msg, n);
    return m;
}

// Move m has ended: its record, and the PC told.
static void ended(const move_t *m, move_end_t how) {
    const odom_report_t *o = body_odom();
    uint8_t stop = how == MOVE_END_SAFETY_STOP ? o->stop_reason : STOP_NONE;
    endings++;
    last = (move_record_t){
        .move = m->number, .command = m->command, .kind = m->kind, .how = how, .stop_reason = stop,
        .moved_m = m->acted ? hypotf(o->x_m - m->start.x_m, o->y_m - m->start.y_m) : 0,
        .rotated_rad = m->acted ? o->yaw_rad - m->start.yaw_rad : 0,
        .goal_m = m->kind == MOVE_GOTO ? m->param : 0,
        .frames_sent = m->sent, .frames_dropped = m->dropped, .imu_sent = m->imu_sent, .imu_dropped = m->imu_dropped,
    };
    ended_queue[(endings - 1) % MOVE_ENDED_KEPT] = last;
    wire_move_end_t e = {.move = m->number, .end = move_wire_end(&last), .t_us = clock_since_start_us(), .frames_sent = m->sent,
                         .frames_dropped = m->dropped, .moved_m = last.moved_m, .rotated_rad = last.rotated_rad, .imu_sent = m->imu_sent,
                         .imu_dropped = m->imu_dropped};
    uint8_t msg[WIRE_HEADER + WIRE_MOVE_END_SIZE];
    uint32_t n = wire_encode_move_end(&e, msg);
    link_send_control(msg, n);
}

// The imu records waiting, as one batch to the bulk lane; counted.
static void send_batch(void) {
    if (!batch.n) return;
    static uint8_t msg[WIRE_HEADER + WIRE_IMU_BATCH_HEADER + WIRE_IMU_BATCH_MAX * WIRE_IMU_RECORD];
    // Evenly spaced (move_imu() starts a new batch at a gap): the period is the first
    // to the last record's time over n - 1, rounded; the nominal one for one record.
    uint32_t span = batch.records[batch.n - 1].t_us - batch.records[0].t_us;
    batch.move = cur.number;
    batch.t_us = clock_since_start_of(batch.records[0].t_us);
    batch.period_us = (uint16_t)(batch.n > 1 ? (span + (batch.n - 1) / 2u) / (batch.n - 1u) : IMU_RECORD_PERIOD_US);
    uint32_t n = wire_encode_imu_batch(&batch, msg);
    if (link_send_bulk(msg, n)) cur.imu_sent++;
    else cur.imu_dropped++;
    batch.n = 0;
}

// The running move ends: its last imu batch, the action's end (it prints how it went),
// then the record and the PC told.
static void finish(move_end_t how) {
    send_batch();
    phase = IDLE;
    running_actor->end(body_odom(), how);
    ended(&cur, how);
    printf("%s %lu: %lu ToF frames sent, %lu dropped; %lu imu batches sent, %lu dropped\n", print_name[cur.kind],
           (unsigned long)cur.number, (unsigned long)cur.sent, (unsigned long)cur.dropped, (unsigned long)cur.imu_sent,
           (unsigned long)cur.imu_dropped);
}

// Ends the running move at once (any phase), saying why; the motors go off unless
// another move follows.
static void end_now(move_end_t how, const char *why, bool motors_off) {
    if (phase == IDLE) return;
    body_drive(0, 0);
    if (motors_off) body_motors(false);
    if (why) printf("%s stopped: %s\n", print_name[cur.kind], why);
    finish(how);
}

void move_start(move_kind_t k, float p, uint32_t c, const move_actor_t *a) {
    end_now(MOVE_END_REPLACED, "another action started", false);
    cur = numbered_move(k, p, c, false);
    running_actor = a;
    body_drive(0, 0);
    body_motors(true);
    phase = STARTING;
    deadline = make_timeout_time_us(START_TIMEOUT_US);
}

void move_refused(move_kind_t k, float p, uint32_t c, move_end_t how) {
    move_t m = numbered_move(k, p, c, true);
    ended(&m, how);
}

void move_stop(void) { end_now(MOVE_END_STOPPED, "by command", true); }

void move_tof(const posed_frame_t *f) {
    if (phase == IDLE) return;
    static uint8_t msg[WIRE_HEADER + WIRE_TOF_HEADER + TOF_FORMAT_MAX];
    uint32_t n = wire_encode_tof(TOF_USE_MAP, cur.number, clock_since_start_of(f->frame.t_us), f, msg);
    if (link_send_bulk(msg, n)) cur.sent++;
    else cur.dropped++;
}

void move_imu(const imu_record_t *r) {
    if (phase == IDLE) return;
    if (batch.n) { // not the next record after the last (one lost on the way): a new batch
        int32_t gap = (int32_t)(r->t_us - batch.records[batch.n - 1].t_us);
        if (gap < IMU_RECORD_PERIOD_US / 2 || gap > IMU_RECORD_PERIOD_US * 3 / 2) send_batch();
    }
    batch.records[batch.n++] = *r;
    if (batch.n == WIRE_IMU_BATCH_MAX) send_batch();
}

void move_update(void) {
    if (phase == IDLE) return;
    const odom_report_t *o = body_odom();
    if (!body_connected()) { end_now(MOVE_END_PICOB_LOST, "PicoB not connected", true); return; }
    if (phase != STARTING && !o->motors_on) {
        printf("%s stopped: PicoB switched the motors off (%s)\n", print_name[cur.kind], stop_reason_text(o->stop_reason));
        end_now(MOVE_END_SAFETY_STOP, NULL, false);
        return;
    }
    switch (phase) {
    case STARTING:
        if (!(o->motors_on && o->stationary)) {
            if (time_reached(deadline))
                end_now(MOVE_END_NO_START, o->motors_on ? "the robot isn't standing still" : "the motors didn't switch on", true);
            break;
        }
        phase = PRE;
        still_until = make_timeout_time_us((uint64_t)pre_ms * 1000);
        // fall through - a pre of 0 ms acts at once
    case PRE:
        if (!time_reached(still_until)) break;
        cur.start = *o;
        cur.acted = true;
        phase = ACT;
        running_actor->begin(o);
        // fall through - the first step at once
    case ACT: {
        move_act_t r = running_actor->step(o);
        if (r == MOVE_ACTING) break;
        body_drive(0, 0);
        cur.obstacle = r == MOVE_ACT_OBSTACLE;
        phase = POST;
        still_until = make_timeout_time_us((uint64_t)post_ms * 1000);
        deadline = make_timeout_time_us(POST_MAX_US);
        break;
    }
    case POST:
        if ((time_reached(still_until) && o->stationary) || time_reached(deadline)) {
            body_motors(false);
            finish(cur.obstacle ? MOVE_END_OBSTACLE : MOVE_END_DONE);
        }
        break;
    case IDLE:
        break;
    }
}
