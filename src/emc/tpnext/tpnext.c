/********************************************************************
* Description: tpnext.c
*   Path parameter trajectory planner, loaded with [TRAJ]TPMOD=tpnextmod.
*
*   The queued moves form one path parameterised by s, the RS274NGC feed
*   length. Where two moves blend, the corner is replaced by a quintic
*   inside the G64 P and E tolerances. Every piece of the path (a blend or the
*   unblended interior of a move) gets a velocity cap and tangential
*   acceleration and jerk limits projected from the per axis limits, so
*   no axis can exceed its own limits whatever the path direction.
*
*   Every servo cycle a jerk limited controller picks the largest jerk
*   that still lets it meet every speed cap and stop ahead of it. Caps
*   beyond the braking horizon are folded into a backward envelope.
*   Feed override changes the requested speed only; the controller
*   follows it with the same jerk limits.
*
*   This file holds the module interface and the queue bookkeeping;
*   tpn_plan.c builds the queue, tpn_run.c is the per cycle controller,
*   tpn_sync.c follows the spindle for G33 and G33.1 and tpn_geom.c
*   holds the geometry.
*
* License: GPL Version 2
* System: Linux
********************************************************************/
#include <rtapi.h>
#include <rtapi_app.h>
#include <rtapi_math.h>
#include <hal.h>
#include <posemath.h>
#include <emcpose.h>
#include <motion_types.h>
#include "../motion/motion.h"
#include "../tp/tp.h"
#include "tpn.h"

MODULE_LICENSE("GPL");

#define TPN_QMARGIN 20

tpn_state tpn = {
    .lin_mask = 0707,
    .ang_mask = 0070,
};

static void (*_DioWrite)(int, char);
static void (*_AioWrite)(int, double);
static void (*_SetRotaryUnlock)(int, int);
static int (*_GetRotaryIsUnlocked)(int);

void tpMotFunctions(void (*pDioWrite)(int, char)
                   ,void (*pAioWrite)(int, double)
                   ,void (*pSetRotaryUnlock)(int, int)
                   ,int (*pGetRotaryIsUnlocked)(int)
                   ,double (*paxis_get_vel_limit)(int)
                   ,double (*paxis_get_acc_limit)(int)
                   ,double (*paxis_get_jerk_limit)(int)
                   ,int (*paxis_is_angular)(int)
                   )
{
    _DioWrite = pDioWrite;
    _AioWrite = pAioWrite;
    _SetRotaryUnlock = pSetRotaryUnlock;
    _GetRotaryIsUnlocked = pGetRotaryIsUnlocked;
    tpn.axis_get_vel_limit = paxis_get_vel_limit;
    tpn.axis_get_acc_limit = paxis_get_acc_limit;
    tpn.axis_get_jerk_limit = paxis_get_jerk_limit;
    tpn.axis_is_angular = paxis_is_angular;
}

void tpMotData(emcmot_status_t *pstatus, emcmot_config_t *pconfig)
{
    tpn.emcmotStatus = pstatus;
    tpn.emcmotConfig = pconfig;
}

/* ------------------------------------------------------------ public API */

int tpCreate(TP_STRUCT * const tp, int _queueSize, int id)
{
    (void)_queueSize;
    (void)id;
    if (!tp) {
        return TP_ERR_FAIL;
    }
    tp->queueSize = TPN_QSIZE;
    return tpInit(tp);
}

int tpClearDIOs(TP_STRUCT * const tp)
{
    int i;
    tp->syncdio.anychanged = 0;
    tp->syncdio.dio_mask = 0;
    tp->syncdio.aio_mask = 0;
    for (i = 0; i < tpn.emcmotConfig->numDIO; i++) {
        tp->syncdio.dios[i] = 0;
    }
    for (i = 0; i < tpn.emcmotConfig->numAIO; i++) {
        tp->syncdio.aios[i] = 0;
    }
    return TP_ERR_OK;
}

static void statusIdle(TP_STRUCT * const tp)
{
    tpn.emcmotStatus->distance_to_go = 0;
    tpn.emcmotStatus->enables_queued = tpn.emcmotStatus->enables_new;
    tpn.emcmotStatus->requested_vel = 0;
    tpn.emcmotStatus->current_vel = 0;
    tpn.emcmotStatus->current_acc = 0;
    tpn.emcmotStatus->current_jerk = 0;
    tpn.emcmotStatus->spindleSync = 0;
    tpn.emcmotStatus->current_dir.x = 0;
    tpn.emcmotStatus->current_dir.y = 0;
    tpn.emcmotStatus->current_dir.z = 0;
    tpn.emcmotStatus->tcqlen = 0;
    ZERO_EMC_POSE(tpn.emcmotStatus->dtg);
    tp->motionType = 0;
    tp->activeDepth = 0;
}

static void queueReset(TP_STRUCT * const tp)
{
    tpn.q_start = 0;
    tpn.q_len = 0;
    tpn.cur_s = tpn.cur_v = tpn.cur_a = tpn.cur_j = 0.0;
    tpnSyncReset();
    tpnRunReset();
    tp->goalPos = tp->currentPos;
    tp->done = 1;
    tp->depth = tp->activeDepth = 0;
    tp->aborting = 0;
    tp->pausing = 0;
    tp->execId = 0;
    tp->motionType = 0;
    tp->queue._len = 0;
}

int tpClear(TP_STRUCT * const tp)
{
    queueReset(tp);
    tp->nextId = 0;
    struct state_tag_t tag = {};
    tp->execTag = tag;
    tp->reverse_run = TC_DIR_FORWARD;
    tp->synchronized = 0;
    tp->uu_per_rev = 0.0;
    statusIdle(tp);
    tpn.emcmotStatus->motionFlag |= EMCMOT_MOTION_INPOS_BIT;
    return tpClearDIOs(tp);
}

int tpInit(TP_STRUCT * const tp)
{
    if (!tpn.emcmotStatus) {
        rtapi_print("tpnext: tpInit without motion data\n");
        return -1;
    }
    tp->cycleTime = 0.0;
    tp->vLimit = 0.0;
    tp->ini_maxvel = 0.0;
    tp->ini_maxjerk = 0.0;
    tp->aLimit = 0.0;
    tp->aMax = 0.0;
    tp->vMax = 0.0;
    tp->wMax = 0.0;
    tp->wDotMax = 0.0;
    tp->spindle.offset = 0.0;
    tp->spindle.revs = 0.0;
    tp->spindle.waiting_for_index = MOTION_INVALID_ID;
    tp->spindle.waiting_for_atspeed = MOTION_INVALID_ID;
    tp->reverse_run = TC_DIR_FORWARD;
    tp->termCond = TC_TERM_COND_PARABOLIC;
    tp->tolerance = 0.0;
    tp->queue.size = TPN_QSIZE;
    ZERO_EMC_POSE(tp->currentPos);
    return tpClear(tp);
}

int tpSetCycleTime(TP_STRUCT * const tp, double secs)
{
    if (!tp || secs <= 0.0) {
        return TP_ERR_FAIL;
    }
    tp->cycleTime = secs;
    return TP_ERR_OK;
}

int tpSetVmax(TP_STRUCT * const tp, double vMax, double ini_maxvel)
{
    if (!tp || vMax <= 0.0 || ini_maxvel <= 0.0) {
        return TP_ERR_FAIL;
    }
    tp->vMax = vMax;
    tp->ini_maxvel = ini_maxvel;
    return TP_ERR_OK;
}

int tpSetVlimit(TP_STRUCT * const tp, double vLimit)
{
    if (!tp) {
        return TP_ERR_FAIL;
    }
    tp->vLimit = vLimit < 0.0 ? 0.0 : vLimit;
    return TP_ERR_OK;
}

int tpSetAmax(TP_STRUCT * const tp, double aMax)
{
    if (!tp || aMax <= 0.0) {
        return TP_ERR_FAIL;
    }
    tp->aMax = aMax;
    return TP_ERR_OK;
}

int tpSetId(TP_STRUCT * const tp, int id)
{
    if (!MOTION_ID_VALID(id)) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tpSetId: invalid motion id %d\n", id);
        return TP_ERR_FAIL;
    }
    if (!tp) {
        return TP_ERR_FAIL;
    }
    tp->nextId = id;
    return TP_ERR_OK;
}

int tpGetExecId(TP_STRUCT * const tp)
{
    if (!tp) {
        return TP_ERR_FAIL;
    }
    return tp->execId;
}

struct state_tag_t tpGetExecTag(TP_STRUCT * const tp)
{
    if (!tp) {
        struct state_tag_t empty = {};
        return empty;
    }
    return tp->execTag;
}

int tpSetTermCond(TP_STRUCT * const tp, int cond, double tolerance, double angular_tolerance)
{
    if (!tp) {
        return TP_ERR_FAIL;
    }
    switch (cond) {
    case TC_TERM_COND_PARABOLIC:
    case TC_TERM_COND_TANGENT:
    case TC_TERM_COND_EXACT:
    case TC_TERM_COND_STOP:
        tp->termCond = cond;
        tp->tolerance = tolerance;
        tpn.ang_tolerance = fmax(0.0, angular_tolerance);
        break;
    default:
        return -1;
    }
    return TP_ERR_OK;
}

int tpSetCurrentPos(TP_STRUCT * const tp, EmcPose const * const pos)
{
    if (!tp) {
        return TP_ERR_FAIL;
    }
    if (!emcPoseValid(pos)) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tpnext: invalid pose in tpSetCurrentPos\n");
        return TP_ERR_INVALID;
    }
    tp->currentPos = *pos;
    return TP_ERR_OK;
}

int tpAddCurrentPos(TP_STRUCT * const tp, EmcPose const * const disp)
{
    if (!tp || !disp) {
        return TP_ERR_MISSING_INPUT;
    }
    if (!emcPoseValid(disp)) {
        return TP_ERR_INVALID;
    }
    emcPoseSelfAdd(&tp->currentPos, disp);
    return TP_ERR_OK;
}

int tpSetPos(TP_STRUCT * const tp, EmcPose const * const pos)
{
    if (!tp) {
        return TP_ERR_FAIL;
    }
    if (tpSetCurrentPos(tp, pos)) {
        return TP_ERR_FAIL;
    }
    tp->goalPos = *pos;
    return TP_ERR_OK;
}

int tpSetSpindleSync(TP_STRUCT * const tp, int spindle, double sync, int mode)
{
    if (sync) {
        tp->synchronized = mode ? TC_SYNC_VELOCITY : TC_SYNC_POSITION;
        tp->uu_per_rev = sync;
        tp->spindle.spindle_num = spindle;
        /* each synchronized move may report again */
        tp->spindle.overrun_reported = 0;
    } else {
        tp->synchronized = 0;
    }
    return TP_ERR_OK;
}

int tpPause(TP_STRUCT * const tp)
{
    if (!tp) {
        return TP_ERR_FAIL;
    }
    tp->pausing = 1;
    return TP_ERR_OK;
}

int tpResume(TP_STRUCT * const tp)
{
    if (!tp) {
        return TP_ERR_FAIL;
    }
    tp->pausing = 0;
    return TP_ERR_OK;
}

int tpAbort(TP_STRUCT * const tp)
{
    if (!tp) {
        return TP_ERR_FAIL;
    }
    if (!tp->aborting) {
        tpPause(tp);
        tp->aborting = 1;
    }
    return tpClearDIOs(tp);
}

int tpGetMotionType(TP_STRUCT * const tp)
{
    return tp->motionType;
}

int tpGetPos(TP_STRUCT const * const tp, EmcPose * const pos)
{
    if (!tp) {
        ZERO_EMC_POSE((*pos));
        return TP_ERR_FAIL;
    }
    *pos = tp->currentPos;
    return TP_ERR_OK;
}

int tpIsDone(TP_STRUCT * const tp)
{
    return tp ? tp->done : TP_ERR_OK;
}

int tpQueueDepth(TP_STRUCT * const tp)
{
    return tp ? tp->depth : TP_ERR_OK;
}

int tpActiveDepth(TP_STRUCT * const tp)
{
    return tp ? tp->activeDepth : TP_ERR_OK;
}

int tpSetAout(TP_STRUCT * const tp, unsigned char index, double start, double end)
{
    (void)end;
    if (!tp) {
        return TP_ERR_FAIL;
    }
    tp->syncdio.anychanged = 1;
    tp->syncdio.aio_mask |= (1 << index);
    tp->syncdio.aios[index] = start;
    return TP_ERR_OK;
}

int tpSetDout(TP_STRUCT * const tp, int index, unsigned char start, unsigned char end)
{
    (void)end;
    if (!tp) {
        return TP_ERR_FAIL;
    }
    tp->syncdio.anychanged = 1;
    tp->syncdio.dio_mask |= (1 << index);
    tp->syncdio.dios[index] = start > 0 ? 1 : -1;
    return TP_ERR_OK;
}

int tpIsMoving(TP_STRUCT const * const tp)
{
    (void)tp;
    return tpn.cur_v > 1e-9 || fabs(tpn.cur_a) > 1e-9 || tpnTapMoving();
}

int tpSetRunDir(TP_STRUCT * const tp, tc_direction_t dir)
{
    if (dir == TC_DIR_FORWARD) {
        tp->reverse_run = TC_DIR_FORWARD;
        return TP_ERR_OK;
    }
    /* running backwards along the path is not supported yet */
    return TP_ERR_FAIL;
}

void tpToggleDIOs(TC_STRUCT * const tc)
{
    (void)tc;
}

int tcqFull(TC_QUEUE_STRUCT const * const tcq)
{
    (void)tcq;
    return tpn.q_len >= TPN_QSIZE - TPN_QMARGIN;
}

static void toggleDIOs(syncdio_t *io)
{
    int i;
    if (!io->anychanged) {
        return;
    }
    for (i = 0; i < tpn.emcmotConfig->numDIO; i++) {
        if (!(io->dio_mask & (1ULL << i))) {
            continue;
        }
        if (io->dios[i] > 0) {
            _DioWrite(i, 1);
        } else if (io->dios[i] < 0) {
            _DioWrite(i, 0);
        }
    }
    for (i = 0; i < tpn.emcmotConfig->numAIO; i++) {
        if (io->aio_mask & (1ULL << i)) {
            _AioWrite(i, io->aios[i]);
        }
    }
    io->anychanged = 0;
}

int tpAddLine(TP_STRUCT * const tp, EmcPose end, int canon_motion_type,
        double vel, double ini_maxvel, double acc, double ini_maxjerk,
        double vlimit_scale, unsigned char enables, char atspeed, int indexer_jnum,
        struct state_tag_t tag)
{
    (void)acc;
    (void)ini_maxjerk;
    if (!tp) {
        return TP_ERR_FAIL;
    }
    if (tpn.q_len >= TPN_QSIZE) {
        return TP_ERR_FAIL;
    }
    tpn_seg *sg = seg(tpn.q_len);
    if (tpnLineInit(&sg->geom, &tp->goalPos, &end)) {
        return TP_ERR_ZERO_LENGTH;
    }
    int res = tpnAddSegment(tp, sg, canon_motion_type, vel, ini_maxvel, vlimit_scale,
            enables, atspeed, indexer_jnum, tag);
    if (res == TP_ERR_OK) {
        tp->goalPos = end;
    }
    return res;
}

int tpAddCircle(TP_STRUCT * const tp, EmcPose end, PmCartesian center,
        PmCartesian normal, int turn, int canon_motion_type, double vel,
        double ini_maxvel, double acc, double ini_maxjerk, double vlimit_scale,
        unsigned char enables, char atspeed, struct state_tag_t tag)
{
    (void)acc;
    (void)ini_maxjerk;
    if (!tp) {
        return TP_ERR_FAIL;
    }
    if (tpn.q_len >= TPN_QSIZE) {
        return TP_ERR_FAIL;
    }
    tpn_seg *sg = seg(tpn.q_len);
    if (tpnArcInit(&sg->geom, &tp->goalPos, &end, &center, &normal, turn)) {
        return TP_ERR_ZERO_LENGTH;
    }
    int res = tpnAddSegment(tp, sg, canon_motion_type, vel, ini_maxvel, vlimit_scale,
            enables, atspeed, -1, tag);
    if (res == TP_ERR_OK) {
        tp->goalPos = end;
    }
    return res;
}

int tpAddRigidTap(TP_STRUCT * const tp, EmcPose end, double vel, double ini_maxvel,
        double acc, double ini_maxjerk, unsigned char enables, double scale,
        struct state_tag_t tag)
{
    (void)acc;
    (void)ini_maxjerk;
    if (!tp) {
        return TP_ERR_FAIL;
    }
    if (!tp->synchronized) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Cannot add unsynchronized rigid tap move.\n");
        return TP_ERR_FAIL;
    }
    if (tpn.q_len >= TPN_QSIZE) {
        return TP_ERR_FAIL;
    }
    tpn_seg *sg = seg(tpn.q_len);
    /* only XYZ move */
    EmcPose bottom = tp->goalPos;
    bottom.tran = end.tran;
    if (tpnLineInit(&sg->geom, &tp->goalPos, &bottom)) {
        return TP_ERR_ZERO_LENGTH;
    }
    /* the spindle always has to be at speed */
    int res = tpnAddSegment(tp, sg, 0, vel, ini_maxvel, 0.0, enables, 1, -1, tag);
    if (res == TP_ERR_OK) {
        sg->tap = 1;
        sg->tap_scale = scale > 0.0 ? scale : 1.0;
    }
    /* the tap ends where it started, goalPos stays */
    return res;
}

/* ------------------------------------------------------------ runtime */

/* pose at path parameter S, from the first move that owns it */
static void poseAt(double S, tpn_vec *p, tpn_vec *d1)
{
    int i;
    tpn_seg *sg = seg(0);
    for (i = 0; i < tpn.q_len; i++) {
        sg = seg(i);
        if (S < ownedEnd(sg) || i == tpn.q_len - 1) {
            break;
        }
    }
    if (sg->h_in > 0.0 && S < sg->S0 + sg->h_in) {
        tpnBlendEval(&sg->bin, S - ownedStart(sg), p, d1);
        return;
    }
    double u = S - sg->S0;
    if (u < 0.0) {
        u = 0.0;
    }
    tpnGeomEval(&sg->geom, u, p, d1, 0);
}

static void updateStatus(TP_STRUCT * const tp, tpn_vec const *d1)
{
    tpn_seg *sg = seg(0);
    EmcPose end;
    double m;
    tp->motionType = sg->canon_type;
    tp->execId = sg->id;
    tp->execTag = sg->tag;
    tp->activeDepth = (sg->h_in > 0.0 && tpn.cur_s < sg->S0 + sg->h_in) ? 2 : 1;
    tpn.emcmotStatus->distance_to_go = fmax(0.0, segEnd(sg) - tpn.cur_s);
    tpn.emcmotStatus->enables_queued = sg->enables;
    tpn.emcmotStatus->requested_vel = sg->vreq;
    tpn.emcmotStatus->current_vel = tpn.cur_v;
    tpn.emcmotStatus->current_acc = tpn.cur_a;
    tpn.emcmotStatus->current_jerk = tpn.cur_j;
    tpn.emcmotStatus->spindleSync = tpnSyncOn();
    tpn.emcmotStatus->tcqlen = tpn.q_len;
    m = sqrt(d1->v[0] * d1->v[0] + d1->v[1] * d1->v[1] + d1->v[2] * d1->v[2]);
    if (m > 1e-12) {
        tpn.emcmotStatus->current_dir.x = d1->v[0] / m;
        tpn.emcmotStatus->current_dir.y = d1->v[1] / m;
        tpn.emcmotStatus->current_dir.z = d1->v[2] / m;
    } else {
        tpn.emcmotStatus->current_dir.x = 0;
        tpn.emcmotStatus->current_dir.y = 0;
        tpn.emcmotStatus->current_dir.z = 0;
    }
    tpnPoseFromVec(&end, &sg->geom.p1);
    emcPoseSub(&end, &tp->currentPos, &tpn.emcmotStatus->dtg);
}

static void popFront(TP_STRUCT * const tp)
{
    tpn.q_start = (tpn.q_start + 1) % TPN_QSIZE;
    tpn.q_len--;
    tp->queue._len = tpn.q_len;
    tp->depth = tpn.q_len;
    tpn.emcmotStatus->tcqlen = tpn.q_len;
}

/* returns nonzero while the move has to wait before it may start */
static int activate(TP_STRUCT * const tp, tpn_seg *sg)
{
    int s;
    if (sg->active) {
        return 0;
    }
    if (sg->atspeed) {
        for (s = 0; s < tpn.emcmotConfig->numSpindles; s++) {
            if (!tpn.emcmotStatus->spindle_status[s].at_speed) {
                tp->spindle.waiting_for_atspeed = sg->id;
                return 1;
            }
        }
        tp->spindle.waiting_for_atspeed = MOTION_INVALID_ID;
    }
    if (sg->indexer_jnum != -1) {
        _SetRotaryUnlock(sg->indexer_jnum, 1);
        if (!_GetRotaryIsUnlocked(sg->indexer_jnum)) {
            return 1;
        }
    }
    if (sg->sync == TC_SYNC_POSITION && !tpnSyncOn() && tpnSyncStart(tp, sg)) {
        return 1;
    }
    sg->active = 1;
    toggleDIOs(&sg->syncdio);
    tp->execTag = sg->tag;
    return 0;
}

/* The tap has come to rest above its start: the rest of the move is a
 * plain line back down to the start, ending where the tap's length
 * ended so the moves after it keep their place along the path. */
static void tapToPath(tpn_seg *sg, double u, double v, double a)
{
    EmcPose from, to;
    tpn_vec q;
    int i;
    double end = segEnd(sg);
    for (i = 0; i < TPN_NAX; i++) {
        q.v[i] = sg->geom.p0.v[i] + sg->geom.g.v[i] * u;
    }
    tpnPoseFromVec(&from, &q);
    tpnPoseFromVec(&to, &sg->geom.p0);
    if (tpnLineInit(&sg->geom, &from, &to)) {
        /* already there */
        sg->geom.p1 = sg->geom.p0;
        sg->geom.L = 0.0;
        for (i = 0; i < TPN_NAX; i++) {
            sg->geom.g.v[i] = 0.0;
        }
        v = a = 0.0;
    }
    sg->S0 = end - sg->geom.L;
    sg->tap = 0;
    sg->sync = TC_SYNC_NONE;
    sg->vreq = sg->lim_int.V;
    sg->lim_int_hi = sg->lim_int;
    sg->E_int = sg->E_int_hi = fmin(sg->lim_int.V, sqrt(sg->lim_int.A * sg->geom.L));
    tpnSyncReset();
    tpnRunReset();
    tpn.cur_s = sg->S0;
    tpn.cur_v = fmax(v, 0.0);
    tpn.cur_a = a;
    tpn.cur_j = 0.0;
}

/* One cycle of a rigid tap that follows the spindle. On the cycle it
 * comes to rest above its start it hands over to the path controller,
 * which takes the next cycle. */
static int tapCycle(TP_STRUCT * const tp, tpn_seg *sg)
{
    double u, v, a;
    int i, r = tpnTapCycle(tp, sg, &u, &v, &a);
    tpn_vec p, d1;
    if (r == TPN_TAP_STOPPED) {
        queueReset(tp);
        statusIdle(tp);
        tp->spindle.waiting_for_index = MOTION_INVALID_ID;
        tp->spindle.waiting_for_atspeed = MOTION_INVALID_ID;
        return TP_ERR_STOPPED;
    }
    for (i = 0; i < TPN_NAX; i++) {
        p.v[i] = sg->geom.p0.v[i] + sg->geom.g.v[i] * u;
        d1.v[i] = v < 0.0 ? -sg->geom.g.v[i] : sg->geom.g.v[i];
    }
    double dtg = fabs(sg->geom.L - u);
    if (r == TPN_TAP_PLACE) {
        tapToPath(sg, u, v, a);
        dtg = sg->geom.L;
    }
    tpnPoseFromVec(&tp->currentPos, &p);
    updateStatus(tp, &d1);
    tpn.emcmotStatus->distance_to_go = dtg;
    tpn.emcmotStatus->current_vel = fabs(v);
    tpn.emcmotStatus->current_acc = a;
    return TP_ERR_OK;
}

/* returns nonzero while a finished move still has to wait */
static int finish(tpn_seg *sg)
{
    if (sg->indexer_jnum != -1) {
        _SetRotaryUnlock(sg->indexer_jnum, 0);
        if (_GetRotaryIsUnlocked(sg->indexer_jnum)) {
            return 1;
        }
    }
    return 0;
}

int tpRunCycle(TP_STRUCT * const tp, long period)
{
    (void)period;
    int stepping = tpn.emcmotStatus->stepping;

    tpnSpindleEstimate(tp);

    if (tpn.q_len == 0) {
        queueReset(tp);
        statusIdle(tp);
        tpResume(tp);
        return TP_ERR_WAITING;
    }

    if (tp->aborting && tpn.cur_v <= 0.0 && tpn.cur_a == 0.0 && !tpnTapMoving()) {
        queueReset(tp);
        statusIdle(tp);
        tp->spindle.waiting_for_index = MOTION_INVALID_ID;
        tp->spindle.waiting_for_atspeed = MOTION_INVALID_ID;
        return TP_ERR_STOPPED;
    }

    /* moves the controller has left */
    while (tpn.q_len > 1 && tpn.cur_s >= ownedEnd(seg(0))) {
        if (finish(seg(0))) {
            return TP_ERR_WAITING;
        }
        popFront(tp);
    }
    if (tpn.q_len == 1 && tpn.cur_s >= segEnd(seg(0)) && tpn.cur_v <= 0.0) {
        if (finish(seg(0))) {
            return TP_ERR_WAITING;
        }
        tpnPoseFromVec(&tp->currentPos, &seg(0)->geom.p1);
        popFront(tp);
        queueReset(tp);
        statusIdle(tp);
        return TP_ERR_OK;
    }
    if (seg(0)->tap) {
        /* a tap starts on the index even right after a thread */
        if (!seg(0)->active) {
            tpnSyncReset();
        }
        if (activate(tp, seg(0))) {
            return TP_ERR_WAITING;
        }
        return tapCycle(tp, seg(0));
    }
    if (seg(0)->sync != TC_SYNC_POSITION) {
        tpnSyncReset();
    }
    if (activate(tp, seg(0))) {
        return TP_ERR_WAITING;
    }

    double scale = tpn.emcmotStatus->net_feed_scale;
    /* a thread runs at the spindle's pace, through feed hold and override */
    tpn.track = tpnSyncOn() && !tp->aborting;
    if (tpn.track) {
        tpnSyncReference(tp);
        scale = 1.0;
    } else if (tp->pausing || tp->aborting) {
        scale = 0.0;
    }
    if (scale < 0.0) {
        scale = 0.0;
    }
    tpnAdvance(tp, scale, stepping);
    if (tpn.track) {
        tpnSyncOverrun(tp);
    }

    tpn_vec p, d1;
    poseAt(tpn.cur_s, &p, &d1);
    tpnPoseFromVec(&tp->currentPos, &p);
    updateStatus(tp, &d1);
    return TP_ERR_OK;
}

/* ------------------------------------------------------------ module */

static int tpnext_id;

int rtapi_app_main(void)
{
    tpnext_id = hal_init("tpnextmod");
    if (tpnext_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tpnextmod: hal_init() failed\n");
        return -1;
    }
    hal_ready(tpnext_id);
    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(tpnext_id);
}

EXPORT_SYMBOL(tpMotFunctions);
EXPORT_SYMBOL(tpMotData);

EXPORT_SYMBOL(tpAbort);
EXPORT_SYMBOL(tpActiveDepth);
EXPORT_SYMBOL(tpAddCircle);
EXPORT_SYMBOL(tpAddLine);
EXPORT_SYMBOL(tpAddRigidTap);
EXPORT_SYMBOL(tpClear);
EXPORT_SYMBOL(tpCreate);
EXPORT_SYMBOL(tpGetExecId);
EXPORT_SYMBOL(tpGetExecTag);
EXPORT_SYMBOL(tpGetMotionType);
EXPORT_SYMBOL(tpGetPos);
EXPORT_SYMBOL(tpIsDone);
EXPORT_SYMBOL(tpPause);
EXPORT_SYMBOL(tpQueueDepth);
EXPORT_SYMBOL(tpResume);
EXPORT_SYMBOL(tpRunCycle);
EXPORT_SYMBOL(tpSetAmax);
EXPORT_SYMBOL(tpSetAout);
EXPORT_SYMBOL(tpSetCycleTime);
EXPORT_SYMBOL(tpSetDout);
EXPORT_SYMBOL(tpSetId);
EXPORT_SYMBOL(tpSetPos);
EXPORT_SYMBOL(tpSetRunDir);
EXPORT_SYMBOL(tpSetSpindleSync);
EXPORT_SYMBOL(tpSetTermCond);
EXPORT_SYMBOL(tpSetVlimit);
EXPORT_SYMBOL(tpSetVmax);

EXPORT_SYMBOL(tcqFull);
