/********************************************************************
* Description: tpnext.c
*   Path parameter trajectory planner, loaded with [TRAJ]TPMOD=tpnextmod.
*
*   The queued moves form one path parameterised by s, the RS274NGC feed
*   length. Where two moves blend, the corner is replaced by a quintic
*   inside the G64 P tolerance. Every piece of the path (a blend or the
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

#define TPN_QSIZE DEFAULT_TC_QUEUE_SIZE
#define TPN_QMARGIN 20
#define TPN_MAXCON 512
#define TPN_BIG 1e30
#define TPN_EPS 1e-12
/* fraction of the INI limits the planner aims at */
#define TPN_LIMIT_SCALE 0.99
/* fraction of the blend tolerance the blend may use */
#define TPN_TOL_SCALE 0.98
#define TPN_DEV_SAMPLES 32
/* braking is planned with this fraction of the tangential limits so the
 * cycle by cycle controller has headroom to follow the curve */
#define TPN_BRAKE_SCALE 0.97

static emcmot_status_t *emcmotStatus;
static emcmot_config_t *emcmotConfig;

static void (*_DioWrite)(int, char);
static void (*_AioWrite)(int, double);
static void (*_SetRotaryUnlock)(int, int);
static int (*_GetRotaryIsUnlocked)(int);
static double (*_axis_get_vel_limit)(int);
static double (*_axis_get_acc_limit)(int);
static double (*_axis_get_jerk_limit)(int);

void tpMotFunctions(void (*pDioWrite)(int, char)
                   ,void (*pAioWrite)(int, double)
                   ,void (*pSetRotaryUnlock)(int, int)
                   ,int (*pGetRotaryIsUnlocked)(int)
                   ,double (*paxis_get_vel_limit)(int)
                   ,double (*paxis_get_acc_limit)(int)
                   ,double (*paxis_get_jerk_limit)(int)
                   )
{
    _DioWrite = pDioWrite;
    _AioWrite = pAioWrite;
    _SetRotaryUnlock = pSetRotaryUnlock;
    _GetRotaryIsUnlocked = pGetRotaryIsUnlocked;
    _axis_get_vel_limit = paxis_get_vel_limit;
    _axis_get_acc_limit = paxis_get_acc_limit;
    _axis_get_jerk_limit = paxis_get_jerk_limit;
}

void tpMotData(emcmot_status_t *pstatus, emcmot_config_t *pconfig)
{
    emcmotStatus = pstatus;
    emcmotConfig = pconfig;
}

/* ------------------------------------------------------------ state */

static tpn_seg queue[TPN_QSIZE];
static int q_start, q_len;
/* controller state along the path parameter */
static double cur_s, cur_v, cur_a, cur_j;
/* deadbeat stop sequence in progress */
#define TPN_DB_MAX 40
static double db_seq[TPN_DB_MAX];
static int db_n, db_i;
static double db_S;

static tpn_seg *seg(int i)
{
    return &queue[(q_start + i) % TPN_QSIZE];
}

static double segEnd(tpn_seg const *sg)
{
    return sg->S0 + sg->geom.L;
}

/* the part of the path a move owns: from the start of its blend with the
 * previous move to the start of its blend with the next one */
static double ownedStart(tpn_seg const *sg)
{
    return sg->S0 - sg->h_in;
}

static double ownedEnd(tpn_seg const *sg)
{
    return sg->S0 + sg->geom.L - sg->h_out;
}

static void readAxisLimits(TP_STRUCT const *tp, tpn_axlim *ax)
{
    int i;
    int trapezoid = emcmotStatus->planner_type != 1;
    for (i = 0; i < TPN_NAX; i++) {
        double v = _axis_get_vel_limit ? _axis_get_vel_limit(i) : 0.0;
        double a = _axis_get_acc_limit ? _axis_get_acc_limit(i) : 0.0;
        double j = _axis_get_jerk_limit ? _axis_get_jerk_limit(i) : 0.0;
        /* an axis without limits is not part of the machine; it can
         * only appear in a move with zero displacement */
        ax->vel[i] = v > 0.0 ? v * TPN_LIMIT_SCALE : TPN_BIG;
        ax->acc[i] = a > 0.0 ? a * TPN_LIMIT_SCALE : TPN_BIG;
        if (trapezoid || j <= 0.0) {
            /* acceleration may change within two cycles */
            j = ax->acc[i] / (2.0 * tp->cycleTime);
        } else {
            j *= TPN_LIMIT_SCALE;
        }
        ax->jerk[i] = j;
    }
}

/* velocity cap from [TRAJ]MAX_LINEAR_VELOCITY on the XYZ speed */
static double xyzCap(TP_STRUCT const *tp, tpn_vec const *G)
{
    double gx = sqrt(G->v[0] * G->v[0] + G->v[1] * G->v[1] + G->v[2] * G->v[2]);
    if (tp->vLimit <= 0.0 || gx < 1e-9) {
        return TPN_BIG;
    }
    return tp->vLimit / gx;
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
    for (i = 0; i < emcmotConfig->numDIO; i++) {
        tp->syncdio.dios[i] = 0;
    }
    for (i = 0; i < emcmotConfig->numAIO; i++) {
        tp->syncdio.aios[i] = 0;
    }
    return TP_ERR_OK;
}

static void statusIdle(TP_STRUCT * const tp)
{
    emcmotStatus->distance_to_go = 0;
    emcmotStatus->enables_queued = emcmotStatus->enables_new;
    emcmotStatus->requested_vel = 0;
    emcmotStatus->current_vel = 0;
    emcmotStatus->current_acc = 0;
    emcmotStatus->current_jerk = 0;
    emcmotStatus->spindleSync = 0;
    emcmotStatus->current_dir.x = 0;
    emcmotStatus->current_dir.y = 0;
    emcmotStatus->current_dir.z = 0;
    emcmotStatus->tcqlen = 0;
    ZERO_EMC_POSE(emcmotStatus->dtg);
    tp->motionType = 0;
    tp->activeDepth = 0;
}

static void queueReset(TP_STRUCT * const tp)
{
    q_start = 0;
    q_len = 0;
    cur_s = cur_v = cur_a = cur_j = 0.0;
    db_n = db_i = 0;
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
    emcmotStatus->motionFlag |= EMCMOT_MOTION_INPOS_BIT;
    return tpClearDIOs(tp);
}

int tpInit(TP_STRUCT * const tp)
{
    if (!emcmotStatus) {
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

int tpSetTermCond(TP_STRUCT * const tp, int cond, double tolerance)
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
    return cur_v > 1e-9 || fabs(cur_a) > 1e-9;
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
    return q_len >= TPN_QSIZE - TPN_QMARGIN;
}

static void toggleDIOs(syncdio_t *io)
{
    int i;
    if (!io->anychanged) {
        return;
    }
    for (i = 0; i < emcmotConfig->numDIO; i++) {
        if (!(io->dio_mask & (1ULL << i))) {
            continue;
        }
        if (io->dios[i] > 0) {
            _DioWrite(i, 1);
        } else if (io->dios[i] < 0) {
            _DioWrite(i, 0);
        }
    }
    for (i = 0; i < emcmotConfig->numAIO; i++) {
        if (io->aio_mask & (1ULL << i)) {
            _AioWrite(i, io->aios[i]);
        }
    }
    io->anychanged = 0;
}

/* ------------------------------------------------------------ constraints */

typedef struct {
    double S;       /* where the constraint starts */
    double Vh;      /* hard: speed cap or envelope at S */
    double Vs;      /* soft: requested speed at S, < 0 for none */
    double Aentry;  /* tangential acceleration limit from S on */
    double Arun;    /* smallest acceleration limit between here and S */
    double Jrun;    /* smallest jerk limit between here and S */
} tpn_con;

static tpn_con con[TPN_MAXCON];
static int ncon;
static double g_dt = 0.001;

/* Hard feasibility of a single constraint at distance d with the brake
 * to its cap; d < 0 means the constraint starts inside this step. */
static int conOk(double v1, double a1, double d, double V, double A, double J)
{
    double bd = tpnBrakeDist(v1, a1, V, A, J);
    return bd <= fmax(d, 0.0) + 1e-12;
}

/* distance needed to bring |a1| down to Aentry */
static int accEntryOk(double v1, double a1, double d, double Aentry, double J)
{
    double aa = fabs(a1);
    if (aa <= Aentry + 1e-9) {
        return 1;
    }
    if (d <= 0.0) {
        return 0;
    }
    double t = (aa - Aentry) / J;
    double dist;
    if (a1 > 0.0) {
        dist = v1 * t + 0.5 * a1 * t * t - J * t * t * t / 6.0;
    } else {
        dist = v1 * t + 0.5 * a1 * t * t + J * t * t * t / 6.0;
    }
    return dist <= d;
}

/* ------------------------------------------------------------ queue build */

static double speedFactor(TP_STRUCT const *tp)
{
    (void)tp;
    return fmax(emcmotConfig->maxFeedScale, 1.0);
}

static double pieceSoft(tpn_seg const *sg, int blend, double scale)
{
    double v = blend ? sg->vreq_bin : sg->vreq;
    if (sg->sync == TC_SYNC_VELOCITY) {
        double speed = fabs(emcmotStatus->spindle_status[0].spindleSpeedIn);
        v = speed * sg->uu_per_rev;
    }
    return v * scale;
}

/* Backward envelope with half of each piece's tangential acceleration and
 * zero speed at the end of the queue and at every stop. The runtime
 * checks the true caps inside its braking horizon; the envelope carries
 * everything beyond it. */
static void backwardPass(void)
{
    double Enext = 0.0;
    int i;
    for (i = q_len - 1; i >= 0; i--) {
        tpn_seg *sg = seg(i);
        double len = fmax(0.0, sg->geom.L - sg->h_in - sg->h_out);
        double Eint = fmin(sg->lim_int.V, sqrt(Enext * Enext + sg->lim_int.A * len));
        double Ebin = Eint;
        if (sg->h_in > 0.0) {
            Ebin = fmin(sg->lim_bin.V, sqrt(Eint * Eint + sg->lim_bin.A * 2.0 * sg->h_in));
        }
        int same = (i < q_len - 2) && Eint == sg->E_int && Ebin == sg->E_bin;
        sg->E_int = Eint;
        sg->E_bin = Ebin;
        Enext = sg->stop_in ? 0.0 : Ebin;
        if (same) {
            break;
        }
    }
}

static void blendBuild(tpn_seg const *prev, tpn_seg const *sg, double h, tpn_blend *b)
{
    tpn_vec p0, d0, dd0, p1, d1, dd1;
    tpnGeomEval(&prev->geom, prev->geom.L - h, &p0, &d0, &dd0);
    tpnGeomEval(&sg->geom, h, &p1, &d1, &dd1);
    tpnBlendInit(b, 2.0 * h, &p0, &d0, &dd0, &p1, &d1, &dd1);
}

static double blendDeviation(tpn_seg const *prev, tpn_seg const *sg, tpn_blend const *b)
{
    double dev = 0.0;
    int k;
    for (k = 1; k < TPN_DEV_SAMPLES; k++) {
        tpn_vec p;
        PmCartesian q;
        tpnBlendEval(b, b->H * k / TPN_DEV_SAMPLES, &p, 0);
        q.x = p.v[0];
        q.y = p.v[1];
        q.z = p.v[2];
        double d = fmin(tpnGeomDistXYZ(&prev->geom, &q), tpnGeomDistXYZ(&sg->geom, &q));
        dev = fmax(dev, d);
    }
    return dev;
}

static void blendLimits(TP_STRUCT const *tp, tpn_axlim const *ax, tpn_seg const *prev,
        tpn_seg const *sg, tpn_blend const *b, tpn_lim *lim)
{
    tpn_vec G, G1, G2;
    tpnBlendBounds(b, &G, &G1, &G2);
    tpnLimits(ax, &G, &G1, &G2, lim);
    lim->V = fmin(lim->V, xyzCap(tp, &G));
    lim->V = fmin(lim->V, fmin(prev->vreq, sg->vreq) * speedFactor(tp));
}

/* Smallest acceleration and jerk limits of the pieces between the
 * current position and S. */
static void runLimits(double S, double *A, double *J)
{
    int i;
    *A = TPN_BIG;
    *J = TPN_BIG;
    for (i = 0; i < q_len; i++) {
        tpn_seg *sg = seg(i);
        if (ownedStart(sg) >= S) {
            break;
        }
        if (sg->h_in > 0.0 && sg->S0 + sg->h_in > cur_s) {
            *A = fmin(*A, sg->lim_bin.A);
            *J = fmin(*J, sg->lim_bin.J);
        }
        if (ownedEnd(sg) > cur_s && sg->S0 + sg->h_in < S) {
            *A = fmin(*A, sg->lim_int.A);
            *J = fmin(*J, sg->lim_int.J);
        }
    }
}

/* Can the controller, from its current state, reach the new blend with a
 * speed at most V? */
static int reachable(double S, double V)
{
    double A, J;
    if (q_len == 0) {
        return 1;
    }
    runLimits(S, &A, &J);
    if (A >= TPN_BIG) {
        return 1;
    }
    /* one cycle of margin for the step that is about to be taken */
    double d = S - cur_s - 2.0 * cur_v * 0.001 - 1e-9;
    return tpnBrakeDist(cur_v, cur_a, V, A * TPN_BRAKE_SCALE, J * TPN_BRAKE_SCALE) <= d;
}

static void joinMoves(TP_STRUCT const *tp, tpn_axlim const *ax, tpn_seg *prev, tpn_seg *sg)
{
    int stop = 0;
    sg->S0 = segEnd(prev);
    sg->h_in = 0.0;
    prev->h_out = 0.0;
    sg->stop_in = 0;

    if (prev->term_cond == TC_TERM_COND_STOP || prev->term_cond == TC_TERM_COND_EXACT) {
        stop = 1;
    }
    if ((prev->canon_type == EMC_MOTION_TYPE_TRAVERSE) ^ (sg->canon_type == EMC_MOTION_TYPE_TRAVERSE)) {
        stop = 1;
    }
    if (sg->atspeed || sg->indexer_jnum != -1 || prev->indexer_jnum != -1) {
        stop = 1;
    }
    if (prev->sync != sg->sync) {
        stop = 1;
    }
    double hmax = fmin(prev->geom.L - prev->h_in, 0.5 * sg->geom.L);
    if (prev == seg(0) || prev->active) {
        /* the blend has to start ahead of the controller */
        hmax = fmin(hmax, segEnd(prev) - cur_s - 4.0 * cur_v * tp->cycleTime - 1e-6);
    }
    if (hmax < 1e-6) {
        stop = 1;
    }
    if (stop) {
        sg->stop_in = 1;
        return;
    }

    double tol = prev->tolerance * TPN_TOL_SCALE;
    double h = hmax;
    tpn_blend b;
    tpn_lim lim;
    int k;
    blendBuild(prev, sg, h, &b);
    if (tol > 0.0) {
        for (k = 0; k < 40; k++) {
            double dev = blendDeviation(prev, sg, &b);
            if (dev <= tol) {
                break;
            }
            h *= fmax(0.1, fmin(0.95, tol / dev));
            blendBuild(prev, sg, h, &b);
        }
        if (k == 40) {
            sg->stop_in = 1;
            return;
        }
    }
    /* no longer than needed for the requested speed */
    double vwant = fmin(prev->vreq, sg->vreq) * speedFactor(tp);
    blendLimits(tp, ax, prev, sg, &b, &lim);
    if (lim.V >= vwant) {
        double lo = h * 1e-3, hi = h;
        for (k = 0; k < 14; k++) {
            double mid = 0.5 * (lo + hi);
            tpn_blend bm;
            tpn_lim lm;
            blendBuild(prev, sg, mid, &bm);
            blendLimits(tp, ax, prev, sg, &bm, &lm);
            if (lm.V >= vwant) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        h = hi;
        blendBuild(prev, sg, h, &b);
        blendLimits(tp, ax, prev, sg, &b, &lim);
    }
    if (lim.V < 1e-6 || lim.A < 1e-9 || !reachable(segEnd(prev) - h, lim.V)) {
        sg->stop_in = 1;
        return;
    }
    sg->h_in = h;
    prev->h_out = h;
    sg->bin = b;
    sg->lim_bin = lim;
    sg->vreq_bin = fmin(prev->vreq, sg->vreq);
}

static int addSegment(TP_STRUCT * const tp, tpn_seg *sg, int canon_type, double vel,
        double ini_maxvel, unsigned char enables, char atspeed, int indexer_jnum,
        struct state_tag_t tag)
{
    tpn_axlim ax;
    tpn_vec G, G1, G2;

    if (!tp || tp->aborting) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tpnext: cannot queue a move while aborting\n");
        return TP_ERR_FAIL;
    }
    if (q_len >= TPN_QSIZE) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tpnext: queue full\n");
        return TP_ERR_FAIL;
    }
    if (tp->synchronized == TC_SYNC_POSITION) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tpnext: spindle synchronized motion is not supported yet\n");
        return TP_ERR_FAIL;
    }
    readAxisLimits(tp, &ax);

    sg->id = tp->nextId;
    sg->tag = tag;
    sg->canon_type = canon_type;
    sg->enables = enables;
    sg->atspeed = atspeed;
    sg->indexer_jnum = indexer_jnum;
    sg->term_cond = tp->termCond;
    sg->tolerance = tp->tolerance;
    sg->sync = tp->synchronized;
    sg->uu_per_rev = tp->uu_per_rev;
    sg->vreq = fmin(vel, ini_maxvel > 0.0 ? ini_maxvel : vel);
    if (sg->vreq <= 0.0) {
        sg->vreq = ini_maxvel;
    }
    sg->h_in = sg->h_out = 0.0;
    sg->stop_in = 0;
    sg->active = 0;
    if (tp->syncdio.anychanged) {
        sg->syncdio = tp->syncdio;
        tpClearDIOs(tp);
    } else {
        sg->syncdio.anychanged = 0;
    }

    tpnGeomBounds(&sg->geom, &G, &G1, &G2);
    tpnLimits(&ax, &G, &G1, &G2, &sg->lim_int);
    sg->lim_int.V = fmin(sg->lim_int.V, xyzCap(tp, &G));
    double vmax = sg->vreq * speedFactor(tp);
    if (ini_maxvel > 0.0) {
        vmax = fmin(vmax, ini_maxvel);
    }
    sg->lim_int.V = fmin(sg->lim_int.V, vmax);

    if (q_len > 0) {
        joinMoves(tp, &ax, seg(q_len - 1), sg);
    } else {
        sg->S0 = cur_s;
        sg->stop_in = 1;
    }

    q_len++;
    tp->queue._len = q_len;
    tp->depth = q_len;
    tp->done = 0;
    emcmotStatus->tcqlen = q_len;
    backwardPass();
    return TP_ERR_OK;
}

int tpAddLine(TP_STRUCT * const tp, EmcPose end, int canon_motion_type,
        double vel, double ini_maxvel, double acc, double ini_maxjerk,
        unsigned char enables, char atspeed, int indexer_jnum, struct state_tag_t tag)
{
    (void)acc;
    (void)ini_maxjerk;
    if (!tp) {
        return TP_ERR_FAIL;
    }
    if (q_len >= TPN_QSIZE) {
        return TP_ERR_FAIL;
    }
    tpn_seg *sg = seg(q_len);
    if (tpnLineInit(&sg->geom, &tp->goalPos, &end)) {
        return TP_ERR_ZERO_LENGTH;
    }
    int res = addSegment(tp, sg, canon_motion_type, vel, ini_maxvel, enables,
            atspeed, indexer_jnum, tag);
    if (res == TP_ERR_OK) {
        tp->goalPos = end;
    }
    return res;
}

int tpAddCircle(TP_STRUCT * const tp, EmcPose end, PmCartesian center,
        PmCartesian normal, int turn, int canon_motion_type, double vel,
        double ini_maxvel, double acc, double ini_maxjerk, unsigned char enables,
        char atspeed, struct state_tag_t tag)
{
    (void)acc;
    (void)ini_maxjerk;
    if (!tp) {
        return TP_ERR_FAIL;
    }
    if (q_len >= TPN_QSIZE) {
        return TP_ERR_FAIL;
    }
    tpn_seg *sg = seg(q_len);
    if (tpnArcInit(&sg->geom, &tp->goalPos, &end, &center, &normal, turn)) {
        return TP_ERR_ZERO_LENGTH;
    }
    int res = addSegment(tp, sg, canon_motion_type, vel, ini_maxvel, enables,
            atspeed, -1, tag);
    if (res == TP_ERR_OK) {
        tp->goalPos = end;
    }
    return res;
}

int tpAddRigidTap(TP_STRUCT * const tp, EmcPose end, double vel, double ini_maxvel,
        double acc, double ini_maxjerk, unsigned char enables, double scale,
        struct state_tag_t tag)
{
    (void)tp;
    (void)end;
    (void)vel;
    (void)ini_maxvel;
    (void)acc;
    (void)ini_maxjerk;
    (void)enables;
    (void)scale;
    (void)tag;
    rtapi_print_msg(RTAPI_MSG_ERR, "tpnext: rigid tapping is not supported yet\n");
    return TP_ERR_FAIL;
}

/* ------------------------------------------------------------ runtime */

/* pose at path parameter S, from the first move that owns it */
static void poseAt(double S, tpn_vec *p, tpn_vec *d1)
{
    int i;
    tpn_seg *sg = seg(0);
    for (i = 0; i < q_len; i++) {
        sg = seg(i);
        if (S < ownedEnd(sg) || i == q_len - 1) {
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

typedef struct {
    double V, A, J;     /* caps of the pieces the step touches */
    double Vs;          /* soft cap there, < 0 for none */
    double Sstop;       /* nearest stop ahead */
} tpn_step;

static void addCon(double S, double Vh, double Vs, double Aentry, double Arun, double Jrun)
{
    if (ncon >= TPN_MAXCON) {
        return;
    }
    con[ncon].S = S;
    con[ncon].Vh = Vh;
    con[ncon].Vs = Vs;
    con[ncon].Aentry = Aentry;
    con[ncon].Arun = Arun;
    con[ncon].Jrun = Jrun;
    ncon++;
}

/* Collect the constraints ahead of cur_s up to the braking horizon of the
 * fastest state reachable this cycle. */
static void gather(TP_STRUCT const *tp, double scale, int stepping, tpn_step *st)
{
    double dt = tp->cycleTime;
    double Arun = TPN_BIG, Jrun = TPN_BIG;
    int i;
    ncon = 0;
    st->V = TPN_BIG;
    st->A = TPN_BIG;
    st->J = TPN_BIG;
    st->Vs = -1.0;
    st->Sstop = TPN_BIG;

    for (i = 0; i < q_len; i++) {
        tpn_seg *sg = seg(i);
        int piece;
        if (sg->stop_in && sg->S0 > cur_s) {
            addCon(sg->S0, 0.0, -1.0, TPN_BIG, Arun, Jrun);
            st->Sstop = fmin(st->Sstop, sg->S0);
        }
        for (piece = 0; piece < 2; piece++) {
            double Pa, Pb, E, soft;
            tpn_lim const *lim;
            if (piece == 0) {
                if (sg->h_in <= 0.0) {
                    continue;
                }
                Pa = ownedStart(sg);
                Pb = sg->S0 + sg->h_in;
                lim = &sg->lim_bin;
                E = sg->E_bin;
            } else {
                Pa = sg->S0 + sg->h_in;
                Pb = ownedEnd(sg);
                lim = &sg->lim_int;
                E = sg->E_int;
            }
            if (Pb <= cur_s && !(i == q_len - 1 && piece == 1)) {
                continue;
            }
            soft = pieceSoft(sg, piece == 0, scale);
            if (Pa <= cur_s) {
                st->V = fmin(st->V, lim->V);
                st->A = fmin(st->A, lim->A);
                st->J = fmin(st->J, lim->J);
                st->Vs = st->Vs < 0.0 ? soft : fmin(st->Vs, soft);
            } else {
                addCon(Pa, E, soft, lim->A, Arun, Jrun);
            }
            Arun = fmin(Arun, lim->A);
            Jrun = fmin(Jrun, lim->J);
        }
        if (stepping && i == 0 && q_len > 1) {
            addCon(ownedEnd(sg), 0.0, -1.0, TPN_BIG, Arun, Jrun);
            st->Sstop = fmin(st->Sstop, ownedEnd(sg));
        }
        if (i == q_len - 1) {
            addCon(segEnd(sg), 0.0, -1.0, TPN_BIG, Arun, Jrun);
            st->Sstop = fmin(st->Sstop, segEnd(sg));
        }
        /* beyond the braking distance of the fastest next state every
         * constraint can be met by stopping */
        double vhi = cur_v + fmax(cur_a, 0.0) * dt + 0.5 * st->J * dt * dt;
        double ahi = fmax(cur_a, 0.0) + st->J * dt;
        double horizon = tpnBrakeDist(vhi, ahi, 0.0, TPN_BRAKE_SCALE * fmin(Arun, st->A),
                TPN_BRAKE_SCALE * fmin(Jrun, st->J))
            + 2.0 * vhi * dt + 8.0 * st->J * dt * dt * dt + 1e-6;
        if (ownedEnd(sg) - cur_s > horizon || ncon >= TPN_MAXCON) {
            break;
        }
    }
}

typedef struct {
    double s1, v1, a1;
} tpn_next;

static void stepState(double j, double dt, tpn_next *n)
{
    n->s1 = cur_s + cur_v * dt + 0.5 * cur_a * dt * dt + j * dt * dt * dt / 6.0;
    n->v1 = cur_v + cur_a * dt + 0.5 * j * dt * dt;
    n->a1 = cur_a + j * dt;
}

enum { CHK_HARD = 1, CHK_SOFT = 2 };

static unsigned char failmask[TPN_MAXCON];

/* check constraint k (or the step caps for k < 0) against the next state */
static int checkOne(int k, int what, tpn_next const *n, tpn_step const *st)
{
    if (k < 0) {
        double A = st->A * TPN_BRAKE_SCALE, J = st->J * TPN_BRAKE_SCALE;
        if (what == CHK_HARD) {
            return conOk(n->v1, n->a1, 0.0, st->V, A, J);
        }
        return st->Vs < 0.0 || conOk(n->v1, n->a1, 0.0, st->Vs, A, J);
    }
    tpn_con const *c = &con[k];
    double d = c->S - n->s1;
    double J = fmin(c->Jrun, st->J) * TPN_BRAKE_SCALE;
    double A = fmin(c->Arun, st->A) * TPN_BRAKE_SCALE;
    if (what == CHK_HARD) {
        if (c->Vh <= 0.0) {
            /* leave the deadbeat finish a little room to land on a cycle */
            d -= 4.0 * J * g_dt * g_dt * g_dt;
        }
        return conOk(n->v1, n->a1, d, c->Vh, A, J) && accEntryOk(n->v1, n->a1, d, c->Aentry, J);
    }
    return c->Vs < 0.0 || conOk(n->v1, n->a1, d, c->Vs, A, J);
}

/* smallest jerk that keeps the motion from reversing */
static double jerkNoReverse(double jlo, double jhi, double J, double dt)
{
    tpn_next n;
    int k;
    stepState(jlo, dt, &n);
    if (n.v1 >= 0.0 && (n.a1 >= 0.0 || n.v1 - 0.5 * n.a1 * n.a1 / J >= -1e-12)) {
        return jlo;
    }
    for (k = 0; k < 40; k++) {
        double mid = 0.5 * (jlo + jhi);
        stepState(mid, dt, &n);
        if (n.v1 >= 0.0 && (n.a1 >= 0.0 || n.v1 - 0.5 * n.a1 * n.a1 / J >= -1e-12)) {
            jhi = mid;
        } else {
            jlo = mid;
        }
    }
    return jhi;
}

/* Jerk that brings the speed down to Vt as fast as the limits allow
 * without undershooting it. */
static double jerkTrack(double Vt, double jlo, double jhi, double J, double dt)
{
    tpn_next n;
    int k;
    stepState(jlo, dt, &n);
    if (n.v1 + 0.5 * n.a1 * fabs(n.a1) / J >= Vt) {
        return jlo;
    }
    stepState(jhi, dt, &n);
    if (n.v1 + 0.5 * n.a1 * fabs(n.a1) / J < Vt) {
        return jhi;
    }
    for (k = 0; k < 40; k++) {
        double mid = 0.5 * (jlo + jhi);
        stepState(mid, dt, &n);
        if (n.v1 + 0.5 * n.a1 * fabs(n.a1) / J >= Vt) {
            jhi = mid;
        } else {
            jlo = mid;
        }
    }
    return jhi;
}

static double chooseJerk(TP_STRUCT const *tp, tpn_step const *st)
{
    double dt = tp->cycleTime;
    double J = st->J;
    double A = st->A;
    double jhi = fmin(J, (A - cur_a) / dt);
    double jlo = fmax(-J, (-A - cur_a) / dt);
    tpn_next n;
    int k, i;
    int nfail = 0;

    if (jlo > jhi) {
        /* the acceleration is above the limit of the piece just entered */
        return cur_a > 0.0 ? -J : J;
    }
    jlo = jerkNoReverse(jlo, jhi, J, dt);

    /* everything that holds at jhi holds for any smaller jerk */
    stepState(jhi, dt, &n);
    for (i = -1; i < ncon; i++) {
        unsigned char m = 0;
        if (!checkOne(i, CHK_HARD, &n, st)) {
            m |= CHK_HARD;
        }
        if (!checkOne(i, CHK_SOFT, &n, st)) {
            m |= CHK_SOFT;
        }
        if (i >= 0) {
            failmask[i] = m;
        }
        nfail += m != 0;
    }
    if (nfail == 0) {
        return jhi;
    }

    /* soft caps that cannot be met even braking at full rate are
     * tracked instead of searched */
    unsigned char curmask = 0;
    double Vtrack = TPN_BIG;
    stepState(jhi, dt, &n);
    if (!checkOne(-1, CHK_HARD, &n, st)) {
        curmask |= CHK_HARD;
    }
    if (!checkOne(-1, CHK_SOFT, &n, st)) {
        curmask |= CHK_SOFT;
    }
    stepState(jlo, dt, &n);
    if ((curmask & CHK_SOFT) && !checkOne(-1, CHK_SOFT, &n, st)) {
        curmask &= ~CHK_SOFT;
        Vtrack = fmin(Vtrack, st->Vs);
    }
    for (i = 0; i < ncon; i++) {
        if ((failmask[i] & CHK_SOFT) && !checkOne(i, CHK_SOFT, &n, st)) {
            failmask[i] &= ~CHK_SOFT;
            Vtrack = fmin(Vtrack, con[i].Vs);
        }
    }

    double lo = jlo, hi = jhi;
    int ok_lo = 1;
    stepState(lo, dt, &n);
    if ((curmask && !(((curmask & CHK_HARD) == 0 || checkOne(-1, CHK_HARD, &n, st))
                     && ((curmask & CHK_SOFT) == 0 || checkOne(-1, CHK_SOFT, &n, st))))) {
        ok_lo = 0;
    }
    for (i = 0; i < ncon && ok_lo; i++) {
        if (((failmask[i] & CHK_HARD) && !checkOne(i, CHK_HARD, &n, st))
                || ((failmask[i] & CHK_SOFT) && !checkOne(i, CHK_SOFT, &n, st))) {
            ok_lo = 0;
        }
    }
    double j;
    if (!ok_lo) {
        j = lo;
    } else {
        for (k = 0; k < 30; k++) {
            double mid = 0.5 * (lo + hi);
            int ok = 1;
            stepState(mid, dt, &n);
            if (((curmask & CHK_HARD) && !checkOne(-1, CHK_HARD, &n, st))
                    || ((curmask & CHK_SOFT) && !checkOne(-1, CHK_SOFT, &n, st))) {
                ok = 0;
            }
            for (i = 0; i < ncon && ok; i++) {
                if (((failmask[i] & CHK_HARD) && !checkOne(i, CHK_HARD, &n, st))
                        || ((failmask[i] & CHK_SOFT) && !checkOne(i, CHK_SOFT, &n, st))) {
                    ok = 0;
                }
            }
            if (ok) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        j = lo;
    }
    if (Vtrack < TPN_BIG) {
        j = fmin(j, jerkTrack(Vtrack, jlo, jhi, J, dt));
    }
    return j;
}

/*
 * Continuous brake profile from (v, a) to rest as up to three constant
 * jerk phases; returns the number of phases.
 */
static int brakePhases(double v, double a, double A, double J, double *pj, double *pt)
{
    int n = 0;
    double t1, tA, vv;
    if (a >= 0.0) {
        vv = v + 0.5 * a * a / J;
        t1 = fmin(sqrt(vv / J), A / J);
        tA = vv > J * t1 * t1 ? (vv - J * t1 * t1) / A : 0.0;
        pj[n] = -J; pt[n++] = a / J + t1;
    } else {
        double Aeff = fmax(A, -a);
        vv = v + 0.5 * a * a / J;
        if (v - 0.5 * a * a / J <= 0.0) {
            pj[n] = J; pt[n++] = -a / J;
            return n;
        }
        t1 = fmin(sqrt(vv / J), Aeff / J);
        tA = vv > J * t1 * t1 ? (vv - J * t1 * t1) / Aeff : 0.0;
        pj[n] = -J; pt[n++] = fmax(0.0, t1 + a / J);
    }
    pj[n] = 0.0; pt[n++] = tA;
    pj[n] = J; pt[n++] = t1;
    return n;
}

/* invert the symmetric 3x3 matrix M */
static int inv3(double M[3][3], double inv[3][3])
{
    double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
               - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
               + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    if (fabs(det) < 1e-300) {
        return -1;
    }
    inv[0][0] = (M[1][1] * M[2][2] - M[1][2] * M[2][1]) / det;
    inv[0][1] = (M[0][2] * M[2][1] - M[0][1] * M[2][2]) / det;
    inv[0][2] = (M[0][1] * M[1][2] - M[0][2] * M[1][1]) / det;
    inv[1][0] = (M[1][2] * M[2][0] - M[1][0] * M[2][2]) / det;
    inv[1][1] = (M[0][0] * M[2][2] - M[0][2] * M[2][0]) / det;
    inv[1][2] = (M[0][2] * M[1][0] - M[0][0] * M[1][2]) / det;
    inv[2][0] = (M[1][0] * M[2][1] - M[1][1] * M[2][0]) / det;
    inv[2][1] = (M[0][1] * M[2][0] - M[0][0] * M[2][1]) / det;
    inv[2][2] = (M[0][0] * M[1][1] - M[0][1] * M[1][0]) / det;
    return 0;
}

/*
 * Jerk sequence over N cycles that ends exactly at distance d with zero
 * speed and acceleration on a cycle boundary: the continuous brake
 * profile averaged over each cycle, plus the smallest correction that
 * removes what the averaging and the fractional last cycle leave over.
 * Returns the largest jerk of the sequence.
 */
static double deadbeat(double d, double v, double a, int N, double dt,
        int nph, double const *pj, double const *pt, double *seq)
{
    double M[3][3] = {{0}}, r[3], inv[3][3], lam[3];
    int k, p, q;
    double sN = 0.0, vN = v, aN = a, t0 = 0.0;
    /* nominal jerk: continuous profile averaged over each cycle */
    for (k = 0; k < N; k++) {
        double ta = k * dt, tb = ta + dt, acc = 0.0;
        t0 = 0.0;
        for (p = 0; p < nph; p++) {
            double lo = fmax(ta, t0), hi = fmin(tb, t0 + pt[p]);
            if (hi > lo) {
                acc += pj[p] * (hi - lo);
            }
            t0 += pt[p];
        }
        seq[k] = acc / dt;
    }
    /* response to the nominal sequence */
    for (k = 0; k < N; k++) {
        double j = seq[k];
        sN += vN * dt + 0.5 * aN * dt * dt + j * dt * dt * dt / 6.0;
        vN += aN * dt + 0.5 * j * dt * dt;
        aN += j * dt;
    }
    r[0] = d - sN;
    r[1] = -vN;
    r[2] = -aN;
    for (k = 1; k <= N; k++) {
        double m = N - k;
        double row[3] = {dt * dt * dt * (1.0 / 6.0 + 0.5 * m + 0.5 * m * m),
                         dt * dt * (0.5 + m), dt};
        for (p = 0; p < 3; p++) {
            for (q = 0; q < 3; q++) {
                M[p][q] += row[p] * row[q];
            }
        }
    }
    if (inv3(M, inv)) {
        return TPN_BIG;
    }
    for (p = 0; p < 3; p++) {
        lam[p] = inv[p][0] * r[0] + inv[p][1] * r[1] + inv[p][2] * r[2];
    }
    double jmax = 0.0;
    for (k = 1; k <= N; k++) {
        double m = N - k;
        seq[k - 1] += lam[0] * dt * dt * dt * (1.0 / 6.0 + 0.5 * m + 0.5 * m * m)
                    + lam[1] * dt * dt * (0.5 + m) + lam[2] * dt;
        jmax = fmax(jmax, fabs(seq[k - 1]));
    }
    return jmax;
}

/* acceleration stays within A and the motion never reverses */
static int deadbeatOk(int N, double dt, double A)
{
    double v = cur_v, a = cur_a;
    int k;
    for (k = 0; k < N; k++) {
        double j = db_seq[k];
        v += a * dt + 0.5 * j * dt * dt;
        a += j * dt;
        if (fabs(a) > A * (1.0 + 1e-6) || v < -1e-9) {
            return 0;
        }
    }
    return 1;
}

/* Land a stop on a cycle boundary: once the controller rides the brake
 * curve into a stop close enough, replace the rest of the curve by a
 * deadbeat sequence and run it to its end unless the stop moves. */
static int finishStop(TP_STRUCT const *tp, tpn_step const *st, double *j)
{
    double dt = tp->cycleTime;
    double d = st->Sstop - cur_s;
    double pj[3], pt[3];
    int nph, N, k;
    if (db_i < db_n && db_S == st->Sstop) {
        *j = db_seq[db_i++];
        return 1;
    }
    db_n = db_i = 0;
    if (st->Sstop >= TPN_BIG || d < 0.0) {
        return 0;
    }
    double Ab = st->A * TPN_BRAKE_SCALE, Jb = st->J * TPN_BRAKE_SCALE;
    double bd = tpnBrakeDist(cur_v, cur_a, 0.0, Ab, Jb);
    double slack = 8.0 * st->J * dt * dt * dt;
    int creep = cur_v <= 1e-9 && fabs(cur_a) <= 1e-9;
    if (creep ? d > fmin(0.01, 100.0 * slack) : d > bd + fmax(1e-6 * d, 0.05 * cur_v * dt) + slack) {
        return 0;
    }
    nph = brakePhases(cur_v, cur_a, Ab, Jb, pj, pt);
    double T = 0.0;
    for (k = 0; k < nph; k++) {
        T += pt[k];
    }
    N = (int)ceil(T / dt - 1e-9);
    int Nmax = creep ? TPN_DB_MAX : N + 6;
    if (N > TPN_DB_MAX - 2) {
        return 0;
    }
    for (; N <= TPN_DB_MAX && N <= Nmax; N++) {
        if (N < 3) {
            continue;
        }
        if (deadbeat(d, cur_v, cur_a, N, dt, nph, pj, pt, db_seq) <= 1.03 * st->J
                && deadbeatOk(N, dt, st->A)) {
            db_n = N;
            db_i = 1;
            db_S = st->Sstop;
            *j = db_seq[0];
            return 1;
        }
    }
    return 0;
}

static void updateStatus(TP_STRUCT * const tp, tpn_vec const *d1)
{
    tpn_seg *sg = seg(0);
    EmcPose end;
    double m;
    tp->motionType = sg->canon_type;
    tp->execId = sg->id;
    tp->execTag = sg->tag;
    tp->activeDepth = (sg->h_in > 0.0 && cur_s < sg->S0 + sg->h_in) ? 2 : 1;
    emcmotStatus->distance_to_go = fmax(0.0, segEnd(sg) - cur_s);
    emcmotStatus->enables_queued = sg->enables;
    emcmotStatus->requested_vel = sg->vreq;
    emcmotStatus->current_vel = cur_v;
    emcmotStatus->current_acc = cur_a;
    emcmotStatus->current_jerk = cur_j;
    emcmotStatus->spindleSync = 0;
    emcmotStatus->tcqlen = q_len;
    m = sqrt(d1->v[0] * d1->v[0] + d1->v[1] * d1->v[1] + d1->v[2] * d1->v[2]);
    if (m > 1e-12) {
        emcmotStatus->current_dir.x = d1->v[0] / m;
        emcmotStatus->current_dir.y = d1->v[1] / m;
        emcmotStatus->current_dir.z = d1->v[2] / m;
    } else {
        emcmotStatus->current_dir.x = 0;
        emcmotStatus->current_dir.y = 0;
        emcmotStatus->current_dir.z = 0;
    }
    tpnPoseFromVec(&end, &sg->geom.p1);
    emcPoseSub(&end, &tp->currentPos, &emcmotStatus->dtg);
}

static void popFront(TP_STRUCT * const tp)
{
    q_start = (q_start + 1) % TPN_QSIZE;
    q_len--;
    tp->queue._len = q_len;
    tp->depth = q_len;
    emcmotStatus->tcqlen = q_len;
}

/* returns nonzero while the move has to wait before it may start */
static int activate(TP_STRUCT * const tp, tpn_seg *sg)
{
    int s;
    if (sg->active) {
        return 0;
    }
    if (sg->atspeed) {
        for (s = 0; s < emcmotConfig->numSpindles; s++) {
            if (!emcmotStatus->spindle_status[s].at_speed) {
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
    sg->active = 1;
    toggleDIOs(&sg->syncdio);
    tp->execTag = sg->tag;
    return 0;
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
    double dt = tp->cycleTime;
    int stepping = emcmotStatus->stepping;
    g_dt = dt;

    if (q_len == 0) {
        queueReset(tp);
        statusIdle(tp);
        tpResume(tp);
        return TP_ERR_WAITING;
    }

    if (tp->aborting && cur_v <= 0.0 && cur_a == 0.0) {
        queueReset(tp);
        statusIdle(tp);
        tp->spindle.waiting_for_index = MOTION_INVALID_ID;
        tp->spindle.waiting_for_atspeed = MOTION_INVALID_ID;
        return TP_ERR_STOPPED;
    }

    /* moves the controller has left */
    while (q_len > 1 && cur_s >= ownedEnd(seg(0))) {
        if (finish(seg(0))) {
            return TP_ERR_WAITING;
        }
        popFront(tp);
    }
    if (q_len == 1 && cur_s >= segEnd(seg(0)) && cur_v <= 0.0) {
        if (finish(seg(0))) {
            return TP_ERR_WAITING;
        }
        tpnPoseFromVec(&tp->currentPos, &seg(0)->geom.p1);
        popFront(tp);
        queueReset(tp);
        statusIdle(tp);
        return TP_ERR_OK;
    }
    if (activate(tp, seg(0))) {
        return TP_ERR_WAITING;
    }

    double scale = emcmotStatus->net_feed_scale;
    if (tp->pausing || tp->aborting) {
        scale = 0.0;
    }
    if (scale < 0.0) {
        scale = 0.0;
    }

    tpn_step st;
    gather(tp, scale, stepping, &st);
    double j = chooseJerk(tp, &st);
    double jf;
    if (scale > 0.0 && finishStop(tp, &st, &jf)) {
        j = jf;
    }
    tpn_next n;
    stepState(j, dt, &n);

    /* land exactly on a stop */
    if (n.s1 >= st.Sstop - 1e-9 || (n.v1 < 1e-6 && st.Sstop - n.s1 < 1e-6 && n.a1 <= 0.0)) {
        n.s1 = fmin(n.s1, st.Sstop);
        if (st.Sstop - n.s1 < 1e-6) {
            n.s1 = st.Sstop;
            n.v1 = 0.0;
            n.a1 = 0.0;
        }
    }
    if (n.v1 <= 0.0) {
        n.v1 = 0.0;
        if (n.a1 < 0.0) {
            n.a1 = 0.0;
        }
    }
    /* paused or aborting and at rest */
    if (scale == 0.0 && n.v1 < 0.01 * st.J * dt * dt && fabs(n.a1) < 0.01 * st.J * dt) {
        n.v1 = 0.0;
        n.a1 = 0.0;
    }
#ifdef TPN_TRACE
    rtapi_print("T s=%.12f v=%.9f a=%.6f j=%.3f Sstop=%.12f db=%d/%d ncon=%d V=%.4f A=%.3f J=%.1f\n",
            n.s1, n.v1, n.a1, j, st.Sstop, db_i, db_n, ncon, st.V, st.A, st.J);
    if (st.Sstop < TPN_BIG) {
        rtapi_print("   d=%.9f bd=%.9f slack=%.9f\n", st.Sstop - n.s1,
                tpnBrakeDist(n.v1, n.a1, 0.0, st.A, st.J), 4.0 * st.J * dt * dt * dt);
    }
#endif
    cur_j = (n.a1 - cur_a) / dt;
    cur_s = n.s1;
    cur_v = n.v1;
    cur_a = n.a1;

    tpn_vec p, d1;
    poseAt(cur_s, &p, &d1);
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
