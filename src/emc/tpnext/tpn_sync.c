/********************************************************************
* Description: tpn_sync.c
*   Spindle position synchronization (G33) for the tpnext trajectory
*   planner.
*
*   Every spindle's encoder position is smoothed by a tracking filter.
*   A synchronized move starts on the spindle index; from then on the
*   path parameter follows the spindle at the move's pitch. The lag of
*   the start is not dropped: a jerk limited catch up, planned on the
*   index, gains it back, so the thread lies at the same place whatever
*   the spindle speed. The per cycle controller follows the reference and
*   corrects what the plan does not know about.
*
* License: GPL Version 2
* System: Linux
********************************************************************/
#include <rtapi.h>
#include <rtapi_math.h>
#include <motion_types.h>
#include "../motion/motion.h"
#include "../tp/tp.h"
#include "tpn.h"

/* bandwidth of the spindle position filter, rad/s: a load dip costs
 * about 3 da / wn^2 of thread, a coarse encoder jitter growing with it */
#define TPN_SPINDLE_WN 80.0
/* the catch up peaks at most this close to the speed cap */
#define TPN_SYNC_VCAP 0.995
/* motion's cubic interpolator puts a trajectory point on the joints this
 * many trajectory cycles later, so the reference leads the spindle by it */
#define TPN_SYNC_LEAD 2.0
/* a thread starts once the spindle changes speed by less than this
 * fraction per second for TPN_SYNC_SETTLE seconds, or after
 * TPN_SYNC_SETTLE_MAX when a coarse encoder never looks that steady */
#define TPN_SYNC_STEADY 0.25
#define TPN_SYNC_SETTLE 0.1
#define TPN_SYNC_SETTLE_MAX 0.5

static struct {
    double pos, vel, acc;
    int init;
    int steady;         /* cycles it has changed speed slowly */
} spest[EMCMOT_MAX_SPINDLES];

/* jerk phases of the catch up, from rest at the start of the move */
#define TPN_PLAN_MAX 8

static struct {
    int on;
    int spindle;
    /* the synchronized move that starts at path parameter S and spindle
     * position R, with length L and pitch p */
    double S, R, L, p;
    /* catch up: phases, elapsed time, and the reference and its speed
     * when it was planned */
    double pj[TPN_PLAN_MAX], pt[TPN_PLAN_MAX];
    int np;
    double T, tau, r0, v0, S0;
    /* spindle revolutions the thread lies behind the index when the catch
     * up could not be done, the way tp.c starts every thread */
    double shift;
    /* cycles the spindle has outrun the axes */
    int cycles;
    /* cycles waited for a steady spindle */
    int waited;
} sy;

void tpnSyncReset(void)
{
    sy.on = 0;
    sy.np = 0;
    sy.cycles = 0;
    tpn.track = 0;
}

int tpnSyncOn(void)
{
    return sy.on;
}

/* the spindle position the joints have to match when this cycle's
 * point reaches them */
static double spindleAhead(int k, double dt)
{
    double t = TPN_SYNC_LEAD * dt;
    return spest[k].pos + spest[k].vel * t + 0.5 * spest[k].acc * t * t;
}

static double signedRevs(int k)
{
    spindle_status_t const *sp = &tpn.emcmotStatus->spindle_status[k];
    return sp->direction < 0 ? -sp->spindleRevs : sp->spindleRevs;
}

/* Track every spindle's position with a critically damped third order
 * filter, so that a synchronized move starts from a settled estimate.
 * A jump (the index reset, a direction change) moves the estimate with
 * it. */
void tpnSpindleEstimate(TP_STRUCT const *tp)
{
    double dt = tp->cycleTime;
    double lam = exp(-TPN_SPINDLE_WN * dt);
    double ka = 1.0 - lam * lam * lam;
    double kv = 1.5 * (1.0 - lam) * (1.0 - lam) * (1.0 + lam) / dt;
    double kc = 2.0 * (1.0 - lam) * (1.0 - lam) * (1.0 - lam) / (dt * dt);
    int k;
    for (k = 0; k < tpn.emcmotConfig->numSpindles && k < EMCMOT_MAX_SPINDLES; k++) {
        double raw = signedRevs(k);
        if (!spest[k].init) {
            spest[k].pos = raw;
            spest[k].vel = spest[k].acc = 0.0;
            spest[k].init = 1;
            spest[k].steady = 0;
            continue;
        }
        double pp = spest[k].pos + spest[k].vel * dt + 0.5 * spest[k].acc * dt * dt;
        double r = raw - pp;
        if (fabs(r) > 0.25) {
            spest[k].pos = raw;
            spest[k].vel += spest[k].acc * dt;
            continue;
        }
        spest[k].pos = pp + ka * r;
        spest[k].vel += spest[k].acc * dt + kv * r;
        spest[k].acc += kc * r;
        if (fabs(spest[k].acc) < TPN_SYNC_STEADY * fabs(spest[k].vel)) {
            spest[k].steady++;
        } else {
            spest[k].steady = 0;
        }
    }
}

/* jerk phases of a speed change from v0 to v1 with zero acceleration at
 * both ends; returns the number of phases added at ph */
static int speedChange(double v0, double v1, double A, double J, double *pj, double *pt)
{
    double dv = fabs(v1 - v0), s = v1 >= v0 ? 1.0 : -1.0;
    if (dv < 1e-12) {
        return 0;
    }
    if (dv >= A * A / J) {
        pj[0] = s * J; pt[0] = A / J;
        pj[1] = 0.0; pt[1] = dv / A - A / J;
        pj[2] = -s * J; pt[2] = A / J;
        return 3;
    }
    pj[0] = s * J; pt[0] = sqrt(dv / J);
    pj[1] = -s * J; pt[1] = pt[0];
    return 2;
}

static double phaseTime(int n, double const *pt)
{
    double T = 0.0;
    int k;
    for (k = 0; k < n; k++) {
        T += pt[k];
    }
    return T;
}

/* Distance gained on a reference running at vr by speeding from rest up
 * to vp, holding vp for tc and slowing to vr, less the lag e0 to take
 * back; builds the phases. The speed changes are symmetric, so each
 * covers its mean speed times its time. */
static double catchUp(double vp, double tc, double vr, double e0, double A, double J)
{
    int n1 = speedChange(0.0, vp, A, J, sy.pj, sy.pt);
    double T1 = phaseTime(n1, sy.pt);
    sy.pj[n1] = 0.0;
    sy.pt[n1] = tc;
    int n3 = speedChange(vp, vr, A, J, sy.pj + n1 + 1, sy.pt + n1 + 1);
    double T3 = phaseTime(n3, sy.pt + n1 + 1);
    sy.np = n1 + 1 + n3;
    sy.T = T1 + tc + T3;
    return (vp - vr) * tc + 0.5 * (vp - vr) * T3 + (0.5 * vp - vr) * T1 - e0;
}

/* Plan the catch up from rest at the start of the move: the fastest
 * jerk limited profile that ends on the reference at its speed, peaking
 * above it just enough to gain back the lag. */
static void planCatchUp(double e0, double vr, tpn_lim const *lim)
{
    double A = lim->A * TPN_SYNC_AMARGIN, J = lim->J * TPN_SYNC_JMARGIN;
    double V = lim->V * TPN_SYNC_VCAP;
    int k;
    sy.np = 0;
    sy.T = 0.0;
    if (vr <= 0.0 || A <= 0.0 || J <= 0.0) {
        return;
    }
    if (vr >= V) {
        /* no speed left to catch up with */
        sy.T = TPN_BIG;
        return;
    }
    double f = catchUp(V, 0.0, vr, e0, A, J);
    if (f < 0.0) {
        catchUp(V, -f / (V - vr), vr, e0, A, J);
        return;
    }
    double lo = vr, hi = V;
    for (k = 0; k < 50; k++) {
        double mid = 0.5 * (lo + hi);
        if (catchUp(mid, 0.0, vr, e0, A, J) < 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    catchUp(hi, 0.0, vr, e0, A, J);
}

/* position, speed and acceleration of the plan at time t */
static void planEval(double t, double *x, double *v, double *a)
{
    int k;
    *x = *v = *a = 0.0;
    for (k = 0; k < sy.np && t > 0.0; k++) {
        double h = fmin(sy.pt[k], t), j = sy.pj[k];
        *x += *v * h + 0.5 * *a * h * h + j * h * h * h / 6.0;
        *v += *a * h + 0.5 * j * h * h;
        *a += j * h;
        t -= h;
    }
    if (t > 0.0) {
        *x += *v * t + 0.5 * *a * t * t;
        *v += *a * t;
    }
}

/* Travel the synchronized moves queued from sg on lack to reach sync
 * before they brake for their end, or TPN_BIG when the axes have no
 * speed left to catch up with; 0 when the catch up fits. */
static double leadInShort(tpn_seg const *sg)
{
    double x, v, a, end = sg->S0;
    int i;
    if (sy.T >= TPN_BIG) {
        return TPN_BIG;
    }
    if (sy.np == 0) {
        return 0.0;
    }
    for (i = 0; i < tpn.q_len && seg(i) != sg; i++) {
    }
    for (; i < tpn.q_len && seg(i)->sync == TC_SYNC_POSITION && seg(i)->S0 == end; i++) {
        end = segEnd(seg(i));
    }
    /* synchronized from the end of the catch up to where the stop at
     * the end of the thread starts to brake */
    planEval(sy.T, &x, &v, &a);
    x += tpnBrakeDist(v, 0.0, 0.0, sg->lim_int.A * TPN_BRAKE_SCALE,
            sg->lim_int.J * TPN_BRAKE_SCALE);
    return fmax(0.0, sy.S0 + x - end);
}

/* Start following the spindle: wait until every spindle is at speed,
 * then for the index of this one; the move starts on the index. Returns
 * nonzero while waiting. */
int tpnSyncStart(TP_STRUCT * const tp, tpn_seg *sg)
{
    int s, k = sg->spindle;
    spindle_status_t *sp = &tpn.emcmotStatus->spindle_status[k];
    double dt = tp->cycleTime;
    if (tp->spindle.waiting_for_index != sg->id) {
        for (s = 0; s < tpn.emcmotConfig->numSpindles; s++) {
            if (!tpn.emcmotStatus->spindle_status[s].at_speed) {
                tp->spindle.waiting_for_atspeed = sg->id;
                sy.waited = 0;
                return 1;
            }
        }
        tp->spindle.waiting_for_atspeed = MOTION_INVALID_ID;
        /* the catch up is planned for the speed on the index: let the
         * estimate settle after a speed change */
        if (spest[k].steady * dt < TPN_SYNC_SETTLE && ++sy.waited * dt < TPN_SYNC_SETTLE_MAX) {
            return 1;
        }
        sy.waited = 0;
        tp->spindle.waiting_for_index = sg->id;
        sp->spindle_index_enable = 1;
        return 1;
    }
    if (sp->spindle_index_enable) {
        return 1;
    }
    /* the encoder counts from the index now */
    tp->spindle.waiting_for_index = MOTION_INVALID_ID;
    spest[k].pos = signedRevs(k);
    sy.on = 1;
    sy.spindle = k;
    sy.S = sg->S0;
    sy.R = 0.0;
    sy.L = sg->geom.L;
    sy.p = sg->uu_per_rev;
    sy.cycles = 0;
    /* this cycle's output aims at the reference now, the axis is at rest
     * one cycle before it */
    sy.S0 = tpn.cur_s;
    sy.r0 = sy.S + sy.p * spindleAhead(k, dt);
    sy.v0 = sy.p * spest[k].vel;
    sy.tau = 0.0;
    sy.shift = 0.0;
    planCatchUp(sy.r0 - sy.v0 * dt - sy.S0, sy.v0, &sg->lim_int);
    double need = leadInShort(sg);
    if (need > 0.0) {
        /* Too short to gain the lag back: ramp up to the spindle's speed
         * and leave the thread behind by the lag, as tp.c does. The shift
         * depends on the speed only, so passes at one speed still line
         * up. */
        double x, v, a;
        sy.np = speedChange(0.0, sy.v0, sg->lim_int.A * TPN_SYNC_AMARGIN,
                sg->lim_int.J * TPN_SYNC_JMARGIN, sy.pj, sy.pt);
        sy.T = phaseTime(sy.np, sy.pt);
        planEval(sy.T, &x, &v, &a);
        sy.shift = (sy.r0 + sy.v0 * (sy.T - dt) - (sy.S0 + x)) / sy.p;
        if (need >= TPN_BIG) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "spindle-synchronized move %d: the axes have no speed left to "
                    "catch up with the spindle, the thread is shifted by %f\n",
                    sg->id, sy.shift * sy.p);
        } else {
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "spindle-synchronized move %d: lead-in too short to reach sync, "
                    "need %f more travel, the thread is shifted by %f\n",
                    sg->id, need, sy.shift * sy.p);
        }
    }
    return 0;
}

/* The spindle reference for the controller. Consecutive synchronized
 * moves continue the thread: each starts where the spindle has turned
 * the previous move's length at its pitch. While the catch up runs the
 * reference is held back by the lag it still has to gain. */
void tpnSyncReference(TP_STRUCT const *tp)
{
    double dt = tp->cycleTime;
    /* while the catch up runs it holds the reference back itself */
    double th = spindleAhead(sy.spindle, dt) - (sy.np > 0 ? 0.0 : sy.shift);
    int i;
    for (;;) {
        double end = sy.S + sy.L;
        if (sy.S + sy.p * (th - sy.R) <= end) {
            break;
        }
        for (i = 0; i < tpn.q_len && seg(i)->S0 != end; i++) {
        }
        if (i == tpn.q_len || seg(i)->sync != TC_SYNC_POSITION) {
            break;
        }
        sy.R += sy.L / sy.p;
        sy.S = end;
        sy.L = seg(i)->geom.L;
        sy.p = seg(i)->uu_per_rev;
    }
    tpn.s_ref = sy.S + sy.p * (th - sy.R);
    tpn.v_ref = sy.p * spest[sy.spindle].vel;
    tpn.a_ref = sy.p * spest[sy.spindle].acc;
    tpn.j_ref = 0.0;
    if (sy.np > 0) {
        double x, v, a, xb, vb, ab;
        sy.tau += dt;
        /* the cycle that holds the end of the plan still follows it */
        if (sy.tau - dt >= sy.T) {
            sy.np = 0;
            tpn.s_ref -= sy.p * sy.shift;
            return;
        }
        planEval(sy.tau, &x, &v, &a);
        /* the jerk of the plan averaged over this cycle */
        planEval(sy.tau - dt, &xb, &vb, &ab);
        tpn.j_ref = (a - ab) / dt;
        tpn.s_ref -= sy.r0 + sy.v0 * (sy.tau - dt) - (sy.S0 + x);
        tpn.v_ref -= sy.v0 - v;
        /* right after a speed change the spindle's acceleration estimate
         * is still settling: the plan alone sets the acceleration */
        tpn.a_ref = a;
    }
}

/* Report a spindle that outruns the axes: its speed at the move's pitch
 * above the speed cap of the pieces under way for the whole window. The estimate is smooth, so unlike tp.c this
 * needs no wait for the motion to pin at its cap, which a short move
 * never does. */
void tpnSyncOverrun(TP_STRUCT * const tp)
{
    double dt = tp->cycleTime;
    int window = (int)(TP_SYNC_OVERRUN_WINDOW / dt);
    double demand = fabs(sy.p * spest[sy.spindle].vel);
    if (tp->spindle.overrun_reported || demand <= tpn.step_V) {
        sy.cycles = 0;
        return;
    }
    if (++sy.cycles < window) {
        return;
    }
    sy.cycles = 0;
    tpn.emcmotStatus->syncOverrunSpindle = sy.spindle + 1;
    tpn.emcmotStatus->syncOverrunError = demand - tpn.step_V;
    tp->spindle.overrun_reported = 1;
}
