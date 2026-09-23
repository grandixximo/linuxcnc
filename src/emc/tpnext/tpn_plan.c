/********************************************************************
* Description: tpn_plan.c
*   Queue build of the tpnext trajectory planner: per piece limits,
*   corner blends sized to G64 P and E, and the backward envelope.
*
* License: GPL Version 2
* System: Linux
********************************************************************/
#include <rtapi.h>
#include <rtapi_math.h>
#include <emcpose.h>
#include <motion_types.h>
#include "../motion/motion.h"
#include "../tp/tp.h"
#include "tpn.h"

/* fraction of the INI limits the planner aims at */
#define TPN_LIMIT_SCALE 0.99
/* fraction of the blend tolerance the blend may use */
#define TPN_TOL_SCALE 0.98
#define TPN_DEV_SAMPLES 32

static void readAxisLimits(TP_STRUCT const *tp, tpn_axlim *ax)
{
    int i;
    int trapezoid = tpn.emcmotStatus->planner_type != 1;
    for (i = 0; i < TPN_NAX; i++) {
        double v = tpn.axis_get_vel_limit ? tpn.axis_get_vel_limit(i) : 0.0;
        double a = tpn.axis_get_acc_limit ? tpn.axis_get_acc_limit(i) : 0.0;
        double j = tpn.axis_get_jerk_limit ? tpn.axis_get_jerk_limit(i) : 0.0;
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
    if (tpn.axis_is_angular) {
        tpn.lin_mask = tpn.ang_mask = 0;
        for (i = 0; i < TPN_NAX; i++) {
            if (tpn.axis_is_angular(i)) {
                tpn.ang_mask |= 1u << i;
            } else {
                tpn.lin_mask |= 1u << i;
            }
        }
    }
}

/* velocity cap from [TRAJ]MAX_LINEAR_VELOCITY on the speed of the linear
 * axes among XYZ, else among UVW, the way the feed is measured */
static double linearCap(TP_STRUCT const *tp, tpn_vec const *G)
{
    double g2[2] = {0.0, 0.0};
    int i;
    for (i = 0; i < TPN_NAX; i++) {
        if ((tpn.lin_mask & (1u << i)) && (i < 3 || i >= 6)) {
            g2[i >= 6] += G->v[i] * G->v[i];
        }
    }
    double gx = sqrt(g2[0] > 1e-18 ? g2[0] : g2[1]);
    if (tp->vLimit <= 0.0 || gx < 1e-9) {
        return TPN_BIG;
    }
    return tp->vLimit / gx;
}

static double speedFactor(TP_STRUCT const *tp)
{
    (void)tp;
    return fmax(tpn.emcmotConfig->maxFeedScale, 1.0);
}

/* Backward envelope with half of each piece's tangential acceleration and
 * zero speed at the end of the queue and at every stop. The runtime
 * checks the true caps inside its braking horizon; the envelope carries
 * everything beyond it. */
static void backwardPass(void)
{
    double Enext = 0.0;
    int i;
    for (i = tpn.q_len - 1; i >= 0; i--) {
        tpn_seg *sg = seg(i);
        double len = fmax(0.0, sg->geom.L - sg->h_in - sg->h_out);
        double Eint = fmin(sg->lim_int.V, sqrt(Enext * Enext + sg->lim_int.A * len));
        double Ebin = Eint;
        if (sg->h_in > 0.0) {
            Ebin = fmin(sg->lim_bin.V, sqrt(Eint * Eint + sg->lim_bin.A * 2.0 * sg->h_in));
        }
        int same = (i < tpn.q_len - 2) && Eint == sg->E_int && Ebin == sg->E_bin;
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

static double blendDevAt(tpn_seg const *prev, tpn_seg const *sg, tpn_blend const *b,
        double sigma, tpn_vec const *w)
{
    tpn_vec p;
    tpnBlendEval(b, sigma, &p, 0);
    return fmin(tpnGeomDist(&prev->geom, &p, w), tpnGeomDist(&sg->geom, &p, w));
}

/* Largest distance of the blend from the programmed path, the linear
 * axes in units of G64 P and the angular ones in units of G64 E, so a
 * value up to 1 keeps every blend point within P and E of one point of
 * the path. Sampled, then refined around the worst sample. */
static double blendDeviation(tpn_seg const *prev, tpn_seg const *sg, tpn_blend const *b,
        tpn_vec const *w)
{
    double dev = 0.0, h = b->H / TPN_DEV_SAMPLES;
    int k, kmax = 1;
    for (k = 1; k < TPN_DEV_SAMPLES; k++) {
        double d = blendDevAt(prev, sg, b, h * k, w);
        if (d > dev) {
            dev = d;
            kmax = k;
        }
    }
    double lo = h * (kmax - 1), hi = h * (kmax + 1);
    for (k = 0; k < 20; k++) {
        double m1 = hi - 0.618034 * (hi - lo), m2 = lo + 0.618034 * (hi - lo);
        double d1 = blendDevAt(prev, sg, b, m1, w);
        double d2 = blendDevAt(prev, sg, b, m2, w);
        dev = fmax(dev, fmax(d1, d2));
        if (d1 > d2) {
            hi = m2;
        } else {
            lo = m1;
        }
    }
    return dev;
}

static void blendLimits(TP_STRUCT const *tp, tpn_axlim const *ax, tpn_seg const *prev,
        tpn_seg const *sg, tpn_blend const *b, tpn_lim *lim)
{
    tpn_vec G, G1, G2;
    tpnBlendBounds(b, &G, &G1, &G2);
    tpnLimits(ax, &G, &G1, &G2, lim);
    lim->V = fmin(lim->V, linearCap(tp, &G));
    lim->V = fmin(lim->V, fmin(prev->vreq, sg->vreq) * speedFactor(tp));
}

/* Smallest acceleration and jerk limits of the pieces between the
 * current position and S. */
static void runLimits(double S, double *A, double *J)
{
    int i;
    *A = TPN_BIG;
    *J = TPN_BIG;
    for (i = 0; i < tpn.q_len; i++) {
        tpn_seg *sg = seg(i);
        if (ownedStart(sg) >= S) {
            break;
        }
        if (sg->h_in > 0.0 && sg->S0 + sg->h_in > tpn.cur_s) {
            *A = fmin(*A, sg->lim_bin.A);
            *J = fmin(*J, sg->lim_bin.J);
        }
        if (ownedEnd(sg) > tpn.cur_s && sg->S0 + sg->h_in < S) {
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
    if (tpn.q_len == 0) {
        return 1;
    }
    runLimits(S, &A, &J);
    if (A >= TPN_BIG) {
        return 1;
    }
    /* one cycle of margin for the step that is about to be taken */
    double d = S - tpn.cur_s - 2.0 * tpn.cur_v * 0.001 - 1e-9;
    return tpnBrakeDist(tpn.cur_v, tpn.cur_a, V, A * TPN_BRAKE_SCALE, J * TPN_BRAKE_SCALE) <= d;
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
        hmax = fmin(hmax, segEnd(prev) - tpn.cur_s - 4.0 * tpn.cur_v * tp->cycleTime - 1e-6);
    }
    if (hmax < 1e-6) {
        stop = 1;
    }
    if (stop) {
        sg->stop_in = 1;
        return;
    }

    double tol = prev->tolerance * TPN_TOL_SCALE;
    double atol = prev->ang_tolerance * TPN_TOL_SCALE;
    double h = hmax;
    tpn_blend b;
    tpn_lim lim;
    int k;
    blendBuild(prev, sg, h, &b);
    if (tol > 0.0 || atol > 0.0) {
        tpn_vec w;
        for (k = 0; k < TPN_NAX; k++) {
            double t = (tpn.ang_mask & (1u << k)) ? atol : tol;
            w.v[k] = t > 0.0 ? 1.0 / t : 0.0;
        }
        for (k = 0; k < 40; k++) {
            double r = blendDeviation(prev, sg, &b, &w);
            if (r <= 1.0) {
                break;
            }
            h *= fmax(0.1, fmin(0.95, 1.0 / r));
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

int tpnAddSegment(TP_STRUCT * const tp, tpn_seg *sg, int canon_type, double vel,
        double ini_maxvel, unsigned char enables, char atspeed, int indexer_jnum,
        struct state_tag_t tag)
{
    tpn_axlim ax;
    tpn_vec G, G1, G2;

    if (!tp || tp->aborting) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tpnext: cannot queue a move while aborting\n");
        return TP_ERR_FAIL;
    }
    if (tpn.q_len >= TPN_QSIZE) {
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
    sg->ang_tolerance = tpn.ang_tolerance;
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
    sg->lim_int.V = fmin(sg->lim_int.V, linearCap(tp, &G));
    double vmax = sg->vreq * speedFactor(tp);
    if (ini_maxvel > 0.0) {
        vmax = fmin(vmax, ini_maxvel);
    }
    sg->lim_int.V = fmin(sg->lim_int.V, vmax);

    if (tpn.q_len > 0) {
        joinMoves(tp, &ax, seg(tpn.q_len - 1), sg);
    } else {
        sg->S0 = tpn.cur_s;
        sg->stop_in = 1;
    }

    tpn.q_len++;
    tp->queue._len = tpn.q_len;
    tp->depth = tpn.q_len;
    tp->done = 0;
    tpn.emcmotStatus->tcqlen = tpn.q_len;
    backwardPass();
    return TP_ERR_OK;
}
