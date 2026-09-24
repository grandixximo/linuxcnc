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
#define TPN_DEV_SAMPLES 16
#define TPN_DEV_REFINE 14
/* blend sizes tried below the tolerance, each half the one before, down
 * to one crossed in TPN_BLEND_CYCLES cycles */
#define TPN_SIZE_TRIES 6
#define TPN_BLEND_CYCLES 4.0

/* A position synchronized move takes the trapezoid jerk whatever the
 * planner: it must follow the spindle, and a lower jerk only delays the
 * catch-up and the turns of a thread chain. */
static void readAxisLimits(TP_STRUCT const *tp, tpn_axlim *ax)
{
    int i;
    int trapezoid = tpn.emcmotStatus->planner_type != 1
            || tp->synchronized == TC_SYNC_POSITION;
    for (i = 0; i < TPN_NAX; i++) {
        double v = tpn.axis_get_vel_limit ? tpn.axis_get_vel_limit(i) : 0.0;
        double a = tpn.axis_get_acc_limit ? tpn.axis_get_acc_limit(i) : 0.0;
        double j = tpn.axis_get_jerk_limit ? tpn.axis_get_jerk_limit(i) : 0.0;
        /* an axis without limits is not part of the machine; it can
         * only appear in a move with zero displacement */
        ax->vel[i] = v > 0.0 ? v * TPN_LIMIT_SCALE : TPN_BIG;
        ax->acc[i] = a > 0.0 ? a * TPN_LIMIT_SCALE : TPN_BIG;
        /* acceleration may change within two cycles, and no faster:
         * a higher jerk cannot be shaped at this cycle time */
        double jtrap = ax->acc[i] / (2.0 * tp->cycleTime);
        ax->jerk[i] = trapezoid || j <= 0.0 ? jtrap : fmin(j * TPN_LIMIT_SCALE, jtrap);
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
        int same = (i < tpn.q_len - 2) && Eint == sg->E_int;
        double E = Eint;
        int k;
        sg->E_int = Eint;
        if (sg->h_in > 0.0) {
            double part = 2.0 * sg->h_in / TPN_NSUB;
            for (k = TPN_NSUB - 1; k >= 0; k--) {
                E = fmin(sg->lim_sub[k].V, sqrt(E * E + sg->lim_sub[k].A * part));
                same = same && E == sg->E_sub[k];
                sg->E_sub[k] = E;
            }
        }
        Enext = sg->stop_in ? 0.0 : E;
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

/* Distance of the blend point at sigma from the path. Once the distance
 * to the nearer move is no more than floor, the other one cannot lift
 * the result above floor, and is skipped. */
static double blendDevAt(tpn_seg const *prev, tpn_seg const *sg, tpn_blend const *b,
        double sigma, tpn_vec const *w, double floor)
{
    tpn_vec p;
    tpnBlendEval(b, sigma, &p, 0);
    int first_prev = sigma < 0.5 * b->H;
    double d = tpnGeomDist(first_prev ? &prev->geom : &sg->geom, &p, w);
    if (d <= floor) {
        return d;
    }
    return fmin(d, tpnGeomDist(first_prev ? &sg->geom : &prev->geom, &p, w));
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
        double d = blendDevAt(prev, sg, b, h * k, w, dev);
        if (d > dev) {
            dev = d;
            kmax = k;
        }
    }
    /* golden section, one new point per step */
    double const g = 0.6180339887498949;
    double lo = h * (kmax - 1), hi = h * (kmax + 1);
    double m1 = hi - g * (hi - lo), m2 = lo + g * (hi - lo);
    double d1 = blendDevAt(prev, sg, b, m1, w, -1.0);
    double d2 = blendDevAt(prev, sg, b, m2, w, -1.0);
    for (k = 0; k < TPN_DEV_REFINE; k++) {
        if (d1 > d2) {
            hi = m2;
            m2 = m1;
            d2 = d1;
            m1 = hi - g * (hi - lo);
            d1 = blendDevAt(prev, sg, b, m1, w, -1.0);
        } else {
            lo = m1;
            m1 = m2;
            d1 = d2;
            m2 = lo + g * (hi - lo);
            d2 = blendDevAt(prev, sg, b, m2, w, -1.0);
        }
    }
    return fmax(dev, fmax(d1, d2));
}

/* derivative bounds and speed caps of each part of the blend */
typedef struct {
    tpn_vec G[TPN_NSUB], G1[TPN_NSUB], G2[TPN_NSUB];
    tpn_caps caps[TPN_NSUB];
    double vcap[TPN_NSUB];
} tpn_parts;

static void blendParts(TP_STRUCT const *tp, tpn_axlim const *ax, tpn_seg const *prev,
        tpn_seg const *sg, tpn_blend const *b, tpn_parts *pt)
{
    double vcap = fmin(prev->vreq, sg->vreq) * speedFactor(tp);
    int k;
    for (k = 0; k < TPN_NSUB; k++) {
        tpn_blend part;
        tpnBlendPart(b, (double)k / TPN_NSUB, (double)(k + 1) / TPN_NSUB, &part);
        tpnBlendBounds(&part, &pt->G[k], &pt->G1[k], &pt->G2[k]);
        tpnLimitCaps(ax, &pt->G[k], &pt->G1[k], &pt->G2[k], &pt->caps[k]);
        pt->vcap[k] = fmin(vcap, pt->caps[k].Vg);
    }
}

/* limits of each part of the blend, with its second and third derivative
 * bounds scaled by r and r^2, and the smallest of them in lim */
static void partLimits(tpn_axlim const *ax, tpn_parts const *pt, double r, tpn_lim *lim,
        tpn_lim *sub)
{
    double s2 = r == 1.0 ? 1.0 : sqrt(r);
    double s3 = r == 1.0 ? 1.0 : pow(r, 2.0 / 3.0);
    int k;
    lim->V = lim->A = lim->J = TPN_BIG;
    for (k = 0; k < TPN_NSUB; k++) {
        tpn_lim l;
        double V = fmin(pt->vcap[k], fmin(pt->caps[k].V2 / s2, pt->caps[k].V3 / s3));
        tpnLimitsAt(ax, &pt->G[k], &pt->G1[k], &pt->G2[k], r, V, pt->caps[k].curved, &l);
        lim->V = fmin(lim->V, l.V);
        lim->A = fmin(lim->A, l.A);
        lim->J = fmin(lim->J, l.J);
        if (sub) {
            sub[k] = l;
        }
    }
}

/* limits of each part of the blend, and the smallest of them in lim */
static void blendLimits(TP_STRUCT const *tp, tpn_axlim const *ax, tpn_seg const *prev,
        tpn_seg const *sg, tpn_blend const *b, tpn_lim *lim, tpn_lim *sub)
{
    tpn_parts pt;
    blendParts(tp, ax, prev, sg, b, &pt);
    partLimits(ax, &pt, 1.0, lim, sub);
}

/* Smallest acceleration and jerk limits of the pieces between from and
 * S, scanned back from the end of the queue. */
static void runLimits(double from, double S, double *A, double *J)
{
    int i;
    *A = TPN_BIG;
    *J = TPN_BIG;
    for (i = tpn.q_len - 1; i >= 0; i--) {
        tpn_seg *sg = seg(i);
        if (ownedEnd(sg) <= from) {
            break;
        }
        if (ownedStart(sg) >= S) {
            continue;
        }
        if (sg->h_in > 0.0 && sg->S0 + sg->h_in > from) {
            *A = fmin(*A, sg->lim_bin.A);
            *J = fmin(*J, sg->lim_bin.J);
        }
        if (sg->S0 + sg->h_in < S) {
            *A = fmin(*A, sg->lim_int.A);
            *J = fmin(*J, sg->lim_int.J);
        }
    }
}

#define TPN_SLOW_POINTS 8

/* The last TPN_SLOW_POINTS pieces or stops that start between the
 * controller and S with a speed cap no higher than V, latest first: start
 * P, cap Vp and acceleration limit Ap. Returns how many. */
static int slowPoints(double S, double V, double *P, double *Vp, double *Ap)
{
    int i, k, n = 0;
    for (i = tpn.q_len - 1; i >= 0 && n < TPN_SLOW_POINTS; i--) {
        tpn_seg const *sg = seg(i);
        if (ownedEnd(sg) <= tpn.cur_s) {
            break;
        }
        if (ownedStart(sg) >= S) {
            continue;
        }
        /* the interior, the parts of the blend and the stop, last first */
        for (k = TPN_NSUB; k >= -1 && n < TPN_SLOW_POINTS; k--) {
            double p, vp, ap;
            if (k == TPN_NSUB) {
                p = sg->S0 + sg->h_in;
                vp = sg->lim_int.V;
                ap = sg->lim_int.A;
            } else if (k >= 0) {
                if (sg->h_in <= 0.0) {
                    continue;
                }
                p = ownedStart(sg) + 2.0 * sg->h_in * k / TPN_NSUB;
                vp = sg->lim_sub[k].V;
                ap = sg->lim_sub[k].A;
            } else {
                if (!sg->stop_in) {
                    continue;
                }
                p = sg->S0;
                vp = ap = 0.0;
            }
            if (p > tpn.cur_s && p < S && vp <= V) {
                P[n] = p;
                Vp[n] = vp;
                Ap[n] = ap;
                n++;
            }
        }
    }
    return n;
}

/* Can the controller, from its current state, reach the new blend with a
 * speed at most V? */
static int reachable(double S, double V)
{
    double A, J;
    if (tpn.q_len == 0) {
        return 1;
    }
    /* one cycle of margin for the step that is about to be taken */
    double d = S - tpn.cur_s - 2.0 * tpn.cur_v * 0.001 - 1e-9;
    /* The limits of every queued piece bound those between the controller
     * and S from below, and a smaller limit only brakes longer: when the
     * bounds suffice the scan of the queue would agree. */
    if (tpn.A_lo < TPN_BIG && tpnBrakeDist(tpn.cur_v, tpn.cur_a, V, tpn.A_lo * TPN_BRAKE_SCALE,
                tpn.J_lo * TPN_BRAKE_SCALE) <= d) {
        return 1;
    }
    /* the controller may already have to slow down to V or below on the
     * way: then only the rest of the way counts */
    double P[TPN_SLOW_POINTS], Vp[TPN_SLOW_POINTS], Ap[TPN_SLOW_POINTS];
    int i, n = slowPoints(S, V, P, Vp, Ap);
    for (i = 0; i < n; i++) {
        /* past P the controller keeps v + a^2 / 2J within Vp, so it brakes
         * as from (Vp, 0) after ramping its acceleration out */
        runLimits(P[i], S, &A, &J);
        A *= TPN_BRAKE_SCALE;
        J *= TPN_BRAKE_SCALE;
        double ramp = Vp[i] * fmin(Ap[i], sqrt(2.0 * J * Vp[i])) / J;
        if (ramp + tpnBrakeDist(Vp[i], 0.0, V, A, J) <= S - P[i] - 1e-9) {
            return 1;
        }
    }
    runLimits(tpn.cur_s, S, &A, &J);
    if (A >= TPN_BIG) {
        return 1;
    }
    /* the scan may stop short of the last move, later ones will not */
    tpn_seg const *last = seg(tpn.q_len - 1);
    tpn.A_lo = fmin(A, last->lim_int.A);
    tpn.J_lo = fmin(J, last->lim_int.J);
    if (last->h_in > 0.0) {
        tpn.A_lo = fmin(tpn.A_lo, last->lim_bin.A);
        tpn.J_lo = fmin(tpn.J_lo, last->lim_bin.J);
    }
    if (tpnBrakeDist(tpn.cur_v, tpn.cur_a, V, A * TPN_BRAKE_SCALE, J * TPN_BRAKE_SCALE) <= d) {
        return 1;
    }
    return 0;
}

/* duration of the velocity change dv with zero acceleration at both ends */
static double rampTime(double dv, double A, double J)
{
    if (dv <= 0.0) {
        return 0.0;
    }
    if (dv * J <= A * A) {
        return 2.0 * sqrt(dv / J);
    }
    return dv / A + A / J;
}

/* Time to cover D from speed V, speeding up to at most vr under A and J */
static double sideTime(double V, double vr, double A, double J, double D)
{
    if (D <= 0.0) {
        return 0.0;
    }
    if (V >= vr) {
        return D / vr;
    }
    double T = rampTime(vr - V, A, J);
    double d = 0.5 * (V + vr) * T;
    if (d <= D) {
        return T + (D - d) / vr;
    }
    /* D ends before vr. A jerk limited ramp by dv = x^2 covers
     * (2 V x + x^3) / sqrt(J), so x solves x^3 + 2 V x = D sqrt(J), whose
     * one real root is Cardano's; past dv = A^2 / J the ramp holds A and
     * covers (V + dv / 2) (dv / A + A / J), a quadratic in dv. */
    double q = D * sqrt(J), p3 = 2.0 * V / 3.0;
    double disc = sqrt(0.25 * q * q + p3 * p3 * p3);
    double ca = pow(disc + 0.5 * q, 1.0 / 3.0), cb = pow(fmax(disc - 0.5 * q, 0.0), 1.0 / 3.0);
    /* ca - cb without the cancellation: ca^3 - cb^3 = q */
    double x = q / (ca * ca + ca * cb + cb * cb);
    double dv = x * x;
    if (dv * J > A * A) {
        /* dv^2 / (2A) + dv (V / A + A / (2J)) - (D - V A / J) = 0 */
        double a2 = 0.5 / A, b2 = V / A + 0.5 * A / J, c2 = D - V * A / J;
        dv = 2.0 * c2 / (b2 + sqrt(b2 * b2 + 4.0 * a2 * c2));
    }
    return rampTime(dv, A, J);
}

/* Estimated time over the interior before the corner, the blend of half
 * length h with the limits sub of its parts (none for a stop) and half of
 * the next move: each part of the blend at its speed cap, and speeding up
 * on the interiors on both sides. */
static double cornerTime(tpn_seg const *prev, tpn_seg const *sg, double h, tpn_lim const *sub,
        double vr)
{
    double A = fmin(prev->lim_int.A, sg->lim_int.A) * TPN_BRAKE_SCALE;
    double J = fmin(prev->lim_int.J, sg->lim_int.J) * TPN_BRAKE_SCALE;
    double Vin = 0.0, Vout = 0.0, tb = 0.0;
    int k;
    if (sub) {
        for (k = 0; k < TPN_NSUB; k++) {
            tb += 2.0 * h / TPN_NSUB / fmin(sub[k].V, vr);
        }
        Vin = fmin(sub[0].V, vr);
        Vout = fmin(sub[TPN_NSUB - 1].V, vr);
    }
    double Din = prev->geom.L - prev->h_in - h;
    double Dout = 0.5 * sg->geom.L - h;
    return tb + sideTime(Vin, vr, A, J, Din) + sideTime(Vout, vr, A, J, Dout);
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
    if (prev->sync != sg->sync || prev->tap) {
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
    tpn_lim lim, sub[TPN_NSUB];
    int k, i;
    int lines = prev->geom.type == TPN_LINE && sg->geom.type == TPN_LINE;
    blendBuild(prev, sg, h, &b);
    if (tol > 0.0 || atol > 0.0) {
        tpn_vec w;
        for (k = 0; k < TPN_NAX; k++) {
            double t = (tpn.ang_mask & (1u << k)) ? atol : tol;
            w.v[k] = t > 0.0 ? 1.0 / t : 0.0;
        }
        /* between two lines the blend only scales about the corner, and
         * its deviation with it */
        for (k = 0; k < 40; k++) {
            double r = blendDeviation(prev, sg, &b, &w);
            if (r <= 1.0) {
                break;
            }
            if (lines) {
                h /= r;
                blendBuild(prev, sg, h, &b);
                break;
            }
            /* aim a little inside so a nearly linear corner ends here */
            h *= fmax(0.1, 0.99 / r);
            blendBuild(prev, sg, h, &b);
        }
        if (k == 40) {
            sg->stop_in = 1;
            return;
        }
    }
    /* A longer blend is faster where it turns hardest, but only with
     * about the cube root of its length when the jerk limits it, and it
     * takes longer to cross: a sharp corner is often passed sooner with a
     * shorter blend, or a stop. Take the fastest of a few sizes inside
     * the tolerance and a stop, among those the controller can reach. */
    double vwant = fmin(prev->vreq, sg->vreq) * speedFactor(tp);
    /* judged at the programmed feed */
    double vr = fmin(fmin(prev->vreq, sg->vreq), fmin(prev->lim_int.V, sg->lim_int.V));
    /* The candidates are judged on the bounds of this blend scaled to
     * their size: exact between two lines, where the blend only scales
     * about the corner, an estimate elsewhere. The one taken gets its own
     * bounds. */
    tpn_parts pt;
    double h0 = h;
    blendParts(tp, ax, prev, sg, &b, &pt);
    partLimits(ax, &pt, 1.0, &lim, sub);
    /* the spindle sets the speed along a thread, and a stop there would
     * lose it: take the blend the tolerance allows */
    if (vr > 1e-6 && sg->sync != TC_SYNC_POSITION) {
        double best = TPN_BIG, hc = h;
        int found = 0, worse = 0;
        for (k = 0; k <= TPN_SIZE_TRIES; k++, hc *= 0.5) {
            tpn_lim lc = lim, subc[TPN_NSUB];
            if (hc < 1e-6) {
                break;
            }
            if (k > 0) {
                partLimits(ax, &pt, h0 / hc, &lc, subc);
                if (2.0 * hc < TPN_BLEND_CYCLES * fmin(lc.V, vr) * tp->cycleTime) {
                    break;
                }
            } else {
                for (i = 0; i < TPN_NSUB; i++) {
                    subc[i] = sub[i];
                }
            }
            if (lc.V < 1e-6 || lc.A < 1e-9 || !reachable(segEnd(prev) - hc, lc.V)) {
                continue;
            }
            double t = cornerTime(prev, sg, hc, subc, vr);
            /* the time has one valley over the sizes: two sizes in a row
             * slower than the best end the search */
            if (found && t >= best && ++worse >= 2) {
                break;
            }
            if (!found || t < 0.99 * best) {
                worse = 0;
                found = 1;
                best = t;
                h = hc;
                lim = lc;
                for (i = 0; i < TPN_NSUB; i++) {
                    sub[i] = subc[i];
                }
            }
        }
        /* and a stop at the corner, which is also all that is left when no
         * blend can be reached */
        if (!found || cornerTime(prev, sg, 0.0, 0, vr) < 0.99 * best) {
            sg->stop_in = 1;
            return;
        }
    }
    /* no longer than needed for the requested speed */
    if (lim.V >= vwant) {
        double lo = h * 1e-3, hi = h;
        for (k = 0; k < 14; k++) {
            double mid = 0.5 * (lo + hi);
            tpn_lim lm;
            partLimits(ax, &pt, h0 / mid, &lm, 0);
            if (lm.V >= vwant) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        h = hi;
        partLimits(ax, &pt, h0 / h, &lim, sub);
    }
    if (h != h0) {
        blendBuild(prev, sg, h, &b);
        if (!lines) {
            blendLimits(tp, ax, prev, sg, &b, &lim, sub);
        }
    }
    if (lim.V < 1e-6 || lim.A < 1e-9 || !reachable(segEnd(prev) - h, lim.V)) {
        sg->stop_in = 1;
        return;
    }
    sg->h_in = h;
    prev->h_out = h;
    sg->bin = b;
    sg->lim_bin = lim;
    for (k = 0; k < TPN_NSUB; k++) {
        sg->lim_sub[k] = sub[k];
    }
    sg->vreq_bin = fmin(prev->vreq, sg->vreq);
    /* the lower of the two caps, where 0 is none */
    sg->vlimit_bin = prev->vlimit_scale <= 0.0 ? sg->vlimit_scale
        : sg->vlimit_scale <= 0.0 ? prev->vlimit_scale
        : fmin(prev->vlimit_scale, sg->vlimit_scale);
}

int tpnAddSegment(TP_STRUCT * const tp, tpn_seg *sg, int canon_type, double vel,
        double ini_maxvel, double vlimit_scale, unsigned char enables, char atspeed,
        int indexer_jnum, struct state_tag_t tag)
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
    sg->spindle = tp->spindle.spindle_num;
    sg->uu_per_rev = tp->uu_per_rev;
    sg->vreq = fmin(vel, ini_maxvel > 0.0 ? ini_maxvel : vel);
    if (sg->vreq <= 0.0 || (sg->sync == TC_SYNC_POSITION && ini_maxvel > 0.0)) {
        /* the spindle sets the speed of a position synchronized move */
        sg->vreq = ini_maxvel;
    }
    /* the max velocity slider caps the length canon measures, which is
     * vlimit_scale times ours, 0 for a move canon measures in degrees; a
     * position synchronized move follows the spindle instead */
    sg->vlimit_scale = sg->sync == TC_SYNC_POSITION ? 0.0 : vlimit_scale;
    sg->vlimit_bin = 0.0;
    sg->h_in = sg->h_out = 0.0;
    sg->stop_in = 0;
    sg->active = 0;
    sg->tap = 0;
    if (tp->syncdio.anychanged) {
        sg->syncdio = tp->syncdio;
        tpClearDIOs(tp);
    } else {
        sg->syncdio.anychanged = 0;
    }

    tpnGeomBounds(&sg->geom, &G, &G1, &G2);
    double vmax = sg->vreq * speedFactor(tp);
    if (ini_maxvel > 0.0) {
        vmax = fmin(vmax, ini_maxvel);
    }
    if (sg->sync == TC_SYNC_POSITION) {
        /* a thread may run up to the axes' own speed, which the
         * interpreter allows; the controller never passes a speed cap */
        tpn_axlim axs = ax;
        int i;
        for (i = 0; i < TPN_NAX; i++) {
            axs.vel[i] /= TPN_LIMIT_SCALE;
        }
        tpnLimits(&axs, &G, &G1, &G2, vmax, &sg->lim_int);
    } else {
        tpnLimits(&ax, &G, &G1, &G2, vmax, &sg->lim_int);
    }

    if (tpn.q_len > 0) {
        joinMoves(tp, &ax, seg(tpn.q_len - 1), sg);
    } else {
        sg->S0 = tpn.cur_s;
        sg->stop_in = 1;
        tpn.A_lo = tpn.J_lo = TPN_BIG;
    }
    if (sg->h_in > 0.0) {
        tpn.A_lo = fmin(tpn.A_lo, sg->lim_bin.A);
        tpn.J_lo = fmin(tpn.J_lo, sg->lim_bin.J);
    }
    tpn.A_lo = fmin(tpn.A_lo, sg->lim_int.A);
    tpn.J_lo = fmin(tpn.J_lo, sg->lim_int.J);

    tpn.q_len++;
    tp->queue._len = tpn.q_len;
    tp->depth = tpn.q_len;
    tp->done = 0;
    tpn.emcmotStatus->tcqlen = tpn.q_len;
    backwardPass();
    return TP_ERR_OK;
}
