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

/* envelope at the entry of a piece of length len from Enext at its end */
static double envelopeIn(tpn_lim const *lim, double Enext, double len, double *E, int *same)
{
    double e = fmin(lim->V, sqrt(Enext * Enext + lim->A * len));
    *same = *same && e == *E;
    *E = e;
    return e;
}

/* Backward envelope with half of each piece's tangential acceleration and
 * zero speed at the end of the queue and at every stop, for both sets of
 * limits. The runtime checks the true caps inside its braking horizon;
 * the envelope carries everything beyond it. */
static void backwardPass(void)
{
    double Enext = 0.0, Enext_hi = 0.0;
    int i;
    for (i = tpn.q_len - 1; i >= 0; i--) {
        tpn_seg *sg = seg(i);
        double len = fmax(0.0, sg->geom.L - sg->h_in - sg->h_out);
        int same = i < tpn.q_len - 2;
        double E = envelopeIn(&sg->lim_int, Enext, len, &sg->E_int, &same);
        double E_hi = envelopeIn(&sg->lim_int_hi, Enext_hi, len, &sg->E_int_hi, &same);
        int k;
        if (sg->h_in > 0.0) {
            double part = 2.0 * sg->h_in / TPN_NSUB;
            for (k = TPN_NSUB - 1; k >= 0; k--) {
                E = envelopeIn(&sg->lim_sub[k], E, part, &sg->E_sub[k], &same);
                E_hi = envelopeIn(&sg->lim_sub_hi[k], E_hi, part, &sg->E_sub_hi[k], &same);
            }
        }
        Enext = sg->stop_in ? 0.0 : E;
        Enext_hi = sg->stop_in ? 0.0 : E_hi;
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

/* ---------------------------------------------------------------- joints
 *
 * With kinematics other than the identity each joint is one more channel
 * of the limits, with its own derivative bounds per piece. The
 * kinematics is asked only at anchors along each move: the joints and
 * the Jacobian there, so q' = J P' is exact at every anchor; q'' and q'''
 * come from how q' changes between anchors. A blend asks nothing: near
 * the corner J is the corner's plus its rate along each move, and the
 * blend's own polynomial gives the rest. */

#define TPN_JANCH_MAX 32
/* anchors per degree of rotary travel and per unit of length */
#define TPN_JANCH_DEG 0.5
#define TPN_JANCH_LEN 0.05
#define TPN_JPART_SAMPLES 5
/* for the peaks of q''' between anchors or samples */
#define TPN_JMARGIN 1.1
/* for the peaks between the samples of a blend part */
#define TPN_JPART_MARGIN 1.03

static int jointsOn(tpn_seg const *sg)
{
    if (!tpn.kins.inverse || !tpn.kins.identity || !tpn.kins.joints) {
        return 0;
    }
    if (sg->sync == TC_SYNC_POSITION || tpn.kins.identity()) {
        return 0;
    }
    return tpn.kins.joints() > 0;
}

static void readJointLimits(TP_STRUCT const *tp, tpn_jlim *jl)
{
    int trapezoid = tpn.emcmotStatus->planner_type != 1;
    int j;
    jl->n = tpn.kins.joints();
    if (jl->n > TPN_NJ) {
        jl->n = TPN_NJ;
    }
    for (j = 0; j < jl->n; j++) {
        double v = tpn.kins.joint_vel_limit ? tpn.kins.joint_vel_limit(j) : 0.0;
        double a = tpn.kins.joint_acc_limit ? tpn.kins.joint_acc_limit(j) : 0.0;
        double k = tpn.kins.joint_jerk_limit ? tpn.kins.joint_jerk_limit(j) : 0.0;
        jl->vel[j] = v > 0.0 ? v * TPN_LIMIT_SCALE : TPN_BIG;
        jl->acc[j] = a > 0.0 ? a * TPN_LIMIT_SCALE : TPN_BIG;
        double jtrap = jl->acc[j] / (2.0 * tp->cycleTime);
        jl->jerk[j] = trapezoid || k <= 0.0 ? jtrap : fmin(k * TPN_LIMIT_SCALE, jtrap);
    }
}

/* the Jacobian at the joints q of pose p, differenced from the inverse
 * where the module has none */
static int jacobianAt(double const *q, tpn_vec const *p, double J[][TPN_NAX])
{
    EmcPose pose;
    int a, j, n = tpn.jl.n;
    tpnPoseFromVec(&pose, p);
    if (tpn.kins.jacobian && tpn.kins.jacobian(q, &pose, J) == 0) {
        return 0;
    }
    for (a = 0; a < TPN_NAX; a++) {
        double qp[TPN_NJ], qm[TPN_NJ];
        double h = 1e-4;
        tpn_vec pp = *p, pm = *p;
        pp.v[a] += h;
        pm.v[a] -= h;
        for (j = 0; j < n; j++) {
            qp[j] = qm[j] = q[j];
        }
        tpnPoseFromVec(&pose, &pp);
        if (tpn.kins.inverse(&pose, qp) != 0) {
            return -1;
        }
        tpnPoseFromVec(&pose, &pm);
        if (tpn.kins.inverse(&pose, qm) != 0) {
            return -1;
        }
        for (j = 0; j < n; j++) {
            J[j][a] = (qp[j] - qm[j]) / (2.0 * h);
        }
    }
    return 0;
}

/* Joint bounds over the whole move sg, in jb, and the joint model at its
 * start (tpn.jhead) and end (tail). Nonzero if the kinematics cannot
 * answer somewhere along it. */
static int jointAnchors(tpn_seg const *sg, tpn_jb *jb, tpn_jend *tail, double *qend)
{
    static double qd[TPN_JANCH_MAX + 1][TPN_NJ];
    static double Jh[3][TPN_NJ][TPN_NAX], Jt[3][TPN_NJ][TPN_NAX];
    tpn_geom const *g = &sg->geom;
    double q[TPN_NJ], rot = 0.0;
    int n = tpn.jl.n, K, k, j, a, i;

    for (a = 0; a < TPN_NAX; a++) {
        if (tpn.ang_mask & (1u << a)) {
            rot = fmax(rot, fabs(g->p1.v[a] - g->p0.v[a]));
        }
    }
    K = 3 + (int)ceil(rot * TPN_JANCH_DEG + g->L * TPN_JANCH_LEN);
    if (K > TPN_JANCH_MAX) {
        K = TPN_JANCH_MAX;
    }
    double du = g->L / K;
    if (tpn.jseed_valid && tpn.q_len > 0) {
        for (j = 0; j < n; j++) {
            q[j] = tpn.jseed[j];
        }
    } else if (tpn.kins.joint_pos) {
        tpn.kins.joint_pos(q);
    } else {
        for (j = 0; j < n; j++) {
            q[j] = 0.0;
        }
    }
    for (k = 0; k <= K; k++) {
        tpn_vec p, d1;
        EmcPose pose;
        double (*Jk)[TPN_NAX] = k < 3 ? Jh[k] : Jt[k % 3];
        tpnGeomEval(g, k == K ? g->L : du * k, &p, &d1, 0);
        tpnPoseFromVec(&pose, &p);
        if (tpn.kins.inverse(&pose, q) != 0 || jacobianAt(q, &p, Jk) != 0) {
            return -1;
        }
        if (k >= K - 2 && k < 3) {
            /* a short move: its first anchors are also its last */
            for (j = 0; j < n; j++) {
                for (a = 0; a < TPN_NAX; a++) {
                    Jt[k % 3][j][a] = Jk[j][a];
                }
            }
        }
        for (j = 0; j < n; j++) {
            double s = 0.0;
            for (a = 0; a < TPN_NAX; a++) {
                s += Jk[j][a] * d1.v[a];
            }
            qd[k][j] = s;
        }
        if (k == 0) {
            for (a = 0; a < TPN_NAX; a++) {
                tpn.jhead.t[a] = d1.v[a];
            }
        }
        if (k == K) {
            for (a = 0; a < TPN_NAX; a++) {
                tail->t[a] = d1.v[a];
            }
        }
    }
    for (j = 0; j < n; j++) {
        double m1 = 0.0, m2 = 0.0, m3 = 0.0, h2 = 0.0, t2 = 0.0;
        double prev = 0.0;
        for (k = 0; k <= K; k++) {
            m1 = fmax(m1, fabs(qd[k][j]));
        }
        for (k = 0; k < K; k++) {
            double dd = (qd[k + 1][j] - qd[k][j]) / du;
            m2 = fmax(m2, fabs(dd));
            if (k > 0) {
                double d3 = fabs(dd - prev) / du;
                m3 = fmax(m3, d3);
                if (k == 1) {
                    h2 = d3;
                }
                if (k == K - 1) {
                    t2 = d3;
                }
            }
            prev = dd;
        }
        m3 *= TPN_JMARGIN;
        /* from the anchors to the peaks between them */
        m2 += 0.5 * du * m3;
        m1 += 0.5 * du * m2;
        jb->G[j] = m1;
        jb->G1s[j] = m2;
        jb->G2s[j] = m3;
        jb->G1u[j] = jb->G2m[j] = jb->G2u[j] = 0.0;
        tpn.jhead.G2[j] = h2 * TPN_JMARGIN;
        tail->G2[j] = t2 * TPN_JMARGIN;
        qend[j] = q[j];
    }
    /* the Jacobian at both ends and its rate along the move there, to
     * second order */
    for (j = 0; j < n; j++) {
        for (a = 0; a < TPN_NAX; a++) {
            double t0 = Jt[K % 3][j][a], t1 = Jt[(K - 1) % 3][j][a], t2 = Jt[(K - 2) % 3][j][a];
            tpn.jhead.J[j][a] = Jh[0][j][a];
            tpn.jhead.D[j][a] = (-3.0 * Jh[0][j][a] + 4.0 * Jh[1][j][a] - Jh[2][j][a]) / (2.0 * du);
            tail->J[j][a] = t0;
            tail->D[j][a] = (3.0 * t0 - 4.0 * t1 + t2) / (2.0 * du);
        }
    }
    for (i = n; i < TPN_NJ; i++) {
        jb->G[i] = jb->G1s[i] = jb->G1u[i] = jb->G2s[i] = jb->G2m[i] = jb->G2u[i] = 0.0;
    }
    tpn.jhead.valid = 1;
    tail->valid = 1;
    return 0;
}

/* position and first three derivatives of a blend at tau, per unit sigma */
static void blendDerivs(tpn_blend const *b, double t, double *p, double *d1, double *d2, double *d3)
{
    double H = b->H;
    int i;
    for (i = 0; i < TPN_NAX; i++) {
        double c0 = b->c[0][i], c1 = b->c[1][i], c2 = b->c[2][i];
        double c3 = b->c[3][i], c4 = b->c[4][i], c5 = b->c[5][i];
        p[i] = c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * c5))));
        d1[i] = (c1 + t * (2.0 * c2 + t * (3.0 * c3 + t * (4.0 * c4 + t * 5.0 * c5)))) / H;
        d2[i] = (2.0 * c2 + t * (6.0 * c3 + t * (12.0 * c4 + t * 20.0 * c5))) / (H * H);
        d3[i] = (6.0 * c3 + t * (24.0 * c4 + t * 60.0 * c5)) / (H * H * H);
    }
}

/* Joint bounds of each part of the blend b between the move ending with
 * the joint model in and the one starting with out, at the corner pc.
 * The blend runs near the plane of the two tangents: each point and
 * derivative of it is split into its parts x, y along them, and J there
 * is in->J + x in->D + y out->D. */
static void jointBlendParts(tpn_blend const *b, tpn_jend const *in, tpn_jend const *out,
        tpn_vec const *pc, tpn_jb *jb)
{
    double const *ti = in->t, *to = out->t;
    double g11 = 0.0, g12 = 0.0, g22 = 0.0;
    int ax[TPN_NAX], na = 0;
    int n = tpn.jl.n, k, i, j, a, d;
    int const M = TPN_JPART_SAMPLES - 1;
    for (a = 0; a < TPN_NAX; a++) {
        g11 += ti[a] * ti[a];
        g12 += ti[a] * to[a];
        g22 += to[a] * to[a];
        /* an axis the blend does not move adds nothing to any sum */
        if (b->c[1][a] != 0.0 || b->c[2][a] != 0.0 || b->c[3][a] != 0.0
                || b->c[4][a] != 0.0 || b->c[5][a] != 0.0) {
            ax[na++] = a;
        }
    }
    double det = g11 * g22 - g12 * g12;
    int flat = det <= 1e-9 * g11 * g22;
    for (k = 0; k < TPN_NSUB; k++) {
        for (j = 0; j < n; j++) {
            jb[k].G[j] = jb[k].G1s[j] = jb[k].G1u[j] = 0.0;
            jb[k].G2s[j] = jb[k].G2m[j] = 0.0;
            jb[k].G2u[j] = fmax(in->G2[j], out->G2[j]);
        }
    }
    /* the samples of each part, the ends shared with its neighbours */
    for (i = 0; i <= TPN_NSUB * M; i++) {
        double tau = (double)i / (TPN_NSUB * M);
        double p[TPN_NAX], d1[TPN_NAX], d2[TPN_NAX], d3[TPN_NAX];
        double x[3], y[3];
        double const *w[3] = {p, d1, d2};
        int k1 = i / M < TPN_NSUB ? i / M : TPN_NSUB - 1;
        int k0 = i % M == 0 && i > 0 && i < TPN_NSUB * M ? k1 - 1 : k1;
        blendDerivs(b, tau, p, d1, d2, d3);
        for (d = 0; d < 3; d++) {
            double r1 = 0.0, r2 = 0.0;
            for (a = 0; a < na; a++) {
                double v = d == 0 ? w[0][ax[a]] - pc->v[ax[a]] : w[d][ax[a]];
                r1 += v * ti[ax[a]];
                r2 += v * to[ax[a]];
            }
            if (flat) {
                x[d] = y[d] = g11 > 0.0 ? 0.5 * r1 / g11 : 0.0;
            } else {
                x[d] = (g22 * r1 - g12 * r2) / det;
                y[d] = (g11 * r2 - g12 * r1) / det;
            }
        }
        for (j = 0; j < n; j++) {
            double jp = 0.0, s1 = 0.0, u1 = 0.0, s2 = 0.0, m2 = 0.0;
            for (a = 0; a < na; a++) {
                int c = ax[a];
                double Di = in->D[j][c], Do = out->D[j][c];
                double J0 = in->J[j][c] + x[0] * Di + y[0] * Do;
                double J1 = x[1] * Di + y[1] * Do;
                double J2 = x[2] * Di + y[2] * Do;
                jp += J0 * d1[c];
                s1 += J0 * d2[c];
                u1 += J1 * d1[c];
                s2 += J0 * d3[c];
                m2 += 2.0 * J1 * d2[c] + J2 * d1[c];
            }
            for (k = k0; k <= k1; k++) {
                jb[k].G[j] = fmax(jb[k].G[j], fabs(jp));
                jb[k].G1s[j] = fmax(jb[k].G1s[j], fabs(s1));
                jb[k].G1u[j] = fmax(jb[k].G1u[j], fabs(u1));
                jb[k].G2s[j] = fmax(jb[k].G2s[j], fabs(s2));
                jb[k].G2m[j] = fmax(jb[k].G2m[j], fabs(m2));
            }
        }
    }
    for (k = 0; k < TPN_NSUB; k++) {
        for (j = 0; j < n; j++) {
            jb[k].G[j] *= TPN_JPART_MARGIN;
            jb[k].G1s[j] *= TPN_JPART_MARGIN;
            jb[k].G1u[j] *= TPN_JPART_MARGIN;
            jb[k].G2s[j] *= TPN_JPART_MARGIN;
            jb[k].G2m[j] *= TPN_JPART_MARGIN;
        }
    }
}

/* derivative bounds and speed caps of each part of the blend */
typedef struct {
    tpn_vec G[TPN_NSUB], G1[TPN_NSUB], G2[TPN_NSUB];
    tpn_caps caps[TPN_NSUB];
    double vcap[TPN_NSUB];
    double vwant;           /* programmed feed through the blend */
    double vtop;            /* the higher cap of the two moves */
    int jon;                /* the joints are bounded too, by jb */
    tpn_jb jb[TPN_NSUB];
} tpn_parts;

static void blendParts(TP_STRUCT const *tp, tpn_axlim const *ax, tpn_seg const *prev,
        tpn_seg const *sg, tpn_blend const *b, tpn_parts *pt)
{
    double vcap = fmin(prev->vreq, sg->vreq) * speedFactor(tp);
    int k;
    pt->vwant = fmin(prev->vreq, sg->vreq);
    /* a blend part between two arcs may bound the curvature closer than
     * the arcs do, but a speed above both only makes the S-curve
     * controller speed up and slow down again */
    pt->vtop = tpn.emcmotStatus->planner_type == 1
        && prev->geom.type == TPN_ARC && sg->geom.type == TPN_ARC
        ? fmax(prev->lim_int.V, sg->lim_int.V) : TPN_BIG;
    for (k = 0; k < TPN_NSUB; k++) {
        tpn_blend part;
        tpnBlendPart(b, (double)k / TPN_NSUB, (double)(k + 1) / TPN_NSUB, &part);
        tpnBlendBounds(&part, &pt->G[k], &pt->G1[k], &pt->G2[k]);
        tpnLimitCaps(ax, &pt->G[k], &pt->G1[k], &pt->G2[k], &pt->caps[k]);
        pt->vcap[k] = fmin(vcap, pt->caps[k].Vg);
    }
    pt->jon = tpn.jtail.valid && tpn.jhead.valid;
    if (pt->jon) {
        jointBlendParts(b, &tpn.jtail, &tpn.jhead, &prev->geom.p1, pt->jb);
    }
}

/* limits of each part of the blend, with its second and third derivative
 * bounds scaled by r and r^2, and the smallest of them in lim */
static void partLimits(tpn_axlim const *ax, tpn_parts const *pt, double r, tpn_lim *lim,
        tpn_lim *sub)
{
    int k;
    lim->V = lim->A = lim->J = TPN_BIG;
    for (k = 0; k < TPN_NSUB; k++) {
        tpn_lim l;
        if (pt->jon) {
            tpn_caps c;
            tpnLimitCapsJ(&pt->caps[k], r, &tpn.jl, &pt->jb[k], &c);
            double V = fmin(fmin(fmin(pt->vcap[k], c.Vg), pt->vtop), tpnCurveCap(&c, 1.0, pt->vwant));
            tpnLimitsAt(ax, &pt->G[k], &pt->G1[k], &pt->G2[k], r, V, c.curved, &l);
            tpnJointLimitsAt(&tpn.jl, &pt->jb[k], r, V, c.curved, &l);
        } else {
            double V = fmin(fmin(pt->vcap[k], pt->vtop), tpnCurveCap(&pt->caps[k], r, pt->vwant));
            tpnLimitsAt(ax, &pt->G[k], &pt->G1[k], &pt->G2[k], r, V, pt->caps[k].curved, &l);
        }
        lim->V = fmin(lim->V, l.V);
        lim->A = fmin(lim->A, l.A);
        lim->J = fmin(lim->J, l.J);
        if (sub) {
            sub[k] = l;
        }
    }
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
     * way: then only the rest of the way counts. Past P the controller
     * keeps v + a^2 / 2J within Vp, so it brakes as from (Vp, 0) after
     * ramping its acceleration out. The slow point found last time is
     * tried first, with the bounds kept of the pieces after it, which
     * spares the scan of a long queue behind one slow piece. */
    if (tpn.slow_A < TPN_BIG && tpn.slow_P > tpn.cur_s && tpn.slow_V <= V) {
        A = tpn.slow_A * TPN_BRAKE_SCALE;
        J = tpn.slow_J * TPN_BRAKE_SCALE;
        double ramp = tpn.slow_V * fmin(tpn.slow_Ap, sqrt(2.0 * J * tpn.slow_V)) / J;
        if (ramp + tpnBrakeDist(tpn.slow_V, 0.0, V, A, J) <= S - tpn.slow_P - 1e-9) {
            return 1;
        }
    }
    double P[TPN_SLOW_POINTS], Vp[TPN_SLOW_POINTS], Ap[TPN_SLOW_POINTS];
    int i, n = slowPoints(S, V, P, Vp, Ap);
    for (i = 0; i < n; i++) {
        runLimits(P[i], S, &A, &J);
        double Ab = A, Jb = J;
        A *= TPN_BRAKE_SCALE;
        J *= TPN_BRAKE_SCALE;
        double ramp = Vp[i] * fmin(Ap[i], sqrt(2.0 * J * Vp[i])) / J;
        if (ramp + tpnBrakeDist(Vp[i], 0.0, V, A, J) <= S - P[i] - 1e-9) {
            tpn.slow_P = P[i];
            tpn.slow_V = Vp[i];
            tpn.slow_Ap = Ap[i];
            tpn.slow_A = Ab;
            tpn.slow_J = Jb;
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
            /* the bounds of the blend taken, for the limits at the
             * highest override below as well */
            blendParts(tp, ax, prev, sg, &b, &pt);
            h0 = h;
            partLimits(ax, &pt, 1.0, &lim, sub);
        } else if (pt.jon) {
            /* the axes scale with a blend between two lines, the joints
             * only roughly: their bounds from the blend taken, put back to
             * the scale of pt */
            double r = h0 / h;
            jointBlendParts(&b, &tpn.jtail, &tpn.jhead, &prev->geom.p1, pt.jb);
            for (k = 0; k < TPN_NSUB; k++) {
                for (i = 0; i < tpn.jl.n; i++) {
                    pt.jb[k].G1s[i] /= r;
                    pt.jb[k].G2s[i] /= r * r;
                    pt.jb[k].G2m[i] /= r;
                }
            }
            partLimits(ax, &pt, r, &lim, sub);
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
    /* pt holds the bounds of the blend of half length h0, which between
     * two lines is this one scaled about the corner */
    pt.vwant *= speedFactor(tp);
    if (pt.vtop < TPN_BIG) {
        pt.vtop = fmax(prev->lim_int_hi.V, sg->lim_int_hi.V);
    }
    partLimits(ax, &pt, h0 / h, &lim, sg->lim_sub_hi);
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
    tpn_jb jb;
    tpn_jend tail;
    double qend[TPN_NJ];
    int jon;

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

    jon = jointsOn(sg);
    tpn.jhead.valid = 0;
    tail.valid = 0;
    if (jon) {
        readJointLimits(tp, &tpn.jl);
        if (jointAnchors(sg, &jb, &tail, qend) != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "tpnext: the kinematics cannot answer along move %d, its joints follow the axis limits only\n",
                    sg->id);
            jon = 0;
            tpn.jhead.valid = 0;
            tail.valid = 0;
        }
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
        tpnLimits(&axs, &G, &G1, &G2, vmax, sg->vreq, &sg->lim_int);
        tpnLimits(&axs, &G, &G1, &G2, vmax, vmax, &sg->lim_int_hi);
    } else if (jon) {
        tpnLimitsJ(&ax, &G, &G1, &G2, &tpn.jl, &jb, vmax, sg->vreq, &sg->lim_int);
        tpnLimitsJ(&ax, &G, &G1, &G2, &tpn.jl, &jb, vmax, vmax, &sg->lim_int_hi);
        if (sg->lim_int.V < 0.01 * sg->vreq) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "tpnext: move %d slowed to %g by its joints, near a singular pose\n",
                    sg->id, sg->lim_int.V);
        }
    } else {
        tpnLimits(&ax, &G, &G1, &G2, vmax, sg->vreq, &sg->lim_int);
        tpnLimits(&ax, &G, &G1, &G2, vmax, vmax, &sg->lim_int_hi);
    }

    if (tpn.q_len > 0) {
        joinMoves(tp, &ax, seg(tpn.q_len - 1), sg);
    } else {
        sg->S0 = tpn.cur_s;
        sg->stop_in = 1;
        tpn.A_lo = tpn.J_lo = TPN_BIG;
        tpn.slow_A = tpn.slow_J = TPN_BIG;
    }
    tpn.jtail = tail;
    tpn.jseed_valid = jon;
    if (jon) {
        int j;
        for (j = 0; j < tpn.jl.n; j++) {
            tpn.jseed[j] = qend[j];
        }
    }
    if (sg->h_in > 0.0) {
        tpn.A_lo = fmin(tpn.A_lo, sg->lim_bin.A);
        tpn.J_lo = fmin(tpn.J_lo, sg->lim_bin.J);
    }
    tpn.A_lo = fmin(tpn.A_lo, sg->lim_int.A);
    tpn.J_lo = fmin(tpn.J_lo, sg->lim_int.J);
    if (tpn.slow_A < TPN_BIG) {
        if (sg->h_in > 0.0) {
            tpn.slow_A = fmin(tpn.slow_A, sg->lim_bin.A);
            tpn.slow_J = fmin(tpn.slow_J, sg->lim_bin.J);
        }
        tpn.slow_A = fmin(tpn.slow_A, sg->lim_int.A);
        tpn.slow_J = fmin(tpn.slow_J, sg->lim_int.J);
    }

    tpn.q_len++;
    tp->queue._len = tpn.q_len;
    tp->depth = tpn.q_len;
    tp->done = 0;
    tpn.emcmotStatus->tcqlen = tpn.q_len;
    backwardPass();
    return TP_ERR_OK;
}
