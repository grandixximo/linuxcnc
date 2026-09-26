/********************************************************************
* Description: tpn_run.c
*   Per cycle controller of the tpnext trajectory planner: picks the
*   largest jerk that keeps every speed cap and stop in the braking
*   horizon reachable, and lands stops on a cycle boundary.
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

#define TPN_MAXCON 512
/* poles of the spindle tracking, rad/s; beyond TPN_SYNC_LAND of error
 * it also keeps the reference reachable */
#define TPN_SYNC_W 80.0
#define TPN_SYNC_LAND 0.02
/* landing plans tried beyond the shortest, in steps */
#define TPN_LAND_EXTRA 4

/* deadbeat stop sequence in progress */
#define TPN_DB_MAX 40
static double db_seq[TPN_DB_MAX];
static int db_n, db_i;
static double db_S;

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
/* the controller's position along its direction of travel: s, or -s in a
 * reverse run */
static double cx;

/* Hard feasibility of a single constraint at distance d with the brake
 * to its cap; d < 0 means the constraint starts inside this step. */
static int conOk(double v1, double a1, double d, double V, double A, double J)
{
    double bd = tpnBrakeDist(v1, a1, V, A, J);
    return bd <= fmax(d, 0.0) + 1e-12;
}

/* Shortest distance in which (v0, a0) gets down to vt, still braking if
 * need be: the deceleration ramps to A and holds, and is let go only
 * past vt. Zero if the speed never exceeds vt. */
static double reachDist(double v0, double a0, double vt, double A, double J)
{
    double d = 0.0;
    if (vt < 0.0) {
        vt = 0.0;
    }
    if (a0 >= 0.0) {
        double t1 = a0 / J;
        double v1 = v0 + 0.5 * a0 * t1;
        if (v1 <= vt) {
            return 0.0;
        }
        d = v0 * t1 + 0.5 * a0 * t1 * t1 - J * t1 * t1 * t1 / 6.0;
        v0 = v1;
        a0 = 0.0;
    } else if (v0 <= vt) {
        return 0.0;
    }
    /* from (vv, 0) the deceleration ramps down through a0 */
    double Aeff = fmax(A, -a0);
    double t0 = -a0 / J;
    double vv = v0 + 0.5 * a0 * a0 / J;
    double dpre = vv * t0 - J * t0 * t0 * t0 / 6.0;
    double dv = vv - vt, dr;
    if (dv <= 0.5 * Aeff * Aeff / J) {
        double t = sqrt(2.0 * dv / J);
        dr = vv * t - J * t * t * t / 6.0;
    } else {
        double t2 = Aeff / J;
        double v2 = vv - 0.5 * Aeff * t2;
        double t3 = (v2 - vt) / Aeff;
        dr = vv * t2 - J * t2 * t2 * t2 / 6.0 + v2 * t3 - 0.5 * Aeff * t3 * t3;
    }
    return d + fmax(0.0, dr - dpre);
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

/* a speed that settles on a cap from below approaches it only slowly */
#define TPN_CAP_NEAR 0.98

/* requested speed of a piece, < 0 for none: a move synchronized to the
 * spindle position follows the spindle instead, unless an abort stops it.
 * Feed override and the max velocity slider apply here, every cycle. */
static double pieceSoft(TP_STRUCT const *tp, tpn_seg const *sg, int blend, double scale)
{
    double v = blend ? sg->vreq_bin : sg->vreq;
    double vls = blend ? sg->vlimit_bin : sg->vlimit_scale;
    if (sg->sync == TC_SYNC_POSITION && tpn.track) {
        return -1.0;
    }
    if (sg->sync == TC_SYNC_VELOCITY) {
        double speed = fabs(tpn.emcmotStatus->spindle_status[sg->spindle].spindleSpeedIn);
        v = speed * sg->uu_per_rev;
    }
    v *= scale;
    if (tp->vLimit > 0.0 && vls > 0.0) {
        v = fmin(v, tp->vLimit * vls);
    }
    return v;
}

typedef struct {
    double V, A, J;     /* caps of the pieces the step touches */
    double Jhi;         /* largest jerk that keeps the step out of pieces
                         * with a lower jerk limit, or leaves it in them
                         * at their limit */
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

/* Piece k of move sg: a part of the blend (k < TPN_NSUB) or the interior
 * (k == TPN_NSUB), from Pa to Pb along the path, with its limits, the
 * opened ones and their envelopes. Zero if the move has no such piece. */
static int pieceOf(tpn_seg const *sg, int k, double *Pa, double *Pb,
        tpn_lim const **lim, tpn_lim const **hi, double *E, double *E_hi)
{
    if (k < TPN_NSUB) {
        if (sg->h_in <= 0.0) {
            return 0;
        }
        *Pa = ownedStart(sg) + 2.0 * sg->h_in * k / TPN_NSUB;
        *Pb = k == TPN_NSUB - 1 ? sg->S0 + sg->h_in
            : ownedStart(sg) + 2.0 * sg->h_in * (k + 1) / TPN_NSUB;
        *lim = &sg->lim_sub[k];
        *hi = &sg->lim_sub_hi[k];
        *E = sg->E_sub[k];
        *E_hi = sg->E_sub_hi[k];
    } else {
        *Pa = sg->S0 + sg->h_in;
        *Pb = ownedEnd(sg);
        *lim = &sg->lim_int;
        *hi = &sg->lim_int_hi;
        *E = sg->E_int;
        *E_hi = sg->E_int_hi;
    }
    return 1;
}

static void stepInit(tpn_step *st)
{
    ncon = 0;
    st->V = TPN_BIG;
    st->A = TPN_BIG;
    st->J = TPN_BIG;
    st->Jhi = TPN_BIG;
    st->Vs = -1.0;
    st->Sstop = TPN_BIG;
}

/* beyond the braking distance of the fastest next state every
 * constraint can be met by stopping */
static double horizonOf(tpn_step const *st, double Arun, double Jrun, double dt)
{
    double vhi = tpn.cur_v + fmax(tpn.cur_a, 0.0) * dt + 0.5 * st->J * dt * dt;
    double ahi = fmax(tpn.cur_a, 0.0) + st->J * dt;
    return tpnBrakeDist(vhi, ahi, 0.0, TPN_BRAKE_SCALE * fmin(Arun, st->A),
            TPN_BRAKE_SCALE * fmin(Jrun, st->J))
        + 2.0 * vhi * dt + 8.0 * st->J * dt * dt * dt + 1e-6;
}

/* Collect the constraints ahead of cur_s up to the braking horizon of the
 * fastest state reachable this cycle. */
static void gather(TP_STRUCT const *tp, double scale, int stepping, tpn_step *st)
{
    double dt = tp->cycleTime;
    double Arun = TPN_BIG, Jrun = TPN_BIG;
    int i;
    stepInit(st);

    for (i = 0; i < tpn.q_len; i++) {
        tpn_seg *sg = seg(i);
        int piece;
        if (sg->stop_in && sg->S0 > tpn.cur_s) {
            addCon(sg->S0, 0.0, -1.0, TPN_BIG, Arun, Jrun);
            st->Sstop = fmin(st->Sstop, sg->S0);
        }
        /* the parts of the blend, then the interior */
        for (piece = 0; piece <= TPN_NSUB; piece++) {
            double Pa, Pb, E, soft;
            tpn_lim const *lim;
            tpn_lim const *hi;
            double E_hi;
            if (!pieceOf(sg, piece, &Pa, &Pb, &lim, &hi, &E, &E_hi)) {
                continue;
            }
            if (Pb <= tpn.cur_s && !(i == tpn.q_len - 1 && piece == TPN_NSUB)) {
                continue;
            }
            soft = pieceSoft(tp, sg, piece < TPN_NSUB, scale);
            /* the opened caps where the override asks for more than the
             * caps allow and they let the motion run faster; in the piece
             * the motion is in only once it is about to reach the caps,
             * since they leave less acceleration along the path, and for
             * as long as it runs faster than the caps allow */
            if (soft >= 0.0 && (Pa <= tpn.cur_s ? tpn.cur_v > lim->V
                        || (soft > lim->V && E_hi > E && tpn.cur_v
                            + 0.5 * tpn.cur_a * fabs(tpn.cur_a) / lim->J
                            >= TPN_CAP_NEAR * lim->V)
                    : soft > lim->V && E_hi > E)) {
                lim = hi;
                E = E_hi;
            }
            if (Pa <= tpn.cur_s) {
                st->V = fmin(st->V, lim->V);
                st->A = fmin(st->A, lim->A);
                st->J = fmin(st->J, lim->J);
                if (soft >= 0.0) {
                    st->Vs = st->Vs < 0.0 ? soft : fmin(st->Vs, soft);
                }
            } else {
                /* a step that ends inside the piece runs part of the
                 * cycle there at the jerk it picks; jstar ends it on Pa */
                double jstar = (Pa - tpn.cur_s - tpn.cur_v * dt - 0.5 * tpn.cur_a * dt * dt)
                    * 6.0 / (dt * dt * dt);
                if (jstar < -lim->J) {
                    /* every jerk within the limits enters it */
                    st->J = fmin(st->J, lim->J);
                } else {
                    st->Jhi = fmin(st->Jhi, fmax(jstar, lim->J));
                }
                addCon(Pa, E, soft, lim->A, Arun, Jrun);
            }
            Arun = fmin(Arun, lim->A);
            Jrun = fmin(Jrun, lim->J);
        }
        if (stepping && i == 0 && tpn.q_len > 1) {
            addCon(ownedEnd(sg), 0.0, -1.0, TPN_BIG, Arun, Jrun);
            st->Sstop = fmin(st->Sstop, ownedEnd(sg));
        }
        if (i == tpn.q_len - 1) {
            addCon(segEnd(sg), 0.0, -1.0, TPN_BIG, Arun, Jrun);
            st->Sstop = fmin(st->Sstop, segEnd(sg));
        }
        if (ownedEnd(sg) - tpn.cur_s > horizonOf(st, Arun, Jrun, dt) || ncon >= TPN_MAXCON) {
            break;
        }
    }
}

/*
 * gather() for a reverse run, in x = -s: the pieces from the one the
 * motion is in back to the start of the oldest move kept that may be run
 * backwards. The planner's backward envelopes hold for the forward
 * direction only, so the envelope is built here over the constraints up
 * to the braking horizon; where the table fills up the motion stops.
 */
static void gatherReverse(TP_STRUCT const *tp, double scale, int stepping, tpn_step *st)
{
    double dt = tp->cycleTime;
    double s = tpn.cur_s;
    double Arun = TPN_BIG, Jrun = TPN_BIG;
    int i;
    stepInit(st);

    for (i = 0; ; i--) {
        tpn_seg *sg = seg(i);
        int piece;
        if (ncon + TPN_NSUB + 3 > TPN_MAXCON) {
            addCon(-ownedEnd(sg), 0.0, -1.0, TPN_BIG, Arun, Jrun);
            st->Sstop = fmin(st->Sstop, -ownedEnd(sg));
            break;
        }
        /* the interior, then the parts of the blend, last first */
        for (piece = TPN_NSUB; piece >= 0; piece--) {
            double Pa, Pb, E, E_hi, soft;
            tpn_lim const *lim, *hi;
            if (!pieceOf(sg, piece, &Pa, &Pb, &lim, &hi, &E, &E_hi) || Pa >= s) {
                continue;
            }
            soft = pieceSoft(tp, sg, piece < TPN_NSUB, scale);
            /* the opened caps as in gather() */
            if (soft >= 0.0 && hi->V > lim->V && (Pb >= s ? tpn.cur_v > lim->V
                        || (soft > lim->V && tpn.cur_v
                            + 0.5 * tpn.cur_a * fabs(tpn.cur_a) / lim->J
                            >= TPN_CAP_NEAR * lim->V)
                    : soft > lim->V)) {
                lim = hi;
            }
            if (Pb >= s) {
                st->V = fmin(st->V, lim->V);
                st->A = fmin(st->A, lim->A);
                st->J = fmin(st->J, lim->J);
                if (soft >= 0.0) {
                    st->Vs = st->Vs < 0.0 ? soft : fmin(st->Vs, soft);
                }
            } else {
                double jstar = (-Pb - cx - tpn.cur_v * dt - 0.5 * tpn.cur_a * dt * dt)
                    * 6.0 / (dt * dt * dt);
                if (jstar < -lim->J) {
                    st->J = fmin(st->J, lim->J);
                } else {
                    st->Jhi = fmin(st->Jhi, fmax(jstar, lim->J));
                }
                addCon(-Pb, lim->V, soft, lim->A, Arun, Jrun);
            }
            Arun = fmin(Arun, lim->A);
            Jrun = fmin(Jrun, lim->J);
        }
        /* a corner without a blend, the end of the history, the start of
         * the move when stepping */
        int last = i == -tpn.h_len || !sg->rev_ok || !seg(i - 1)->rev_ok;
        if ((sg->stop_in && sg->S0 < s) || last || (stepping && i == 0 && ownedStart(sg) < s)) {
            addCon(-ownedStart(sg), 0.0, -1.0, TPN_BIG, Arun, Jrun);
            st->Sstop = fmin(st->Sstop, -ownedStart(sg));
        }
        if (last || s - ownedStart(sg) > horizonOf(st, Arun, Jrun, dt)) {
            break;
        }
    }
    /* the backward envelope of what was gathered, as the planner's
     * backwardPass() builds it forward */
    for (i = ncon - 2; i >= 0; i--) {
        if (con[i].Aentry < TPN_BIG) {
            con[i].Vh = fmin(con[i].Vh, sqrt(con[i + 1].Vh * con[i + 1].Vh
                        + con[i].Aentry * (con[i + 1].S - con[i].S)));
        }
    }
    if (st->J >= TPN_BIG) {
        /* at the start of the move, which is the end of the reverse run:
         * the limits of the piece the move starts with */
        double Pa, Pb, E, E_hi;
        tpn_lim const *lim, *hi;
        tpn_seg const *sg = seg(0);
        pieceOf(sg, sg->h_in > 0.0 ? 0 : TPN_NSUB, &Pa, &Pb, &lim, &hi, &E, &E_hi);
        st->V = lim->V;
        st->A = lim->A;
        st->J = lim->J;
    }
}

typedef struct {
    double s1, v1, a1;
} tpn_next;

static void stepFrom(double s, double v, double a, double j, double dt, tpn_next *n)
{
    n->s1 = s + v * dt + 0.5 * a * dt * dt + j * dt * dt * dt / 6.0;
    n->v1 = v + a * dt + 0.5 * j * dt * dt;
    n->a1 = a + j * dt;
}

static void stepState(double j, double dt, tpn_next *n)
{
    stepFrom(cx, tpn.cur_v, tpn.cur_a, j, dt, n);
}

/* CHK_SPEED: the speed part of a hard constraint alone */
enum { CHK_HARD = 1, CHK_SOFT = 2, CHK_SPEED = 4 };

static unsigned char failmask[TPN_MAXCON];

/* check constraint k (or the step caps for k < 0) against the next state */
static int checkOne(int k, int what, tpn_next const *n, tpn_step const *st)
{
    if (k < 0) {
        double A = st->A * TPN_BRAKE_SCALE, J = st->J * TPN_BRAKE_SCALE;
        if (what == CHK_HARD || what == CHK_SPEED) {
            return conOk(n->v1, n->a1, 0.0, st->V, A, J);
        }
        return st->Vs < 0.0 || conOk(n->v1, n->a1, 0.0, st->Vs, A, J);
    }
    tpn_con const *c = &con[k];
    double d = c->S - n->s1;
    /* inside a chain of caps stepping down, a cap may be met still
     * braking: the speed only goes on down to the next one. Met at rest
     * in acceleration, each would be a small plateau of its own, and the
     * jerk would swing from one to the next. */
    int chain = k + 1 < ncon && con[k + 1].Vh > 0.0 && con[k + 1].Vh < c->Vh;
    int chain_s = k + 1 < ncon && con[k + 1].Vs >= 0.0 && con[k + 1].Vs < c->Vs;
    double J = fmin(c->Jrun, st->J) * TPN_BRAKE_SCALE;
    double A = fmin(c->Arun, st->A) * TPN_BRAKE_SCALE;
    if (what == CHK_HARD || what == CHK_SPEED) {
        if (c->Vh <= 0.0) {
            /* leave the deadbeat finish a little room to land on a cycle */
            d -= 4.0 * J * g_dt * g_dt * g_dt;
        }
        int ok = chain ? reachDist(n->v1, n->a1, c->Vh, A, J) <= fmax(d, 0.0) + 1e-12
            : conOk(n->v1, n->a1, d, c->Vh, A, J);
        if (what == CHK_SPEED) {
            return ok;
        }
        return ok && accEntryOk(n->v1, n->a1, d, c->Aentry, J);
    }
    if (c->Vs < 0.0) {
        return 1;
    }
    return chain_s ? reachDist(n->v1, n->a1, c->Vs, A, J) <= fmax(d, 0.0) + 1e-12
        : conOk(n->v1, n->a1, d, c->Vs, A, J);
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
    double jhi = fmin(fmin(J, st->Jhi), (A - tpn.cur_a) / dt);
    double jlo = fmax(-J, (-A - tpn.cur_a) / dt);
    tpn_next n;
    int k, i;
    int nfail = 0;

    if (jlo > jhi) {
        /* the acceleration is above the limit of the piece just entered */
        return tpn.cur_a > 0.0 ? -J : J;
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
        /* No jerk meets every hard constraint: the step went a hair past
         * the point where a speed cap and the acceleration cap of the
         * piece after it turn tight together. Keep the speeds and release
         * the acceleration as fast as they allow; braking harder would
         * only push the acceleration further past the cap. */
        lo = jlo;
        hi = jhi;
        for (k = 0; k < 30; k++) {
            double mid = k ? 0.5 * (lo + hi) : hi;
            int ok = 1;
            stepState(mid, dt, &n);
            if ((curmask & CHK_HARD) && !checkOne(-1, CHK_SPEED, &n, st)) {
                ok = 0;
            }
            for (i = 0; i < ncon && ok; i++) {
                if ((failmask[i] & CHK_HARD) && !checkOne(i, CHK_SPEED, &n, st)) {
                    ok = 0;
                }
            }
            if (ok) {
                lo = mid;
                if (!k) {
                    break;
                }
            } else {
                hi = mid;
            }
        }
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

/* Jerk j_k = c0 + c1 k, k = 0 .. N-1, that takes (v, a) to (Vt, 0) in N
 * steps. */
static int landPlan(double v, double a, double Vt, int N, double dt, double *c0, double *c1)
{
    double S1 = 0.5 * N * (N - 1.0), S2 = (N - 1.0) * N * (2.0 * N - 1.0) / 6.0;
    double m10 = 0.5 * N * N, m11 = (N - 0.5) * S1 - S2;
    double r0 = -a / dt, r1 = (Vt - v - N * a * dt) / (dt * dt);
    double det = N * m11 - S1 * m10;
    if (fabs(det) < 1e-12) {
        return 0;
    }
    *c0 = (r0 * m11 - S1 * r1) / det;
    *c1 = (N * r1 - m10 * r0) / det;
    return 1;
}

/* A speed plateau. Riding the braking curve onto a speed cap ends with an
 * acceleration below one cycle of jerk, which no jerk takes to zero
 * without passing the cap: the largest jerk under it overshoots into the
 * opposite acceleration, and back, the jerk flipping sign cycle after
 * cycle. Nor can the curve be left at its end without a reversal of the
 * jerk. So leave it at its start: once N steps of jerk of one sign,
 * changing linearly, land (v, a) exactly on the cap with no
 * acceleration, follow them. A plan braking no later than the jerk
 * chosen keeps every cap; one braking later only if they all hold. */
static double landJerk(TP_STRUCT const *tp, tpn_step const *st, double j)
{
    double dt = tp->cycleTime, a = tpn.cur_a, v = tpn.cur_v;
    double Vt = st->Vs >= 0.0 ? fmin(st->Vs, st->V) : st->V;
    double Jf = st->J, sg = a > 0.0 ? 1.0 : -1.0;
    tpn_next n;
    int n0, N, i;
    if (Vt >= TPN_BIG || a == 0.0 || (a > 0.0) != (v < Vt)) {
        return j;
    }
    n0 = (int)ceil(fabs(a) / (Jf * dt));
    if (n0 < 2) {
        n0 = 2;
    }
    for (N = n0; N <= n0 + TPN_LAND_EXTRA; N++) {
        double c0, c1;
        if (!landPlan(v, a, Vt, N, dt, &c0, &c1)) {
            continue;
        }
        double jN = c0 + c1 * (N - 1);
        if (sg * c0 > 1e-9 * Jf || sg * jN > 1e-9 * Jf || fabs(c0) > Jf || fabs(jN) > Jf) {
            continue;
        }
        if (c0 > j) {
            int ok;
            stepState(c0, dt, &n);
            ok = checkOne(-1, CHK_HARD, &n, st) && checkOne(-1, CHK_SOFT, &n, st);
            for (i = 0; i < ncon && ok; i++) {
                ok = checkOne(i, CHK_HARD, &n, st) && checkOne(i, CHK_SOFT, &n, st);
            }
            if (!ok) {
                continue;
            }
        }
        return c0;
    }
    return j;
}

/*
 * Jerk that follows the spindle reference s_ref, v_ref, a_ref, j_ref from
 * the state (s, v, a): the feedforward jerk plus state feedback on the
 * errors it would leave at the end of the step, with all three poles at
 * -w, within the jerk limit J and the acceleration limit A.
 */
static double followJerk(double dt, double s, double v, double a, double A, double J, double w)
{
    double jf = tpn.j_ref;
    /* the errors where the step ends with the feedforward jerk */
    double es = tpn.s_ref - (s + v * dt + 0.5 * a * dt * dt + jf * dt * dt * dt / 6.0);
    double ev = tpn.v_ref - (v + a * dt + 0.5 * jf * dt * dt);
    double ea = tpn.a_ref - (a + jf * dt);
    double j = jf + w * w * w * es + 3.0 * w * w * ev + 3.0 * w * ea;
    double jlo = fmax(-J, (-A - a) / dt);
    double jhi = fmin(J, (A - a) / dt);
    j = fmax(jlo, fmin(jhi, j));
    if (fabs(es) > TPN_SYNC_LAND && jlo < jhi) {
        /* Far from the reference the linear law saturates and would
         * overshoot. In the frame moving with the reference the axis is
         * again a triple integrator: keep the jerk where it can still
         * land on the reference, the way a stop is kept reachable. */
        double side = es > 0.0 ? 1.0 : -1.0;
        double Al = fmax(TPN_SYNC_AMARGIN * A - fabs(tpn.a_ref), 0.1 * A);
        double Jl = TPN_SYNC_JMARGIN * J;
        double lo = jlo, hi = jhi;
        tpn_next n;
        int k;
        for (k = 0; k < 40; k++) {
            double mid = 0.5 * (lo + hi);
            stepFrom(s, v, a, mid, dt, &n);
            double gap = side * (tpn.s_ref - n.s1);
            double u = side * (n.v1 - tpn.v_ref), ar = side * (n.a1 - tpn.a_ref);
            int ok = tpnBrakeDist(u, ar, 0.0, Al, Jl) <= gap;
            /* from behind more jerk is worse, from ahead less */
            if (ok == (side > 0.0)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        j = side > 0.0 ? fmin(j, lo) : fmax(j, hi);
    }
    return j;
}

/* the spindle following jerk of the path, within the hard jerk range
 * [jerkNoReverse, jhard] of this step */
static double syncJerk(TP_STRUCT const *tp, tpn_step const *st, double jhard)
{
    double dt = tp->cycleTime;
    double j = followJerk(dt, tpn.cur_s, tpn.cur_v, tpn.cur_a, st->A, st->J, TPN_SYNC_W);
    double jlo = fmax(-st->J, (-st->A - tpn.cur_a) / dt);
    if (jlo > jhard) {
        return jhard;
    }
    jlo = jerkNoReverse(jlo, jhard, st->J, dt);
    return fmax(jlo, fmin(j, jhard));
}

/* where the speed ends once the acceleration is ramped out at J */
static double speedAhead(tpn_next const *n, double J)
{
    return n->v1 + 0.5 * n->a1 * fabs(n->a1) / J;
}

void tpnTapAdvance(TP_STRUCT const *tp, double *s, double *v, double *a, tpn_lim const *lim)
{
    double dt = tp->cycleTime, V = lim->V, A = lim->A, J = lim->J;
    double j = followJerk(dt, *s, *v, *a, A, J, TPN_TAP_W);
    tpn_next n;
    int k;
    stepFrom(*s, *v, *a, j, dt, &n);
    /* keep the speed where it can still be held within V either way */
    double side = speedAhead(&n, J) > V ? 1.0 : speedAhead(&n, J) < -V ? -1.0 : 0.0;
    if (side != 0.0) {
        double lo = side > 0.0 ? fmax(-J, (-A - *a) / dt) : j;
        double hi = side > 0.0 ? j : fmin(J, (A - *a) / dt);
        for (k = 0; k < 40; k++) {
            double mid = 0.5 * (lo + hi);
            stepFrom(*s, *v, *a, mid, dt, &n);
            if (side * speedAhead(&n, J) > V) {
                if (side > 0.0) {
                    hi = mid;
                } else {
                    lo = mid;
                }
            } else if (side > 0.0) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        j = side > 0.0 ? lo : hi;
        stepFrom(*s, *v, *a, j, dt, &n);
    }
    *s = n.s1;
    *v = n.v1;
    *a = n.a1;
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
    double v = tpn.cur_v, a = tpn.cur_a;
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
    double d = st->Sstop - cx;
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
    double bd = tpnBrakeDist(tpn.cur_v, tpn.cur_a, 0.0, Ab, Jb);
    double slack = 8.0 * st->J * dt * dt * dt;
    int creep = tpn.cur_v <= 1e-9 && fabs(tpn.cur_a) <= 1e-9;
    if (creep ? d > fmin(0.01, 100.0 * slack) : d > bd + fmax(1e-6 * d, 0.05 * tpn.cur_v * dt) + slack) {
        return 0;
    }
    nph = brakePhases(tpn.cur_v, tpn.cur_a, Ab, Jb, pj, pt);
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
        if (deadbeat(d, tpn.cur_v, tpn.cur_a, N, dt, nph, pj, pt, db_seq) <= 1.03 * st->J
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

void tpnRunReset(void)
{
    db_n = db_i = 0;
}

void tpnAdvance(TP_STRUCT const *tp, double scale, int stepping)
{
    double dt = tp->cycleTime;
    int rev = tp->reverse_run == TC_DIR_REVERSE;
    g_dt = dt;
    cx = rev ? -tpn.cur_s : tpn.cur_s;

    tpn_step st;
    if (rev) {
        gatherReverse(tp, scale, stepping, &st);
    } else {
        gather(tp, scale, stepping, &st);
    }
    double j = chooseJerk(tp, &st);
    double jf;
    if (tpn.track) {
        j = syncJerk(tp, &st, j);
    }
    tpn.step_V = st.V;
    if (!tpn.track) {
        j = landJerk(tp, &st, j);
    }
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
    tpn.cur_j = (n.a1 - tpn.cur_a) / dt;
    tpn.cur_s = rev ? -n.s1 : n.s1;
    tpn.cur_v = n.v1;
    tpn.cur_a = n.a1;
}
