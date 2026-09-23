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

static double pieceSoft(tpn_seg const *sg, int blend, double scale)
{
    double v = blend ? sg->vreq_bin : sg->vreq;
    if (sg->sync == TC_SYNC_VELOCITY) {
        double speed = fabs(tpn.emcmotStatus->spindle_status[0].spindleSpeedIn);
        v = speed * sg->uu_per_rev;
    }
    return v * scale;
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

    for (i = 0; i < tpn.q_len; i++) {
        tpn_seg *sg = seg(i);
        int piece;
        if (sg->stop_in && sg->S0 > tpn.cur_s) {
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
            if (Pb <= tpn.cur_s && !(i == tpn.q_len - 1 && piece == 1)) {
                continue;
            }
            soft = pieceSoft(sg, piece == 0, scale);
            if (Pa <= tpn.cur_s) {
                st->V = fmin(st->V, lim->V);
                st->A = fmin(st->A, lim->A);
                st->J = fmin(st->J, lim->J);
                st->Vs = st->Vs < 0.0 ? soft : fmin(st->Vs, soft);
            } else {
                /* a step that ends inside the piece runs part of the
                 * cycle there at the jerk it picks */
                if (st->J < TPN_BIG && Pa < tpn.cur_s + tpn.cur_v * dt
                        + 0.5 * tpn.cur_a * dt * dt + st->J * dt * dt * dt / 6.0) {
                    st->J = fmin(st->J, lim->J);
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
        /* beyond the braking distance of the fastest next state every
         * constraint can be met by stopping */
        double vhi = tpn.cur_v + fmax(tpn.cur_a, 0.0) * dt + 0.5 * st->J * dt * dt;
        double ahi = fmax(tpn.cur_a, 0.0) + st->J * dt;
        double horizon = tpnBrakeDist(vhi, ahi, 0.0, TPN_BRAKE_SCALE * fmin(Arun, st->A),
                TPN_BRAKE_SCALE * fmin(Jrun, st->J))
            + 2.0 * vhi * dt + 8.0 * st->J * dt * dt * dt + 1e-6;
        if (ownedEnd(sg) - tpn.cur_s > horizon || ncon >= TPN_MAXCON) {
            break;
        }
    }
}

typedef struct {
    double s1, v1, a1;
} tpn_next;

static void stepState(double j, double dt, tpn_next *n)
{
    n->s1 = tpn.cur_s + tpn.cur_v * dt + 0.5 * tpn.cur_a * dt * dt + j * dt * dt * dt / 6.0;
    n->v1 = tpn.cur_v + tpn.cur_a * dt + 0.5 * j * dt * dt;
    n->a1 = tpn.cur_a + j * dt;
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
    double J = fmin(c->Jrun, st->J) * TPN_BRAKE_SCALE;
    double A = fmin(c->Arun, st->A) * TPN_BRAKE_SCALE;
    if (what == CHK_HARD || what == CHK_SPEED) {
        if (c->Vh <= 0.0) {
            /* leave the deadbeat finish a little room to land on a cycle */
            d -= 4.0 * J * g_dt * g_dt * g_dt;
        }
        if (what == CHK_SPEED) {
            return conOk(n->v1, n->a1, d, c->Vh, A, J);
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
    double jhi = fmin(J, (A - tpn.cur_a) / dt);
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
    double d = st->Sstop - tpn.cur_s;
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
    g_dt = dt;

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
    tpn.cur_j = (n.a1 - tpn.cur_a) / dt;
    tpn.cur_s = n.s1;
    tpn.cur_v = n.v1;
    tpn.cur_a = n.a1;
}
