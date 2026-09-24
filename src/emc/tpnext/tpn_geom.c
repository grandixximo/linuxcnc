/********************************************************************
* Description: tpn_geom.c
*   Geometry, blends and limit projection for the tpnext planner.
*
* License: GPL Version 2
* System: Linux
********************************************************************/
#include <rtapi_math.h>
#include "tpn.h"

#define TPN_BIG 1e30
#define TPN_TINY 1e-12

void tpnVecFromPose(tpn_vec *v, EmcPose const *p)
{
    v->v[0] = p->tran.x;
    v->v[1] = p->tran.y;
    v->v[2] = p->tran.z;
    v->v[3] = p->a;
    v->v[4] = p->b;
    v->v[5] = p->c;
    v->v[6] = p->u;
    v->v[7] = p->v;
    v->v[8] = p->w;
}

void tpnPoseFromVec(EmcPose *p, tpn_vec const *v)
{
    p->tran.x = v->v[0];
    p->tran.y = v->v[1];
    p->tran.z = v->v[2];
    p->a = v->v[3];
    p->b = v->v[4];
    p->c = v->v[5];
    p->u = v->v[6];
    p->v = v->v[7];
    p->w = v->v[8];
}

static double norm3(double const *x)
{
    return sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
}

/* feed length the way RS274NGC defines it: XYZ if XYZ moves, else UVW,
 * else ABC */
static double feedLength(tpn_vec const *d)
{
    double l = norm3(&d->v[0]);
    if (l > 1e-9) {
        return l;
    }
    l = norm3(&d->v[6]);
    if (l > 1e-9) {
        return l;
    }
    return norm3(&d->v[3]);
}

int tpnLineInit(tpn_geom *g, EmcPose const *start, EmcPose const *end)
{
    tpn_vec d;
    int i;
    g->type = TPN_LINE;
    tpnVecFromPose(&g->p0, start);
    tpnVecFromPose(&g->p1, end);
    for (i = 0; i < TPN_NAX; i++) {
        d.v[i] = g->p1.v[i] - g->p0.v[i];
    }
    g->L = feedLength(&d);
    if (g->L <= 1e-9) {
        return -1;
    }
    for (i = 0; i < TPN_NAX; i++) {
        g->g.v[i] = d.v[i] / g->L;
    }
    return 0;
}

int tpnArcInit(tpn_geom *g, EmcPose const *start, EmcPose const *end,
        PmCartesian const *center, PmCartesian const *normal, int turn)
{
    PmCircle c;
    double helix;
    int i;
    g->type = TPN_ARC;
    tpnVecFromPose(&g->p0, start);
    tpnVecFromPose(&g->p1, end);
    if (pmCircleInit(&c, &start->tran, &end->tran, center, normal, turn)) {
        return -1;
    }
    if (c.radius <= 1e-9 || c.angle <= 1e-9) {
        return -1;
    }
    g->center = c.center;
    g->rTan = c.rTan;
    g->rPerp = c.rPerp;
    g->rHelix = c.rHelix;
    g->radius = c.radius;
    g->spiral = c.spiral;
    g->angle = c.angle;
    pmCartMag(&c.rHelix, &helix);
    double arc = c.angle * (c.radius + 0.5 * c.spiral);
    g->L = sqrt(arc * arc + helix * helix);
    if (g->L <= 1e-9) {
        return -1;
    }
    for (i = 0; i < 3; i++) {
        g->g.v[i] = 0.0;
    }
    for (i = 3; i < TPN_NAX; i++) {
        g->g.v[i] = (g->p1.v[i] - g->p0.v[i]) / g->L;
    }
    return 0;
}

void tpnGeomEval(tpn_geom const *g, double u, tpn_vec *p, tpn_vec *d1, tpn_vec *d2)
{
    int i;
    if (u >= g->L && p) {
        *p = g->p1;
        p = 0;
    }
    if (g->type == TPN_LINE) {
        for (i = 0; i < TPN_NAX; i++) {
            if (p) {
                p->v[i] = g->p0.v[i] + g->g.v[i] * u;
            }
            if (d1) {
                d1->v[i] = g->g.v[i];
            }
            if (d2) {
                d2->v[i] = 0.0;
            }
        }
        return;
    }
    double k = g->angle / g->L;
    double phi = k * u;
    double cs = cos(phi), sn = sin(phi);
    double R = g->radius;
    double r = R + g->spiral * phi / g->angle;
    double rp = g->spiral / g->angle;
    double const *rt = &g->rTan.x;
    double const *rq = &g->rPerp.x;
    double const *rh = &g->rHelix.x;
    double const *ce = &g->center.x;
    for (i = 0; i < 3; i++) {
        double dir = (rt[i] * cs + rq[i] * sn) / R;
        double dirp = (-rt[i] * sn + rq[i] * cs) / R;
        if (p) {
            p->v[i] = ce[i] + r * dir + rh[i] * phi / g->angle;
        }
        if (d1) {
            d1->v[i] = k * (rp * dir + r * dirp + rh[i] / g->angle);
        }
        if (d2) {
            d2->v[i] = k * k * (2.0 * rp * dirp - r * dir);
        }
    }
    for (i = 3; i < TPN_NAX; i++) {
        if (p) {
            p->v[i] = g->p0.v[i] + g->g.v[i] * u;
        }
        if (d1) {
            d1->v[i] = g->g.v[i];
        }
        if (d2) {
            d2->v[i] = 0.0;
        }
    }
}

/* per axis bounds of |dP/ds|, |d2P/ds2|, |d3P/ds3| over the whole move */
void tpnGeomBounds(tpn_geom const *g, tpn_vec *G, tpn_vec *G1, tpn_vec *G2)
{
    int i;
    for (i = 0; i < TPN_NAX; i++) {
        G->v[i] = fabs(g->g.v[i]);
        G1->v[i] = 0.0;
        G2->v[i] = 0.0;
    }
    if (g->type != TPN_ARC) {
        return;
    }
    double k = g->angle / g->L;
    double R = g->radius;
    double rmax = R + fmax(0.0, g->spiral);
    double rp = fabs(g->spiral / g->angle);
    double const *rt = &g->rTan.x;
    double const *rq = &g->rPerp.x;
    double const *rh = &g->rHelix.x;
    for (i = 0; i < 3; i++) {
        double m = sqrt(rt[i] * rt[i] + rq[i] * rq[i]) / R;
        G->v[i] = k * (rp * m + rmax * m + fabs(rh[i]) / g->angle);
        G1->v[i] = k * k * (2.0 * rp * m + rmax * m);
        G2->v[i] = k * k * k * (3.0 * rp * m + rmax * m);
    }
}

static double weightedDist2(tpn_vec const *p, tpn_vec const *q, tpn_vec const *w)
{
    double s = 0.0;
    int i;
    for (i = 0; i < TPN_NAX; i++) {
        double e = (q->v[i] - p->v[i]) * w->v[i];
        s += e * e;
    }
    return s;
}

/* Newton steps on the squared distance from u, kept inside [0, L] */
static double refineDist2(tpn_geom const *g, tpn_vec const *q, tpn_vec const *w, double u)
{
    tpn_vec p, d1, d2;
    int k, i;
    for (k = 0; k < 6; k++) {
        double f1 = 0.0, f2 = 0.0;
        tpnGeomEval(g, u, &p, &d1, &d2);
        for (i = 0; i < TPN_NAX; i++) {
            double w2 = w->v[i] * w->v[i];
            double e = p.v[i] - q->v[i];
            f1 += w2 * e * d1.v[i];
            f2 += w2 * (d1.v[i] * d1.v[i] + e * d2.v[i]);
        }
        if (f2 <= TPN_TINY) {
            break;
        }
        double un = fmin(g->L, fmax(0.0, u - f1 / f2));
        if (fabs(un - u) < 1e-12 * g->L) {
            u = un;
            break;
        }
        u = un;
    }
    tpnGeomEval(g, u, &p, 0, 0);
    return weightedDist2(&p, q, w);
}

double tpnGeomDist(tpn_geom const *g, tpn_vec const *q, tpn_vec const *w)
{
    double best = fmin(weightedDist2(&g->p0, q, w), weightedDist2(&g->p1, q, w));
    int i;
    if (g->type == TPN_LINE) {
        /* the line is linear in every axis, so is its weighted projection */
        double t = 0.0, L2 = 0.0;
        for (i = 0; i < TPN_NAX; i++) {
            double w2 = w->v[i] * w->v[i];
            t += w2 * (q->v[i] - g->p0.v[i]) * g->g.v[i];
            L2 += w2 * g->g.v[i] * g->g.v[i];
        }
        if (L2 > TPN_TINY) {
            tpn_vec p;
            tpnGeomEval(g, fmin(g->L, fmax(0.0, t / L2)), &p, 0, 0);
            best = fmin(best, weightedDist2(&p, q, w));
        }
        return sqrt(best);
    }
    /* start from the angle of q around the arc axis on every turn, then
     * refine on the weighted distance */
    double const *rt = &g->rTan.x;
    double const *rq = &g->rPerp.x;
    double x = 0.0, y = 0.0;
    for (i = 0; i < 3; i++) {
        double d = q->v[i] - (&g->center.x)[i];
        x += d * rt[i];
        y += d * rq[i];
    }
    double phi = atan2(y, x);
    if (phi < 0.0) {
        phi += 2.0 * M_PI;
    }
    for (; phi <= g->angle + M_PI; phi += 2.0 * M_PI) {
        best = fmin(best, refineDist2(g, q, w, fmin(g->L, phi * g->L / g->angle)));
    }
    return sqrt(best);
}

/* Quintic Hermite from (p0, d0, dd0) to (p1, d1, dd1), derivatives taken
 * with respect to sigma, sigma in [0, H]. */
void tpnBlendInit(tpn_blend *b, double H, tpn_vec const *p0, tpn_vec const *d0,
        tpn_vec const *dd0, tpn_vec const *p1, tpn_vec const *d1, tpn_vec const *dd1)
{
    int i;
    b->H = H;
    for (i = 0; i < TPN_NAX; i++) {
        double D = p1->v[i] - p0->v[i];
        double v0 = H * d0->v[i], v1 = H * d1->v[i];
        double a0 = H * H * dd0->v[i], a1 = H * H * dd1->v[i];
        b->c[0][i] = p0->v[i];
        b->c[1][i] = v0;
        b->c[2][i] = 0.5 * a0;
        b->c[3][i] = 10.0 * D - 6.0 * v0 - 4.0 * v1 - 1.5 * a0 + 0.5 * a1;
        b->c[4][i] = -15.0 * D + 8.0 * v0 + 7.0 * v1 + 1.5 * a0 - a1;
        b->c[5][i] = 6.0 * D - 3.0 * v0 - 3.0 * v1 - 0.5 * a0 + 0.5 * a1;
    }
}

void tpnBlendEval(tpn_blend const *b, double sigma, tpn_vec *p, tpn_vec *d1)
{
    double t = sigma / b->H;
    int i;
    if (t < 0.0) {
        t = 0.0;
    } else if (t > 1.0) {
        t = 1.0;
    }
    for (i = 0; i < TPN_NAX; i++) {
        double const *c[6] = {&b->c[0][i], &b->c[1][i], &b->c[2][i],
                              &b->c[3][i], &b->c[4][i], &b->c[5][i]};
        if (p) {
            p->v[i] = *c[0] + t * (*c[1] + t * (*c[2] + t * (*c[3] + t * (*c[4] + t * *c[5]))));
        }
        if (d1) {
            d1->v[i] = (*c[1] + t * (2.0 * *c[2] + t * (3.0 * *c[3] + t * (4.0 * *c[4] + t * 5.0 * *c[5])))) / b->H;
        }
    }
}

/* Bounds from the Bernstein coefficients of the derivatives: a polynomial
 * on [0, 1] stays inside the hull of its Bernstein coefficients. */
void tpnBlendBounds(tpn_blend const *b, tpn_vec *G, tpn_vec *G1, tpn_vec *G2)
{
    int i, k;
    double H = b->H;
    for (i = 0; i < TPN_NAX; i++) {
        double bz[6];
        double c0 = b->c[0][i], c1 = b->c[1][i], c2 = b->c[2][i];
        double c3 = b->c[3][i], c4 = b->c[4][i], c5 = b->c[5][i];
        if (c1 == 0.0 && c2 == 0.0 && c3 == 0.0 && c4 == 0.0 && c5 == 0.0) {
            G->v[i] = G1->v[i] = G2->v[i] = 0.0;
            continue;
        }
        /* power to Bernstein, degree 5 */
        bz[0] = c0;
        bz[1] = c0 + c1 / 5.0;
        bz[2] = c0 + 2.0 * c1 / 5.0 + c2 / 10.0;
        bz[3] = c0 + 3.0 * c1 / 5.0 + 3.0 * c2 / 10.0 + c3 / 10.0;
        bz[4] = c0 + 4.0 * c1 / 5.0 + 6.0 * c2 / 10.0 + 4.0 * c3 / 10.0 + c4 / 5.0;
        bz[5] = c0 + c1 + c2 + c3 + c4 + c5;
        double m1 = 0.0, m2 = 0.0, m3 = 0.0;
        for (k = 0; k < 5; k++) {
            m1 = fmax(m1, fabs(5.0 * (bz[k + 1] - bz[k])));
        }
        for (k = 0; k < 4; k++) {
            m2 = fmax(m2, fabs(20.0 * (bz[k + 2] - 2.0 * bz[k + 1] + bz[k])));
        }
        for (k = 0; k < 3; k++) {
            m3 = fmax(m3, fabs(60.0 * (bz[k + 3] - 3.0 * bz[k + 2] + 3.0 * bz[k + 1] - bz[k])));
        }
        G->v[i] = m1 / H;
        G1->v[i] = m2 / (H * H);
        G2->v[i] = m3 / (H * H * H);
    }
}

void tpnBlendPart(tpn_blend const *b, double t0, double t1, tpn_blend *part)
{
    static const double binom[6][6] = {
        {1, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0}, {1, 2, 1, 0, 0, 0},
        {1, 3, 3, 1, 0, 0}, {1, 4, 6, 4, 1, 0}, {1, 5, 10, 10, 5, 1}};
    double d = t1 - t0;
    int i, k, m;
    part->H = b->H * d;
    for (i = 0; i < TPN_NAX; i++) {
        double dk = 1.0;
        if (b->c[1][i] == 0.0 && b->c[2][i] == 0.0 && b->c[3][i] == 0.0
                && b->c[4][i] == 0.0 && b->c[5][i] == 0.0) {
            /* an axis the blend does not move */
            part->c[0][i] = b->c[0][i];
            for (k = 1; k < 6; k++) {
                part->c[k][i] = 0.0;
            }
            continue;
        }
        for (k = 0; k < 6; k++) {
            double c = 0.0, tp = 1.0;
            for (m = k; m < 6; m++) {
                c += binom[m][k] * b->c[m][i] * tp;
                tp *= t0;
            }
            part->c[k][i] = c * dk;
            dk *= d;
        }
    }
}

static double cbrt_pos(double x)
{
    return x > 0.0 ? pow(x, 1.0 / 3.0) : 0.0;
}

/*
 * Tangential limits of a piece from per axis limits. With s the path
 * parameter and P(s) the pose, axis j moves with
 *   x'   = s' P'
 *   x''  = s'' P' + s'^2 P''
 *   x''' = s''' P' + 3 s' s'' P'' + s'^3 P'''
 * On curved pieces the speed cap takes at most half of the acceleration
 * for s'^2 P'' and a quarter of the jerk for s'^3 P'''. Only what the cap
 * takes is reserved; of the rest of the jerk two thirds go to s''' P' and
 * one third to the cross term.
 */
void tpnLimits(tpn_axlim const *ax, tpn_vec const *G, tpn_vec const *G1,
        tpn_vec const *G2, double vcap, tpn_lim *lim)
{
    int i;
    int curved = 0;
    for (i = 0; i < TPN_NAX; i++) {
        if (G1->v[i] > TPN_TINY || G2->v[i] > TPN_TINY) {
            curved = 1;
        }
    }
    double fa = curved ? 0.5 : 0.0;
    double fj2 = curved ? 0.25 : 0.0;
    double fj = curved ? 2.0 / 3.0 : 1.0;
    double V = vcap, A = TPN_BIG, J = TPN_BIG;
    double V2 = TPN_BIG, V3 = TPN_BIG;
    for (i = 0; i < TPN_NAX; i++) {
        if (G->v[i] > TPN_TINY) {
            V = fmin(V, ax->vel[i] / G->v[i]);
        }
        if (G1->v[i] > TPN_TINY) {
            V2 = fmin(V2, fa * ax->acc[i] / G1->v[i]);
        }
        if (G2->v[i] > TPN_TINY) {
            V3 = fmin(V3, fj2 * ax->jerk[i] / G2->v[i]);
        }
    }
    if (V2 < TPN_BIG) {
        V = fmin(V, sqrt(V2));
    }
    if (V3 < TPN_BIG) {
        V = fmin(V, cbrt_pos(V3));
    }
    for (i = 0; i < TPN_NAX; i++) {
        double ra = ax->acc[i] - V * V * G1->v[i];
        double rj = ax->jerk[i] - V * V * V * G2->v[i];
        if (G->v[i] > TPN_TINY) {
            A = fmin(A, ra / G->v[i]);
            J = fmin(J, fj * rj / G->v[i]);
        }
        if (G1->v[i] > TPN_TINY && V > TPN_TINY) {
            A = fmin(A, (1.0 - fj) * rj / (3.0 * V * G1->v[i]));
        }
    }
    lim->V = V;
    lim->A = A;
    lim->J = J;
}

/* distance of the symmetric velocity change from (v1, 0) down to (vt, 0) */
static double symDist(double v1, double vt, double A, double J)
{
    double dv = v1 - vt;
    double T;
    if (dv <= 0.0) {
        return 0.0;
    }
    if (dv * J <= A * A) {
        T = 2.0 * sqrt(dv / J);
    } else {
        T = dv / A + A / J;
    }
    return 0.5 * (v1 + vt) * T;
}

/*
 * Shortest distance in which the state (v0, a0) can be brought to a
 * velocity no higher than vt with zero acceleration, under tangential
 * limits A and J. Zero if the state never exceeds vt anyway.
 */
double tpnBrakeDist(double v0, double a0, double vt, double A, double J)
{
    if (vt < 0.0) {
        vt = 0.0;
    }
    if (a0 >= 0.0) {
        double t1 = a0 / J;
        double v1 = v0 + 0.5 * a0 * t1;
        if (v1 <= vt) {
            return 0.0;
        }
        double d1 = v0 * t1 + 0.5 * a0 * t1 * t1 - J * t1 * t1 * t1 / 6.0;
        return d1 + symDist(v1, vt, A, J);
    }
    if (v0 <= vt) {
        return 0.0;
    }
    double vz = v0 - 0.5 * a0 * a0 / J;
    if (vz <= vt) {
        /* ramping the deceleration out right away already reaches vt */
        double disc = a0 * a0 - 2.0 * J * (v0 - vt);
        double t = (-a0 - sqrt(fmax(disc, 0.0))) / J;
        return v0 * t + 0.5 * a0 * t * t + J * t * t * t / 6.0;
    }
    /* continue a profile that started at (vv, 0) and ramped down to a0 */
    double Aeff = fmax(A, -a0);
    double t0 = -a0 / J;
    double vv = v0 + 0.5 * a0 * a0 / J;
    double dpre = vv * t0 - J * t0 * t0 * t0 / 6.0;
    return fmax(0.0, symDist(vv, vt, Aeff, J) - dpre);
}
