/********************************************************************
* Description: orient_smooth.cc
*   Smoothing of the rotary axes along runs of short lines
*   ([TRAJ]ORIENT_SMOOTH), for emccanon.
*
*   Each rotary axis of a run becomes a clamped quintic B-spline of the
*   path length on uniform breakpoints. Its coefficients minimise the
*   squared distance to the programmed points plus lambda times the
*   squared third differences of the coefficients (a P-spline), the last
*   held at the end of the run and the first at its start, or the first
*   three on the curve where a run sent in part left off. A bisection on
*   log lambda finds the smoothest curve still within the tolerance, for a
*   few breakpoint spacings, and the one estimated fastest is taken. The
*   planner follows each line as the quintic Hermite of the value, rate
*   and second derivative at its ends, so the tolerance and the estimate
*   are taken on those quintics.
*
* License: GPL Version 2
* System: Linux
********************************************************************/
#include "orient_smooth.hh"

#include <algorithm>
#include <cmath>

namespace {

const int P = 5;                    // degree: the lines join up to their second derivative
const int NB = P + 1;               // basis functions not zero at a point
const double LOG_LO = -6.0, LOG_HI = 8.0;   // log10 lambda searched
const double LOG_STEP = 0.05;       // the bisection stops this close
const int BISECT_MAX = 30;
const int SPACINGS = 4;             // spacing, spacing / 2, / 4 and / 8
const int SEG_PER_LINE = 4;         // at most this many breakpoint intervals per line
const int MAX_LINES = 100000;
const size_t MIN_PART = 8;          // the fewest moves a part of a run is fitted in
const int CHECKS = 8;               // the tolerance is checked at t = k / CHECKS along a line
const double VBIG = 1e9;            // speed estimate where an axis does not bend

/* The basis functions not zero at x, span - P to span, and their first
 * two derivatives (The NURBS Book, A2.3). */
void dersBasis(const std::vector<double> &U, int span, double x, double ders[3][NB])
{
    double ndu[NB][NB], left[NB], right[NB], a[2][NB];
    ndu[0][0] = 1.0;
    for (int j = 1; j <= P; j++) {
        left[j] = x - U[span + 1 - j];
        right[j] = U[span + j] - x;
        double saved = 0.0;
        for (int r = 0; r < j; r++) {
            ndu[j][r] = right[r + 1] + left[j - r];
            double temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    for (int j = 0; j <= P; j++) {
        ders[0][j] = ndu[j][P];
    }
    for (int r = 0; r <= P; r++) {
        int s1 = 0, s2 = 1;
        a[0][0] = 1.0;
        for (int k = 1; k <= 2; k++) {
            double d = 0.0;
            int rk = r - k, pk = P - k;
            if (r >= k) {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }
            int j1 = rk >= -1 ? 1 : -rk;
            int j2 = r - 1 <= pk ? k - 1 : P - r;
            for (int j = j1; j <= j2; j++) {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                d += a[s2][j] * ndu[rk + j][pk];
            }
            if (r <= pk) {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                d += a[s2][k] * ndu[r][pk];
            }
            ders[k][r] = d;
            std::swap(s1, s2);
        }
    }
    for (int j = 0; j <= P; j++) {
        ders[1][j] *= P;
        ders[2][j] *= P * (P - 1);
    }
}

/* The quintic of a line of length H from (v0, d0, a0) to (v1, d1, a1),
 * value, rate and second derivative per unit length, in t = u / H. */
void hermite(double v0, double d0, double a0, double v1, double d1, double a1, double H,
        double c[6])
{
    double D = v1 - v0;
    double r0 = H * d0, r1 = H * d1, q0 = H * H * a0, q1 = H * H * a1;
    c[0] = v0;
    c[1] = r0;
    c[2] = 0.5 * q0;
    c[3] = 10.0 * D - 6.0 * r0 - 4.0 * r1 - 1.5 * q0 + 0.5 * q1;
    c[4] = -15.0 * D + 8.0 * r0 + 7.0 * r1 + 1.5 * q0 - q1;
    c[5] = 6.0 * D - 3.0 * r0 - 3.0 * r1 - 0.5 * q0 + 0.5 * q1;
}

/* the largest third derivative per unit length of that quintic */
double hermiteJerk(double const c[6], double H)
{
    /* 6 c3 + 24 c4 t + 60 c5 t^2: the ends, and the vertex if inside */
    double f0 = 6.0 * c[3], f1 = 6.0 * c[3] + 24.0 * c[4] + 60.0 * c[5];
    double m = std::max(std::fabs(f0), std::fabs(f1));
    if (c[5] != 0.0) {
        double t = -c[4] / (5.0 * c[5]);
        if (t > 0.0 && t < 1.0) {
            m = std::max(m, std::fabs(f0 + 24.0 * c[4] * t + 60.0 * c[5] * t * t));
        }
    }
    return m / (H * H * H);
}

/* speed under jerk J where the path bends at g3 per unit length cubed */
double jerkSpeed(double J, double g3)
{
    return g3 > 1e-12 ? std::cbrt(J / g3) : VBIG;
}

/* Rough time of the programmed lines under the jerk limits alone: the
 * slope step at each point taken in a blend half a line long. */
double timeLines(const std::vector<double> &s, const std::vector<OrientSmoothAxis> &axes,
        const std::vector<double> &J)
{
    double t = 0.0;
    size_t n = s.size();
    for (size_t j = 1; j + 1 < n; j++) {
        double L0 = s[j] - s[j - 1], L1 = s[j + 1] - s[j];
        double h = 0.5 * std::min(L0, L1);
        double v = VBIG;
        for (size_t k = 0; k < axes.size(); k++) {
            if (J[k] <= 0.0) {
                continue;
            }
            const std::vector<double> &y = axes[k].y;
            double dsl = std::fabs((y[j + 1] - y[j]) / L1 - (y[j] - y[j - 1]) / L0);
            /* a quintic over 2h turning the slope by dsl: jerk 7.5 dsl v^3 / (2h)^2 */
            v = std::min(v, jerkSpeed(J[k], 7.5 * dsl / (4.0 * h * h)));
        }
        t += 0.5 * (L0 + L1) / v;
    }
    return t;
}

/* clamped quintic B-spline on nseg uniform intervals over the run, with
 * the basis functions and their derivatives at each point */
struct Spline {
    int nseg, m;                    // intervals, coefficients
    std::vector<double> U;          // knots
    std::vector<int> span;          // at each point
    std::vector<double> B;          // at each point 3 * NB: value, rate, second derivative
};

void splineInit(Spline &sp, const std::vector<double> &s, int nseg)
{
    size_t n = s.size();
    double s0 = s[0], s1 = s[n - 1], h = (s1 - s0) / nseg;
    sp.nseg = nseg;
    sp.m = nseg + P;
    sp.U.assign(sp.m + P + 1, 0.0);
    for (int j = 0; j <= P; j++) {
        sp.U[j] = s0;
        sp.U[sp.m + j] = s1;
    }
    for (int i = 1; i < nseg; i++) {
        sp.U[P + i] = s0 + i * h;
    }
    sp.span.resize(n);
    sp.B.resize(n * 3 * NB);
    for (size_t j = 0; j < n; j++) {
        int i = (int)std::floor((s[j] - s0) / h);
        i = std::max(0, std::min(nseg - 1, i));
        sp.span[j] = P + i;
        double ders[3][NB];
        dersBasis(sp.U, sp.span[j], s[j], ders);
        for (int d = 0; d < 3; d++) {
            for (int a = 0; a < NB; a++) {
                sp.B[(j * 3 + d) * NB + a] = ders[d][a];
            }
        }
    }
}

/* symmetric band matrix, lower half: at(i, d) is entry (i, i - d) */
struct Band {
    int n;
    std::vector<double> v;
    void init(int size)
    {
        n = size;
        v.assign((size_t)size * NB, 0.0);
    }
    double &at(int i, int d)
    {
        return v[(size_t)i * NB + d];
    }
    double at(int i, int d) const
    {
        return v[(size_t)i * NB + d];
    }
};

/* in place Cholesky of a band of half width P; false if not positive */
bool bandCholesky(Band &L)
{
    for (int i = 0; i < L.n; i++) {
        for (int j = std::max(0, i - P); j <= i; j++) {
            double sum = L.at(i, i - j);
            for (int k = std::max(0, i - P); k < j; k++) {
                sum -= L.at(i, i - k) * L.at(j, j - k);
            }
            if (i == j) {
                if (!(sum > 0.0)) {
                    return false;
                }
                L.at(i, 0) = std::sqrt(sum);
            } else {
                L.at(i, i - j) = sum / L.at(j, 0);
            }
        }
    }
    return true;
}

void bandSolve(const Band &L, std::vector<double> &x)
{
    for (int i = 0; i < L.n; i++) {
        double sum = x[i];
        for (int k = std::max(0, i - P); k < i; k++) {
            sum -= L.at(i, i - k) * x[k];
        }
        x[i] = sum / L.at(i, 0);
    }
    for (int i = L.n - 1; i >= 0; i--) {
        double sum = x[i];
        for (int k = i + 1; k <= std::min(L.n - 1, i + P); k++) {
            sum -= L.at(k, k - i) * x[k];
        }
        x[i] = sum / L.at(i, 0);
    }
}

/* One breakpoint spacing: the normal equations of the fit, then a curve
 * for each lambda asked. */
class Fit {
  public:
    Fit(const std::vector<double> &s, const std::vector<OrientSmoothAxis> &axes, int nseg)
        : s_(s), axes_(axes)
    {
        splineInit(sp_, s, nseg);
        int m = sp_.m;
        A_.init(m);
        D_.init(m);
        b_.assign(axes.size(), std::vector<double>(m, 0.0));
        for (size_t j = 0; j < s.size(); j++) {
            int i0 = sp_.span[j] - P;
            const double *N = &sp_.B[j * 3 * NB];
            for (int a = 0; a < NB; a++) {
                for (int c = 0; c <= a; c++) {
                    A_.at(i0 + a, a - c) += N[a] * N[c];
                }
                for (size_t k = 0; k < axes.size(); k++) {
                    b_[k][i0 + a] += N[a] * axes[k].y[j];
                }
            }
        }
        /* the squared third differences of the coefficients */
        static const double del[4] = {-1.0, 3.0, -3.0, 1.0};
        for (int r = 0; r + 3 < m; r++) {
            for (int a = 0; a < 4; a++) {
                for (int c = 0; c <= a; c++) {
                    D_.at(r + a, a - c) += del[a] * del[c];
                }
            }
        }
    }

    /* the curve at the points for lambda into val, d1, d2 of every axis;
     * false if the equations fail */
    bool curve(double lambda, std::vector<OrientSmoothAxis> &out)
    {
        int m = sp_.m;
        size_t n = s_.size();
        /* The last coefficient is the end of the run, the first its start:
         * f = 1 fixed. On a curve sent before, the rate and the second
         * derivative there fix the next two: f = 3. */
        bool factored[4] = {false, false, false, false};
        std::vector<double> c(m), x;
        for (size_t k = 0; k < axes_.size(); k++) {
            const OrientSmoothAxis &ax = axes_[k];
            int f = ax.start_fixed ? 3 : 1;
            Band &L = f == 3 ? L3_ : L1_;
            if (!factored[f]) {
                L.init(m - 1 - f);
                for (int i = f; i <= m - 2; i++) {
                    for (int d = 0; d <= std::min(P, i - f); d++) {
                        L.at(i - f, d) = A_.at(i, d) + lambda * D_.at(i, d);
                    }
                }
                if (!bandCholesky(L)) {
                    return false;
                }
                factored[f] = true;
            }
            c[0] = ax.start_fixed ? ax.start_val : ax.y[0];
            c[m - 1] = ax.y[n - 1];
            if (f == 3) {
                /* at the start only functions 0 and 1 have a rate, 0 to 2
                 * a second derivative */
                const double *N = &sp_.B[0];
                c[1] = (ax.start_d1 - N[NB] * c[0]) / N[NB + 1];
                c[2] = (ax.start_d2 - N[2 * NB] * c[0] - N[2 * NB + 1] * c[1]) / N[2 * NB + 2];
            }
            x.resize(m - 1 - f);
            for (int i = f; i <= m - 2; i++) {
                double r = b_[k][i];
                for (int q = 0; q < f; q++) {
                    if (i - q <= P) {
                        r -= (A_.at(i, i - q) + lambda * D_.at(i, i - q)) * c[q];
                    }
                }
                if (m - 1 - i <= P) {
                    r -= (A_.at(m - 1, m - 1 - i) + lambda * D_.at(m - 1, m - 1 - i)) * c[m - 1];
                }
                x[i - f] = r;
            }
            bandSolve(L, x);
            std::copy(x.begin(), x.end(), c.begin() + f);
            OrientSmoothAxis &o = out[k];
            o.val.resize(n);
            o.d1.resize(n);
            o.d2.resize(n);
            for (size_t j = 0; j < n; j++) {
                int i0 = sp_.span[j] - P;
                const double *N = &sp_.B[j * 3 * NB];
                double e[3] = {0.0, 0.0, 0.0};
                for (int d = 0; d < 3; d++) {
                    for (int a = 0; a < NB; a++) {
                        e[d] += N[d * NB + a] * c[i0 + a];
                    }
                }
                o.val[j] = e[0];
                o.d1[j] = e[1];
                o.d2[j] = e[2];
            }
            o.val[0] = c[0];
            o.val[n - 1] = c[m - 1];
        }
        return true;
    }

  private:
    const std::vector<double> &s_;
    const std::vector<OrientSmoothAxis> &axes_;
    Spline sp_;
    Band A_, D_, L1_, L3_;
    std::vector<std::vector<double>> b_;
};

/* Does the curve keep within tol of the programmed moves over all the
 * axes, and each axis within its limits, at the points and along each
 * move? */
bool within(const std::vector<double> &s, const std::vector<OrientSmoothAxis> &axes, double tol)
{
    size_t n = s.size(), na = axes.size();
    double tol2 = tol * tol;
    for (size_t j = 1; j + 1 < n; j++) {
        double e2 = 0.0;
        for (size_t k = 0; k < na; k++) {
            double v = axes[k].val[j], e = v - axes[k].y[j];
            if (v < axes[k].min || v > axes[k].max) {
                return false;
            }
            e2 += e * e;
        }
        if (e2 > tol2) {
            return false;
        }
    }
    std::vector<double> c(6 * na);
    for (size_t j = 0; j + 1 < n; j++) {
        double H = s[j + 1] - s[j];
        for (size_t k = 0; k < na; k++) {
            const OrientSmoothAxis &a = axes[k];
            double *ck = &c[6 * k];
            hermite(a.val[j], a.d1[j], a.d2[j], a.val[j + 1], a.d1[j + 1], a.d2[j + 1], H, ck);
            /* less the programmed line */
            ck[0] -= a.y[j];
            ck[1] -= a.y[j + 1] - a.y[j];
        }
        for (int q = 1; q < CHECKS; q++) {
            double t = (double)q / CHECKS, e2 = 0.0;
            for (size_t k = 0; k < na; k++) {
                const OrientSmoothAxis &a = axes[k];
                const double *ck = &c[6 * k];
                double e = ck[0] + t * (ck[1] + t * (ck[2] + t * (ck[3] + t * (ck[4] + t * ck[5]))));
                double v = a.y[j] + t * (a.y[j + 1] - a.y[j]) + e;
                if (v < a.min || v > a.max) {
                    return false;
                }
                e2 += e * e;
            }
            if (e2 > tol2) {
                return false;
            }
        }
    }
    return true;
}

/* rough time of the curve under the jerk limits alone */
double timeCurve(const std::vector<double> &s, const std::vector<OrientSmoothAxis> &axes,
        const std::vector<double> &J)
{
    double t = 0.0, c[6];
    for (size_t j = 0; j + 1 < s.size(); j++) {
        double H = s[j + 1] - s[j], v = VBIG;
        for (size_t k = 0; k < axes.size(); k++) {
            if (J[k] <= 0.0) {
                continue;
            }
            const OrientSmoothAxis &a = axes[k];
            hermite(a.val[j], a.d1[j], a.d2[j], a.val[j + 1], a.d1[j + 1], a.d2[j + 1], H, c);
            v = std::min(v, jerkSpeed(J[k], hermiteJerk(c, H)));
        }
        t += H / v;
    }
    return t;
}

} // namespace

bool orientSmooth(const std::vector<double> &s, std::vector<OrientSmoothAxis> &axes,
        double tol, double spacing, OrientSmoothInfo *info)
{
    size_t n = s.size();
    if (info) {
        *info = OrientSmoothInfo{0.0, 0.0, 0.0, 0.0};
    }
    if (n < 4 || n > (size_t)MAX_LINES + 1 || axes.empty() || !(tol > 0.0) || !(spacing > 0.0)) {
        return false;
    }
    double span = s[n - 1] - s[0];
    for (size_t j = 1; j < n; j++) {
        if (!(s[j] > s[j - 1])) {
            return false;
        }
    }
    for (const OrientSmoothAxis &a : axes) {
        if (a.y.size() != n) {
            return false;
        }
    }
    /* the jerk limits; with none at all the estimates compare the shapes */
    std::vector<double> J(axes.size());
    bool anyJ = false;
    for (size_t k = 0; k < axes.size(); k++) {
        J[k] = axes[k].jerk;
        anyJ = anyJ || J[k] > 0.0;
    }
    if (!anyJ) {
        std::fill(J.begin(), J.end(), 1.0);
    }

    double tbest = timeLines(s, axes, J);
    if (info) {
        info->t_raw = tbest;
    }
    std::vector<OrientSmoothAxis> cur(axes), ok(axes), best;
    int nprev = 0;
    for (int q = 0; q < SPACINGS; q++) {
        double ds = spacing / (1 << q);
        double want = std::ceil(span / ds);
        int nseg = (int)std::max(1.0, std::min(want, (double)SEG_PER_LINE * (n - 1)));
        if (nseg == nprev) {
            continue;
        }
        nprev = nseg;
        Fit fit(s, axes, nseg);
        /* the smoothest curve within tol: the largest lambda that keeps it */
        double lo = LOG_LO, hi = LOG_HI, lam = 0.0;
        bool found = false;
        for (int it = 0; it < BISECT_MAX; it++) {
            double lg = it == 0 ? hi : 0.5 * (lo + hi);
            if (!fit.curve(std::pow(10.0, lg), cur)) {
                /* too little to hold the curve: smoother */
                lo = lg;
            } else if (within(s, cur, tol)) {
                ok.swap(cur);
                found = true;
                lam = std::pow(10.0, lg);
                lo = lg;
            } else {
                hi = lg;
            }
            if (hi - lo < LOG_STEP) {
                break;
            }
        }
        if (!found) {
            continue;
        }
        double t = timeCurve(s, ok, J);
        if (t < tbest) {
            tbest = t;
            best = ok;
            if (info) {
                info->spacing = span / nseg;
                info->lambda = lam;
                info->t_fit = t;
            }
        }
    }
    if (best.empty()) {
        return false;
    }
    for (size_t k = 0; k < axes.size(); k++) {
        axes[k].val.swap(best[k].val);
        axes[k].d1.swap(best[k].d1);
        axes[k].d2.swap(best[k].d2);
    }
    return true;
}

bool orientSmoothRun(const double start_abc[3], const OrientAxes &ax,
        std::vector<OrientMove> &moves, size_t emit, double tol, double spacing,
        const OrientCarry &in, OrientCarry &out, OrientSmoothInfo *info)
{
    size_t n = moves.size();
    out.mask = 0;
    for (OrientMove &m : moves) {
        m.mask = 0;
    }
    if (info) {
        *info = OrientSmoothInfo{0.0, 0.0, 0.0, 0.0};
    }
    if (n < 3 || emit == 0 || emit > n) {
        return false;
    }
    std::vector<double> s(n + 1);
    s[0] = 0.0;
    for (size_t i = 0; i < n; i++) {
        s[i + 1] = s[i] + moves[i].length;
    }
    /* the angular axes that move along the run or start on the curve */
    std::vector<OrientSmoothAxis> axes;
    int which[3];
    unsigned mask = 0;
    for (int k = 0; k < 3; k++) {
        if (!(ax.angular & (1u << k))) {
            continue;
        }
        OrientSmoothAxis a;
        bool moves_k = false;
        a.y.resize(n + 1);
        a.y[0] = start_abc[k];
        for (size_t i = 0; i < n; i++) {
            a.y[i + 1] = moves[i].abc[k];
            moves_k = moves_k || a.y[i + 1] != a.y[0];
        }
        if (in.mask & (1u << k)) {
            a.start_fixed = true;
            a.start_val = in.val[k];
            a.start_d1 = in.d1[k];
            a.start_d2 = in.d2[k];
        } else if (!moves_k) {
            continue;
        }
        a.jerk = ax.jerk[k];
        a.min = ax.min[k];
        a.max = ax.max[k];
        which[axes.size()] = k;
        axes.push_back(std::move(a));
        mask |= 1u << k;
    }
    if (!mask || !orientSmooth(s, axes, tol, spacing, info)) {
        return false;
    }
    for (size_t i = 0; i < emit; i++) {
        OrientMove &m = moves[i];
        m.mask = mask;
        for (int q = 0; q < 12; q++) {
            m.shape[q] = 0.0;
        }
        for (size_t a = 0; a < axes.size(); a++) {
            int k = which[a];
            m.out[k] = axes[a].val[i + 1];
            m.shape[k] = axes[a].d1[i];
            m.shape[3 + k] = axes[a].d2[i];
            m.shape[6 + k] = axes[a].d1[i + 1];
            m.shape[9 + k] = axes[a].d2[i + 1];
        }
    }
    if (emit < n) {
        out.mask = mask;
        for (size_t a = 0; a < axes.size(); a++) {
            int k = which[a];
            out.val[k] = axes[a].val[emit];
            out.d1[k] = axes[a].d1[emit];
            out.d2[k] = axes[a].d2[emit];
        }
    }
    return true;
}

size_t orientSmoothSend(const double start_abc[3], const OrientAxes &ax,
        std::vector<OrientMove> &moves, size_t emit, double tol, double spacing,
        const OrientCarry &in, OrientCarry &out)
{
    /* the parts smoothed so far: where each starts, what it starts on, and
     * whether it hands its curve on to the moves after it */
    struct Part {
        size_t begin, end;
        double start[3];
        OrientCarry in;
        bool open;
    };
    std::vector<Part> parts;
    size_t n = moves.size(), done = 0, want = n;
    double start[3] = {start_abc[0], start_abc[1], start_abc[2]};
    OrientCarry carry = in;
    std::vector<OrientMove> part;
    out.mask = 0;
    for (OrientMove &m : moves) {
        m.mask = 0;
    }
    emit = std::min(emit, n);
    while (done < emit) {
        size_t avail = n - done, m = std::min(avail, want);
        /* a part short of the moves held leaves its last quarter to the next */
        size_t e = m == avail ? emit - done : std::min(emit - done, std::max<size_t>(1, m - m / 4));
        part.assign(moves.begin() + done, moves.begin() + done + m);
        OrientCarry next;
        if (orientSmoothRun(start, ax, part, e, tol, spacing, carry, next)) {
            std::copy(part.begin(), part.begin() + e, moves.begin() + done);
            parts.push_back({done, done + e, {start[0], start[1], start[2]}, carry, next.mask != 0});
            carry = next;
            done += e;
            want = std::min(n, 2 * m);
        } else if (m > MIN_PART) {
            want = m / 2;
            continue;
        } else {
            /* Even a few moves fail, and the first of them stay as
             * programmed. A part running into them on its curve would
             * leave them off their programmed start, a corner the planner
             * blends on top of that: it ends on the programmed point
             * instead, or stays as programmed too. */
            size_t at = done;
            while (!parts.empty() && parts.back().end == at && parts.back().open) {
                Part &p = parts.back();
                part.assign(moves.begin() + p.begin, moves.begin() + at);
                OrientCarry none;
                if (orientSmoothRun(p.start, ax, part, part.size(), tol, spacing, p.in, none)) {
                    std::copy(part.begin(), part.end(), moves.begin() + p.begin);
                    p.open = false;
                    break;
                }
                for (size_t i = p.begin; i < at; i++) {
                    moves[i].mask = 0;
                }
                at = p.begin;
                parts.pop_back();
            }
            e = std::min(emit - done, std::max<size_t>(1, m / 2));
            carry.mask = 0;
            done += e;
            want = MIN_PART;
        }
        for (int k = 0; k < 3; k++) {
            start[k] = moves[done - 1].abc[k];
        }
    }
    out = carry;
    size_t smoothed = 0;
    for (size_t i = 0; i < emit; i++) {
        smoothed += moves[i].mask != 0;
    }
    return smoothed;
}
