/********************************************************************
* Description: orient_smooth.hh
*   Smoothing of the rotary axes along runs of short lines
*   ([TRAJ]ORIENT_SMOOTH), for emccanon.
*
* License: GPL Version 2
* System: Linux
********************************************************************/
#ifndef ORIENT_SMOOTH_HH
#define ORIENT_SMOOTH_HH

#include <vector>

/* One rotary axis of a run: the programmed value at each point, its jerk
 * limit (0 = none), whether the run starts on a curve sent before, with
 * its value, rate and second derivative there, and on success the value,
 * the rate and the second derivative per unit path length of the curve at
 * each point. */
struct OrientSmoothAxis {
    std::vector<double> y;
    double jerk = 0.0;
    bool start_fixed = false;
    double start_val = 0.0, start_d1 = 0.0, start_d2 = 0.0;
    std::vector<double> val, d1, d2;
};

/* what the fit settled on, for tests */
struct OrientSmoothInfo {
    double spacing;             // breakpoint spacing of the curve taken
    double lambda;              // its smoothing weight
    double t_raw, t_fit;        // estimated times of the lines and of the curve
};

/* The rotary axes along a run of lines, s the path length at each point
 * (s[0] = 0, increasing). Each axis becomes one smooth curve of the path
 * length through both ends of the run, or from the curve it starts on,
 * the same smoothing for every axis, so that the distance from the
 * programmed lines over all the axes, the curve followed line by line as
 * the quintic its ends give, stays within tol. spacing is the coarsest
 * breakpoint spacing tried. False, leaving the run as programmed, when no
 * curve is found or none is estimated to run faster than the lines under
 * the axis jerk limits. */
bool orientSmooth(const std::vector<double> &s, std::vector<OrientSmoothAxis> &axes,
        double tol, double spacing, OrientSmoothInfo *info = nullptr);

/* A line of a run held back: its end, in, and on the curve, out. */
struct OrientLine {
    double xyz[3];              // end of the line
    double abc[3];              // programmed A, B and C at its end
    unsigned mask;              // the rotary axes on the curve, bit k for A + k
    double out[3];              // their end on the curve
    double shape[12];           // d0[3], dd0[3], d1[3], dd1[3], see tpSetOrientShape()
};

/* where a run sent in part leaves the curve for the rest of it */
struct OrientCarry {
    unsigned mask;              // axes on the curve there
    double val[3], d1[3], d2[3];
};

/* Fit the run of lines from start (programmed), on the curve in leaves
 * off, for the angular axes among A, B and C (bit k for A + k), and fill
 * mask, out and shape of the first emit lines; out is where the rest of
 * the run then starts. False, leaving the lines as programmed, when the
 * run is not smoothed. */
bool orientSmoothRun(const double start_xyz[3], const double start_abc[3], unsigned angular,
        const double jerk[3], std::vector<OrientLine> &lines, size_t emit, double tol,
        double spacing, const OrientCarry &in, OrientCarry &out,
        OrientSmoothInfo *info = nullptr);

#endif
