/********************************************************************
 * Description: nonrt_kins.h
 *   Interface a kinematics module exports so that a non-RT caller can
 *   evaluate it.
 *
 *   A trajectory planner needs forward and inverse kinematics at poses
 *   the machine has not reached yet, which means calling them outside
 *   the servo thread.  A module opts in by exporting nonrt_attach().
 *
 *   The caller dlopens the module, calls nonrt_attach() once with the
 *   coordinates string from the INI file, and gets back the module's
 *   own forward and inverse entry points.  nonrt_attach() points the
 *   module's haldata at the value cells of the pins the running RT
 *   instance already owns, so both copies read the same numbers and
 *   the kinematics code itself does not change.
 *
 *   Bind input pins only.  Output pins and any scratch storage must
 *   stay private to the non-RT copy, or the two copies will write to
 *   each other's state.
 *
 *   Modules that do not export nonrt_attach() still work as they
 *   always have; the caller sees that the symbol is missing and does
 *   without kinematic limits for that machine.
 *
 * Author: LinuxCNC
 * License: GPL Version 2
 * System: Linux
 *
 * Copyright (c) 2024 All rights reserved.
 ********************************************************************/

#ifndef NONRT_KINS_H
#define NONRT_KINS_H

#include <stdarg.h>

#include <rtapi.h>
#include <hal.h>
#include <emcpos.h>
#include <kinematics.h>

/* Filled in by nonrt_attach().  A module that reports is_identity has
   joints equal to axes and the caller needs no module code at all, so
   forward and inverse may be left NULL. */
typedef struct {
    int (*forward)(const double *joints, EmcPose *pos,
                   const KINEMATICS_FORWARD_FLAGS *fflags,
                   KINEMATICS_INVERSE_FLAGS *iflags);
    int (*inverse)(const EmcPose *pos, double *joints,
                   const KINEMATICS_INVERSE_FLAGS *iflags,
                   KINEMATICS_FORWARD_FLAGS *fflags);
    int is_identity;
} nonrt_ops_t;

/* Exported by a participating module:
     int nonrt_attach(const char *coordinates, nonrt_ops_t *ops);
   Returns 0 on success. */

/* Point 'dst' at the value cell of an existing HAL pin, so that
   hal_get_real(*dst) reads whatever the RT instance currently sees.
   The cell moves if the pin is later linked to a signal, so bind after
   the configuration is up and re-bind if that assumption is broken. */
static inline int nonrt_bind_real(hal_real_t *dst, const char *fmt, ...)
{
    char name[HAL_NAME_LEN + 1];
    hal_data_u *cell;
    hal_type_t type;
    va_list ap;

    va_start(ap, fmt);
    rtapi_vsnprintf(name, sizeof(name), fmt, ap);
    va_end(ap);

    if (hal_get_pin_value_by_name(name, &type, &cell, NULL) != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "nonrt_bind_real: no such pin '%s'\n", name);
        return -1;
    }
    if (type != HAL_FLOAT) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "nonrt_bind_real: pin '%s' is not a float\n", name);
        return -1;
    }
    *dst = (hal_real_t)(void *)cell;
    return 0;
}

#endif /* NONRT_KINS_H */
