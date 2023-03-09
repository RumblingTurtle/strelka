#ifndef SCALING_H
#define SCALING_H

#ifdef __cplusplus
extern "C" {
#endif // ifdef __cplusplus

// Functions to scale problem data
#include "constants.h"
#include "lin_alg.h"
#include "types.h"

// Enable data scaling if EMBEDDED is disabled or if EMBEDDED == 2
#if EMBEDDED != 1

/**
 * Scale problem matrices
 * @param  work Workspace
 * @return      exitflag
 */
c_int scale_data(struct OSQPWorkspace *work);
#endif // if EMBEDDED != 1

/**
 * Unscale problem matrices
 * @param  work Workspace
 * @return      exitflag
 */
c_int unscale_data(struct OSQPWorkspace *work);

/**
 * Unscale solution
 * @param  work Workspace
 * @return      exitflag
 */
c_int unscale_solution(struct OSQPWorkspace *work);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // ifndef SCALING_H
