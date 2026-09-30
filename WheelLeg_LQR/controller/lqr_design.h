#ifndef LQR_DESIGN_H
#define LQR_DESIGN_H
#include "lqr.h"
/* Fixed 4-state/1-input solver. Row-major matrices; no allocation.
 * Returns iteration count, or negative status. Computation in double, MCU gains in float. */
LQR_API int LQR_Design(const double *a, const double *b, const double *q,
                       double r, double ts, double *ad, double *bd, double *p, double *k);
#endif
