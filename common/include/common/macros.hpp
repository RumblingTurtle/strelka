#ifndef MACROS_H
#define MACROS_H

#define EPS 0.000001
#define FOR_EACH_LEG for (int LEG_ID = 0; LEG_ID < 4; LEG_ID++)
#define APPROX_EQUAL(x, y) (x - y) * ((x - y > 0) - (x - y < 0)) < EPS
#endif