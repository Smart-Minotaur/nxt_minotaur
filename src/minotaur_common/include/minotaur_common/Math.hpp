#ifndef MINOTAUR_MATH_HPP
#define MINOTAUR_MATH_HPP

#include <cmath>

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

#define DEG_PER_CIRCLE 360
#define RAD_PER_CIRCLE (2 * M_PI) 

#define DEG_TO_RAD(deg) ((M_PI * deg) / 180)
#define RAD_TO_DEG(rad) ((rad * 180) / M_PI)

#define MSEC_PER_SEC 1000
#define USEC_PER_SEC 1000000

#define USEC_TO_SEC(usec) (((float) usec) / ((float) USEC_PER_SEC))
#define MSEC_TO_SEC(msec) (((float) msec) / ((float) MSEC_PER_SEC))
#define SEC_TO_USEC(sec) ((int) (sec * USEC_PER_SEC))
#define SEC_TO_MSEC(sec) ((int) (sec * MSEC_PER_SEC))

#define CM_PER_METER 100
#define CM_TO_METER(cm) (((float) cm) / ((float) CM_PER_METER))
#define METER_TO_CM(m) ((int) (m * CM_PER_METER))

#define RADIUS_TO_CIRCUMFERENCE(rad) (rad * 2 * M_PI)

#endif
