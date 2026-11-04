/*
 * foc_math.h
 *
 * Math helpers shared by all FOC modules:
 *   - fast sin / cos / atan2 (lookup table, copied from SimpleFOC)
 *   - _constrain / _sign macros and sqrt(3) constants
 *
 * Formerly foc_utils.h + the macro section of motor_control.h.
 */

#ifndef INC_FOC_MATH_H_
#define INC_FOC_MATH_H_

#include <math.h>

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _sign(x) ((x)>0?1:((x)<0?-1:0))
#define _SQRT3 1.73205080757f
#define _SQRT3_2 0.86602540378f
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f

/**
 *  Function approximating the sine calculation by using fixed size array
 * - execution time ~40us (Arduino UNO)
 *
 * @param a angle in between 0 and 2PI
 */
float _sin(float a);
/**
 * Function approximating cosine calculation by using fixed size array
 * - execution time ~50us (Arduino UNO)
 *
 * @param a angle in between 0 and 2PI
 */
float _cos(float a);
/**
 * Function returning both sine and cosine of the angle in one call.
 * Internally it uses the _sin and _cos functions, but you may wish to
 * provide your own implementation which is more optimized.
 */
void _sincos(float a, float* s, float* c);

/**
 * Function approximating atan2
 *
 */
float _atan2(float y, float x);

#endif /* INC_FOC_MATH_H_ */
