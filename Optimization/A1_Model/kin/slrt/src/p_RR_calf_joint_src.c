/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:44 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RR_calf_joint_src.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE static __inline__        /* use standard inline */
#endif

/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
INLINE double Power(double x, double y) { return pow(x, y); }
INLINE double Sqrt(double x) { return sqrt(x); }

INLINE double Abs(double x) { return fabs(x); }

INLINE double Exp(double x) { return exp(x); }
INLINE double Log(double x) { return log(x); }

INLINE double Sin(double x) { return sin(x); }
INLINE double Cos(double x) { return cos(x); }
INLINE double Tan(double x) { return tan(x); }

INLINE double Csc(double x) { return 1.0/sin(x); }
INLINE double Sec(double x) { return 1.0/cos(x); }

INLINE double ArcSin(double x) { return asin(x); }
INLINE double ArcCos(double x) { return acos(x); }

/* update ArcTan function to use atan2 instead. */
INLINE double ArcTan(double x, double y) { return atan2(y,x); }

INLINE double Sinh(double x) { return sinh(x); }
INLINE double Cosh(double x) { return cosh(x); }
INLINE double Tanh(double x) { return tanh(x); }

#define E 2.71828182845904523536029
#define Pi 3.14159265358979323846264
#define Degree 0.01745329251994329576924

/*
 * Sub functions
 */
static void output1(double *p_output1,const double *var1)
{
  double t3374;
  double t3385;
  double t3388;
  double t3386;
  double t3389;
  double t3380;
  double t3387;
  double t3391;
  double t3392;
  double t3396;
  double t3405;
  double t3407;
  double t3408;
  double t3375;
  double t3416;
  double t3349;
  double t3419;
  double t3420;
  double t3421;
  double t3423;
  double t3424;
  double t3425;
  double t3428;
  double t3432;
  double t3433;
  double t3434;
  double t3441;
  double t3442;
  double t3444;
  double t3365;
  double t3367;
  double t3368;
  double t3371;
  double t3381;
  double t3383;
  double t3400;
  double t3402;
  double t3403;
  double t3404;
  double t3458;
  double t3459;
  double t3460;
  double t3462;
  double t3463;
  double t3464;
  double t3417;
  double t3418;
  double t3427;
  double t3429;
  double t3430;
  double t3477;
  double t3478;
  double t3479;
  double t3436;
  double t3437;
  double t3440;
  double t3481;
  double t3482;
  double t3483;
  double t3485;
  double t3486;
  double t3487;
  double t3505;
  double t3506;
  double t3507;
  double t3509;
  double t3510;
  double t3511;
  double t3513;
  double t3514;
  double t3515;
  t3374 = Cos(var1[3]);
  t3385 = Cos(var1[5]);
  t3388 = Sin(var1[3]);
  t3386 = Sin(var1[4]);
  t3389 = Sin(var1[5]);
  t3380 = Sin(var1[12]);
  t3387 = t3374*t3385*t3386;
  t3391 = t3388*t3389;
  t3392 = t3387 + t3391;
  t3396 = Cos(var1[12]);
  t3405 = -1.*t3385*t3388;
  t3407 = t3374*t3386*t3389;
  t3408 = t3405 + t3407;
  t3375 = Cos(var1[4]);
  t3416 = Sin(var1[13]);
  t3349 = Cos(var1[13]);
  t3419 = t3396*t3392;
  t3420 = -1.*t3380*t3408;
  t3421 = t3419 + t3420;
  t3423 = Cos(var1[14]);
  t3424 = -1.*t3423;
  t3425 = 1. + t3424;
  t3428 = Sin(var1[14]);
  t3432 = t3374*t3375*t3416;
  t3433 = t3349*t3421;
  t3434 = t3432 + t3433;
  t3441 = t3349*t3374*t3375;
  t3442 = -1.*t3416*t3421;
  t3444 = t3441 + t3442;
  t3365 = -1.*t3349;
  t3367 = 1. + t3365;
  t3368 = -0.183*t3367;
  t3371 = 0. + t3368;
  t3381 = 0.047*t3380;
  t3383 = 0. + t3381;
  t3400 = -1.*t3396;
  t3402 = 1. + t3400;
  t3403 = -0.047*t3402;
  t3404 = 0. + t3403;
  t3458 = t3385*t3388*t3386;
  t3459 = -1.*t3374*t3389;
  t3460 = t3458 + t3459;
  t3462 = t3374*t3385;
  t3463 = t3388*t3386*t3389;
  t3464 = t3462 + t3463;
  t3417 = -0.183*t3416;
  t3418 = 0. + t3417;
  t3427 = -0.2*t3425;
  t3429 = -0.183*t3428;
  t3430 = 0. + t3427 + t3429;
  t3477 = t3396*t3460;
  t3478 = -1.*t3380*t3464;
  t3479 = t3477 + t3478;
  t3436 = -0.183*t3425;
  t3437 = 0.2*t3428;
  t3440 = 0. + t3436 + t3437;
  t3481 = t3375*t3416*t3388;
  t3482 = t3349*t3479;
  t3483 = t3481 + t3482;
  t3485 = t3349*t3375*t3388;
  t3486 = -1.*t3416*t3479;
  t3487 = t3485 + t3486;
  t3505 = t3396*t3375*t3385;
  t3506 = -1.*t3375*t3380*t3389;
  t3507 = t3505 + t3506;
  t3509 = -1.*t3416*t3386;
  t3510 = t3349*t3507;
  t3511 = t3509 + t3510;
  t3513 = -1.*t3349*t3386;
  t3514 = -1.*t3416*t3507;
  t3515 = t3513 + t3514;
  p_output1[0]=0. + t3371*t3374*t3375 + t3383*t3392 + t3404*t3408 - 0.13205*(t3380*t3392 + t3396*t3408) + t3418*t3421 + t3430*t3434 + t3440*t3444 - 0.183*(-1.*t3428*t3434 + t3423*t3444) - 0.2*(t3423*t3434 + t3428*t3444) + var1[0];
  p_output1[1]=0. + t3371*t3375*t3388 + t3383*t3460 + t3404*t3464 - 0.13205*(t3380*t3460 + t3396*t3464) + t3418*t3479 + t3430*t3483 + t3440*t3487 - 0.183*(-1.*t3428*t3483 + t3423*t3487) - 0.2*(t3423*t3483 + t3428*t3487) + var1[1];
  p_output1[2]=0. + t3375*t3383*t3385 - 1.*t3371*t3386 - 0.13205*(t3375*t3380*t3385 + t3375*t3389*t3396) + t3375*t3389*t3404 + t3418*t3507 + t3430*t3511 + t3440*t3515 - 0.183*(-1.*t3428*t3511 + t3423*t3515) - 0.2*(t3423*t3511 + t3428*t3515) + var1[2];
}



void p_RR_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
