/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:05 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FLfoot_src.h"

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
  double t601;
  double t2459;
  double t3041;
  double t2774;
  double t3360;
  double t665;
  double t617;
  double t618;
  double t1489;
  double t2803;
  double t3426;
  double t3586;
  double t3642;
  double t6126;
  double t6127;
  double t6183;
  double t6194;
  double t6225;
  double t6227;
  double t573;
  double t6265;
  double t6282;
  double t6285;
  double t6290;
  double t6291;
  double t6292;
  double t6293;
  double t6294;
  double t6295;
  double t6300;
  double t6336;
  double t6339;
  double t6340;
  double t619;
  double t6237;
  double t6246;
  double t6266;
  double t6267;
  double t6268;
  double t6280;
  double t6302;
  double t6304;
  double t6310;
  double t6315;
  double t6324;
  double t6332;
  double t6341;
  double t6342;
  double t6344;
  double t6347;
  double t6365;
  double t6368;
  double t6369;
  double t6372;
  double t6446;
  double t6450;
  double t6257;
  double t6269;
  double t6272;
  double t6381;
  double t6383;
  double t6385;
  double t6420;
  double t6422;
  double t6423;
  double t6425;
  double t6428;
  double t6429;
  double t6431;
  double t6432;
  double t6434;
  double t6437;
  double t6373;
  double t6376;
  double t6377;
  double t6442;
  double t6443;
  double t6454;
  double t6456;
  double t6462;
  double t6464;
  double t6465;
  double t6467;
  double t6309;
  double t6328;
  double t6330;
  double t6387;
  double t6389;
  double t6393;
  double t6378;
  double t6379;
  double t6380;
  double t6343;
  double t6366;
  double t6367;
  double t6408;
  double t6412;
  double t6414;
  t601 = Cos(var1[3]);
  t2459 = Cos(var1[5]);
  t3041 = Sin(var1[3]);
  t2774 = Sin(var1[4]);
  t3360 = Sin(var1[5]);
  t665 = Cos(var1[10]);
  t617 = Cos(var1[4]);
  t618 = Sin(var1[10]);
  t1489 = Cos(var1[9]);
  t2803 = t601*t2459*t2774;
  t3426 = t3041*t3360;
  t3586 = t2803 + t3426;
  t3642 = t1489*t3586;
  t6126 = Sin(var1[9]);
  t6127 = -1.*t2459*t3041;
  t6183 = t601*t2774*t3360;
  t6194 = t6127 + t6183;
  t6225 = -1.*t6126*t6194;
  t6227 = t3642 + t6225;
  t573 = Sin(var1[11]);
  t6265 = Cos(var1[11]);
  t6282 = t2459*t3041*t2774;
  t6285 = -1.*t601*t3360;
  t6290 = t6282 + t6285;
  t6291 = t1489*t6290;
  t6292 = t601*t2459;
  t6293 = t3041*t2774*t3360;
  t6294 = t6292 + t6293;
  t6295 = -1.*t6126*t6294;
  t6300 = t6291 + t6295;
  t6336 = t1489*t617*t2459;
  t6339 = -1.*t617*t6126*t3360;
  t6340 = t6336 + t6339;
  t619 = t601*t617*t618;
  t6237 = t665*t6227;
  t6246 = t619 + t6237;
  t6266 = t665*t601*t617;
  t6267 = -1.*t618*t6227;
  t6268 = t6266 + t6267;
  t6280 = t617*t618*t3041;
  t6302 = t665*t6300;
  t6304 = t6280 + t6302;
  t6310 = t665*t617*t3041;
  t6315 = -1.*t618*t6300;
  t6324 = t6310 + t6315;
  t6332 = -1.*t618*t2774;
  t6341 = t665*t6340;
  t6342 = t6332 + t6341;
  t6344 = -1.*t665*t2774;
  t6347 = -1.*t618*t6340;
  t6365 = t6344 + t6347;
  t6368 = t6126*t3586;
  t6369 = t1489*t6194;
  t6372 = t6368 + t6369;
  t6446 = -1.*t6265;
  t6450 = 1. + t6446;
  t6257 = -1.*t573*t6246;
  t6269 = t6265*t6268;
  t6272 = t6257 + t6269;
  t6381 = t6265*t6246;
  t6383 = t573*t6268;
  t6385 = t6381 + t6383;
  t6420 = -1.*t665;
  t6422 = 1. + t6420;
  t6423 = 0.183*t6422;
  t6425 = 0. + t6423;
  t6428 = -0.047*t6126;
  t6429 = 0. + t6428;
  t6431 = -1.*t1489;
  t6432 = 1. + t6431;
  t6434 = 0.047*t6432;
  t6437 = 0. + t6434;
  t6373 = t6126*t6290;
  t6376 = t1489*t6294;
  t6377 = t6373 + t6376;
  t6442 = 0.183*t618;
  t6443 = 0. + t6442;
  t6454 = -0.2*t6450;
  t6456 = 0.183*t573;
  t6462 = 0. + t6454 + t6456;
  t6464 = 0.183*t6450;
  t6465 = 0.2*t573;
  t6467 = 0. + t6464 + t6465;
  t6309 = -1.*t573*t6304;
  t6328 = t6265*t6324;
  t6330 = t6309 + t6328;
  t6387 = t6265*t6304;
  t6389 = t573*t6324;
  t6393 = t6387 + t6389;
  t6378 = t617*t2459*t6126;
  t6379 = t1489*t617*t3360;
  t6380 = t6378 + t6379;
  t6343 = -1.*t573*t6342;
  t6366 = t6265*t6365;
  t6367 = t6343 + t6366;
  t6408 = t6265*t6342;
  t6412 = t573*t6365;
  t6414 = t6408 + t6412;
  p_output1[0]=t6272;
  p_output1[1]=t6330;
  p_output1[2]=t6367;
  p_output1[3]=0.;
  p_output1[4]=t6372;
  p_output1[5]=t6377;
  p_output1[6]=t6380;
  p_output1[7]=0.;
  p_output1[8]=t6385;
  p_output1[9]=t6393;
  p_output1[10]=t6414;
  p_output1[11]=0.;
  p_output1[12]=0. + 0.183*t6272 + 0.13205*t6372 - 0.4*t6385 + t601*t617*t6425 + t3586*t6429 + t6194*t6437 + t6227*t6443 + t6246*t6462 + t6268*t6467 + var1[0];
  p_output1[13]=0. + 0.183*t6330 + 0.13205*t6377 - 0.4*t6393 + t3041*t617*t6425 + t6290*t6429 + t6294*t6437 + t6300*t6443 + t6304*t6462 + t6324*t6467 + var1[1];
  p_output1[14]=0. + 0.183*t6367 + 0.13205*t6380 - 0.4*t6414 - 1.*t2774*t6425 + t2459*t617*t6429 + t3360*t617*t6437 + t6340*t6443 + t6342*t6462 + t6365*t6467 + var1[2];
  p_output1[15]=1.;
}



void H_FLfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
