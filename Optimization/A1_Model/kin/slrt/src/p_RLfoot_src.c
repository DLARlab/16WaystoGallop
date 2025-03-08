/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:11 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RLfoot_src.h"

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
  double t3784;
  double t7208;
  double t7216;
  double t7209;
  double t7217;
  double t7187;
  double t7212;
  double t7220;
  double t7222;
  double t7224;
  double t7234;
  double t7238;
  double t7242;
  double t6902;
  double t7255;
  double t1423;
  double t7259;
  double t7262;
  double t7263;
  double t7265;
  double t7266;
  double t7267;
  double t7269;
  double t7272;
  double t7273;
  double t7276;
  double t7282;
  double t7283;
  double t7285;
  double t1432;
  double t1447;
  double t2197;
  double t3717;
  double t7197;
  double t7205;
  double t7226;
  double t7227;
  double t7231;
  double t7232;
  double t7303;
  double t7304;
  double t7305;
  double t7307;
  double t7308;
  double t7309;
  double t7257;
  double t7258;
  double t7268;
  double t7270;
  double t7271;
  double t7315;
  double t7316;
  double t7319;
  double t7278;
  double t7279;
  double t7281;
  double t7323;
  double t7327;
  double t7328;
  double t7330;
  double t7331;
  double t7332;
  double t7350;
  double t7351;
  double t7352;
  double t7354;
  double t7355;
  double t7356;
  double t7358;
  double t7359;
  double t7360;
  t3784 = Cos(var1[3]);
  t7208 = Cos(var1[5]);
  t7216 = Sin(var1[3]);
  t7209 = Sin(var1[4]);
  t7217 = Sin(var1[5]);
  t7187 = Sin(var1[15]);
  t7212 = t3784*t7208*t7209;
  t7220 = t7216*t7217;
  t7222 = t7212 + t7220;
  t7224 = Cos(var1[15]);
  t7234 = -1.*t7208*t7216;
  t7238 = t3784*t7209*t7217;
  t7242 = t7234 + t7238;
  t6902 = Cos(var1[4]);
  t7255 = Sin(var1[16]);
  t1423 = Cos(var1[16]);
  t7259 = t7224*t7222;
  t7262 = -1.*t7187*t7242;
  t7263 = t7259 + t7262;
  t7265 = Cos(var1[17]);
  t7266 = -1.*t7265;
  t7267 = 1. + t7266;
  t7269 = Sin(var1[17]);
  t7272 = t3784*t6902*t7255;
  t7273 = t1423*t7263;
  t7276 = t7272 + t7273;
  t7282 = t1423*t3784*t6902;
  t7283 = -1.*t7255*t7263;
  t7285 = t7282 + t7283;
  t1432 = -1.*t1423;
  t1447 = 1. + t1432;
  t2197 = -0.183*t1447;
  t3717 = 0. + t2197;
  t7197 = -0.047*t7187;
  t7205 = 0. + t7197;
  t7226 = -1.*t7224;
  t7227 = 1. + t7226;
  t7231 = 0.047*t7227;
  t7232 = 0. + t7231;
  t7303 = t7208*t7216*t7209;
  t7304 = -1.*t3784*t7217;
  t7305 = t7303 + t7304;
  t7307 = t3784*t7208;
  t7308 = t7216*t7209*t7217;
  t7309 = t7307 + t7308;
  t7257 = -0.183*t7255;
  t7258 = 0. + t7257;
  t7268 = -0.2*t7267;
  t7270 = -0.183*t7269;
  t7271 = 0. + t7268 + t7270;
  t7315 = t7224*t7305;
  t7316 = -1.*t7187*t7309;
  t7319 = t7315 + t7316;
  t7278 = -0.183*t7267;
  t7279 = 0.2*t7269;
  t7281 = 0. + t7278 + t7279;
  t7323 = t6902*t7255*t7216;
  t7327 = t1423*t7319;
  t7328 = t7323 + t7327;
  t7330 = t1423*t6902*t7216;
  t7331 = -1.*t7255*t7319;
  t7332 = t7330 + t7331;
  t7350 = t7224*t6902*t7208;
  t7351 = -1.*t6902*t7187*t7217;
  t7352 = t7350 + t7351;
  t7354 = -1.*t7255*t7209;
  t7355 = t1423*t7352;
  t7356 = t7354 + t7355;
  t7358 = -1.*t1423*t7209;
  t7359 = -1.*t7255*t7352;
  t7360 = t7358 + t7359;
  p_output1[0]=0. + t3717*t3784*t6902 + t7205*t7222 + t7232*t7242 + 0.13205*(t7187*t7222 + t7224*t7242) + t7258*t7263 + t7271*t7276 + t7281*t7285 - 0.183*(-1.*t7269*t7276 + t7265*t7285) - 0.4*(t7265*t7276 + t7269*t7285) + var1[0];
  p_output1[1]=0. + t3717*t6902*t7216 + t7205*t7305 + t7232*t7309 + 0.13205*(t7187*t7305 + t7224*t7309) + t7258*t7319 + t7271*t7328 + t7281*t7332 - 0.183*(-1.*t7269*t7328 + t7265*t7332) - 0.4*(t7265*t7328 + t7269*t7332) + var1[1];
  p_output1[2]=0. + t6902*t7205*t7208 - 1.*t3717*t7209 + 0.13205*(t6902*t7187*t7208 + t6902*t7217*t7224) + t6902*t7217*t7232 + t7258*t7352 + t7271*t7356 + t7281*t7360 - 0.183*(-1.*t7269*t7356 + t7265*t7360) - 0.4*(t7265*t7356 + t7269*t7360) + var1[2];
}



void p_RLfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
