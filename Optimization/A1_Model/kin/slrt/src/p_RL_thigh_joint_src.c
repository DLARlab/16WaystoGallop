/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:52 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RL_thigh_joint_src.h"

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
  double t4296;
  double t4307;
  double t4310;
  double t4308;
  double t4311;
  double t4302;
  double t4309;
  double t4313;
  double t4314;
  double t4318;
  double t4323;
  double t4324;
  double t4325;
  double t4273;
  double t4297;
  double t4333;
  double t4336;
  double t4337;
  double t4340;
  double t4287;
  double t4289;
  double t4290;
  double t4293;
  double t4303;
  double t4305;
  double t4319;
  double t4320;
  double t4321;
  double t4322;
  double t4348;
  double t4349;
  double t4350;
  double t4352;
  double t4353;
  double t4356;
  double t4334;
  double t4335;
  double t4362;
  double t4363;
  double t4364;
  double t4378;
  double t4379;
  double t4380;
  t4296 = Cos(var1[3]);
  t4307 = Cos(var1[5]);
  t4310 = Sin(var1[3]);
  t4308 = Sin(var1[4]);
  t4311 = Sin(var1[5]);
  t4302 = Sin(var1[15]);
  t4309 = t4296*t4307*t4308;
  t4313 = t4310*t4311;
  t4314 = t4309 + t4313;
  t4318 = Cos(var1[15]);
  t4323 = -1.*t4307*t4310;
  t4324 = t4296*t4308*t4311;
  t4325 = t4323 + t4324;
  t4273 = Cos(var1[16]);
  t4297 = Cos(var1[4]);
  t4333 = Sin(var1[16]);
  t4336 = t4318*t4314;
  t4337 = -1.*t4302*t4325;
  t4340 = t4336 + t4337;
  t4287 = -1.*t4273;
  t4289 = 1. + t4287;
  t4290 = -0.183*t4289;
  t4293 = 0. + t4290;
  t4303 = -0.047*t4302;
  t4305 = 0. + t4303;
  t4319 = -1.*t4318;
  t4320 = 1. + t4319;
  t4321 = 0.047*t4320;
  t4322 = 0. + t4321;
  t4348 = t4307*t4310*t4308;
  t4349 = -1.*t4296*t4311;
  t4350 = t4348 + t4349;
  t4352 = t4296*t4307;
  t4353 = t4310*t4308*t4311;
  t4356 = t4352 + t4353;
  t4334 = -0.183*t4333;
  t4335 = 0. + t4334;
  t4362 = t4318*t4350;
  t4363 = -1.*t4302*t4356;
  t4364 = t4362 + t4363;
  t4378 = t4318*t4297*t4307;
  t4379 = -1.*t4297*t4302*t4311;
  t4380 = t4378 + t4379;
  p_output1[0]=0. + t4293*t4296*t4297 + t4305*t4314 + t4322*t4325 + 0.13205*(t4302*t4314 + t4318*t4325) + t4335*t4340 - 0.183*(t4273*t4296*t4297 - 1.*t4333*t4340) + var1[0];
  p_output1[1]=0. + t4293*t4297*t4310 + t4305*t4350 + t4322*t4356 + 0.13205*(t4302*t4350 + t4318*t4356) + t4335*t4364 - 0.183*(t4273*t4297*t4310 - 1.*t4333*t4364) + var1[1];
  p_output1[2]=0. + t4297*t4305*t4307 - 1.*t4293*t4308 + 0.13205*(t4297*t4302*t4307 + t4297*t4311*t4318) + t4297*t4311*t4322 + t4335*t4380 - 0.183*(-1.*t4273*t4308 - 1.*t4333*t4380) + var1[2];
}



void p_RL_thigh_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
