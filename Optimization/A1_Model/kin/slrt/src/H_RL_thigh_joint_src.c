/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:54 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RL_thigh_joint_src.h"

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
  double t4407;
  double t4449;
  double t4458;
  double t4456;
  double t4464;
  double t811;
  double t4427;
  double t4440;
  double t4441;
  double t4477;
  double t4457;
  double t4466;
  double t4468;
  double t4483;
  double t4490;
  double t4491;
  double t4497;
  double t4501;
  double t4510;
  double t4513;
  double t4514;
  double t4515;
  double t4470;
  double t4492;
  double t4493;
  double t4512;
  double t4516;
  double t4518;
  double t4529;
  double t4533;
  double t4536;
  double t4542;
  double t4543;
  double t4544;
  double t4439;
  double t4494;
  double t4495;
  double t4574;
  double t4575;
  double t4578;
  double t4579;
  double t4581;
  double t4582;
  double t4584;
  double t4587;
  double t4588;
  double t4590;
  double t4545;
  double t4546;
  double t4547;
  double t4593;
  double t4594;
  double t4496;
  double t4519;
  double t4523;
  double t4548;
  double t4554;
  double t4559;
  double t4524;
  double t4537;
  double t4539;
  t4407 = Cos(var1[3]);
  t4449 = Cos(var1[5]);
  t4458 = Sin(var1[3]);
  t4456 = Sin(var1[4]);
  t4464 = Sin(var1[5]);
  t811 = Cos(var1[16]);
  t4427 = Cos(var1[4]);
  t4440 = Sin(var1[16]);
  t4441 = Cos(var1[15]);
  t4477 = Sin(var1[15]);
  t4457 = t4407*t4449*t4456;
  t4466 = t4458*t4464;
  t4468 = t4457 + t4466;
  t4483 = -1.*t4449*t4458;
  t4490 = t4407*t4456*t4464;
  t4491 = t4483 + t4490;
  t4497 = t4449*t4458*t4456;
  t4501 = -1.*t4407*t4464;
  t4510 = t4497 + t4501;
  t4513 = t4407*t4449;
  t4514 = t4458*t4456*t4464;
  t4515 = t4513 + t4514;
  t4470 = t4441*t4468;
  t4492 = -1.*t4477*t4491;
  t4493 = t4470 + t4492;
  t4512 = t4441*t4510;
  t4516 = -1.*t4477*t4515;
  t4518 = t4512 + t4516;
  t4529 = t4441*t4427*t4449;
  t4533 = -1.*t4427*t4477*t4464;
  t4536 = t4529 + t4533;
  t4542 = t4477*t4468;
  t4543 = t4441*t4491;
  t4544 = t4542 + t4543;
  t4439 = t811*t4407*t4427;
  t4494 = -1.*t4440*t4493;
  t4495 = t4439 + t4494;
  t4574 = -1.*t811;
  t4575 = 1. + t4574;
  t4578 = -0.183*t4575;
  t4579 = 0. + t4578;
  t4581 = -0.047*t4477;
  t4582 = 0. + t4581;
  t4584 = -1.*t4441;
  t4587 = 1. + t4584;
  t4588 = 0.047*t4587;
  t4590 = 0. + t4588;
  t4545 = t4477*t4510;
  t4546 = t4441*t4515;
  t4547 = t4545 + t4546;
  t4593 = -0.183*t4440;
  t4594 = 0. + t4593;
  t4496 = t811*t4427*t4458;
  t4519 = -1.*t4440*t4518;
  t4523 = t4496 + t4519;
  t4548 = t4427*t4449*t4477;
  t4554 = t4441*t4427*t4464;
  t4559 = t4548 + t4554;
  t4524 = -1.*t811*t4456;
  t4537 = -1.*t4440*t4536;
  t4539 = t4524 + t4537;
  p_output1[0]=t4495;
  p_output1[1]=t4523;
  p_output1[2]=t4539;
  p_output1[3]=0.;
  p_output1[4]=t4544;
  p_output1[5]=t4547;
  p_output1[6]=t4559;
  p_output1[7]=0.;
  p_output1[8]=t4407*t4427*t4440 + t4493*t811;
  p_output1[9]=t4427*t4440*t4458 + t4518*t811;
  p_output1[10]=-1.*t4440*t4456 + t4536*t811;
  p_output1[11]=0.;
  p_output1[12]=0. - 0.183*t4495 + 0.13205*t4544 + t4407*t4427*t4579 + t4468*t4582 + t4491*t4590 + t4493*t4594 + var1[0];
  p_output1[13]=0. - 0.183*t4523 + 0.13205*t4547 + t4427*t4458*t4579 + t4510*t4582 + t4515*t4590 + t4518*t4594 + var1[1];
  p_output1[14]=0. - 0.183*t4539 + 0.13205*t4559 - 1.*t4456*t4579 + t4427*t4449*t4582 + t4427*t4464*t4590 + t4536*t4594 + var1[2];
  p_output1[15]=1.;
}



void H_RL_thigh_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
