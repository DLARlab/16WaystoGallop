/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:53 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RL_thigh_joint_src.h"

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
  double t4361;
  double t4385;
  double t4378;
  double t4382;
  double t4386;
  double t4375;
  double t4384;
  double t4391;
  double t4392;
  double t4394;
  double t4399;
  double t4401;
  double t4403;
  double t152;
  double t4360;
  double t4412;
  double t4417;
  double t4418;
  double t4419;
  double t4324;
  double t4336;
  double t4345;
  double t4359;
  double t4376;
  double t4377;
  double t4395;
  double t4396;
  double t4397;
  double t4398;
  double t4427;
  double t4428;
  double t4431;
  double t4433;
  double t4434;
  double t4435;
  double t4415;
  double t4416;
  double t4441;
  double t4442;
  double t4443;
  double t4459;
  double t4461;
  double t4463;
  double t4479;
  double t4480;
  double t4481;
  double t4498;
  double t4501;
  double t4504;
  double t4516;
  double t4517;
  double t4518;
  double t4521;
  double t4523;
  double t4524;
  double t4533;
  double t4534;
  double t4535;
  double t4409;
  double t4538;
  double t4539;
  double t4549;
  double t4550;
  double t4551;
  double t4561;
  double t4562;
  double t4568;
  double t4569;
  double t4570;
  double t4572;
  double t4573;
  double t4542;
  double t4552;
  double t4553;
  double t4554;
  double t4555;
  double t4556;
  double t4557;
  double t4576;
  double t4577;
  t4361 = Sin(var1[3]);
  t4385 = Cos(var1[3]);
  t4378 = Cos(var1[5]);
  t4382 = Sin(var1[4]);
  t4386 = Sin(var1[5]);
  t4375 = Sin(var1[15]);
  t4384 = -1.*t4378*t4361*t4382;
  t4391 = t4385*t4386;
  t4392 = t4384 + t4391;
  t4394 = Cos(var1[15]);
  t4399 = -1.*t4385*t4378;
  t4401 = -1.*t4361*t4382*t4386;
  t4403 = t4399 + t4401;
  t152 = Cos(var1[16]);
  t4360 = Cos(var1[4]);
  t4412 = Sin(var1[16]);
  t4417 = t4394*t4392;
  t4418 = -1.*t4375*t4403;
  t4419 = t4417 + t4418;
  t4324 = -1.*t152;
  t4336 = 1. + t4324;
  t4345 = -0.183*t4336;
  t4359 = 0. + t4345;
  t4376 = -0.047*t4375;
  t4377 = 0. + t4376;
  t4395 = -1.*t4394;
  t4396 = 1. + t4395;
  t4397 = 0.047*t4396;
  t4398 = 0. + t4397;
  t4427 = t4385*t4378*t4382;
  t4428 = t4361*t4386;
  t4431 = t4427 + t4428;
  t4433 = -1.*t4378*t4361;
  t4434 = t4385*t4382*t4386;
  t4435 = t4433 + t4434;
  t4415 = -0.183*t4412;
  t4416 = 0. + t4415;
  t4441 = t4394*t4431;
  t4442 = -1.*t4375*t4435;
  t4443 = t4441 + t4442;
  t4459 = t4394*t4385*t4360*t4378;
  t4461 = -1.*t4385*t4360*t4375*t4386;
  t4463 = t4459 + t4461;
  t4479 = t4394*t4360*t4378*t4361;
  t4480 = -1.*t4360*t4375*t4361*t4386;
  t4481 = t4479 + t4480;
  t4498 = -1.*t4394*t4378*t4382;
  t4501 = t4375*t4382*t4386;
  t4504 = t4498 + t4501;
  t4516 = t4378*t4361;
  t4517 = -1.*t4385*t4382*t4386;
  t4518 = t4516 + t4517;
  t4521 = -1.*t4375*t4431;
  t4523 = t4394*t4518;
  t4524 = t4521 + t4523;
  t4533 = t4378*t4361*t4382;
  t4534 = -1.*t4385*t4386;
  t4535 = t4533 + t4534;
  t4409 = t4394*t4403;
  t4538 = -1.*t4375*t4535;
  t4539 = t4538 + t4409;
  t4549 = -1.*t4360*t4378*t4375;
  t4550 = -1.*t4394*t4360*t4386;
  t4551 = t4549 + t4550;
  t4561 = -1.*t4394*t4435;
  t4562 = t4521 + t4561;
  t4568 = t4385*t4378;
  t4569 = t4361*t4382*t4386;
  t4570 = t4568 + t4569;
  t4572 = -1.*t4394*t4570;
  t4573 = t4538 + t4572;
  t4542 = t4394*t4535;
  t4552 = t4416*t4551;
  t4553 = 0.183*t4412*t4551;
  t4554 = t4394*t4360*t4378;
  t4555 = -1.*t4360*t4375*t4386;
  t4556 = t4554 + t4555;
  t4557 = 0.13205*t4556;
  t4576 = -1.*t4375*t4570;
  t4577 = t4542 + t4576;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=-1.*t4359*t4360*t4361 + t4377*t4392 + t4398*t4403 + 0.13205*(t4375*t4392 + t4409) + t4416*t4419 - 0.183*(-1.*t152*t4360*t4361 - 1.*t4412*t4419);
  p_output1[10]=t4359*t4360*t4385 + t4377*t4431 + t4398*t4435 + 0.13205*(t4375*t4431 + t4394*t4435) + t4416*t4443 - 0.183*(t152*t4360*t4385 - 1.*t4412*t4443);
  p_output1[11]=0;
  p_output1[12]=t4360*t4377*t4378*t4385 - 1.*t4359*t4382*t4385 + 0.13205*(t4360*t4375*t4378*t4385 + t4360*t4385*t4386*t4394) + t4360*t4385*t4386*t4398 + t4416*t4463 - 0.183*(-1.*t152*t4382*t4385 - 1.*t4412*t4463);
  p_output1[13]=t4360*t4361*t4377*t4378 - 1.*t4359*t4361*t4382 + 0.13205*(t4360*t4361*t4375*t4378 + t4360*t4361*t4386*t4394) + t4360*t4361*t4386*t4398 + t4416*t4481 - 0.183*(-1.*t152*t4361*t4382 - 1.*t4412*t4481);
  p_output1[14]=-1.*t4359*t4360 - 1.*t4377*t4378*t4382 + 0.13205*(-1.*t4375*t4378*t4382 - 1.*t4382*t4386*t4394) - 1.*t4382*t4386*t4398 + t4416*t4504 - 0.183*(-1.*t152*t4360 - 1.*t4412*t4504);
  p_output1[15]=t4398*t4431 + t4377*t4518 + 0.13205*(t4441 + t4375*t4518) + 0.183*t4412*t4524 + t4416*t4524;
  p_output1[16]=t4377*t4403 + t4398*t4535 + 0.183*t4412*t4539 + t4416*t4539 + 0.13205*(t4375*t4403 + t4542);
  p_output1[17]=-1.*t4360*t4377*t4386 + t4360*t4378*t4398 + t4552 + t4553 + t4557;
  p_output1[18]=0;
  p_output1[19]=0;
  p_output1[20]=0;
  p_output1[21]=0;
  p_output1[22]=0;
  p_output1[23]=0;
  p_output1[24]=0;
  p_output1[25]=0;
  p_output1[26]=0;
  p_output1[27]=0;
  p_output1[28]=0;
  p_output1[29]=0;
  p_output1[30]=0;
  p_output1[31]=0;
  p_output1[32]=0;
  p_output1[33]=0;
  p_output1[34]=0;
  p_output1[35]=0;
  p_output1[36]=0;
  p_output1[37]=0;
  p_output1[38]=0;
  p_output1[39]=0;
  p_output1[40]=0;
  p_output1[41]=0;
  p_output1[42]=0;
  p_output1[43]=0;
  p_output1[44]=0;
  p_output1[45]=-0.047*t4394*t4431 + 0.047*t4375*t4435 + 0.13205*t4443 + 0.183*t4412*t4562 + t4416*t4562;
  p_output1[46]=-0.047*t4394*t4535 + 0.047*t4375*t4570 + 0.183*t4412*t4573 + t4416*t4573 + 0.13205*t4577;
  p_output1[47]=0.047*t4360*t4375*t4386 - 0.047*t4360*t4378*t4394 + t4552 + t4553 + t4557;
  p_output1[48]=-0.183*t4360*t4385*t4412 - 0.183*t152*t4443 - 0.183*(-1.*t4360*t4385*t4412 - 1.*t152*t4443);
  p_output1[49]=-0.183*t4360*t4361*t4412 - 0.183*t152*t4577 - 0.183*(-1.*t4360*t4361*t4412 - 1.*t152*t4577);
  p_output1[50]=0.183*t4382*t4412 - 0.183*t152*t4556 - 0.183*(t4382*t4412 - 1.*t152*t4556);
  p_output1[51]=0;
  p_output1[52]=0;
  p_output1[53]=0;
}



void Jp_RL_thigh_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
