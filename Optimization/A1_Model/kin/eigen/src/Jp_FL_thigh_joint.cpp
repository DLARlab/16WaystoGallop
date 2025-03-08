/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:57 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FL_thigh_joint.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE static inline        /* use standard inline */
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
static void output1(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t1459;
  double t2852;
  double t2376;
  double t2413;
  double t2949;
  double t1894;
  double t2789;
  double t3037;
  double t3409;
  double t4538;
  double t5144;
  double t5407;
  double t5555;
  double t169;
  double t1436;
  double t8317;
  double t8325;
  double t8327;
  double t8330;
  double t480;
  double t518;
  double t1347;
  double t1386;
  double t2187;
  double t2372;
  double t4566;
  double t4762;
  double t5119;
  double t5126;
  double t8374;
  double t8375;
  double t8376;
  double t8396;
  double t8402;
  double t8406;
  double t8319;
  double t8323;
  double t8416;
  double t8422;
  double t8423;
  double t8448;
  double t8449;
  double t8450;
  double t8472;
  double t8473;
  double t8474;
  double t8491;
  double t8492;
  double t8496;
  double t8505;
  double t8506;
  double t8507;
  double t8511;
  double t8513;
  double t8514;
  double t8521;
  double t8522;
  double t8525;
  double t8272;
  double t8533;
  double t8534;
  double t8547;
  double t8548;
  double t8549;
  double t8561;
  double t8562;
  double t8568;
  double t8569;
  double t8570;
  double t8572;
  double t8573;
  double t8538;
  double t8552;
  double t8553;
  double t8554;
  double t8555;
  double t8556;
  double t8557;
  double t8576;
  double t8577;
  t1459 = Sin(var1[3]);
  t2852 = Cos(var1[3]);
  t2376 = Cos(var1[5]);
  t2413 = Sin(var1[4]);
  t2949 = Sin(var1[5]);
  t1894 = Sin(var1[9]);
  t2789 = -1.*t2376*t1459*t2413;
  t3037 = t2852*t2949;
  t3409 = t2789 + t3037;
  t4538 = Cos(var1[9]);
  t5144 = -1.*t2852*t2376;
  t5407 = -1.*t1459*t2413*t2949;
  t5555 = t5144 + t5407;
  t169 = Cos(var1[10]);
  t1436 = Cos(var1[4]);
  t8317 = Sin(var1[10]);
  t8325 = t4538*t3409;
  t8327 = -1.*t1894*t5555;
  t8330 = t8325 + t8327;
  t480 = -1.*t169;
  t518 = 1. + t480;
  t1347 = 0.183*t518;
  t1386 = 0. + t1347;
  t2187 = -0.047*t1894;
  t2372 = 0. + t2187;
  t4566 = -1.*t4538;
  t4762 = 1. + t4566;
  t5119 = 0.047*t4762;
  t5126 = 0. + t5119;
  t8374 = t2852*t2376*t2413;
  t8375 = t1459*t2949;
  t8376 = t8374 + t8375;
  t8396 = -1.*t2376*t1459;
  t8402 = t2852*t2413*t2949;
  t8406 = t8396 + t8402;
  t8319 = 0.183*t8317;
  t8323 = 0. + t8319;
  t8416 = t4538*t8376;
  t8422 = -1.*t1894*t8406;
  t8423 = t8416 + t8422;
  t8448 = t4538*t2852*t1436*t2376;
  t8449 = -1.*t2852*t1436*t1894*t2949;
  t8450 = t8448 + t8449;
  t8472 = t4538*t1436*t2376*t1459;
  t8473 = -1.*t1436*t1894*t1459*t2949;
  t8474 = t8472 + t8473;
  t8491 = -1.*t4538*t2376*t2413;
  t8492 = t1894*t2413*t2949;
  t8496 = t8491 + t8492;
  t8505 = t2376*t1459;
  t8506 = -1.*t2852*t2413*t2949;
  t8507 = t8505 + t8506;
  t8511 = -1.*t1894*t8376;
  t8513 = t4538*t8507;
  t8514 = t8511 + t8513;
  t8521 = t2376*t1459*t2413;
  t8522 = -1.*t2852*t2949;
  t8525 = t8521 + t8522;
  t8272 = t4538*t5555;
  t8533 = -1.*t1894*t8525;
  t8534 = t8533 + t8272;
  t8547 = -1.*t1436*t2376*t1894;
  t8548 = -1.*t4538*t1436*t2949;
  t8549 = t8547 + t8548;
  t8561 = -1.*t4538*t8406;
  t8562 = t8511 + t8561;
  t8568 = t2852*t2376;
  t8569 = t1459*t2413*t2949;
  t8570 = t8568 + t8569;
  t8572 = -1.*t4538*t8570;
  t8573 = t8533 + t8572;
  t8538 = t4538*t8525;
  t8552 = t8323*t8549;
  t8553 = -0.183*t8317*t8549;
  t8554 = t4538*t1436*t2376;
  t8555 = -1.*t1436*t1894*t2949;
  t8556 = t8554 + t8555;
  t8557 = 0.13205*t8556;
  t8576 = -1.*t1894*t8570;
  t8577 = t8538 + t8576;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=-1.*t1386*t1436*t1459 + t2372*t3409 + t5126*t5555 + 0.13205*(t1894*t3409 + t8272) + t8323*t8330 + 0.183*(-1.*t1436*t1459*t169 - 1.*t8317*t8330);
  p_output1(10)=t1386*t1436*t2852 + t2372*t8376 + t5126*t8406 + 0.13205*(t1894*t8376 + t4538*t8406) + t8323*t8423 + 0.183*(t1436*t169*t2852 - 1.*t8317*t8423);
  p_output1(11)=0;
  p_output1(12)=t1436*t2372*t2376*t2852 - 1.*t1386*t2413*t2852 + 0.13205*(t1436*t1894*t2376*t2852 + t1436*t2852*t2949*t4538) + t1436*t2852*t2949*t5126 + t8323*t8450 + 0.183*(-1.*t169*t2413*t2852 - 1.*t8317*t8450);
  p_output1(13)=t1436*t1459*t2372*t2376 - 1.*t1386*t1459*t2413 + 0.13205*(t1436*t1459*t1894*t2376 + t1436*t1459*t2949*t4538) + t1436*t1459*t2949*t5126 + t8323*t8474 + 0.183*(-1.*t1459*t169*t2413 - 1.*t8317*t8474);
  p_output1(14)=-1.*t1386*t1436 - 1.*t2372*t2376*t2413 + 0.13205*(-1.*t1894*t2376*t2413 - 1.*t2413*t2949*t4538) - 1.*t2413*t2949*t5126 + t8323*t8496 + 0.183*(-1.*t1436*t169 - 1.*t8317*t8496);
  p_output1(15)=t5126*t8376 + t2372*t8507 + 0.13205*(t8416 + t1894*t8507) - 0.183*t8317*t8514 + t8323*t8514;
  p_output1(16)=t2372*t5555 + t5126*t8525 - 0.183*t8317*t8534 + t8323*t8534 + 0.13205*(t1894*t5555 + t8538);
  p_output1(17)=-1.*t1436*t2372*t2949 + t1436*t2376*t5126 + t8552 + t8553 + t8557;
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=-0.047*t4538*t8376 + 0.047*t1894*t8406 + 0.13205*t8423 - 0.183*t8317*t8562 + t8323*t8562;
  p_output1(28)=-0.047*t4538*t8525 + 0.047*t1894*t8570 - 0.183*t8317*t8573 + t8323*t8573 + 0.13205*t8577;
  p_output1(29)=0.047*t1436*t1894*t2949 - 0.047*t1436*t2376*t4538 + t8552 + t8553 + t8557;
  p_output1(30)=0.183*t1436*t2852*t8317 + 0.183*t169*t8423 + 0.183*(-1.*t1436*t2852*t8317 - 1.*t169*t8423);
  p_output1(31)=0.183*t1436*t1459*t8317 + 0.183*t169*t8577 + 0.183*(-1.*t1436*t1459*t8317 - 1.*t169*t8577);
  p_output1(32)=-0.183*t2413*t8317 + 0.183*t169*t8556 + 0.183*(t2413*t8317 - 1.*t169*t8556);
  p_output1(33)=0;
  p_output1(34)=0;
  p_output1(35)=0;
  p_output1(36)=0;
  p_output1(37)=0;
  p_output1(38)=0;
  p_output1(39)=0;
  p_output1(40)=0;
  p_output1(41)=0;
  p_output1(42)=0;
  p_output1(43)=0;
  p_output1(44)=0;
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
  p_output1(48)=0;
  p_output1(49)=0;
  p_output1(50)=0;
  p_output1(51)=0;
  p_output1(52)=0;
  p_output1(53)=0;
}


       
Eigen::Matrix<double,3,18> Jp_FL_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void Jp_FL_thigh_joint(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



