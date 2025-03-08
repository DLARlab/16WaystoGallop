/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:54 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FL_hip_joint.h"

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
  double t1832;
  double t8226;
  double t3658;
  double t8222;
  double t8230;
  double t2688;
  double t8224;
  double t8243;
  double t8245;
  double t8253;
  double t8259;
  double t8260;
  double t8261;
  double t1373;
  double t2727;
  double t3033;
  double t8254;
  double t8256;
  double t8257;
  double t8258;
  double t8274;
  double t8279;
  double t8280;
  double t8282;
  double t8283;
  double t8284;
  double t8360;
  double t8363;
  double t8364;
  double t8376;
  double t8377;
  double t8378;
  double t8368;
  double t8385;
  double t8419;
  double t8420;
  double t8421;
  double t8403;
  double t8404;
  double t8406;
  double t8407;
  t1832 = Sin(var1[3]);
  t8226 = Cos(var1[3]);
  t3658 = Cos(var1[5]);
  t8222 = Sin(var1[4]);
  t8230 = Sin(var1[5]);
  t2688 = Sin(var1[9]);
  t8224 = -1.*t3658*t1832*t8222;
  t8243 = t8226*t8230;
  t8245 = t8224 + t8243;
  t8253 = Cos(var1[9]);
  t8259 = -1.*t8226*t3658;
  t8260 = -1.*t1832*t8222*t8230;
  t8261 = t8259 + t8260;
  t1373 = Cos(var1[4]);
  t2727 = -0.047*t2688;
  t3033 = 0. + t2727;
  t8254 = -1.*t8253;
  t8256 = 1. + t8254;
  t8257 = 0.047*t8256;
  t8258 = 0. + t8257;
  t8274 = t8226*t3658*t8222;
  t8279 = t1832*t8230;
  t8280 = t8274 + t8279;
  t8282 = -1.*t3658*t1832;
  t8283 = t8226*t8222*t8230;
  t8284 = t8282 + t8283;
  t8360 = t3658*t1832;
  t8363 = -1.*t8226*t8222*t8230;
  t8364 = t8360 + t8363;
  t8376 = t3658*t1832*t8222;
  t8377 = -1.*t8226*t8230;
  t8378 = t8376 + t8377;
  t8368 = t8253*t8280;
  t8385 = t8253*t8378;
  t8419 = t8226*t3658;
  t8420 = t1832*t8222*t8230;
  t8421 = t8419 + t8420;
  t8403 = t8253*t1373*t3658;
  t8404 = -1.*t1373*t2688*t8230;
  t8406 = t8403 + t8404;
  t8407 = 0.047*t8406;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=-0.183*t1373*t1832 + t3033*t8245 + t8258*t8261 + 0.047*(t2688*t8245 + t8253*t8261);
  p_output1(10)=0.183*t1373*t8226 + t3033*t8280 + t8258*t8284 + 0.047*(t2688*t8280 + t8253*t8284);
  p_output1(11)=0;
  p_output1(12)=t1373*t3033*t3658*t8226 - 0.183*t8222*t8226 + 0.047*(t1373*t2688*t3658*t8226 + t1373*t8226*t8230*t8253) + t1373*t8226*t8230*t8258;
  p_output1(13)=t1373*t1832*t3033*t3658 - 0.183*t1832*t8222 + 0.047*(t1373*t1832*t2688*t3658 + t1373*t1832*t8230*t8253) + t1373*t1832*t8230*t8258;
  p_output1(14)=-0.183*t1373 - 1.*t3033*t3658*t8222 + 0.047*(-1.*t2688*t3658*t8222 - 1.*t8222*t8230*t8253) - 1.*t8222*t8230*t8258;
  p_output1(15)=t8258*t8280 + t3033*t8364 + 0.047*(t2688*t8364 + t8368);
  p_output1(16)=t3033*t8261 + t8258*t8378 + 0.047*(t2688*t8261 + t8385);
  p_output1(17)=-1.*t1373*t3033*t8230 + t1373*t3658*t8258 + t8407;
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=-0.047*t8253*t8280 + 0.047*t2688*t8284 + 0.047*(-1.*t2688*t8284 + t8368);
  p_output1(28)=-0.047*t8253*t8378 + 0.047*t2688*t8421 + 0.047*(t8385 - 1.*t2688*t8421);
  p_output1(29)=0.047*t1373*t2688*t8230 - 0.047*t1373*t3658*t8253 + t8407;
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
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


       
Eigen::Matrix<double,3,18> Jp_FL_hip_joint(const Eigen::Matrix<double,18,1> &var1)
//void Jp_FL_hip_joint(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



