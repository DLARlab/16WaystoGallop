/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:04 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RR_hip_joint.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t1023;
  double t3567;
  double t4586;
  double t3706;
  double t5404;
  double t2139;
  double t4515;
  double t5636;
  double t7061;
  double t8492;
  double t8519;
  double t8520;
  double t8532;
  double t1927;
  double t2143;
  double t2644;
  double t8505;
  double t8510;
  double t8514;
  double t8516;
  double t8603;
  double t8604;
  double t8605;
  double t8608;
  double t8609;
  double t8611;
  t1023 = Cos(var1[3]);
  t3567 = Cos(var1[5]);
  t4586 = Sin(var1[3]);
  t3706 = Sin(var1[4]);
  t5404 = Sin(var1[5]);
  t2139 = Sin(var1[12]);
  t4515 = t1023*t3567*t3706;
  t5636 = t4586*t5404;
  t7061 = t4515 + t5636;
  t8492 = Cos(var1[12]);
  t8519 = -1.*t3567*t4586;
  t8520 = t1023*t3706*t5404;
  t8532 = t8519 + t8520;
  t1927 = Cos(var1[4]);
  t2143 = 0.047*t2139;
  t2644 = 0. + t2143;
  t8505 = -1.*t8492;
  t8510 = 1. + t8505;
  t8514 = -0.047*t8510;
  t8516 = 0. + t8514;
  t8603 = t3567*t4586*t3706;
  t8604 = -1.*t1023*t5404;
  t8605 = t8603 + t8604;
  t8608 = t1023*t3567;
  t8609 = t4586*t3706*t5404;
  t8611 = t8608 + t8609;

  p_output1(0)=0. - 0.183*t1023*t1927 + t2644*t7061 + t8516*t8532 - 0.047*(t2139*t7061 + t8492*t8532) + var1(0);
  p_output1(1)=0. - 0.183*t1927*t4586 + t2644*t8605 + t8516*t8611 - 0.047*(t2139*t8605 + t8492*t8611) + var1(1);
  p_output1(2)=0. + t1927*t2644*t3567 + 0.183*t3706 - 0.047*(t1927*t2139*t3567 + t1927*t5404*t8492) + t1927*t5404*t8516 + var1(2);
}


       
Eigen::Matrix<double,3,1> p_RR_hip_joint(const Eigen::Matrix<double,18,1> &var1)
//void p_RR_hip_joint(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



