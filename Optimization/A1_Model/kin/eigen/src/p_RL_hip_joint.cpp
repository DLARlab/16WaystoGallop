/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:15 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RL_hip_joint.h"

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
  double t210;
  double t2433;
  double t3261;
  double t2524;
  double t3262;
  double t1645;
  double t3168;
  double t4534;
  double t4543;
  double t4758;
  double t4953;
  double t4954;
  double t4998;
  double t809;
  double t1661;
  double t1914;
  double t4765;
  double t4900;
  double t4933;
  double t4939;
  double t5237;
  double t5247;
  double t5425;
  double t5508;
  double t5553;
  double t5617;
  t210 = Cos(var1[3]);
  t2433 = Cos(var1[5]);
  t3261 = Sin(var1[3]);
  t2524 = Sin(var1[4]);
  t3262 = Sin(var1[5]);
  t1645 = Sin(var1[15]);
  t3168 = t210*t2433*t2524;
  t4534 = t3261*t3262;
  t4543 = t3168 + t4534;
  t4758 = Cos(var1[15]);
  t4953 = -1.*t2433*t3261;
  t4954 = t210*t2524*t3262;
  t4998 = t4953 + t4954;
  t809 = Cos(var1[4]);
  t1661 = -0.047*t1645;
  t1914 = 0. + t1661;
  t4765 = -1.*t4758;
  t4900 = 1. + t4765;
  t4933 = 0.047*t4900;
  t4939 = 0. + t4933;
  t5237 = t2433*t3261*t2524;
  t5247 = -1.*t210*t3262;
  t5425 = t5237 + t5247;
  t5508 = t210*t2433;
  t5553 = t3261*t2524*t3262;
  t5617 = t5508 + t5553;

  p_output1(0)=0. + t1914*t4543 + t4939*t4998 + 0.047*(t1645*t4543 + t4758*t4998) - 0.183*t210*t809 + var1(0);
  p_output1(1)=0. + t1914*t5425 + t4939*t5617 + 0.047*(t1645*t5425 + t4758*t5617) - 0.183*t3261*t809 + var1(1);
  p_output1(2)=0. + 0.183*t2524 + t1914*t2433*t809 + t3262*t4939*t809 + 0.047*(t1645*t2433*t809 + t3262*t4758*t809) + var1(2);
}


       
Eigen::Matrix<double,3,1> p_RL_hip_joint(const Eigen::Matrix<double,18,1> &var1)
//void p_RL_hip_joint(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



