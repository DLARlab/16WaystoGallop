/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:53 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FL_hip_joint.h"

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
  double t1354;
  double t3620;
  double t3781;
  double t3768;
  double t3951;
  double t1373;
  double t3779;
  double t4214;
  double t5044;
  double t8164;
  double t8225;
  double t8226;
  double t8227;
  double t1364;
  double t2727;
  double t2728;
  double t8191;
  double t8221;
  double t8222;
  double t8224;
  double t8245;
  double t8246;
  double t8249;
  double t8254;
  double t8256;
  double t8257;
  t1354 = Cos(var1[3]);
  t3620 = Cos(var1[5]);
  t3781 = Sin(var1[3]);
  t3768 = Sin(var1[4]);
  t3951 = Sin(var1[5]);
  t1373 = Sin(var1[9]);
  t3779 = t1354*t3620*t3768;
  t4214 = t3781*t3951;
  t5044 = t3779 + t4214;
  t8164 = Cos(var1[9]);
  t8225 = -1.*t3620*t3781;
  t8226 = t1354*t3768*t3951;
  t8227 = t8225 + t8226;
  t1364 = Cos(var1[4]);
  t2727 = -0.047*t1373;
  t2728 = 0. + t2727;
  t8191 = -1.*t8164;
  t8221 = 1. + t8191;
  t8222 = 0.047*t8221;
  t8224 = 0. + t8222;
  t8245 = t3620*t3781*t3768;
  t8246 = -1.*t1354*t3951;
  t8249 = t8245 + t8246;
  t8254 = t1354*t3620;
  t8256 = t3781*t3768*t3951;
  t8257 = t8254 + t8256;

  p_output1(0)=0. + 0.183*t1354*t1364 + t2728*t5044 + t8224*t8227 + 0.047*(t1373*t5044 + t8164*t8227) + var1(0);
  p_output1(1)=0. + 0.183*t1364*t3781 + t2728*t8249 + t8224*t8257 + 0.047*(t1373*t8249 + t8164*t8257) + var1(1);
  p_output1(2)=0. + t1364*t2728*t3620 - 0.183*t3768 + 0.047*(t1364*t1373*t3620 + t1364*t3951*t8164) + t1364*t3951*t8224 + var1(2);
}


       
Eigen::Matrix<double,3,1> p_FL_hip_joint(const Eigen::Matrix<double,18,1> &var1)
//void p_FL_hip_joint(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



