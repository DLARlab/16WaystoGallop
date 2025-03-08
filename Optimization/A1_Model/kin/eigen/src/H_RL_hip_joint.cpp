/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:17 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RL_hip_joint.h"

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
static void output1(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t1902;
  double t1184;
  double t4340;
  double t3087;
  double t4998;
  double t5635;
  double t4924;
  double t6098;
  double t5504;
  double t5865;
  double t5871;
  double t6105;
  double t6205;
  double t6208;
  double t6248;
  double t6298;
  double t6305;
  double t6434;
  double t6447;
  double t6484;
  double t6060;
  double t6213;
  double t6236;
  double t6972;
  double t6974;
  double t6983;
  double t6990;
  double t7002;
  double t7147;
  double t6432;
  double t6552;
  double t6611;
  double t6791;
  double t6800;
  double t6804;
  t1902 = Cos(var1[4]);
  t1184 = Cos(var1[3]);
  t4340 = Sin(var1[4]);
  t3087 = Sin(var1[3]);
  t4998 = Cos(var1[5]);
  t5635 = Sin(var1[5]);
  t4924 = Sin(var1[15]);
  t6098 = Cos(var1[15]);
  t5504 = t1184*t4998*t4340;
  t5865 = t3087*t5635;
  t5871 = t5504 + t5865;
  t6105 = -1.*t4998*t3087;
  t6205 = t1184*t4340*t5635;
  t6208 = t6105 + t6205;
  t6248 = t4998*t3087*t4340;
  t6298 = -1.*t1184*t5635;
  t6305 = t6248 + t6298;
  t6434 = t1184*t4998;
  t6447 = t3087*t4340*t5635;
  t6484 = t6434 + t6447;
  t6060 = t4924*t5871;
  t6213 = t6098*t6208;
  t6236 = t6060 + t6213;
  t6972 = -0.047*t4924;
  t6974 = 0. + t6972;
  t6983 = -1.*t6098;
  t6990 = 1. + t6983;
  t7002 = 0.047*t6990;
  t7147 = 0. + t7002;
  t6432 = t4924*t6305;
  t6552 = t6098*t6484;
  t6611 = t6432 + t6552;
  t6791 = t1902*t4998*t4924;
  t6800 = t6098*t1902*t5635;
  t6804 = t6791 + t6800;

  p_output1(0)=t1184*t1902;
  p_output1(1)=t1902*t3087;
  p_output1(2)=-1.*t4340;
  p_output1(3)=0.;
  p_output1(4)=t6236;
  p_output1(5)=t6611;
  p_output1(6)=t6804;
  p_output1(7)=0.;
  p_output1(8)=t5871*t6098 - 1.*t4924*t6208;
  p_output1(9)=t6098*t6305 - 1.*t4924*t6484;
  p_output1(10)=-1.*t1902*t4924*t5635 + t1902*t4998*t6098;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.183*t1184*t1902 + 0.047*t6236 + t5871*t6974 + t6208*t7147 + var1(0);
  p_output1(13)=0. - 0.183*t1902*t3087 + 0.047*t6611 + t6305*t6974 + t6484*t7147 + var1(1);
  p_output1(14)=0. + 0.183*t4340 + 0.047*t6804 + t1902*t4998*t6974 + t1902*t5635*t7147 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_RL_hip_joint(const Eigen::Matrix<double,18,1> &var1)
//void H_RL_hip_joint(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



