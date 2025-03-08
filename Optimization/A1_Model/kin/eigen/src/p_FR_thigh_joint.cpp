/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:45 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FR_thigh_joint.h"

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
  double t477;
  double t7880;
  double t7885;
  double t7882;
  double t7888;
  double t3944;
  double t7884;
  double t7890;
  double t7891;
  double t7893;
  double t7894;
  double t7895;
  double t7897;
  double t938;
  double t949;
  double t7909;
  double t7910;
  double t7911;
  double t7912;
  double t985;
  double t2064;
  double t2743;
  double t2748;
  double t7847;
  double t7867;
  double t7868;
  double t7877;
  double t7902;
  double t7903;
  double t7926;
  double t7927;
  double t7928;
  double t7930;
  double t7931;
  double t7932;
  double t7914;
  double t7915;
  double t7938;
  double t7939;
  double t7942;
  double t7956;
  double t7957;
  double t7958;
  t477 = Cos(var1[3]);
  t7880 = Cos(var1[5]);
  t7885 = Sin(var1[4]);
  t7882 = Sin(var1[3]);
  t7888 = Sin(var1[5]);
  t3944 = Cos(var1[6]);
  t7884 = -1.*t7880*t7882;
  t7890 = t477*t7885*t7888;
  t7891 = t7884 + t7890;
  t7893 = t477*t7880*t7885;
  t7894 = t7882*t7888;
  t7895 = t7893 + t7894;
  t7897 = Sin(var1[6]);
  t938 = Cos(var1[4]);
  t949 = Cos(var1[7]);
  t7909 = t3944*t7895;
  t7910 = -1.*t7891*t7897;
  t7911 = t7909 + t7910;
  t7912 = Sin(var1[7]);
  t985 = -1.*t949;
  t2064 = 1. + t985;
  t2743 = 0.183*t2064;
  t2748 = 0. + t2743;
  t7847 = -1.*t3944;
  t7867 = 1. + t7847;
  t7868 = -0.047*t7867;
  t7877 = 0. + t7868;
  t7902 = 0.047*t7897;
  t7903 = 0. + t7902;
  t7926 = t477*t7880;
  t7927 = t7882*t7885*t7888;
  t7928 = t7926 + t7927;
  t7930 = t7880*t7882*t7885;
  t7931 = -1.*t477*t7888;
  t7932 = t7930 + t7931;
  t7914 = 0.183*t7912;
  t7915 = 0. + t7914;
  t7938 = t3944*t7932;
  t7939 = -1.*t7928*t7897;
  t7942 = t7938 + t7939;
  t7956 = t938*t7880*t3944;
  t7957 = -1.*t938*t7888*t7897;
  t7958 = t7956 + t7957;

  p_output1(0)=0. + t7877*t7891 - 0.13205*(t3944*t7891 + t7895*t7897) + t7895*t7903 + t7911*t7915 + t2748*t477*t938 + 0.183*(-1.*t7911*t7912 + t477*t938*t949) + var1(0);
  p_output1(1)=0. + t7877*t7928 + t7903*t7932 - 0.13205*(t3944*t7928 + t7897*t7932) + t7915*t7942 + t2748*t7882*t938 + 0.183*(-1.*t7912*t7942 + t7882*t938*t949) + var1(1);
  p_output1(2)=0. - 1.*t2748*t7885 + t7915*t7958 + t7877*t7888*t938 + t7880*t7903*t938 - 0.13205*(t3944*t7888*t938 + t7880*t7897*t938) + 0.183*(-1.*t7912*t7958 - 1.*t7885*t949) + var1(2);
}


       
Eigen::Matrix<double,3,1> p_FR_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void p_FR_thigh_joint(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



