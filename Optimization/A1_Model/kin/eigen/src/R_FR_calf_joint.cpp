/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:52 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_FR_calf_joint.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t641;
  double t3243;
  double t3891;
  double t3804;
  double t5816;
  double t803;
  double t2155;
  double t3889;
  double t7994;
  double t8003;
  double t8041;
  double t8096;
  double t8107;
  double t8114;
  double t8117;
  double t8134;
  double t8136;
  double t656;
  double t8140;
  double t523;
  double t8168;
  double t8169;
  double t8172;
  double t8176;
  double t8179;
  double t8180;
  double t8183;
  double t8187;
  double t8188;
  double t8159;
  double t8198;
  double t8199;
  double t8200;
  double t8148;
  double t8152;
  double t8156;
  double t810;
  double t8141;
  double t8144;
  double t8192;
  double t8193;
  double t8194;
  double t8165;
  double t8189;
  double t8190;
  double t8218;
  double t8219;
  double t8220;
  double t8197;
  double t8203;
  double t8204;
  t641 = Cos(var1[3]);
  t3243 = Cos(var1[5]);
  t3891 = Sin(var1[3]);
  t3804 = Sin(var1[4]);
  t5816 = Sin(var1[5]);
  t803 = Cos(var1[7]);
  t2155 = Cos(var1[6]);
  t3889 = t641*t3243*t3804;
  t7994 = t3891*t5816;
  t8003 = t3889 + t7994;
  t8041 = t2155*t8003;
  t8096 = -1.*t3243*t3891;
  t8107 = t641*t3804*t5816;
  t8114 = t8096 + t8107;
  t8117 = Sin(var1[6]);
  t8134 = -1.*t8114*t8117;
  t8136 = t8041 + t8134;
  t656 = Cos(var1[4]);
  t8140 = Sin(var1[7]);
  t523 = Cos(var1[8]);
  t8168 = t3243*t3891*t3804;
  t8169 = -1.*t641*t5816;
  t8172 = t8168 + t8169;
  t8176 = t2155*t8172;
  t8179 = t641*t3243;
  t8180 = t3891*t3804*t5816;
  t8183 = t8179 + t8180;
  t8187 = -1.*t8183*t8117;
  t8188 = t8176 + t8187;
  t8159 = Sin(var1[8]);
  t8198 = t656*t3243*t2155;
  t8199 = -1.*t656*t5816*t8117;
  t8200 = t8198 + t8199;
  t8148 = t803*t8136;
  t8152 = t641*t656*t8140;
  t8156 = t8148 + t8152;
  t810 = t641*t656*t803;
  t8141 = -1.*t8136*t8140;
  t8144 = t810 + t8141;
  t8192 = t803*t8188;
  t8193 = t656*t3891*t8140;
  t8194 = t8192 + t8193;
  t8165 = t656*t803*t3891;
  t8189 = -1.*t8188*t8140;
  t8190 = t8165 + t8189;
  t8218 = t803*t8200;
  t8219 = -1.*t3804*t8140;
  t8220 = t8218 + t8219;
  t8197 = -1.*t803*t3804;
  t8203 = -1.*t8200*t8140;
  t8204 = t8197 + t8203;

  p_output1(0)=t523*t8144 - 1.*t8156*t8159;
  p_output1(1)=t523*t8190 - 1.*t8159*t8194;
  p_output1(2)=t523*t8204 - 1.*t8159*t8220;
  p_output1(3)=t2155*t8114 + t8003*t8117;
  p_output1(4)=t8117*t8172 + t2155*t8183;
  p_output1(5)=t2155*t5816*t656 + t3243*t656*t8117;
  p_output1(6)=t523*t8156 + t8144*t8159;
  p_output1(7)=t8159*t8190 + t523*t8194;
  p_output1(8)=t8159*t8204 + t523*t8220;
}


       
Eigen::Matrix<double,3,3> R_FR_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void R_FR_calf_joint(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



