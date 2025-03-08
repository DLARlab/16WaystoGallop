/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:50 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RL_hip_joint_src.h"

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
  double t4146;
  double t4144;
  double t4163;
  double t4159;
  double t4173;
  double t4177;
  double t4170;
  double t4184;
  double t4174;
  double t4179;
  double t4180;
  double t4185;
  double t4186;
  double t4187;
  double t4193;
  double t4194;
  double t4196;
  double t4198;
  double t4199;
  double t4200;
  double t4183;
  double t4188;
  double t4189;
  double t4226;
  double t4228;
  double t4231;
  double t4232;
  double t4233;
  double t4234;
  double t4197;
  double t4202;
  double t4204;
  double t4205;
  double t4207;
  double t4210;
  t4146 = Cos(var1[4]);
  t4144 = Cos(var1[3]);
  t4163 = Sin(var1[4]);
  t4159 = Sin(var1[3]);
  t4173 = Cos(var1[5]);
  t4177 = Sin(var1[5]);
  t4170 = Sin(var1[15]);
  t4184 = Cos(var1[15]);
  t4174 = t4144*t4173*t4163;
  t4179 = t4159*t4177;
  t4180 = t4174 + t4179;
  t4185 = -1.*t4173*t4159;
  t4186 = t4144*t4163*t4177;
  t4187 = t4185 + t4186;
  t4193 = t4173*t4159*t4163;
  t4194 = -1.*t4144*t4177;
  t4196 = t4193 + t4194;
  t4198 = t4144*t4173;
  t4199 = t4159*t4163*t4177;
  t4200 = t4198 + t4199;
  t4183 = t4170*t4180;
  t4188 = t4184*t4187;
  t4189 = t4183 + t4188;
  t4226 = -0.047*t4170;
  t4228 = 0. + t4226;
  t4231 = -1.*t4184;
  t4232 = 1. + t4231;
  t4233 = 0.047*t4232;
  t4234 = 0. + t4233;
  t4197 = t4170*t4196;
  t4202 = t4184*t4200;
  t4204 = t4197 + t4202;
  t4205 = t4146*t4173*t4170;
  t4207 = t4184*t4146*t4177;
  t4210 = t4205 + t4207;
  p_output1[0]=t4144*t4146;
  p_output1[1]=t4146*t4159;
  p_output1[2]=-1.*t4163;
  p_output1[3]=0.;
  p_output1[4]=t4189;
  p_output1[5]=t4204;
  p_output1[6]=t4210;
  p_output1[7]=0.;
  p_output1[8]=t4180*t4184 - 1.*t4170*t4187;
  p_output1[9]=t4184*t4196 - 1.*t4170*t4200;
  p_output1[10]=-1.*t4146*t4170*t4177 + t4146*t4173*t4184;
  p_output1[11]=0.;
  p_output1[12]=0. - 0.183*t4144*t4146 + 0.047*t4189 + t4180*t4228 + t4187*t4234 + var1[0];
  p_output1[13]=0. - 0.183*t4146*t4159 + 0.047*t4204 + t4196*t4228 + t4200*t4234 + var1[1];
  p_output1[14]=0. + 0.183*t4163 + 0.047*t4210 + t4146*t4173*t4228 + t4146*t4177*t4234 + var1[2];
  p_output1[15]=1.;
}



void H_RL_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
