/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:49 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RL_hip_joint_src.h"

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
  double t4085;
  double t4125;
  double t4121;
  double t4123;
  double t4126;
  double t4111;
  double t4124;
  double t4127;
  double t4128;
  double t4130;
  double t4137;
  double t4138;
  double t4139;
  double t4070;
  double t4112;
  double t4117;
  double t4131;
  double t4132;
  double t4133;
  double t4135;
  double t4147;
  double t4148;
  double t4149;
  double t4151;
  double t4152;
  double t4153;
  double t4189;
  double t4191;
  double t4193;
  double t4200;
  double t4201;
  double t4202;
  double t4195;
  double t4206;
  double t4225;
  double t4226;
  double t4227;
  double t4213;
  double t4214;
  double t4215;
  double t4216;
  t4085 = Sin(var1[3]);
  t4125 = Cos(var1[3]);
  t4121 = Cos(var1[5]);
  t4123 = Sin(var1[4]);
  t4126 = Sin(var1[5]);
  t4111 = Sin(var1[15]);
  t4124 = -1.*t4121*t4085*t4123;
  t4127 = t4125*t4126;
  t4128 = t4124 + t4127;
  t4130 = Cos(var1[15]);
  t4137 = -1.*t4125*t4121;
  t4138 = -1.*t4085*t4123*t4126;
  t4139 = t4137 + t4138;
  t4070 = Cos(var1[4]);
  t4112 = -0.047*t4111;
  t4117 = 0. + t4112;
  t4131 = -1.*t4130;
  t4132 = 1. + t4131;
  t4133 = 0.047*t4132;
  t4135 = 0. + t4133;
  t4147 = t4125*t4121*t4123;
  t4148 = t4085*t4126;
  t4149 = t4147 + t4148;
  t4151 = -1.*t4121*t4085;
  t4152 = t4125*t4123*t4126;
  t4153 = t4151 + t4152;
  t4189 = t4121*t4085;
  t4191 = -1.*t4125*t4123*t4126;
  t4193 = t4189 + t4191;
  t4200 = t4121*t4085*t4123;
  t4201 = -1.*t4125*t4126;
  t4202 = t4200 + t4201;
  t4195 = t4130*t4149;
  t4206 = t4130*t4202;
  t4225 = t4125*t4121;
  t4226 = t4085*t4123*t4126;
  t4227 = t4225 + t4226;
  t4213 = t4130*t4070*t4121;
  t4214 = -1.*t4070*t4111*t4126;
  t4215 = t4213 + t4214;
  t4216 = 0.047*t4215;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=0.183*t4070*t4085 + t4117*t4128 + t4135*t4139 + 0.047*(t4111*t4128 + t4130*t4139);
  p_output1[10]=-0.183*t4070*t4125 + t4117*t4149 + t4135*t4153 + 0.047*(t4111*t4149 + t4130*t4153);
  p_output1[11]=0;
  p_output1[12]=t4070*t4117*t4121*t4125 + 0.183*t4123*t4125 + 0.047*(t4070*t4111*t4121*t4125 + t4070*t4125*t4126*t4130) + t4070*t4125*t4126*t4135;
  p_output1[13]=t4070*t4085*t4117*t4121 + 0.183*t4085*t4123 + 0.047*(t4070*t4085*t4111*t4121 + t4070*t4085*t4126*t4130) + t4070*t4085*t4126*t4135;
  p_output1[14]=0.183*t4070 - 1.*t4117*t4121*t4123 + 0.047*(-1.*t4111*t4121*t4123 - 1.*t4123*t4126*t4130) - 1.*t4123*t4126*t4135;
  p_output1[15]=t4135*t4149 + t4117*t4193 + 0.047*(t4111*t4193 + t4195);
  p_output1[16]=t4117*t4139 + t4135*t4202 + 0.047*(t4111*t4139 + t4206);
  p_output1[17]=-1.*t4070*t4117*t4126 + t4070*t4121*t4135 + t4216;
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
  p_output1[45]=-0.047*t4130*t4149 + 0.047*t4111*t4153 + 0.047*(-1.*t4111*t4153 + t4195);
  p_output1[46]=-0.047*t4130*t4202 + 0.047*t4111*t4227 + 0.047*(t4206 - 1.*t4111*t4227);
  p_output1[47]=0.047*t4070*t4111*t4126 - 0.047*t4070*t4121*t4130 + t4216;
  p_output1[48]=0;
  p_output1[49]=0;
  p_output1[50]=0;
  p_output1[51]=0;
  p_output1[52]=0;
  p_output1[53]=0;
}



void Jp_RL_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
