/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:14 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FR_hip_joint_src.h"

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
  double t109;
  double t129;
  double t132;
  double t128;
  double t133;
  double t119;
  double t130;
  double t134;
  double t136;
  double t139;
  double t140;
  double t141;
  double t142;
  double t101;
  double t121;
  double t123;
  double t125;
  double t127;
  double t143;
  double t144;
  double t152;
  double t153;
  double t154;
  double t156;
  double t157;
  double t158;
  double t194;
  double t196;
  double t198;
  double t205;
  double t206;
  double t207;
  double t200;
  double t211;
  double t230;
  double t231;
  double t232;
  double t218;
  double t219;
  double t220;
  double t221;
  t109 = Sin(var1[3]);
  t129 = Cos(var1[5]);
  t132 = Sin(var1[4]);
  t128 = Cos(var1[3]);
  t133 = Sin(var1[5]);
  t119 = Cos(var1[6]);
  t130 = -1.*t128*t129;
  t134 = -1.*t109*t132*t133;
  t136 = t130 + t134;
  t139 = -1.*t129*t109*t132;
  t140 = t128*t133;
  t141 = t139 + t140;
  t142 = Sin(var1[6]);
  t101 = Cos(var1[4]);
  t121 = -1.*t119;
  t123 = 1. + t121;
  t125 = -0.047*t123;
  t127 = 0. + t125;
  t143 = 0.047*t142;
  t144 = 0. + t143;
  t152 = -1.*t129*t109;
  t153 = t128*t132*t133;
  t154 = t152 + t153;
  t156 = t128*t129*t132;
  t157 = t109*t133;
  t158 = t156 + t157;
  t194 = t129*t109;
  t196 = -1.*t128*t132*t133;
  t198 = t194 + t196;
  t205 = t129*t109*t132;
  t206 = -1.*t128*t133;
  t207 = t205 + t206;
  t200 = t119*t158;
  t211 = t119*t207;
  t230 = t128*t129;
  t231 = t109*t132*t133;
  t232 = t230 + t231;
  t218 = t101*t129*t119;
  t219 = -1.*t101*t133*t142;
  t220 = t218 + t219;
  t221 = -0.047*t220;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=-0.183*t101*t109 + t127*t136 - 0.047*(t119*t136 + t141*t142) + t141*t144;
  p_output1[10]=0.183*t101*t128 + t127*t154 + t144*t158 - 0.047*(t119*t154 + t142*t158);
  p_output1[11]=0;
  p_output1[12]=-0.183*t128*t132 + t101*t127*t128*t133 - 0.047*(t101*t119*t128*t133 + t101*t128*t129*t142) + t101*t128*t129*t144;
  p_output1[13]=-0.183*t109*t132 + t101*t109*t127*t133 - 0.047*(t101*t109*t119*t133 + t101*t109*t129*t142) + t101*t109*t129*t144;
  p_output1[14]=-0.183*t101 - 1.*t127*t132*t133 - 0.047*(-1.*t119*t132*t133 - 1.*t129*t132*t142) - 1.*t129*t132*t144;
  p_output1[15]=t127*t158 + t144*t198 - 0.047*(t142*t198 + t200);
  p_output1[16]=t136*t144 + t127*t207 - 0.047*(t136*t142 + t211);
  p_output1[17]=t101*t127*t129 - 1.*t101*t133*t144 + t221;
  p_output1[18]=-0.047*t142*t154 + 0.047*t119*t158 - 0.047*(-1.*t142*t154 + t200);
  p_output1[19]=0.047*t119*t207 - 0.047*t142*t232 - 0.047*(t211 - 1.*t142*t232);
  p_output1[20]=0.047*t101*t119*t129 - 0.047*t101*t133*t142 + t221;
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
  p_output1[45]=0;
  p_output1[46]=0;
  p_output1[47]=0;
  p_output1[48]=0;
  p_output1[49]=0;
  p_output1[50]=0;
  p_output1[51]=0;
  p_output1[52]=0;
  p_output1[53]=0;
}



void Jp_FR_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
