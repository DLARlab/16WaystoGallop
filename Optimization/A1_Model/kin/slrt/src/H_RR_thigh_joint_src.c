/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:42 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RR_thigh_joint_src.h"

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
  double t3109;
  double t3129;
  double t3139;
  double t3131;
  double t3142;
  double t3092;
  double t3112;
  double t3119;
  double t3124;
  double t3159;
  double t3135;
  double t3143;
  double t3145;
  double t3163;
  double t3164;
  double t3165;
  double t3183;
  double t3184;
  double t3185;
  double t3187;
  double t3189;
  double t3191;
  double t3156;
  double t3166;
  double t3167;
  double t3186;
  double t3194;
  double t3195;
  double t3208;
  double t3210;
  double t3211;
  double t3216;
  double t3217;
  double t3218;
  double t3118;
  double t3168;
  double t3174;
  double t3250;
  double t3251;
  double t3252;
  double t3253;
  double t3255;
  double t3258;
  double t3261;
  double t3262;
  double t3263;
  double t3264;
  double t3219;
  double t3225;
  double t3230;
  double t3267;
  double t3268;
  double t3182;
  double t3198;
  double t3199;
  double t3231;
  double t3233;
  double t3234;
  double t3204;
  double t3212;
  double t3215;
  t3109 = Cos(var1[3]);
  t3129 = Cos(var1[5]);
  t3139 = Sin(var1[3]);
  t3131 = Sin(var1[4]);
  t3142 = Sin(var1[5]);
  t3092 = Cos(var1[13]);
  t3112 = Cos(var1[4]);
  t3119 = Sin(var1[13]);
  t3124 = Cos(var1[12]);
  t3159 = Sin(var1[12]);
  t3135 = t3109*t3129*t3131;
  t3143 = t3139*t3142;
  t3145 = t3135 + t3143;
  t3163 = -1.*t3129*t3139;
  t3164 = t3109*t3131*t3142;
  t3165 = t3163 + t3164;
  t3183 = t3129*t3139*t3131;
  t3184 = -1.*t3109*t3142;
  t3185 = t3183 + t3184;
  t3187 = t3109*t3129;
  t3189 = t3139*t3131*t3142;
  t3191 = t3187 + t3189;
  t3156 = t3124*t3145;
  t3166 = -1.*t3159*t3165;
  t3167 = t3156 + t3166;
  t3186 = t3124*t3185;
  t3194 = -1.*t3159*t3191;
  t3195 = t3186 + t3194;
  t3208 = t3124*t3112*t3129;
  t3210 = -1.*t3112*t3159*t3142;
  t3211 = t3208 + t3210;
  t3216 = t3159*t3145;
  t3217 = t3124*t3165;
  t3218 = t3216 + t3217;
  t3118 = t3092*t3109*t3112;
  t3168 = -1.*t3119*t3167;
  t3174 = t3118 + t3168;
  t3250 = -1.*t3092;
  t3251 = 1. + t3250;
  t3252 = -0.183*t3251;
  t3253 = 0. + t3252;
  t3255 = 0.047*t3159;
  t3258 = 0. + t3255;
  t3261 = -1.*t3124;
  t3262 = 1. + t3261;
  t3263 = -0.047*t3262;
  t3264 = 0. + t3263;
  t3219 = t3159*t3185;
  t3225 = t3124*t3191;
  t3230 = t3219 + t3225;
  t3267 = -0.183*t3119;
  t3268 = 0. + t3267;
  t3182 = t3092*t3112*t3139;
  t3198 = -1.*t3119*t3195;
  t3199 = t3182 + t3198;
  t3231 = t3112*t3129*t3159;
  t3233 = t3124*t3112*t3142;
  t3234 = t3231 + t3233;
  t3204 = -1.*t3092*t3131;
  t3212 = -1.*t3119*t3211;
  t3215 = t3204 + t3212;
  p_output1[0]=t3174;
  p_output1[1]=t3199;
  p_output1[2]=t3215;
  p_output1[3]=0.;
  p_output1[4]=t3218;
  p_output1[5]=t3230;
  p_output1[6]=t3234;
  p_output1[7]=0.;
  p_output1[8]=t3109*t3112*t3119 + t3092*t3167;
  p_output1[9]=t3112*t3119*t3139 + t3092*t3195;
  p_output1[10]=-1.*t3119*t3131 + t3092*t3211;
  p_output1[11]=0.;
  p_output1[12]=0. - 0.183*t3174 - 0.13205*t3218 + t3109*t3112*t3253 + t3145*t3258 + t3165*t3264 + t3167*t3268 + var1[0];
  p_output1[13]=0. - 0.183*t3199 - 0.13205*t3230 + t3112*t3139*t3253 + t3185*t3258 + t3191*t3264 + t3195*t3268 + var1[1];
  p_output1[14]=0. - 0.183*t3215 - 0.13205*t3234 - 1.*t3131*t3253 + t3112*t3129*t3258 + t3112*t3142*t3264 + t3211*t3268 + var1[2];
  p_output1[15]=1.;
}



void H_RR_thigh_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
