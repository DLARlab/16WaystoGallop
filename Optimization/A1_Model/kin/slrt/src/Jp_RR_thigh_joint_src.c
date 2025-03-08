/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:41 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RR_thigh_joint_src.h"

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
  double t3039;
  double t3056;
  double t3052;
  double t3054;
  double t3061;
  double t3046;
  double t3055;
  double t3062;
  double t3063;
  double t3065;
  double t3071;
  double t3073;
  double t3077;
  double t2994;
  double t3031;
  double t3085;
  double t3088;
  double t3089;
  double t3090;
  double t3006;
  double t3015;
  double t3029;
  double t3030;
  double t3047;
  double t3048;
  double t3066;
  double t3067;
  double t3068;
  double t3069;
  double t3098;
  double t3101;
  double t3102;
  double t3104;
  double t3105;
  double t3106;
  double t3086;
  double t3087;
  double t3112;
  double t3113;
  double t3114;
  double t3131;
  double t3133;
  double t3134;
  double t3150;
  double t3151;
  double t3152;
  double t3171;
  double t3174;
  double t3178;
  double t3187;
  double t3188;
  double t3189;
  double t3193;
  double t3194;
  double t3195;
  double t3204;
  double t3205;
  double t3206;
  double t3080;
  double t3209;
  double t3210;
  double t3220;
  double t3221;
  double t3222;
  double t3232;
  double t3233;
  double t3239;
  double t3240;
  double t3241;
  double t3243;
  double t3244;
  double t3213;
  double t3223;
  double t3224;
  double t3225;
  double t3226;
  double t3227;
  double t3228;
  double t3247;
  double t3248;
  t3039 = Sin(var1[3]);
  t3056 = Cos(var1[3]);
  t3052 = Cos(var1[5]);
  t3054 = Sin(var1[4]);
  t3061 = Sin(var1[5]);
  t3046 = Sin(var1[12]);
  t3055 = -1.*t3052*t3039*t3054;
  t3062 = t3056*t3061;
  t3063 = t3055 + t3062;
  t3065 = Cos(var1[12]);
  t3071 = -1.*t3056*t3052;
  t3073 = -1.*t3039*t3054*t3061;
  t3077 = t3071 + t3073;
  t2994 = Cos(var1[13]);
  t3031 = Cos(var1[4]);
  t3085 = Sin(var1[13]);
  t3088 = t3065*t3063;
  t3089 = -1.*t3046*t3077;
  t3090 = t3088 + t3089;
  t3006 = -1.*t2994;
  t3015 = 1. + t3006;
  t3029 = -0.183*t3015;
  t3030 = 0. + t3029;
  t3047 = 0.047*t3046;
  t3048 = 0. + t3047;
  t3066 = -1.*t3065;
  t3067 = 1. + t3066;
  t3068 = -0.047*t3067;
  t3069 = 0. + t3068;
  t3098 = t3056*t3052*t3054;
  t3101 = t3039*t3061;
  t3102 = t3098 + t3101;
  t3104 = -1.*t3052*t3039;
  t3105 = t3056*t3054*t3061;
  t3106 = t3104 + t3105;
  t3086 = -0.183*t3085;
  t3087 = 0. + t3086;
  t3112 = t3065*t3102;
  t3113 = -1.*t3046*t3106;
  t3114 = t3112 + t3113;
  t3131 = t3065*t3056*t3031*t3052;
  t3133 = -1.*t3056*t3031*t3046*t3061;
  t3134 = t3131 + t3133;
  t3150 = t3065*t3031*t3052*t3039;
  t3151 = -1.*t3031*t3046*t3039*t3061;
  t3152 = t3150 + t3151;
  t3171 = -1.*t3065*t3052*t3054;
  t3174 = t3046*t3054*t3061;
  t3178 = t3171 + t3174;
  t3187 = t3052*t3039;
  t3188 = -1.*t3056*t3054*t3061;
  t3189 = t3187 + t3188;
  t3193 = -1.*t3046*t3102;
  t3194 = t3065*t3189;
  t3195 = t3193 + t3194;
  t3204 = t3052*t3039*t3054;
  t3205 = -1.*t3056*t3061;
  t3206 = t3204 + t3205;
  t3080 = t3065*t3077;
  t3209 = -1.*t3046*t3206;
  t3210 = t3209 + t3080;
  t3220 = -1.*t3031*t3052*t3046;
  t3221 = -1.*t3065*t3031*t3061;
  t3222 = t3220 + t3221;
  t3232 = -1.*t3065*t3106;
  t3233 = t3193 + t3232;
  t3239 = t3056*t3052;
  t3240 = t3039*t3054*t3061;
  t3241 = t3239 + t3240;
  t3243 = -1.*t3065*t3241;
  t3244 = t3209 + t3243;
  t3213 = t3065*t3206;
  t3223 = t3087*t3222;
  t3224 = 0.183*t3085*t3222;
  t3225 = t3065*t3031*t3052;
  t3226 = -1.*t3031*t3046*t3061;
  t3227 = t3225 + t3226;
  t3228 = -0.13205*t3227;
  t3247 = -1.*t3046*t3241;
  t3248 = t3213 + t3247;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=-1.*t3030*t3031*t3039 + t3048*t3063 + t3069*t3077 - 0.13205*(t3046*t3063 + t3080) + t3087*t3090 - 0.183*(-1.*t2994*t3031*t3039 - 1.*t3085*t3090);
  p_output1[10]=t3030*t3031*t3056 + t3048*t3102 + t3069*t3106 - 0.13205*(t3046*t3102 + t3065*t3106) + t3087*t3114 - 0.183*(t2994*t3031*t3056 - 1.*t3085*t3114);
  p_output1[11]=0;
  p_output1[12]=t3031*t3048*t3052*t3056 - 1.*t3030*t3054*t3056 - 0.13205*(t3031*t3046*t3052*t3056 + t3031*t3056*t3061*t3065) + t3031*t3056*t3061*t3069 + t3087*t3134 - 0.183*(-1.*t2994*t3054*t3056 - 1.*t3085*t3134);
  p_output1[13]=t3031*t3039*t3048*t3052 - 1.*t3030*t3039*t3054 - 0.13205*(t3031*t3039*t3046*t3052 + t3031*t3039*t3061*t3065) + t3031*t3039*t3061*t3069 + t3087*t3152 - 0.183*(-1.*t2994*t3039*t3054 - 1.*t3085*t3152);
  p_output1[14]=-1.*t3030*t3031 - 1.*t3048*t3052*t3054 - 0.13205*(-1.*t3046*t3052*t3054 - 1.*t3054*t3061*t3065) - 1.*t3054*t3061*t3069 + t3087*t3178 - 0.183*(-1.*t2994*t3031 - 1.*t3085*t3178);
  p_output1[15]=t3069*t3102 + t3048*t3189 - 0.13205*(t3112 + t3046*t3189) + 0.183*t3085*t3195 + t3087*t3195;
  p_output1[16]=t3048*t3077 + t3069*t3206 + 0.183*t3085*t3210 + t3087*t3210 - 0.13205*(t3046*t3077 + t3213);
  p_output1[17]=-1.*t3031*t3048*t3061 + t3031*t3052*t3069 + t3223 + t3224 + t3228;
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
  p_output1[36]=0.047*t3065*t3102 - 0.047*t3046*t3106 - 0.13205*t3114 + 0.183*t3085*t3233 + t3087*t3233;
  p_output1[37]=0.047*t3065*t3206 - 0.047*t3046*t3241 + 0.183*t3085*t3244 + t3087*t3244 - 0.13205*t3248;
  p_output1[38]=-0.047*t3031*t3046*t3061 + 0.047*t3031*t3052*t3065 + t3223 + t3224 + t3228;
  p_output1[39]=-0.183*t3031*t3056*t3085 - 0.183*t2994*t3114 - 0.183*(-1.*t3031*t3056*t3085 - 1.*t2994*t3114);
  p_output1[40]=-0.183*t3031*t3039*t3085 - 0.183*t2994*t3248 - 0.183*(-1.*t3031*t3039*t3085 - 1.*t2994*t3248);
  p_output1[41]=0.183*t3054*t3085 - 0.183*t2994*t3227 - 0.183*(t3054*t3085 - 1.*t2994*t3227);
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



void Jp_RR_thigh_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
