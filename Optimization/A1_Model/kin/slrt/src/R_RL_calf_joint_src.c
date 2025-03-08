/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:59 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RL_calf_joint_src.h"

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
  double t5188;
  double t5257;
  double t5263;
  double t5260;
  double t5264;
  double t5220;
  double t5189;
  double t5209;
  double t5242;
  double t5261;
  double t5265;
  double t5266;
  double t5267;
  double t5268;
  double t5269;
  double t5270;
  double t5271;
  double t5273;
  double t5275;
  double t5163;
  double t5280;
  double t5291;
  double t5292;
  double t5293;
  double t5294;
  double t5295;
  double t5296;
  double t5297;
  double t5298;
  double t5299;
  double t5311;
  double t5313;
  double t5314;
  double t5216;
  double t5276;
  double t5277;
  double t5283;
  double t5284;
  double t5285;
  double t5290;
  double t5300;
  double t5301;
  double t5305;
  double t5306;
  double t5307;
  double t5310;
  double t5315;
  double t5316;
  double t5319;
  double t5320;
  double t5321;
  t5188 = Cos(var1[3]);
  t5257 = Cos(var1[5]);
  t5263 = Sin(var1[3]);
  t5260 = Sin(var1[4]);
  t5264 = Sin(var1[5]);
  t5220 = Cos(var1[16]);
  t5189 = Cos(var1[4]);
  t5209 = Sin(var1[16]);
  t5242 = Cos(var1[15]);
  t5261 = t5188*t5257*t5260;
  t5265 = t5263*t5264;
  t5266 = t5261 + t5265;
  t5267 = t5242*t5266;
  t5268 = Sin(var1[15]);
  t5269 = -1.*t5257*t5263;
  t5270 = t5188*t5260*t5264;
  t5271 = t5269 + t5270;
  t5273 = -1.*t5268*t5271;
  t5275 = t5267 + t5273;
  t5163 = Sin(var1[17]);
  t5280 = Cos(var1[17]);
  t5291 = t5257*t5263*t5260;
  t5292 = -1.*t5188*t5264;
  t5293 = t5291 + t5292;
  t5294 = t5242*t5293;
  t5295 = t5188*t5257;
  t5296 = t5263*t5260*t5264;
  t5297 = t5295 + t5296;
  t5298 = -1.*t5268*t5297;
  t5299 = t5294 + t5298;
  t5311 = t5242*t5189*t5257;
  t5313 = -1.*t5189*t5268*t5264;
  t5314 = t5311 + t5313;
  t5216 = t5188*t5189*t5209;
  t5276 = t5220*t5275;
  t5277 = t5216 + t5276;
  t5283 = t5220*t5188*t5189;
  t5284 = -1.*t5209*t5275;
  t5285 = t5283 + t5284;
  t5290 = t5189*t5209*t5263;
  t5300 = t5220*t5299;
  t5301 = t5290 + t5300;
  t5305 = t5220*t5189*t5263;
  t5306 = -1.*t5209*t5299;
  t5307 = t5305 + t5306;
  t5310 = -1.*t5209*t5260;
  t5315 = t5220*t5314;
  t5316 = t5310 + t5315;
  t5319 = -1.*t5220*t5260;
  t5320 = -1.*t5209*t5314;
  t5321 = t5319 + t5320;
  p_output1[0]=-1.*t5163*t5277 + t5280*t5285;
  p_output1[1]=-1.*t5163*t5301 + t5280*t5307;
  p_output1[2]=-1.*t5163*t5316 + t5280*t5321;
  p_output1[3]=t5266*t5268 + t5242*t5271;
  p_output1[4]=t5268*t5293 + t5242*t5297;
  p_output1[5]=t5189*t5242*t5264 + t5189*t5257*t5268;
  p_output1[6]=t5277*t5280 + t5163*t5285;
  p_output1[7]=t5280*t5301 + t5163*t5307;
  p_output1[8]=t5280*t5316 + t5163*t5321;
}



void R_RL_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
