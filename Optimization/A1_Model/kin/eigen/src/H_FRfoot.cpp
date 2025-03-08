/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:29 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FRfoot.h"

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
  double t539;
  double t783;
  double t1430;
  double t1414;
  double t1482;
  double t707;
  double t748;
  double t1420;
  double t1708;
  double t1800;
  double t1811;
  double t1840;
  double t1843;
  double t2151;
  double t2273;
  double t2324;
  double t2330;
  double t701;
  double t2333;
  double t345;
  double t3078;
  double t3119;
  double t3211;
  double t3228;
  double t3335;
  double t3385;
  double t3388;
  double t3405;
  double t3616;
  double t2727;
  double t3876;
  double t3893;
  double t3942;
  double t2599;
  double t2623;
  double t2626;
  double t708;
  double t2341;
  double t2393;
  double t3726;
  double t3736;
  double t3742;
  double t3044;
  double t3627;
  double t3631;
  double t4227;
  double t4233;
  double t4253;
  double t3871;
  double t4111;
  double t4201;
  double t4272;
  double t4274;
  double t4275;
  double t5328;
  double t5331;
  double t2443;
  double t2871;
  double t2877;
  double t4462;
  double t4480;
  double t4600;
  double t4898;
  double t4902;
  double t5036;
  double t5053;
  double t5084;
  double t5096;
  double t5133;
  double t5278;
  double t5288;
  double t5289;
  double t4276;
  double t4277;
  double t4280;
  double t5322;
  double t5326;
  double t5332;
  double t5335;
  double t5336;
  double t5344;
  double t5346;
  double t5348;
  double t3694;
  double t3773;
  double t3792;
  double t4670;
  double t4671;
  double t4723;
  double t4281;
  double t4285;
  double t4314;
  double t4202;
  double t4269;
  double t4270;
  double t4769;
  double t4787;
  double t4885;
  t539 = Cos(var1[3]);
  t783 = Cos(var1[5]);
  t1430 = Sin(var1[3]);
  t1414 = Sin(var1[4]);
  t1482 = Sin(var1[5]);
  t707 = Cos(var1[7]);
  t748 = Cos(var1[6]);
  t1420 = t539*t783*t1414;
  t1708 = t1430*t1482;
  t1800 = t1420 + t1708;
  t1811 = t748*t1800;
  t1840 = -1.*t783*t1430;
  t1843 = t539*t1414*t1482;
  t2151 = t1840 + t1843;
  t2273 = Sin(var1[6]);
  t2324 = -1.*t2151*t2273;
  t2330 = t1811 + t2324;
  t701 = Cos(var1[4]);
  t2333 = Sin(var1[7]);
  t345 = Cos(var1[8]);
  t3078 = t783*t1430*t1414;
  t3119 = -1.*t539*t1482;
  t3211 = t3078 + t3119;
  t3228 = t748*t3211;
  t3335 = t539*t783;
  t3385 = t1430*t1414*t1482;
  t3388 = t3335 + t3385;
  t3405 = -1.*t3388*t2273;
  t3616 = t3228 + t3405;
  t2727 = Sin(var1[8]);
  t3876 = t701*t783*t748;
  t3893 = -1.*t701*t1482*t2273;
  t3942 = t3876 + t3893;
  t2599 = t707*t2330;
  t2623 = t539*t701*t2333;
  t2626 = t2599 + t2623;
  t708 = t539*t701*t707;
  t2341 = -1.*t2330*t2333;
  t2393 = t708 + t2341;
  t3726 = t707*t3616;
  t3736 = t701*t1430*t2333;
  t3742 = t3726 + t3736;
  t3044 = t701*t707*t1430;
  t3627 = -1.*t3616*t2333;
  t3631 = t3044 + t3627;
  t4227 = t707*t3942;
  t4233 = -1.*t1414*t2333;
  t4253 = t4227 + t4233;
  t3871 = -1.*t707*t1414;
  t4111 = -1.*t3942*t2333;
  t4201 = t3871 + t4111;
  t4272 = t748*t2151;
  t4274 = t1800*t2273;
  t4275 = t4272 + t4274;
  t5328 = -1.*t345;
  t5331 = 1. + t5328;
  t2443 = t345*t2393;
  t2871 = -1.*t2626*t2727;
  t2877 = t2443 + t2871;
  t4462 = t345*t2626;
  t4480 = t2393*t2727;
  t4600 = t4462 + t4480;
  t4898 = -1.*t707;
  t4902 = 1. + t4898;
  t5036 = 0.183*t4902;
  t5053 = 0. + t5036;
  t5084 = -1.*t748;
  t5096 = 1. + t5084;
  t5133 = -0.047*t5096;
  t5278 = 0. + t5133;
  t5288 = 0.047*t2273;
  t5289 = 0. + t5288;
  t4276 = t748*t3388;
  t4277 = t3211*t2273;
  t4280 = t4276 + t4277;
  t5322 = 0.183*t2333;
  t5326 = 0. + t5322;
  t5332 = -0.2*t5331;
  t5335 = 0.183*t2727;
  t5336 = 0. + t5332 + t5335;
  t5344 = 0.183*t5331;
  t5346 = 0.2*t2727;
  t5348 = 0. + t5344 + t5346;
  t3694 = t345*t3631;
  t3773 = -1.*t3742*t2727;
  t3792 = t3694 + t3773;
  t4670 = t345*t3742;
  t4671 = t3631*t2727;
  t4723 = t4670 + t4671;
  t4281 = t701*t748*t1482;
  t4285 = t701*t783*t2273;
  t4314 = t4281 + t4285;
  t4202 = t345*t4201;
  t4269 = -1.*t4253*t2727;
  t4270 = t4202 + t4269;
  t4769 = t345*t4253;
  t4787 = t4201*t2727;
  t4885 = t4769 + t4787;

  p_output1(0)=t2877;
  p_output1(1)=t3792;
  p_output1(2)=t4270;
  p_output1(3)=0.;
  p_output1(4)=t4275;
  p_output1(5)=t4280;
  p_output1(6)=t4314;
  p_output1(7)=0.;
  p_output1(8)=t4600;
  p_output1(9)=t4723;
  p_output1(10)=t4885;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.183*t2877 - 0.13205*t4275 - 0.4*t4600 + t2151*t5278 + t1800*t5289 + t2330*t5326 + t2626*t5336 + t2393*t5348 + t5053*t539*t701 + var1(0);
  p_output1(13)=0. + 0.183*t3792 - 0.13205*t4280 - 0.4*t4723 + t3388*t5278 + t3211*t5289 + t3616*t5326 + t3742*t5336 + t3631*t5348 + t1430*t5053*t701 + var1(1);
  p_output1(14)=0. + 0.183*t4270 - 0.13205*t4314 - 0.4*t4885 - 1.*t1414*t5053 + t3942*t5326 + t4253*t5336 + t4201*t5348 + t1482*t5278*t701 + t5289*t701*t783 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_FRfoot(const Eigen::Matrix<double,18,1> &var1)
//void H_FRfoot(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



