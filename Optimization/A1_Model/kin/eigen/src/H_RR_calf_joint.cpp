/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:13 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RR_calf_joint.h"

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
  double t678;
  double t1306;
  double t1340;
  double t1319;
  double t1422;
  double t1047;
  double t679;
  double t680;
  double t1210;
  double t1336;
  double t1433;
  double t1508;
  double t1556;
  double t1634;
  double t1798;
  double t2060;
  double t2076;
  double t2088;
  double t2231;
  double t595;
  double t2277;
  double t2448;
  double t2451;
  double t2452;
  double t2462;
  double t2472;
  double t2489;
  double t2523;
  double t2526;
  double t2530;
  double t2571;
  double t2573;
  double t2574;
  double t907;
  double t2236;
  double t2267;
  double t2353;
  double t2355;
  double t2358;
  double t2418;
  double t2551;
  double t2552;
  double t2562;
  double t2563;
  double t2565;
  double t2570;
  double t2575;
  double t2579;
  double t2581;
  double t2584;
  double t2605;
  double t2620;
  double t2642;
  double t2643;
  double t4242;
  double t4243;
  double t2275;
  double t2375;
  double t2401;
  double t2906;
  double t2948;
  double t3158;
  double t3764;
  double t3801;
  double t3851;
  double t3939;
  double t4166;
  double t4186;
  double t4212;
  double t4213;
  double t4220;
  double t4226;
  double t2698;
  double t2729;
  double t2759;
  double t4236;
  double t4237;
  double t4308;
  double t4380;
  double t4390;
  double t4407;
  double t4451;
  double t4501;
  double t2559;
  double t2566;
  double t2567;
  double t3184;
  double t3367;
  double t3429;
  double t2805;
  double t2841;
  double t2897;
  double t2580;
  double t2615;
  double t2619;
  double t3617;
  double t3619;
  double t3623;
  t678 = Cos(var1[3]);
  t1306 = Cos(var1[5]);
  t1340 = Sin(var1[3]);
  t1319 = Sin(var1[4]);
  t1422 = Sin(var1[5]);
  t1047 = Cos(var1[13]);
  t679 = Cos(var1[4]);
  t680 = Sin(var1[13]);
  t1210 = Cos(var1[12]);
  t1336 = t678*t1306*t1319;
  t1433 = t1340*t1422;
  t1508 = t1336 + t1433;
  t1556 = t1210*t1508;
  t1634 = Sin(var1[12]);
  t1798 = -1.*t1306*t1340;
  t2060 = t678*t1319*t1422;
  t2076 = t1798 + t2060;
  t2088 = -1.*t1634*t2076;
  t2231 = t1556 + t2088;
  t595 = Sin(var1[14]);
  t2277 = Cos(var1[14]);
  t2448 = t1306*t1340*t1319;
  t2451 = -1.*t678*t1422;
  t2452 = t2448 + t2451;
  t2462 = t1210*t2452;
  t2472 = t678*t1306;
  t2489 = t1340*t1319*t1422;
  t2523 = t2472 + t2489;
  t2526 = -1.*t1634*t2523;
  t2530 = t2462 + t2526;
  t2571 = t1210*t679*t1306;
  t2573 = -1.*t679*t1634*t1422;
  t2574 = t2571 + t2573;
  t907 = t678*t679*t680;
  t2236 = t1047*t2231;
  t2267 = t907 + t2236;
  t2353 = t1047*t678*t679;
  t2355 = -1.*t680*t2231;
  t2358 = t2353 + t2355;
  t2418 = t679*t680*t1340;
  t2551 = t1047*t2530;
  t2552 = t2418 + t2551;
  t2562 = t1047*t679*t1340;
  t2563 = -1.*t680*t2530;
  t2565 = t2562 + t2563;
  t2570 = -1.*t680*t1319;
  t2575 = t1047*t2574;
  t2579 = t2570 + t2575;
  t2581 = -1.*t1047*t1319;
  t2584 = -1.*t680*t2574;
  t2605 = t2581 + t2584;
  t2620 = t1634*t1508;
  t2642 = t1210*t2076;
  t2643 = t2620 + t2642;
  t4242 = -1.*t2277;
  t4243 = 1. + t4242;
  t2275 = -1.*t595*t2267;
  t2375 = t2277*t2358;
  t2401 = t2275 + t2375;
  t2906 = t2277*t2267;
  t2948 = t595*t2358;
  t3158 = t2906 + t2948;
  t3764 = -1.*t1047;
  t3801 = 1. + t3764;
  t3851 = -0.183*t3801;
  t3939 = 0. + t3851;
  t4166 = 0.047*t1634;
  t4186 = 0. + t4166;
  t4212 = -1.*t1210;
  t4213 = 1. + t4212;
  t4220 = -0.047*t4213;
  t4226 = 0. + t4220;
  t2698 = t1634*t2452;
  t2729 = t1210*t2523;
  t2759 = t2698 + t2729;
  t4236 = -0.183*t680;
  t4237 = 0. + t4236;
  t4308 = -0.2*t4243;
  t4380 = -0.183*t595;
  t4390 = 0. + t4308 + t4380;
  t4407 = -0.183*t4243;
  t4451 = 0.2*t595;
  t4501 = 0. + t4407 + t4451;
  t2559 = -1.*t595*t2552;
  t2566 = t2277*t2565;
  t2567 = t2559 + t2566;
  t3184 = t2277*t2552;
  t3367 = t595*t2565;
  t3429 = t3184 + t3367;
  t2805 = t679*t1306*t1634;
  t2841 = t1210*t679*t1422;
  t2897 = t2805 + t2841;
  t2580 = -1.*t595*t2579;
  t2615 = t2277*t2605;
  t2619 = t2580 + t2615;
  t3617 = t2277*t2579;
  t3619 = t595*t2605;
  t3623 = t3617 + t3619;

  p_output1(0)=t2401;
  p_output1(1)=t2567;
  p_output1(2)=t2619;
  p_output1(3)=0.;
  p_output1(4)=t2643;
  p_output1(5)=t2759;
  p_output1(6)=t2897;
  p_output1(7)=0.;
  p_output1(8)=t3158;
  p_output1(9)=t3429;
  p_output1(10)=t3623;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.183*t2401 - 0.13205*t2643 - 0.2*t3158 + t1508*t4186 + t2076*t4226 + t2231*t4237 + t2267*t4390 + t2358*t4501 + t3939*t678*t679 + var1(0);
  p_output1(13)=0. - 0.183*t2567 - 0.13205*t2759 - 0.2*t3429 + t2452*t4186 + t2523*t4226 + t2530*t4237 + t2552*t4390 + t2565*t4501 + t1340*t3939*t679 + var1(1);
  p_output1(14)=0. - 0.183*t2619 - 0.13205*t2897 - 0.2*t3623 - 1.*t1319*t3939 + t2574*t4237 + t2579*t4390 + t2605*t4501 + t1306*t4186*t679 + t1422*t4226*t679 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_RR_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void H_RR_calf_joint(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



