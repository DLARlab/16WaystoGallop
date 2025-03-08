/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:32 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FLfoot.h"

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
  double t109;
  double t978;
  double t1026;
  double t1015;
  double t1104;
  double t922;
  double t540;
  double t602;
  double t943;
  double t1017;
  double t1214;
  double t1290;
  double t1810;
  double t1947;
  double t2077;
  double t2082;
  double t2101;
  double t2109;
  double t2170;
  double t50;
  double t2290;
  double t2716;
  double t2731;
  double t2786;
  double t2849;
  double t3153;
  double t3379;
  double t3615;
  double t3622;
  double t3638;
  double t4049;
  double t4099;
  double t4122;
  double t778;
  double t2199;
  double t2219;
  double t2304;
  double t2436;
  double t2491;
  double t2711;
  double t3661;
  double t3720;
  double t3789;
  double t3790;
  double t3796;
  double t3843;
  double t4171;
  double t4210;
  double t4223;
  double t4306;
  double t4316;
  double t4399;
  double t4498;
  double t4535;
  double t5906;
  double t5992;
  double t2252;
  double t2587;
  double t2617;
  double t5362;
  double t5391;
  double t5392;
  double t5524;
  double t5548;
  double t5552;
  double t5651;
  double t5672;
  double t5685;
  double t5708;
  double t5759;
  double t5800;
  double t5808;
  double t4690;
  double t4698;
  double t4956;
  double t5863;
  double t5870;
  double t6057;
  double t6079;
  double t6137;
  double t6151;
  double t6174;
  double t6175;
  double t3769;
  double t3837;
  double t3839;
  double t5409;
  double t5471;
  double t5475;
  double t4966;
  double t5118;
  double t5155;
  double t4219;
  double t4328;
  double t4377;
  double t5477;
  double t5481;
  double t5492;
  t109 = Cos(var1[3]);
  t978 = Cos(var1[5]);
  t1026 = Sin(var1[3]);
  t1015 = Sin(var1[4]);
  t1104 = Sin(var1[5]);
  t922 = Cos(var1[10]);
  t540 = Cos(var1[4]);
  t602 = Sin(var1[10]);
  t943 = Cos(var1[9]);
  t1017 = t109*t978*t1015;
  t1214 = t1026*t1104;
  t1290 = t1017 + t1214;
  t1810 = t943*t1290;
  t1947 = Sin(var1[9]);
  t2077 = -1.*t978*t1026;
  t2082 = t109*t1015*t1104;
  t2101 = t2077 + t2082;
  t2109 = -1.*t1947*t2101;
  t2170 = t1810 + t2109;
  t50 = Sin(var1[11]);
  t2290 = Cos(var1[11]);
  t2716 = t978*t1026*t1015;
  t2731 = -1.*t109*t1104;
  t2786 = t2716 + t2731;
  t2849 = t943*t2786;
  t3153 = t109*t978;
  t3379 = t1026*t1015*t1104;
  t3615 = t3153 + t3379;
  t3622 = -1.*t1947*t3615;
  t3638 = t2849 + t3622;
  t4049 = t943*t540*t978;
  t4099 = -1.*t540*t1947*t1104;
  t4122 = t4049 + t4099;
  t778 = t109*t540*t602;
  t2199 = t922*t2170;
  t2219 = t778 + t2199;
  t2304 = t922*t109*t540;
  t2436 = -1.*t602*t2170;
  t2491 = t2304 + t2436;
  t2711 = t540*t602*t1026;
  t3661 = t922*t3638;
  t3720 = t2711 + t3661;
  t3789 = t922*t540*t1026;
  t3790 = -1.*t602*t3638;
  t3796 = t3789 + t3790;
  t3843 = -1.*t602*t1015;
  t4171 = t922*t4122;
  t4210 = t3843 + t4171;
  t4223 = -1.*t922*t1015;
  t4306 = -1.*t602*t4122;
  t4316 = t4223 + t4306;
  t4399 = t1947*t1290;
  t4498 = t943*t2101;
  t4535 = t4399 + t4498;
  t5906 = -1.*t2290;
  t5992 = 1. + t5906;
  t2252 = -1.*t50*t2219;
  t2587 = t2290*t2491;
  t2617 = t2252 + t2587;
  t5362 = t2290*t2219;
  t5391 = t50*t2491;
  t5392 = t5362 + t5391;
  t5524 = -1.*t922;
  t5548 = 1. + t5524;
  t5552 = 0.183*t5548;
  t5651 = 0. + t5552;
  t5672 = -0.047*t1947;
  t5685 = 0. + t5672;
  t5708 = -1.*t943;
  t5759 = 1. + t5708;
  t5800 = 0.047*t5759;
  t5808 = 0. + t5800;
  t4690 = t1947*t2786;
  t4698 = t943*t3615;
  t4956 = t4690 + t4698;
  t5863 = 0.183*t602;
  t5870 = 0. + t5863;
  t6057 = -0.2*t5992;
  t6079 = 0.183*t50;
  t6137 = 0. + t6057 + t6079;
  t6151 = 0.183*t5992;
  t6174 = 0.2*t50;
  t6175 = 0. + t6151 + t6174;
  t3769 = -1.*t50*t3720;
  t3837 = t2290*t3796;
  t3839 = t3769 + t3837;
  t5409 = t2290*t3720;
  t5471 = t50*t3796;
  t5475 = t5409 + t5471;
  t4966 = t540*t978*t1947;
  t5118 = t943*t540*t1104;
  t5155 = t4966 + t5118;
  t4219 = -1.*t50*t4210;
  t4328 = t2290*t4316;
  t4377 = t4219 + t4328;
  t5477 = t2290*t4210;
  t5481 = t50*t4316;
  t5492 = t5477 + t5481;

  p_output1(0)=t2617;
  p_output1(1)=t3839;
  p_output1(2)=t4377;
  p_output1(3)=0.;
  p_output1(4)=t4535;
  p_output1(5)=t4956;
  p_output1(6)=t5155;
  p_output1(7)=0.;
  p_output1(8)=t5392;
  p_output1(9)=t5475;
  p_output1(10)=t5492;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.183*t2617 + 0.13205*t4535 - 0.4*t5392 + t109*t540*t5651 + t1290*t5685 + t2101*t5808 + t2170*t5870 + t2219*t6137 + t2491*t6175 + var1(0);
  p_output1(13)=0. + 0.183*t3839 + 0.13205*t4956 - 0.4*t5475 + t1026*t540*t5651 + t2786*t5685 + t3615*t5808 + t3638*t5870 + t3720*t6137 + t3796*t6175 + var1(1);
  p_output1(14)=0. + 0.183*t4377 + 0.13205*t5155 - 0.4*t5492 - 1.*t1015*t5651 + t1104*t540*t5808 + t4122*t5870 + t4210*t6137 + t4316*t6175 + t540*t5685*t978 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_FLfoot(const Eigen::Matrix<double,18,1> &var1)
//void H_FLfoot(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



