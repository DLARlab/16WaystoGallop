/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:23 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RL_calf_joint.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t2250;
  double t4396;
  double t4700;
  double t4429;
  double t4717;
  double t2625;
  double t4494;
  double t5081;
  double t5082;
  double t5258;
  double t5959;
  double t6386;
  double t6444;
  double t2347;
  double t7345;
  double t957;
  double t8137;
  double t8873;
  double t8896;
  double t8963;
  double t8981;
  double t9003;
  double t9023;
  double t9049;
  double t9050;
  double t9054;
  double t9077;
  double t9080;
  double t9089;
  double t995;
  double t1282;
  double t1490;
  double t1532;
  double t2761;
  double t2878;
  double t5415;
  double t5530;
  double t5753;
  double t5758;
  double t9117;
  double t9118;
  double t9121;
  double t9125;
  double t9127;
  double t9128;
  double t7865;
  double t8046;
  double t9006;
  double t9041;
  double t9046;
  double t9169;
  double t9175;
  double t9183;
  double t9072;
  double t9075;
  double t9076;
  double t9185;
  double t9186;
  double t9187;
  double t9193;
  double t9199;
  double t9200;
  double t9276;
  double t9283;
  double t9285;
  double t9287;
  double t9289;
  double t9290;
  double t9293;
  double t9295;
  double t9296;
  t2250 = Cos(var1[3]);
  t4396 = Cos(var1[5]);
  t4700 = Sin(var1[3]);
  t4429 = Sin(var1[4]);
  t4717 = Sin(var1[5]);
  t2625 = Sin(var1[15]);
  t4494 = t2250*t4396*t4429;
  t5081 = t4700*t4717;
  t5082 = t4494 + t5081;
  t5258 = Cos(var1[15]);
  t5959 = -1.*t4396*t4700;
  t6386 = t2250*t4429*t4717;
  t6444 = t5959 + t6386;
  t2347 = Cos(var1[4]);
  t7345 = Sin(var1[16]);
  t957 = Cos(var1[16]);
  t8137 = t5258*t5082;
  t8873 = -1.*t2625*t6444;
  t8896 = t8137 + t8873;
  t8963 = Cos(var1[17]);
  t8981 = -1.*t8963;
  t9003 = 1. + t8981;
  t9023 = Sin(var1[17]);
  t9049 = t2250*t2347*t7345;
  t9050 = t957*t8896;
  t9054 = t9049 + t9050;
  t9077 = t957*t2250*t2347;
  t9080 = -1.*t7345*t8896;
  t9089 = t9077 + t9080;
  t995 = -1.*t957;
  t1282 = 1. + t995;
  t1490 = -0.183*t1282;
  t1532 = 0. + t1490;
  t2761 = -0.047*t2625;
  t2878 = 0. + t2761;
  t5415 = -1.*t5258;
  t5530 = 1. + t5415;
  t5753 = 0.047*t5530;
  t5758 = 0. + t5753;
  t9117 = t4396*t4700*t4429;
  t9118 = -1.*t2250*t4717;
  t9121 = t9117 + t9118;
  t9125 = t2250*t4396;
  t9127 = t4700*t4429*t4717;
  t9128 = t9125 + t9127;
  t7865 = -0.183*t7345;
  t8046 = 0. + t7865;
  t9006 = -0.2*t9003;
  t9041 = -0.183*t9023;
  t9046 = 0. + t9006 + t9041;
  t9169 = t5258*t9121;
  t9175 = -1.*t2625*t9128;
  t9183 = t9169 + t9175;
  t9072 = -0.183*t9003;
  t9075 = 0.2*t9023;
  t9076 = 0. + t9072 + t9075;
  t9185 = t2347*t7345*t4700;
  t9186 = t957*t9183;
  t9187 = t9185 + t9186;
  t9193 = t957*t2347*t4700;
  t9199 = -1.*t7345*t9183;
  t9200 = t9193 + t9199;
  t9276 = t5258*t2347*t4396;
  t9283 = -1.*t2347*t2625*t4717;
  t9285 = t9276 + t9283;
  t9287 = -1.*t7345*t4429;
  t9289 = t957*t9285;
  t9290 = t9287 + t9289;
  t9293 = -1.*t957*t4429;
  t9295 = -1.*t7345*t9285;
  t9296 = t9293 + t9295;

  p_output1(0)=0. + t1532*t2250*t2347 + t2878*t5082 + t5758*t6444 + 0.13205*(t2625*t5082 + t5258*t6444) + t8046*t8896 + t9046*t9054 + t9076*t9089 - 0.183*(-1.*t9023*t9054 + t8963*t9089) - 0.2*(t8963*t9054 + t9023*t9089) + var1(0);
  p_output1(1)=0. + t1532*t2347*t4700 + t2878*t9121 + t5758*t9128 + 0.13205*(t2625*t9121 + t5258*t9128) + t8046*t9183 + t9046*t9187 + t9076*t9200 - 0.183*(-1.*t9023*t9187 + t8963*t9200) - 0.2*(t8963*t9187 + t9023*t9200) + var1(1);
  p_output1(2)=0. + t2347*t2878*t4396 - 1.*t1532*t4429 + 0.13205*(t2347*t2625*t4396 + t2347*t4717*t5258) + t2347*t4717*t5758 + t8046*t9285 + t9046*t9290 + t9076*t9296 - 0.183*(-1.*t9023*t9290 + t8963*t9296) - 0.2*(t8963*t9290 + t9023*t9296) + var1(2);
}


       
Eigen::Matrix<double,3,1> p_RL_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void p_RL_calf_joint(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



