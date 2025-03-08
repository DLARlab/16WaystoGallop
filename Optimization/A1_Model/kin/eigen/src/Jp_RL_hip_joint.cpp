/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:16 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RL_hip_joint.h"

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
static void output1(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t1645;
  double t3529;
  double t2245;
  double t2490;
  double t4939;
  double t1854;
  double t3257;
  double t4954;
  double t4998;
  double t5237;
  double t5553;
  double t5617;
  double t5635;
  double t567;
  double t1902;
  double t1914;
  double t5425;
  double t5455;
  double t5504;
  double t5508;
  double t6208;
  double t6213;
  double t6236;
  double t6289;
  double t6305;
  double t6351;
  double t7471;
  double t7477;
  double t7485;
  double t8090;
  double t8390;
  double t8756;
  double t7507;
  double t8811;
  double t8987;
  double t8991;
  double t8998;
  double t8899;
  double t8917;
  double t8920;
  double t8921;
  t1645 = Sin(var1[3]);
  t3529 = Cos(var1[3]);
  t2245 = Cos(var1[5]);
  t2490 = Sin(var1[4]);
  t4939 = Sin(var1[5]);
  t1854 = Sin(var1[15]);
  t3257 = -1.*t2245*t1645*t2490;
  t4954 = t3529*t4939;
  t4998 = t3257 + t4954;
  t5237 = Cos(var1[15]);
  t5553 = -1.*t3529*t2245;
  t5617 = -1.*t1645*t2490*t4939;
  t5635 = t5553 + t5617;
  t567 = Cos(var1[4]);
  t1902 = -0.047*t1854;
  t1914 = 0. + t1902;
  t5425 = -1.*t5237;
  t5455 = 1. + t5425;
  t5504 = 0.047*t5455;
  t5508 = 0. + t5504;
  t6208 = t3529*t2245*t2490;
  t6213 = t1645*t4939;
  t6236 = t6208 + t6213;
  t6289 = -1.*t2245*t1645;
  t6305 = t3529*t2490*t4939;
  t6351 = t6289 + t6305;
  t7471 = t2245*t1645;
  t7477 = -1.*t3529*t2490*t4939;
  t7485 = t7471 + t7477;
  t8090 = t2245*t1645*t2490;
  t8390 = -1.*t3529*t4939;
  t8756 = t8090 + t8390;
  t7507 = t5237*t6236;
  t8811 = t5237*t8756;
  t8987 = t3529*t2245;
  t8991 = t1645*t2490*t4939;
  t8998 = t8987 + t8991;
  t8899 = t5237*t567*t2245;
  t8917 = -1.*t567*t1854*t4939;
  t8920 = t8899 + t8917;
  t8921 = 0.047*t8920;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1914*t4998 + t5508*t5635 + 0.047*(t1854*t4998 + t5237*t5635) + 0.183*t1645*t567;
  p_output1(10)=-0.183*t3529*t567 + t1914*t6236 + t5508*t6351 + 0.047*(t1854*t6236 + t5237*t6351);
  p_output1(11)=0;
  p_output1(12)=0.183*t2490*t3529 + t1914*t2245*t3529*t567 + t3529*t4939*t5508*t567 + 0.047*(t1854*t2245*t3529*t567 + t3529*t4939*t5237*t567);
  p_output1(13)=0.183*t1645*t2490 + t1645*t1914*t2245*t567 + t1645*t4939*t5508*t567 + 0.047*(t1645*t1854*t2245*t567 + t1645*t4939*t5237*t567);
  p_output1(14)=-1.*t1914*t2245*t2490 + 0.047*(-1.*t1854*t2245*t2490 - 1.*t2490*t4939*t5237) - 1.*t2490*t4939*t5508 + 0.183*t567;
  p_output1(15)=t5508*t6236 + t1914*t7485 + 0.047*(t1854*t7485 + t7507);
  p_output1(16)=t1914*t5635 + t5508*t8756 + 0.047*(t1854*t5635 + t8811);
  p_output1(17)=-1.*t1914*t4939*t567 + t2245*t5508*t567 + t8921;
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=0;
  p_output1(34)=0;
  p_output1(35)=0;
  p_output1(36)=0;
  p_output1(37)=0;
  p_output1(38)=0;
  p_output1(39)=0;
  p_output1(40)=0;
  p_output1(41)=0;
  p_output1(42)=0;
  p_output1(43)=0;
  p_output1(44)=0;
  p_output1(45)=-0.047*t5237*t6236 + 0.047*t1854*t6351 + 0.047*(-1.*t1854*t6351 + t7507);
  p_output1(46)=-0.047*t5237*t8756 + 0.047*t1854*t8998 + 0.047*(t8811 - 1.*t1854*t8998);
  p_output1(47)=0.047*t1854*t4939*t567 - 0.047*t2245*t5237*t567 + t8921;
  p_output1(48)=0;
  p_output1(49)=0;
  p_output1(50)=0;
  p_output1(51)=0;
  p_output1(52)=0;
  p_output1(53)=0;
}


       
Eigen::Matrix<double,3,18> Jp_RL_hip_joint(const Eigen::Matrix<double,18,1> &var1)
//void Jp_RL_hip_joint(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



