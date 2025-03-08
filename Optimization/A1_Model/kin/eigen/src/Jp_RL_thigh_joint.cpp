/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:20 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RL_thigh_joint.h"

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
  double t696;
  double t1389;
  double t951;
  double t1253;
  double t1469;
  double t799;
  double t1359;
  double t1510;
  double t1737;
  double t1869;
  double t2740;
  double t2847;
  double t3129;
  double t60;
  double t586;
  double t3226;
  double t3772;
  double t4238;
  double t4970;
  double t439;
  double t445;
  double t457;
  double t561;
  double t882;
  double t925;
  double t1892;
  double t2279;
  double t2531;
  double t2669;
  double t7149;
  double t7154;
  double t7420;
  double t7485;
  double t8101;
  double t8756;
  double t3255;
  double t3396;
  double t8981;
  double t8991;
  double t8999;
  double t9054;
  double t9056;
  double t9058;
  double t9099;
  double t9100;
  double t9101;
  double t9124;
  double t9125;
  double t9126;
  double t9140;
  double t9141;
  double t9142;
  double t9161;
  double t9169;
  double t9175;
  double t9187;
  double t9188;
  double t9189;
  double t3163;
  double t9195;
  double t9199;
  double t9220;
  double t9223;
  double t9228;
  double t9247;
  double t9249;
  double t9267;
  double t9269;
  double t9270;
  double t9272;
  double t9273;
  double t9207;
  double t9229;
  double t9232;
  double t9235;
  double t9236;
  double t9239;
  double t9240;
  double t9277;
  double t9281;
  t696 = Sin(var1[3]);
  t1389 = Cos(var1[3]);
  t951 = Cos(var1[5]);
  t1253 = Sin(var1[4]);
  t1469 = Sin(var1[5]);
  t799 = Sin(var1[15]);
  t1359 = -1.*t951*t696*t1253;
  t1510 = t1389*t1469;
  t1737 = t1359 + t1510;
  t1869 = Cos(var1[15]);
  t2740 = -1.*t1389*t951;
  t2847 = -1.*t696*t1253*t1469;
  t3129 = t2740 + t2847;
  t60 = Cos(var1[16]);
  t586 = Cos(var1[4]);
  t3226 = Sin(var1[16]);
  t3772 = t1869*t1737;
  t4238 = -1.*t799*t3129;
  t4970 = t3772 + t4238;
  t439 = -1.*t60;
  t445 = 1. + t439;
  t457 = -0.183*t445;
  t561 = 0. + t457;
  t882 = -0.047*t799;
  t925 = 0. + t882;
  t1892 = -1.*t1869;
  t2279 = 1. + t1892;
  t2531 = 0.047*t2279;
  t2669 = 0. + t2531;
  t7149 = t1389*t951*t1253;
  t7154 = t696*t1469;
  t7420 = t7149 + t7154;
  t7485 = -1.*t951*t696;
  t8101 = t1389*t1253*t1469;
  t8756 = t7485 + t8101;
  t3255 = -0.183*t3226;
  t3396 = 0. + t3255;
  t8981 = t1869*t7420;
  t8991 = -1.*t799*t8756;
  t8999 = t8981 + t8991;
  t9054 = t1869*t1389*t586*t951;
  t9056 = -1.*t1389*t586*t799*t1469;
  t9058 = t9054 + t9056;
  t9099 = t1869*t586*t951*t696;
  t9100 = -1.*t586*t799*t696*t1469;
  t9101 = t9099 + t9100;
  t9124 = -1.*t1869*t951*t1253;
  t9125 = t799*t1253*t1469;
  t9126 = t9124 + t9125;
  t9140 = t951*t696;
  t9141 = -1.*t1389*t1253*t1469;
  t9142 = t9140 + t9141;
  t9161 = -1.*t799*t7420;
  t9169 = t1869*t9142;
  t9175 = t9161 + t9169;
  t9187 = t951*t696*t1253;
  t9188 = -1.*t1389*t1469;
  t9189 = t9187 + t9188;
  t3163 = t1869*t3129;
  t9195 = -1.*t799*t9189;
  t9199 = t9195 + t3163;
  t9220 = -1.*t586*t951*t799;
  t9223 = -1.*t1869*t586*t1469;
  t9228 = t9220 + t9223;
  t9247 = -1.*t1869*t8756;
  t9249 = t9161 + t9247;
  t9267 = t1389*t951;
  t9269 = t696*t1253*t1469;
  t9270 = t9267 + t9269;
  t9272 = -1.*t1869*t9270;
  t9273 = t9195 + t9272;
  t9207 = t1869*t9189;
  t9229 = t3396*t9228;
  t9232 = 0.183*t3226*t9228;
  t9235 = t1869*t586*t951;
  t9236 = -1.*t586*t799*t1469;
  t9239 = t9235 + t9236;
  t9240 = 0.13205*t9239;
  t9277 = -1.*t799*t9270;
  t9281 = t9207 + t9277;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t2669*t3129 + t3396*t4970 - 1.*t561*t586*t696 - 0.183*(-1.*t3226*t4970 - 1.*t586*t60*t696) + 0.13205*(t3163 + t1737*t799) + t1737*t925;
  p_output1(10)=t1389*t561*t586 + t2669*t8756 + 0.13205*(t7420*t799 + t1869*t8756) + t3396*t8999 - 0.183*(t1389*t586*t60 - 1.*t3226*t8999) + t7420*t925;
  p_output1(11)=0;
  p_output1(12)=-1.*t1253*t1389*t561 + t1389*t1469*t2669*t586 + t3396*t9058 - 0.183*(-1.*t1253*t1389*t60 - 1.*t3226*t9058) + t1389*t586*t925*t951 + 0.13205*(t1389*t1469*t1869*t586 + t1389*t586*t799*t951);
  p_output1(13)=-1.*t1253*t561*t696 + t1469*t2669*t586*t696 + t3396*t9101 - 0.183*(-1.*t1253*t60*t696 - 1.*t3226*t9101) + t586*t696*t925*t951 + 0.13205*(t1469*t1869*t586*t696 + t586*t696*t799*t951);
  p_output1(14)=-1.*t1253*t1469*t2669 - 1.*t561*t586 + t3396*t9126 - 0.183*(-1.*t586*t60 - 1.*t3226*t9126) - 1.*t1253*t925*t951 + 0.13205*(-1.*t1253*t1469*t1869 - 1.*t1253*t799*t951);
  p_output1(15)=t2669*t7420 + 0.13205*(t8981 + t799*t9142) + 0.183*t3226*t9175 + t3396*t9175 + t9142*t925;
  p_output1(16)=t2669*t9189 + 0.183*t3226*t9199 + t3396*t9199 + 0.13205*(t3129*t799 + t9207) + t3129*t925;
  p_output1(17)=t9229 + t9232 + t9240 - 1.*t1469*t586*t925 + t2669*t586*t951;
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
  p_output1(45)=-0.047*t1869*t7420 + 0.047*t799*t8756 + 0.13205*t8999 + 0.183*t3226*t9249 + t3396*t9249;
  p_output1(46)=-0.047*t1869*t9189 + 0.047*t799*t9270 + 0.183*t3226*t9273 + t3396*t9273 + 0.13205*t9281;
  p_output1(47)=0.047*t1469*t586*t799 + t9229 + t9232 + t9240 - 0.047*t1869*t586*t951;
  p_output1(48)=-0.183*t1389*t3226*t586 - 0.183*t60*t8999 - 0.183*(-1.*t1389*t3226*t586 - 1.*t60*t8999);
  p_output1(49)=-0.183*t3226*t586*t696 - 0.183*t60*t9281 - 0.183*(-1.*t3226*t586*t696 - 1.*t60*t9281);
  p_output1(50)=0.183*t1253*t3226 - 0.183*t60*t9239 - 0.183*(t1253*t3226 - 1.*t60*t9239);
  p_output1(51)=0;
  p_output1(52)=0;
  p_output1(53)=0;
}


       
Eigen::Matrix<double,3,18> Jp_RL_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void Jp_RL_thigh_joint(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



