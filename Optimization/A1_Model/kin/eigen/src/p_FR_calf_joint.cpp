/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:49 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FR_calf_joint.h"

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
  double t442;
  double t3836;
  double t4478;
  double t4023;
  double t4937;
  double t2734;
  double t4450;
  double t5428;
  double t5431;
  double t7979;
  double t7988;
  double t7989;
  double t7990;
  double t682;
  double t8008;
  double t8009;
  double t8014;
  double t535;
  double t8016;
  double t8033;
  double t8034;
  double t8039;
  double t8042;
  double t8049;
  double t8050;
  double t8051;
  double t8027;
  double t8029;
  double t8032;
  double t852;
  double t1831;
  double t1872;
  double t2029;
  double t2859;
  double t2973;
  double t3676;
  double t3704;
  double t7991;
  double t7993;
  double t8089;
  double t8090;
  double t8093;
  double t8096;
  double t8101;
  double t8102;
  double t8017;
  double t8019;
  double t8110;
  double t8111;
  double t8112;
  double t8041;
  double t8043;
  double t8044;
  double t8052;
  double t8058;
  double t8061;
  double t8118;
  double t8119;
  double t8120;
  double t8114;
  double t8115;
  double t8116;
  double t8145;
  double t8146;
  double t8147;
  double t8153;
  double t8154;
  double t8155;
  double t8149;
  double t8150;
  double t8151;
  t442 = Cos(var1[3]);
  t3836 = Cos(var1[5]);
  t4478 = Sin(var1[4]);
  t4023 = Sin(var1[3]);
  t4937 = Sin(var1[5]);
  t2734 = Cos(var1[6]);
  t4450 = -1.*t3836*t4023;
  t5428 = t442*t4478*t4937;
  t5431 = t4450 + t5428;
  t7979 = t442*t3836*t4478;
  t7988 = t4023*t4937;
  t7989 = t7979 + t7988;
  t7990 = Sin(var1[6]);
  t682 = Cos(var1[7]);
  t8008 = t2734*t7989;
  t8009 = -1.*t5431*t7990;
  t8014 = t8008 + t8009;
  t535 = Cos(var1[4]);
  t8016 = Sin(var1[7]);
  t8033 = Cos(var1[8]);
  t8034 = -1.*t8033;
  t8039 = 1. + t8034;
  t8042 = Sin(var1[8]);
  t8049 = t442*t535*t682;
  t8050 = -1.*t8014*t8016;
  t8051 = t8049 + t8050;
  t8027 = t682*t8014;
  t8029 = t442*t535*t8016;
  t8032 = t8027 + t8029;
  t852 = -1.*t682;
  t1831 = 1. + t852;
  t1872 = 0.183*t1831;
  t2029 = 0. + t1872;
  t2859 = -1.*t2734;
  t2973 = 1. + t2859;
  t3676 = -0.047*t2973;
  t3704 = 0. + t3676;
  t7991 = 0.047*t7990;
  t7993 = 0. + t7991;
  t8089 = t442*t3836;
  t8090 = t4023*t4478*t4937;
  t8093 = t8089 + t8090;
  t8096 = t3836*t4023*t4478;
  t8101 = -1.*t442*t4937;
  t8102 = t8096 + t8101;
  t8017 = 0.183*t8016;
  t8019 = 0. + t8017;
  t8110 = t2734*t8102;
  t8111 = -1.*t8093*t7990;
  t8112 = t8110 + t8111;
  t8041 = -0.2*t8039;
  t8043 = 0.183*t8042;
  t8044 = 0. + t8041 + t8043;
  t8052 = 0.183*t8039;
  t8058 = 0.2*t8042;
  t8061 = 0. + t8052 + t8058;
  t8118 = t535*t682*t4023;
  t8119 = -1.*t8112*t8016;
  t8120 = t8118 + t8119;
  t8114 = t682*t8112;
  t8115 = t535*t4023*t8016;
  t8116 = t8114 + t8115;
  t8145 = t535*t3836*t2734;
  t8146 = -1.*t535*t4937*t7990;
  t8147 = t8145 + t8146;
  t8153 = -1.*t682*t4478;
  t8154 = -1.*t8147*t8016;
  t8155 = t8153 + t8154;
  t8149 = t682*t8147;
  t8150 = -1.*t4478*t8016;
  t8151 = t8149 + t8150;

  p_output1(0)=0. + t2029*t442*t535 + t3704*t5431 - 0.13205*(t2734*t5431 + t7989*t7990) + t7989*t7993 + t8014*t8019 + t8032*t8044 + 0.183*(-1.*t8032*t8042 + t8033*t8051) - 0.2*(t8032*t8033 + t8042*t8051) + t8051*t8061 + var1(0);
  p_output1(1)=0. + t2029*t4023*t535 + t3704*t8093 + t7993*t8102 - 0.13205*(t2734*t8093 + t7990*t8102) + t8019*t8112 + t8044*t8116 + t8061*t8120 + 0.183*(-1.*t8042*t8116 + t8033*t8120) - 0.2*(t8033*t8116 + t8042*t8120) + var1(1);
  p_output1(2)=0. - 1.*t2029*t4478 + t3704*t4937*t535 - 0.13205*(t2734*t4937*t535 + t3836*t535*t7990) + t3836*t535*t7993 + t8019*t8147 + t8044*t8151 + t8061*t8155 + 0.183*(-1.*t8042*t8151 + t8033*t8155) - 0.2*(t8033*t8151 + t8042*t8155) + var1(2);
}


       
Eigen::Matrix<double,3,1> p_FR_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void p_FR_calf_joint(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



