/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:51 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FR_calf_joint.h"

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
  double t243;
  double t657;
  double t927;
  double t660;
  double t931;
  double t553;
  double t640;
  double t886;
  double t983;
  double t1379;
  double t1383;
  double t1411;
  double t1498;
  double t1802;
  double t1908;
  double t2074;
  double t2259;
  double t483;
  double t2319;
  double t230;
  double t3734;
  double t3803;
  double t3804;
  double t3806;
  double t3821;
  double t3833;
  double t3845;
  double t3856;
  double t3858;
  double t2895;
  double t3906;
  double t3910;
  double t3937;
  double t2732;
  double t2820;
  double t2881;
  double t624;
  double t2444;
  double t2685;
  double t3891;
  double t3894;
  double t3897;
  double t3583;
  double t3880;
  double t3883;
  double t4336;
  double t4374;
  double t4862;
  double t3903;
  double t3942;
  double t3947;
  double t5199;
  double t5677;
  double t5816;
  double t8160;
  double t8162;
  double t2689;
  double t3578;
  double t3580;
  double t8021;
  double t8027;
  double t8029;
  double t8108;
  double t8109;
  double t8114;
  double t8115;
  double t8121;
  double t8125;
  double t8134;
  double t8136;
  double t8141;
  double t8144;
  double t6345;
  double t7994;
  double t8003;
  double t8152;
  double t8156;
  double t8163;
  double t8164;
  double t8165;
  double t8169;
  double t8172;
  double t8176;
  double t3889;
  double t3898;
  double t3899;
  double t8041;
  double t8043;
  double t8052;
  double t8006;
  double t8008;
  double t8019;
  double t4293;
  double t4884;
  double t4909;
  double t8085;
  double t8096;
  double t8107;
  t243 = Cos(var1[3]);
  t657 = Cos(var1[5]);
  t927 = Sin(var1[3]);
  t660 = Sin(var1[4]);
  t931 = Sin(var1[5]);
  t553 = Cos(var1[7]);
  t640 = Cos(var1[6]);
  t886 = t243*t657*t660;
  t983 = t927*t931;
  t1379 = t886 + t983;
  t1383 = t640*t1379;
  t1411 = -1.*t657*t927;
  t1498 = t243*t660*t931;
  t1802 = t1411 + t1498;
  t1908 = Sin(var1[6]);
  t2074 = -1.*t1802*t1908;
  t2259 = t1383 + t2074;
  t483 = Cos(var1[4]);
  t2319 = Sin(var1[7]);
  t230 = Cos(var1[8]);
  t3734 = t657*t927*t660;
  t3803 = -1.*t243*t931;
  t3804 = t3734 + t3803;
  t3806 = t640*t3804;
  t3821 = t243*t657;
  t3833 = t927*t660*t931;
  t3845 = t3821 + t3833;
  t3856 = -1.*t3845*t1908;
  t3858 = t3806 + t3856;
  t2895 = Sin(var1[8]);
  t3906 = t483*t657*t640;
  t3910 = -1.*t483*t931*t1908;
  t3937 = t3906 + t3910;
  t2732 = t553*t2259;
  t2820 = t243*t483*t2319;
  t2881 = t2732 + t2820;
  t624 = t243*t483*t553;
  t2444 = -1.*t2259*t2319;
  t2685 = t624 + t2444;
  t3891 = t553*t3858;
  t3894 = t483*t927*t2319;
  t3897 = t3891 + t3894;
  t3583 = t483*t553*t927;
  t3880 = -1.*t3858*t2319;
  t3883 = t3583 + t3880;
  t4336 = t553*t3937;
  t4374 = -1.*t660*t2319;
  t4862 = t4336 + t4374;
  t3903 = -1.*t553*t660;
  t3942 = -1.*t3937*t2319;
  t3947 = t3903 + t3942;
  t5199 = t640*t1802;
  t5677 = t1379*t1908;
  t5816 = t5199 + t5677;
  t8160 = -1.*t230;
  t8162 = 1. + t8160;
  t2689 = t230*t2685;
  t3578 = -1.*t2881*t2895;
  t3580 = t2689 + t3578;
  t8021 = t230*t2881;
  t8027 = t2685*t2895;
  t8029 = t8021 + t8027;
  t8108 = -1.*t553;
  t8109 = 1. + t8108;
  t8114 = 0.183*t8109;
  t8115 = 0. + t8114;
  t8121 = -1.*t640;
  t8125 = 1. + t8121;
  t8134 = -0.047*t8125;
  t8136 = 0. + t8134;
  t8141 = 0.047*t1908;
  t8144 = 0. + t8141;
  t6345 = t640*t3845;
  t7994 = t3804*t1908;
  t8003 = t6345 + t7994;
  t8152 = 0.183*t2319;
  t8156 = 0. + t8152;
  t8163 = -0.2*t8162;
  t8164 = 0.183*t2895;
  t8165 = 0. + t8163 + t8164;
  t8169 = 0.183*t8162;
  t8172 = 0.2*t2895;
  t8176 = 0. + t8169 + t8172;
  t3889 = t230*t3883;
  t3898 = -1.*t3897*t2895;
  t3899 = t3889 + t3898;
  t8041 = t230*t3897;
  t8043 = t3883*t2895;
  t8052 = t8041 + t8043;
  t8006 = t483*t640*t931;
  t8008 = t483*t657*t1908;
  t8019 = t8006 + t8008;
  t4293 = t230*t3947;
  t4884 = -1.*t4862*t2895;
  t4909 = t4293 + t4884;
  t8085 = t230*t4862;
  t8096 = t3947*t2895;
  t8107 = t8085 + t8096;

  p_output1(0)=t3580;
  p_output1(1)=t3899;
  p_output1(2)=t4909;
  p_output1(3)=0.;
  p_output1(4)=t5816;
  p_output1(5)=t8003;
  p_output1(6)=t8019;
  p_output1(7)=0.;
  p_output1(8)=t8029;
  p_output1(9)=t8052;
  p_output1(10)=t8107;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.183*t3580 - 0.13205*t5816 - 0.2*t8029 + t243*t483*t8115 + t1802*t8136 + t1379*t8144 + t2259*t8156 + t2881*t8165 + t2685*t8176 + var1(0);
  p_output1(13)=0. + 0.183*t3899 - 0.13205*t8003 - 0.2*t8052 + t3845*t8136 + t3804*t8144 + t3858*t8156 + t3897*t8165 + t3883*t8176 + t483*t8115*t927 + var1(1);
  p_output1(14)=0. + 0.183*t4909 - 0.13205*t8019 - 0.2*t8107 - 1.*t660*t8115 + t483*t657*t8144 + t3937*t8156 + t4862*t8165 + t3947*t8176 + t483*t8136*t931 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_FR_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void H_FR_calf_joint(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



