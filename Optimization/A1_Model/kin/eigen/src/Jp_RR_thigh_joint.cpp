/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:09 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RR_thigh_joint.h"

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
  double t622;
  double t1146;
  double t1106;
  double t1111;
  double t1163;
  double t1048;
  double t1133;
  double t1655;
  double t1839;
  double t2569;
  double t3335;
  double t3338;
  double t3339;
  double t206;
  double t621;
  double t3346;
  double t3352;
  double t3353;
  double t3354;
  double t329;
  double t356;
  double t369;
  double t599;
  double t1081;
  double t1093;
  double t3041;
  double t3205;
  double t3316;
  double t3329;
  double t3425;
  double t3593;
  double t3636;
  double t3722;
  double t3812;
  double t3873;
  double t3350;
  double t3351;
  double t5582;
  double t6171;
  double t6220;
  double t8759;
  double t8762;
  double t8763;
  double t8828;
  double t8829;
  double t8830;
  double t8859;
  double t8860;
  double t8863;
  double t8877;
  double t8881;
  double t8883;
  double t8886;
  double t8887;
  double t8895;
  double t8908;
  double t8910;
  double t8911;
  double t3343;
  double t8915;
  double t8917;
  double t8932;
  double t8935;
  double t8936;
  double t8951;
  double t8952;
  double t8958;
  double t8959;
  double t8960;
  double t8966;
  double t8970;
  double t8922;
  double t8937;
  double t8938;
  double t8939;
  double t8940;
  double t8941;
  double t8942;
  double t8974;
  double t8975;
  t622 = Sin(var1[3]);
  t1146 = Cos(var1[3]);
  t1106 = Cos(var1[5]);
  t1111 = Sin(var1[4]);
  t1163 = Sin(var1[5]);
  t1048 = Sin(var1[12]);
  t1133 = -1.*t1106*t622*t1111;
  t1655 = t1146*t1163;
  t1839 = t1133 + t1655;
  t2569 = Cos(var1[12]);
  t3335 = -1.*t1146*t1106;
  t3338 = -1.*t622*t1111*t1163;
  t3339 = t3335 + t3338;
  t206 = Cos(var1[13]);
  t621 = Cos(var1[4]);
  t3346 = Sin(var1[13]);
  t3352 = t2569*t1839;
  t3353 = -1.*t1048*t3339;
  t3354 = t3352 + t3353;
  t329 = -1.*t206;
  t356 = 1. + t329;
  t369 = -0.183*t356;
  t599 = 0. + t369;
  t1081 = 0.047*t1048;
  t1093 = 0. + t1081;
  t3041 = -1.*t2569;
  t3205 = 1. + t3041;
  t3316 = -0.047*t3205;
  t3329 = 0. + t3316;
  t3425 = t1146*t1106*t1111;
  t3593 = t622*t1163;
  t3636 = t3425 + t3593;
  t3722 = -1.*t1106*t622;
  t3812 = t1146*t1111*t1163;
  t3873 = t3722 + t3812;
  t3350 = -0.183*t3346;
  t3351 = 0. + t3350;
  t5582 = t2569*t3636;
  t6171 = -1.*t1048*t3873;
  t6220 = t5582 + t6171;
  t8759 = t2569*t1146*t621*t1106;
  t8762 = -1.*t1146*t621*t1048*t1163;
  t8763 = t8759 + t8762;
  t8828 = t2569*t621*t1106*t622;
  t8829 = -1.*t621*t1048*t622*t1163;
  t8830 = t8828 + t8829;
  t8859 = -1.*t2569*t1106*t1111;
  t8860 = t1048*t1111*t1163;
  t8863 = t8859 + t8860;
  t8877 = t1106*t622;
  t8881 = -1.*t1146*t1111*t1163;
  t8883 = t8877 + t8881;
  t8886 = -1.*t1048*t3636;
  t8887 = t2569*t8883;
  t8895 = t8886 + t8887;
  t8908 = t1106*t622*t1111;
  t8910 = -1.*t1146*t1163;
  t8911 = t8908 + t8910;
  t3343 = t2569*t3339;
  t8915 = -1.*t1048*t8911;
  t8917 = t8915 + t3343;
  t8932 = -1.*t621*t1106*t1048;
  t8935 = -1.*t2569*t621*t1163;
  t8936 = t8932 + t8935;
  t8951 = -1.*t2569*t3873;
  t8952 = t8886 + t8951;
  t8958 = t1146*t1106;
  t8959 = t622*t1111*t1163;
  t8960 = t8958 + t8959;
  t8966 = -1.*t2569*t8960;
  t8970 = t8915 + t8966;
  t8922 = t2569*t8911;
  t8937 = t3351*t8936;
  t8938 = 0.183*t3346*t8936;
  t8939 = t2569*t621*t1106;
  t8940 = -1.*t621*t1048*t1163;
  t8941 = t8939 + t8940;
  t8942 = -0.13205*t8941;
  t8974 = -1.*t1048*t8960;
  t8975 = t8922 + t8974;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1093*t1839 + t3329*t3339 - 0.13205*(t1048*t1839 + t3343) + t3351*t3354 - 1.*t599*t621*t622 - 0.183*(-1.*t3346*t3354 - 1.*t206*t621*t622);
  p_output1(10)=t1093*t3636 + t3329*t3873 - 0.13205*(t1048*t3636 + t2569*t3873) + t1146*t599*t621 + t3351*t6220 - 0.183*(t1146*t206*t621 - 1.*t3346*t6220);
  p_output1(11)=0;
  p_output1(12)=-1.*t1111*t1146*t599 + t1093*t1106*t1146*t621 + t1146*t1163*t3329*t621 - 0.13205*(t1048*t1106*t1146*t621 + t1146*t1163*t2569*t621) + t3351*t8763 - 0.183*(-1.*t1111*t1146*t206 - 1.*t3346*t8763);
  p_output1(13)=-1.*t1111*t599*t622 + t1093*t1106*t621*t622 + t1163*t3329*t621*t622 - 0.13205*(t1048*t1106*t621*t622 + t1163*t2569*t621*t622) + t3351*t8830 - 0.183*(-1.*t1111*t206*t622 - 1.*t3346*t8830);
  p_output1(14)=-1.*t1093*t1106*t1111 - 0.13205*(-1.*t1048*t1106*t1111 - 1.*t1111*t1163*t2569) - 1.*t1111*t1163*t3329 - 1.*t599*t621 + t3351*t8863 - 0.183*(-1.*t206*t621 - 1.*t3346*t8863);
  p_output1(15)=t3329*t3636 + t1093*t8883 - 0.13205*(t5582 + t1048*t8883) + 0.183*t3346*t8895 + t3351*t8895;
  p_output1(16)=t1093*t3339 + t3329*t8911 + 0.183*t3346*t8917 + t3351*t8917 - 0.13205*(t1048*t3339 + t8922);
  p_output1(17)=-1.*t1093*t1163*t621 + t1106*t3329*t621 + t8937 + t8938 + t8942;
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
  p_output1(36)=0.047*t2569*t3636 - 0.047*t1048*t3873 - 0.13205*t6220 + 0.183*t3346*t8952 + t3351*t8952;
  p_output1(37)=0.047*t2569*t8911 - 0.047*t1048*t8960 + 0.183*t3346*t8970 + t3351*t8970 - 0.13205*t8975;
  p_output1(38)=-0.047*t1048*t1163*t621 + 0.047*t1106*t2569*t621 + t8937 + t8938 + t8942;
  p_output1(39)=-0.183*t1146*t3346*t621 - 0.183*t206*t6220 - 0.183*(-1.*t1146*t3346*t621 - 1.*t206*t6220);
  p_output1(40)=-0.183*t3346*t621*t622 - 0.183*t206*t8975 - 0.183*(-1.*t3346*t621*t622 - 1.*t206*t8975);
  p_output1(41)=0.183*t1111*t3346 - 0.183*t206*t8941 - 0.183*(t1111*t3346 - 1.*t206*t8941);
  p_output1(42)=0;
  p_output1(43)=0;
  p_output1(44)=0;
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
  p_output1(48)=0;
  p_output1(49)=0;
  p_output1(50)=0;
  p_output1(51)=0;
  p_output1(52)=0;
  p_output1(53)=0;
}


       
Eigen::Matrix<double,3,18> Jp_RR_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void Jp_RR_thigh_joint(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



