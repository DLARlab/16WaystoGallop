/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:46 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FR_thigh_joint.h"

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
  double t1464;
  double t2979;
  double t3179;
  double t2791;
  double t3210;
  double t2284;
  double t3112;
  double t3516;
  double t3925;
  double t3932;
  double t3936;
  double t3940;
  double t4216;
  double t140;
  double t391;
  double t7918;
  double t7920;
  double t7921;
  double t7923;
  double t671;
  double t694;
  double t885;
  double t1110;
  double t2299;
  double t2555;
  double t2718;
  double t2768;
  double t4927;
  double t4945;
  double t7945;
  double t7946;
  double t7947;
  double t7949;
  double t7950;
  double t7952;
  double t7927;
  double t7929;
  double t7962;
  double t7963;
  double t7964;
  double t7980;
  double t7981;
  double t7984;
  double t8000;
  double t8001;
  double t8002;
  double t8018;
  double t8019;
  double t8020;
  double t8029;
  double t8030;
  double t8032;
  double t8039;
  double t8040;
  double t8041;
  double t8046;
  double t8047;
  double t8048;
  double t5202;
  double t8062;
  double t8063;
  double t8077;
  double t8079;
  double t8080;
  double t8087;
  double t8088;
  double t8051;
  double t8093;
  double t8094;
  double t8095;
  double t8100;
  double t8101;
  double t8070;
  double t8071;
  double t8072;
  double t8074;
  double t8081;
  double t8082;
  double t8097;
  double t8098;
  t1464 = Sin(var1[3]);
  t2979 = Cos(var1[5]);
  t3179 = Sin(var1[4]);
  t2791 = Cos(var1[3]);
  t3210 = Sin(var1[5]);
  t2284 = Cos(var1[6]);
  t3112 = -1.*t2791*t2979;
  t3516 = -1.*t1464*t3179*t3210;
  t3925 = t3112 + t3516;
  t3932 = -1.*t2979*t1464*t3179;
  t3936 = t2791*t3210;
  t3940 = t3932 + t3936;
  t4216 = Sin(var1[6]);
  t140 = Cos(var1[4]);
  t391 = Cos(var1[7]);
  t7918 = t2284*t3940;
  t7920 = -1.*t3925*t4216;
  t7921 = t7918 + t7920;
  t7923 = Sin(var1[7]);
  t671 = -1.*t391;
  t694 = 1. + t671;
  t885 = 0.183*t694;
  t1110 = 0. + t885;
  t2299 = -1.*t2284;
  t2555 = 1. + t2299;
  t2718 = -0.047*t2555;
  t2768 = 0. + t2718;
  t4927 = 0.047*t4216;
  t4945 = 0. + t4927;
  t7945 = -1.*t2979*t1464;
  t7946 = t2791*t3179*t3210;
  t7947 = t7945 + t7946;
  t7949 = t2791*t2979*t3179;
  t7950 = t1464*t3210;
  t7952 = t7949 + t7950;
  t7927 = 0.183*t7923;
  t7929 = 0. + t7927;
  t7962 = t2284*t7952;
  t7963 = -1.*t7947*t4216;
  t7964 = t7962 + t7963;
  t7980 = t2791*t140*t2979*t2284;
  t7981 = -1.*t2791*t140*t3210*t4216;
  t7984 = t7980 + t7981;
  t8000 = t140*t2979*t2284*t1464;
  t8001 = -1.*t140*t1464*t3210*t4216;
  t8002 = t8000 + t8001;
  t8018 = -1.*t2979*t2284*t3179;
  t8019 = t3179*t3210*t4216;
  t8020 = t8018 + t8019;
  t8029 = t2979*t1464;
  t8030 = -1.*t2791*t3179*t3210;
  t8032 = t8029 + t8030;
  t8039 = t2284*t8032;
  t8040 = -1.*t7952*t4216;
  t8041 = t8039 + t8040;
  t8046 = t2979*t1464*t3179;
  t8047 = -1.*t2791*t3210;
  t8048 = t8046 + t8047;
  t5202 = t2284*t3925;
  t8062 = -1.*t8048*t4216;
  t8063 = t5202 + t8062;
  t8077 = -1.*t140*t2284*t3210;
  t8079 = -1.*t140*t2979*t4216;
  t8080 = t8077 + t8079;
  t8087 = -1.*t2284*t7947;
  t8088 = t8087 + t8040;
  t8051 = t2284*t8048;
  t8093 = t2791*t2979;
  t8094 = t1464*t3179*t3210;
  t8095 = t8093 + t8094;
  t8100 = -1.*t2284*t8095;
  t8101 = t8100 + t8062;
  t8070 = t140*t2979*t2284;
  t8071 = -1.*t140*t3210*t4216;
  t8072 = t8070 + t8071;
  t8074 = -0.13205*t8072;
  t8081 = t8080*t7929;
  t8082 = -0.183*t8080*t7923;
  t8097 = -1.*t8095*t4216;
  t8098 = t8051 + t8097;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=-1.*t1110*t140*t1464 + t2768*t3925 + t3940*t4945 - 0.13205*(t3940*t4216 + t5202) + 0.183*(-1.*t140*t1464*t391 - 1.*t7921*t7923) + t7921*t7929;
  p_output1(10)=t1110*t140*t2791 + t2768*t7947 + t4945*t7952 - 0.13205*(t2284*t7947 + t4216*t7952) + t7929*t7964 + 0.183*(t140*t2791*t391 - 1.*t7923*t7964);
  p_output1(11)=0;
  p_output1(12)=-1.*t1110*t2791*t3179 + t140*t2768*t2791*t3210 - 0.13205*(t140*t2284*t2791*t3210 + t140*t2791*t2979*t4216) + t140*t2791*t2979*t4945 + t7929*t7984 + 0.183*(-1.*t2791*t3179*t391 - 1.*t7923*t7984);
  p_output1(13)=-1.*t1110*t1464*t3179 + t140*t1464*t2768*t3210 - 0.13205*(t140*t1464*t2284*t3210 + t140*t1464*t2979*t4216) + t140*t1464*t2979*t4945 + t7929*t8002 + 0.183*(-1.*t1464*t3179*t391 - 1.*t7923*t8002);
  p_output1(14)=-1.*t1110*t140 - 1.*t2768*t3179*t3210 - 0.13205*(-1.*t2284*t3179*t3210 - 1.*t2979*t3179*t4216) - 1.*t2979*t3179*t4945 + t7929*t8020 + 0.183*(-1.*t140*t391 - 1.*t7923*t8020);
  p_output1(15)=t2768*t7952 + t4945*t8032 - 0.13205*(t7962 + t4216*t8032) - 0.183*t7923*t8041 + t7929*t8041;
  p_output1(16)=t3925*t4945 + t2768*t8048 - 0.13205*(t3925*t4216 + t8051) - 0.183*t7923*t8063 + t7929*t8063;
  p_output1(17)=t140*t2768*t2979 - 1.*t140*t3210*t4945 + t8074 + t8081 + t8082;
  p_output1(18)=-0.047*t4216*t7947 + 0.047*t2284*t7952 - 0.13205*t7964 - 0.183*t7923*t8088 + t7929*t8088;
  p_output1(19)=0.047*t2284*t8048 - 0.047*t4216*t8095 - 0.13205*t8098 - 0.183*t7923*t8101 + t7929*t8101;
  p_output1(20)=0.047*t140*t2284*t2979 - 0.047*t140*t3210*t4216 + t8074 + t8081 + t8082;
  p_output1(21)=0.183*t140*t2791*t7923 + 0.183*t391*t7964 + 0.183*(-1.*t140*t2791*t7923 - 1.*t391*t7964);
  p_output1(22)=0.183*t140*t1464*t7923 + 0.183*t391*t8098 + 0.183*(-1.*t140*t1464*t7923 - 1.*t391*t8098);
  p_output1(23)=-0.183*t3179*t7923 + 0.183*t391*t8072 + 0.183*(t3179*t7923 - 1.*t391*t8072);
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


       
Eigen::Matrix<double,3,18> Jp_FR_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void Jp_FR_thigh_joint(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



