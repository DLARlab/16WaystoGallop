/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:12 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RR_calf_joint.h"

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
  double t993;
  double t1654;
  double t1529;
  double t1574;
  double t1748;
  double t1014;
  double t1644;
  double t1783;
  double t1787;
  double t1828;
  double t2255;
  double t2262;
  double t2294;
  double t981;
  double t2549;
  double t162;
  double t2698;
  double t2704;
  double t2742;
  double t2819;
  double t2837;
  double t2885;
  double t3155;
  double t3896;
  double t3939;
  double t3941;
  double t4475;
  double t4476;
  double t4522;
  double t363;
  double t471;
  double t595;
  double t765;
  double t1032;
  double t1326;
  double t2165;
  double t2166;
  double t2231;
  double t2234;
  double t4794;
  double t4901;
  double t4926;
  double t4958;
  double t5033;
  double t5034;
  double t2581;
  double t2653;
  double t2906;
  double t3158;
  double t3617;
  double t5553;
  double t5765;
  double t5914;
  double t4308;
  double t4380;
  double t4454;
  double t6236;
  double t6289;
  double t6382;
  double t6484;
  double t6522;
  double t7237;
  double t8921;
  double t8922;
  double t8924;
  double t8930;
  double t8936;
  double t8948;
  double t8954;
  double t8955;
  double t8956;
  double t9015;
  double t9019;
  double t9020;
  double t9022;
  double t9023;
  double t9024;
  double t9026;
  double t9027;
  double t9028;
  double t9053;
  double t9054;
  double t9055;
  double t9057;
  double t9058;
  double t9061;
  double t9065;
  double t9069;
  double t9070;
  double t9082;
  double t9083;
  double t9084;
  double t9087;
  double t9089;
  double t9091;
  double t9113;
  double t9115;
  double t9116;
  double t2419;
  double t9119;
  double t9120;
  double t9143;
  double t9144;
  double t9145;
  double t9180;
  double t9181;
  double t9199;
  double t9200;
  double t9203;
  double t9205;
  double t9206;
  double t9124;
  double t9146;
  double t9147;
  double t9148;
  double t9149;
  double t9150;
  double t9151;
  double t9152;
  double t9153;
  double t9154;
  double t9161;
  double t9167;
  double t9169;
  double t9171;
  double t9173;
  double t9174;
  double t9230;
  double t9231;
  double t9232;
  double t8756;
  double t9210;
  double t9211;
  double t9250;
  double t9251;
  double t9254;
  double t9258;
  double t9259;
  double t9274;
  double t9275;
  double t9276;
  double t9278;
  double t9279;
  double t9280;
  double t8752;
  double t8789;
  double t9243;
  double t9293;
  double t9295;
  double t9296;
  double t9298;
  double t9299;
  double t9306;
  double t9307;
  double t9308;
  double t9263;
  double t9268;
  double t9318;
  double t9319;
  double t9320;
  double t9284;
  double t9288;
  t993 = Sin(var1[3]);
  t1654 = Cos(var1[3]);
  t1529 = Cos(var1[5]);
  t1574 = Sin(var1[4]);
  t1748 = Sin(var1[5]);
  t1014 = Sin(var1[12]);
  t1644 = -1.*t1529*t993*t1574;
  t1783 = t1654*t1748;
  t1787 = t1644 + t1783;
  t1828 = Cos(var1[12]);
  t2255 = -1.*t1654*t1529;
  t2262 = -1.*t993*t1574*t1748;
  t2294 = t2255 + t2262;
  t981 = Cos(var1[4]);
  t2549 = Sin(var1[13]);
  t162 = Cos(var1[13]);
  t2698 = t1828*t1787;
  t2704 = -1.*t1014*t2294;
  t2742 = t2698 + t2704;
  t2819 = Cos(var1[14]);
  t2837 = -1.*t2819;
  t2885 = 1. + t2837;
  t3155 = Sin(var1[14]);
  t3896 = -1.*t981*t2549*t993;
  t3939 = t162*t2742;
  t3941 = t3896 + t3939;
  t4475 = -1.*t162*t981*t993;
  t4476 = -1.*t2549*t2742;
  t4522 = t4475 + t4476;
  t363 = -1.*t162;
  t471 = 1. + t363;
  t595 = -0.183*t471;
  t765 = 0. + t595;
  t1032 = 0.047*t1014;
  t1326 = 0. + t1032;
  t2165 = -1.*t1828;
  t2166 = 1. + t2165;
  t2231 = -0.047*t2166;
  t2234 = 0. + t2231;
  t4794 = t1654*t1529*t1574;
  t4901 = t993*t1748;
  t4926 = t4794 + t4901;
  t4958 = -1.*t1529*t993;
  t5033 = t1654*t1574*t1748;
  t5034 = t4958 + t5033;
  t2581 = -0.183*t2549;
  t2653 = 0. + t2581;
  t2906 = -0.2*t2885;
  t3158 = -0.183*t3155;
  t3617 = 0. + t2906 + t3158;
  t5553 = t1828*t4926;
  t5765 = -1.*t1014*t5034;
  t5914 = t5553 + t5765;
  t4308 = -0.183*t2885;
  t4380 = 0.2*t3155;
  t4454 = 0. + t4308 + t4380;
  t6236 = t1654*t981*t2549;
  t6289 = t162*t5914;
  t6382 = t6236 + t6289;
  t6484 = t162*t1654*t981;
  t6522 = -1.*t2549*t5914;
  t7237 = t6484 + t6522;
  t8921 = t1828*t1654*t981*t1529;
  t8922 = -1.*t1654*t981*t1014*t1748;
  t8924 = t8921 + t8922;
  t8930 = -1.*t1654*t2549*t1574;
  t8936 = t162*t8924;
  t8948 = t8930 + t8936;
  t8954 = -1.*t162*t1654*t1574;
  t8955 = -1.*t2549*t8924;
  t8956 = t8954 + t8955;
  t9015 = t1828*t981*t1529*t993;
  t9019 = -1.*t981*t1014*t993*t1748;
  t9020 = t9015 + t9019;
  t9022 = -1.*t2549*t993*t1574;
  t9023 = t162*t9020;
  t9024 = t9022 + t9023;
  t9026 = -1.*t162*t993*t1574;
  t9027 = -1.*t2549*t9020;
  t9028 = t9026 + t9027;
  t9053 = -1.*t1828*t1529*t1574;
  t9054 = t1014*t1574*t1748;
  t9055 = t9053 + t9054;
  t9057 = -1.*t981*t2549;
  t9058 = t162*t9055;
  t9061 = t9057 + t9058;
  t9065 = -1.*t162*t981;
  t9069 = -1.*t2549*t9055;
  t9070 = t9065 + t9069;
  t9082 = t1529*t993;
  t9083 = -1.*t1654*t1574*t1748;
  t9084 = t9082 + t9083;
  t9087 = -1.*t1014*t4926;
  t9089 = t1828*t9084;
  t9091 = t9087 + t9089;
  t9113 = t1529*t993*t1574;
  t9115 = -1.*t1654*t1748;
  t9116 = t9113 + t9115;
  t2419 = t1828*t2294;
  t9119 = -1.*t1014*t9116;
  t9120 = t9119 + t2419;
  t9143 = -1.*t981*t1529*t1014;
  t9144 = -1.*t1828*t981*t1748;
  t9145 = t9143 + t9144;
  t9180 = -1.*t1828*t5034;
  t9181 = t9087 + t9180;
  t9199 = t1654*t1529;
  t9200 = t993*t1574*t1748;
  t9203 = t9199 + t9200;
  t9205 = -1.*t1828*t9203;
  t9206 = t9119 + t9205;
  t9124 = t1828*t9116;
  t9146 = t2653*t9145;
  t9147 = t162*t3617*t9145;
  t9148 = -1.*t2549*t4454*t9145;
  t9149 = t1828*t981*t1529;
  t9150 = -1.*t981*t1014*t1748;
  t9151 = t9149 + t9150;
  t9152 = -0.13205*t9151;
  t9153 = -1.*t2819*t2549*t9145;
  t9154 = -1.*t162*t3155*t9145;
  t9161 = t9153 + t9154;
  t9167 = -0.183*t9161;
  t9169 = t162*t2819*t9145;
  t9171 = -1.*t2549*t3155*t9145;
  t9173 = t9169 + t9171;
  t9174 = -0.2*t9173;
  t9230 = -1.*t1654*t981*t2549;
  t9231 = -1.*t162*t5914;
  t9232 = t9230 + t9231;
  t8756 = t2819*t7237;
  t9210 = -1.*t1014*t9203;
  t9211 = t9124 + t9210;
  t9250 = -1.*t162*t9211;
  t9251 = t3896 + t9250;
  t9254 = t162*t981*t993;
  t9258 = -1.*t2549*t9211;
  t9259 = t9254 + t9258;
  t9274 = t2549*t1574;
  t9275 = -1.*t162*t9151;
  t9276 = t9274 + t9275;
  t9278 = -1.*t162*t1574;
  t9279 = -1.*t2549*t9151;
  t9280 = t9278 + t9279;
  t8752 = -1.*t3155*t6382;
  t8789 = t8752 + t8756;
  t9243 = -1.*t3155*t7237;
  t9293 = -0.183*t2819;
  t9295 = -0.2*t3155;
  t9296 = t9293 + t9295;
  t9298 = 0.2*t2819;
  t9299 = t9298 + t3158;
  t9306 = t981*t2549*t993;
  t9307 = t162*t9211;
  t9308 = t9306 + t9307;
  t9263 = t2819*t9259;
  t9268 = -1.*t3155*t9259;
  t9318 = -1.*t2549*t1574;
  t9319 = t162*t9151;
  t9320 = t9318 + t9319;
  t9284 = t2819*t9280;
  t9288 = -1.*t3155*t9280;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1326*t1787 + t2234*t2294 - 0.13205*(t1014*t1787 + t2419) + t2653*t2742 + t3617*t3941 + t4454*t4522 - 0.183*(-1.*t3155*t3941 + t2819*t4522) - 0.2*(t2819*t3941 + t3155*t4522) - 1.*t765*t981*t993;
  p_output1(10)=t1326*t4926 + t2234*t5034 - 0.13205*(t1014*t4926 + t1828*t5034) + t2653*t5914 + t3617*t6382 + t4454*t7237 - 0.2*(t2819*t6382 + t3155*t7237) - 0.183*t8789 + t1654*t765*t981;
  p_output1(11)=0;
  p_output1(12)=-1.*t1574*t1654*t765 + t2653*t8924 + t3617*t8948 + t4454*t8956 - 0.183*(-1.*t3155*t8948 + t2819*t8956) - 0.2*(t2819*t8948 + t3155*t8956) + t1326*t1529*t1654*t981 + t1654*t1748*t2234*t981 - 0.13205*(t1014*t1529*t1654*t981 + t1654*t1748*t1828*t981);
  p_output1(13)=t2653*t9020 + t3617*t9024 + t4454*t9028 - 0.183*(-1.*t3155*t9024 + t2819*t9028) - 0.2*(t2819*t9024 + t3155*t9028) - 1.*t1574*t765*t993 + t1326*t1529*t981*t993 + t1748*t2234*t981*t993 - 0.13205*(t1014*t1529*t981*t993 + t1748*t1828*t981*t993);
  p_output1(14)=-1.*t1326*t1529*t1574 - 0.13205*(-1.*t1014*t1529*t1574 - 1.*t1574*t1748*t1828) - 1.*t1574*t1748*t2234 + t2653*t9055 + t3617*t9061 + t4454*t9070 - 0.183*(-1.*t3155*t9061 + t2819*t9070) - 0.2*(t2819*t9061 + t3155*t9070) - 1.*t765*t981;
  p_output1(15)=t2234*t4926 + t1326*t9084 - 0.13205*(t5553 + t1014*t9084) + t2653*t9091 + t162*t3617*t9091 - 1.*t2549*t4454*t9091 - 0.183*(-1.*t2549*t2819*t9091 - 1.*t162*t3155*t9091) - 0.2*(t162*t2819*t9091 - 1.*t2549*t3155*t9091);
  p_output1(16)=t1326*t2294 + t2234*t9116 + t2653*t9120 + t162*t3617*t9120 - 1.*t2549*t4454*t9120 - 0.183*(-1.*t2549*t2819*t9120 - 1.*t162*t3155*t9120) - 0.2*(t162*t2819*t9120 - 1.*t2549*t3155*t9120) - 0.13205*(t1014*t2294 + t9124);
  p_output1(17)=t9146 + t9147 + t9148 + t9152 + t9167 + t9174 - 1.*t1326*t1748*t981 + t1529*t2234*t981;
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
  p_output1(36)=0.047*t1828*t4926 - 0.047*t1014*t5034 - 0.13205*t5914 + t2653*t9181 + t162*t3617*t9181 - 1.*t2549*t4454*t9181 - 0.183*(-1.*t2549*t2819*t9181 - 1.*t162*t3155*t9181) - 0.2*(t162*t2819*t9181 - 1.*t2549*t3155*t9181);
  p_output1(37)=0.047*t1828*t9116 - 0.047*t1014*t9203 + t2653*t9206 + t162*t3617*t9206 - 1.*t2549*t4454*t9206 - 0.183*(-1.*t2549*t2819*t9206 - 1.*t162*t3155*t9206) - 0.2*(t162*t2819*t9206 - 1.*t2549*t3155*t9206) - 0.13205*t9211;
  p_output1(38)=t9146 + t9147 + t9148 + t9152 + t9167 + t9174 - 0.047*t1014*t1748*t981 + 0.047*t1529*t1828*t981;
  p_output1(39)=-0.183*t162*t5914 + t3617*t7237 + t4454*t9232 - 0.2*(t8756 + t3155*t9232) - 0.183*(t2819*t9232 + t9243) - 0.183*t1654*t2549*t981;
  p_output1(40)=-0.183*t162*t9211 + t4454*t9251 + t3617*t9259 - 0.2*(t3155*t9251 + t9263) - 0.183*(t2819*t9251 + t9268) - 0.183*t2549*t981*t993;
  p_output1(41)=0.183*t1574*t2549 - 0.183*t162*t9151 + t4454*t9276 + t3617*t9280 - 0.2*(t3155*t9276 + t9284) - 0.183*(t2819*t9276 + t9288);
  p_output1(42)=-0.2*t8789 - 0.183*(-1.*t2819*t6382 + t9243) + t6382*t9296 + t7237*t9299;
  p_output1(43)=t9259*t9299 + t9296*t9308 - 0.183*(t9268 - 1.*t2819*t9308) - 0.2*(t9263 - 1.*t3155*t9308);
  p_output1(44)=t9280*t9299 + t9296*t9320 - 0.183*(t9288 - 1.*t2819*t9320) - 0.2*(t9284 - 1.*t3155*t9320);
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


       
Eigen::Matrix<double,3,18> Jp_RR_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void Jp_RR_calf_joint(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



