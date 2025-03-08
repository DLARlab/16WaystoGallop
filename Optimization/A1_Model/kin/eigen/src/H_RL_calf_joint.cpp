/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:25 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RL_calf_joint.h"

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
  double t58;
  double t1079;
  double t1155;
  double t1142;
  double t1162;
  double t476;
  double t127;
  double t144;
  double t954;
  double t1144;
  double t1205;
  double t1303;
  double t1401;
  double t1425;
  double t1429;
  double t1460;
  double t1617;
  double t1813;
  double t1830;
  double t45;
  double t1939;
  double t2473;
  double t2485;
  double t2496;
  double t2503;
  double t2692;
  double t2754;
  double t2761;
  double t2766;
  double t2767;
  double t3220;
  double t3304;
  double t3307;
  double t200;
  double t1918;
  double t1925;
  double t2052;
  double t2062;
  double t2126;
  double t2469;
  double t2831;
  double t2884;
  double t2977;
  double t2980;
  double t2993;
  double t3130;
  double t3312;
  double t3320;
  double t3323;
  double t3324;
  double t3332;
  double t3560;
  double t3573;
  double t3659;
  double t4464;
  double t4506;
  double t1928;
  double t2424;
  double t2453;
  double t3904;
  double t3938;
  double t3945;
  double t4252;
  double t4254;
  double t4256;
  double t4257;
  double t4264;
  double t4266;
  double t4318;
  double t4326;
  double t4356;
  double t4363;
  double t3673;
  double t3716;
  double t3759;
  double t4379;
  double t4427;
  double t4565;
  double t4606;
  double t4718;
  double t4881;
  double t4892;
  double t4957;
  double t2968;
  double t3013;
  double t3026;
  double t4009;
  double t4016;
  double t4193;
  double t3841;
  double t3842;
  double t3866;
  double t3322;
  double t3426;
  double t3470;
  double t4244;
  double t4246;
  double t4248;
  t58 = Cos(var1[3]);
  t1079 = Cos(var1[5]);
  t1155 = Sin(var1[3]);
  t1142 = Sin(var1[4]);
  t1162 = Sin(var1[5]);
  t476 = Cos(var1[16]);
  t127 = Cos(var1[4]);
  t144 = Sin(var1[16]);
  t954 = Cos(var1[15]);
  t1144 = t58*t1079*t1142;
  t1205 = t1155*t1162;
  t1303 = t1144 + t1205;
  t1401 = t954*t1303;
  t1425 = Sin(var1[15]);
  t1429 = -1.*t1079*t1155;
  t1460 = t58*t1142*t1162;
  t1617 = t1429 + t1460;
  t1813 = -1.*t1425*t1617;
  t1830 = t1401 + t1813;
  t45 = Sin(var1[17]);
  t1939 = Cos(var1[17]);
  t2473 = t1079*t1155*t1142;
  t2485 = -1.*t58*t1162;
  t2496 = t2473 + t2485;
  t2503 = t954*t2496;
  t2692 = t58*t1079;
  t2754 = t1155*t1142*t1162;
  t2761 = t2692 + t2754;
  t2766 = -1.*t1425*t2761;
  t2767 = t2503 + t2766;
  t3220 = t954*t127*t1079;
  t3304 = -1.*t127*t1425*t1162;
  t3307 = t3220 + t3304;
  t200 = t58*t127*t144;
  t1918 = t476*t1830;
  t1925 = t200 + t1918;
  t2052 = t476*t58*t127;
  t2062 = -1.*t144*t1830;
  t2126 = t2052 + t2062;
  t2469 = t127*t144*t1155;
  t2831 = t476*t2767;
  t2884 = t2469 + t2831;
  t2977 = t476*t127*t1155;
  t2980 = -1.*t144*t2767;
  t2993 = t2977 + t2980;
  t3130 = -1.*t144*t1142;
  t3312 = t476*t3307;
  t3320 = t3130 + t3312;
  t3323 = -1.*t476*t1142;
  t3324 = -1.*t144*t3307;
  t3332 = t3323 + t3324;
  t3560 = t1425*t1303;
  t3573 = t954*t1617;
  t3659 = t3560 + t3573;
  t4464 = -1.*t1939;
  t4506 = 1. + t4464;
  t1928 = -1.*t45*t1925;
  t2424 = t1939*t2126;
  t2453 = t1928 + t2424;
  t3904 = t1939*t1925;
  t3938 = t45*t2126;
  t3945 = t3904 + t3938;
  t4252 = -1.*t476;
  t4254 = 1. + t4252;
  t4256 = -0.183*t4254;
  t4257 = 0. + t4256;
  t4264 = -0.047*t1425;
  t4266 = 0. + t4264;
  t4318 = -1.*t954;
  t4326 = 1. + t4318;
  t4356 = 0.047*t4326;
  t4363 = 0. + t4356;
  t3673 = t1425*t2496;
  t3716 = t954*t2761;
  t3759 = t3673 + t3716;
  t4379 = -0.183*t144;
  t4427 = 0. + t4379;
  t4565 = -0.2*t4506;
  t4606 = -0.183*t45;
  t4718 = 0. + t4565 + t4606;
  t4881 = -0.183*t4506;
  t4892 = 0.2*t45;
  t4957 = 0. + t4881 + t4892;
  t2968 = -1.*t45*t2884;
  t3013 = t1939*t2993;
  t3026 = t2968 + t3013;
  t4009 = t1939*t2884;
  t4016 = t45*t2993;
  t4193 = t4009 + t4016;
  t3841 = t127*t1079*t1425;
  t3842 = t954*t127*t1162;
  t3866 = t3841 + t3842;
  t3322 = -1.*t45*t3320;
  t3426 = t1939*t3332;
  t3470 = t3322 + t3426;
  t4244 = t1939*t3320;
  t4246 = t45*t3332;
  t4248 = t4244 + t4246;

  p_output1(0)=t2453;
  p_output1(1)=t3026;
  p_output1(2)=t3470;
  p_output1(3)=0.;
  p_output1(4)=t3659;
  p_output1(5)=t3759;
  p_output1(6)=t3866;
  p_output1(7)=0.;
  p_output1(8)=t3945;
  p_output1(9)=t4193;
  p_output1(10)=t4248;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.183*t2453 + 0.13205*t3659 - 0.2*t3945 + t1303*t4266 + t1617*t4363 + t1830*t4427 + t1925*t4718 + t2126*t4957 + t127*t4257*t58 + var1(0);
  p_output1(13)=0. - 0.183*t3026 + 0.13205*t3759 - 0.2*t4193 + t1155*t127*t4257 + t2496*t4266 + t2761*t4363 + t2767*t4427 + t2884*t4718 + t2993*t4957 + var1(1);
  p_output1(14)=0. - 0.183*t3470 + 0.13205*t3866 - 0.2*t4248 - 1.*t1142*t4257 + t1079*t127*t4266 + t1162*t127*t4363 + t3307*t4427 + t3320*t4718 + t3332*t4957 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_RL_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void H_RL_calf_joint(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



