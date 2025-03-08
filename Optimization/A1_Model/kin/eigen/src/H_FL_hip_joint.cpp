/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:55 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FL_hip_joint.h"

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
  double t819;
  double t592;
  double t991;
  double t966;
  double t1275;
  double t1297;
  double t1257;
  double t1309;
  double t1294;
  double t1299;
  double t1300;
  double t1310;
  double t1311;
  double t1313;
  double t1317;
  double t1318;
  double t1322;
  double t1324;
  double t1325;
  double t1328;
  double t1302;
  double t1314;
  double t1316;
  double t1811;
  double t1820;
  double t1841;
  double t1845;
  double t1923;
  double t2090;
  double t1323;
  double t1329;
  double t1330;
  double t1331;
  double t1332;
  double t1333;
  t819 = Cos(var1[4]);
  t592 = Cos(var1[3]);
  t991 = Sin(var1[4]);
  t966 = Sin(var1[3]);
  t1275 = Cos(var1[5]);
  t1297 = Sin(var1[5]);
  t1257 = Sin(var1[9]);
  t1309 = Cos(var1[9]);
  t1294 = t592*t1275*t991;
  t1299 = t966*t1297;
  t1300 = t1294 + t1299;
  t1310 = -1.*t1275*t966;
  t1311 = t592*t991*t1297;
  t1313 = t1310 + t1311;
  t1317 = t1275*t966*t991;
  t1318 = -1.*t592*t1297;
  t1322 = t1317 + t1318;
  t1324 = t592*t1275;
  t1325 = t966*t991*t1297;
  t1328 = t1324 + t1325;
  t1302 = t1257*t1300;
  t1314 = t1309*t1313;
  t1316 = t1302 + t1314;
  t1811 = -0.047*t1257;
  t1820 = 0. + t1811;
  t1841 = -1.*t1309;
  t1845 = 1. + t1841;
  t1923 = 0.047*t1845;
  t2090 = 0. + t1923;
  t1323 = t1257*t1322;
  t1329 = t1309*t1328;
  t1330 = t1323 + t1329;
  t1331 = t819*t1275*t1257;
  t1332 = t1309*t819*t1297;
  t1333 = t1331 + t1332;

  p_output1(0)=t592*t819;
  p_output1(1)=t819*t966;
  p_output1(2)=-1.*t991;
  p_output1(3)=0.;
  p_output1(4)=t1316;
  p_output1(5)=t1330;
  p_output1(6)=t1333;
  p_output1(7)=0.;
  p_output1(8)=t1300*t1309 - 1.*t1257*t1313;
  p_output1(9)=t1309*t1322 - 1.*t1257*t1328;
  p_output1(10)=-1.*t1257*t1297*t819 + t1275*t1309*t819;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.047*t1316 + t1300*t1820 + t1313*t2090 + 0.183*t592*t819 + var1(0);
  p_output1(13)=0. + 0.047*t1330 + t1322*t1820 + t1328*t2090 + 0.183*t819*t966 + var1(1);
  p_output1(14)=0. + 0.047*t1333 + t1275*t1820*t819 + t1297*t2090*t819 - 0.183*t991 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_FL_hip_joint(const Eigen::Matrix<double,18,1> &var1)
//void H_FL_hip_joint(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



