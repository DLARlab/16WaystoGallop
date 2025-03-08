/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:30 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_FRfoot.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t447;
  double t1582;
  double t2785;
  double t1595;
  double t2823;
  double t713;
  double t727;
  double t2784;
  double t3228;
  double t3468;
  double t3726;
  double t3736;
  double t4198;
  double t4280;
  double t4322;
  double t4462;
  double t4480;
  double t545;
  double t4885;
  double t437;
  double t5332;
  double t5335;
  double t5336;
  double t5339;
  double t5344;
  double t5346;
  double t5348;
  double t5350;
  double t5352;
  double t5309;
  double t5366;
  double t5369;
  double t5370;
  double t5288;
  double t5289;
  double t5308;
  double t718;
  double t5133;
  double t5278;
  double t5358;
  double t5359;
  double t5360;
  double t5327;
  double t5355;
  double t5356;
  double t5383;
  double t5384;
  double t5385;
  double t5363;
  double t5375;
  double t5377;
  t447 = Cos(var1[3]);
  t1582 = Cos(var1[5]);
  t2785 = Sin(var1[3]);
  t1595 = Sin(var1[4]);
  t2823 = Sin(var1[5]);
  t713 = Cos(var1[7]);
  t727 = Cos(var1[6]);
  t2784 = t447*t1582*t1595;
  t3228 = t2785*t2823;
  t3468 = t2784 + t3228;
  t3726 = t727*t3468;
  t3736 = -1.*t1582*t2785;
  t4198 = t447*t1595*t2823;
  t4280 = t3736 + t4198;
  t4322 = Sin(var1[6]);
  t4462 = -1.*t4280*t4322;
  t4480 = t3726 + t4462;
  t545 = Cos(var1[4]);
  t4885 = Sin(var1[7]);
  t437 = Cos(var1[8]);
  t5332 = t1582*t2785*t1595;
  t5335 = -1.*t447*t2823;
  t5336 = t5332 + t5335;
  t5339 = t727*t5336;
  t5344 = t447*t1582;
  t5346 = t2785*t1595*t2823;
  t5348 = t5344 + t5346;
  t5350 = -1.*t5348*t4322;
  t5352 = t5339 + t5350;
  t5309 = Sin(var1[8]);
  t5366 = t545*t1582*t727;
  t5369 = -1.*t545*t2823*t4322;
  t5370 = t5366 + t5369;
  t5288 = t713*t4480;
  t5289 = t447*t545*t4885;
  t5308 = t5288 + t5289;
  t718 = t447*t545*t713;
  t5133 = -1.*t4480*t4885;
  t5278 = t718 + t5133;
  t5358 = t713*t5352;
  t5359 = t545*t2785*t4885;
  t5360 = t5358 + t5359;
  t5327 = t545*t713*t2785;
  t5355 = -1.*t5352*t4885;
  t5356 = t5327 + t5355;
  t5383 = t713*t5370;
  t5384 = -1.*t1595*t4885;
  t5385 = t5383 + t5384;
  t5363 = -1.*t713*t1595;
  t5375 = -1.*t5370*t4885;
  t5377 = t5363 + t5375;

  p_output1(0)=t437*t5278 - 1.*t5308*t5309;
  p_output1(1)=t437*t5356 - 1.*t5309*t5360;
  p_output1(2)=t437*t5377 - 1.*t5309*t5385;
  p_output1(3)=t3468*t4322 + t4280*t727;
  p_output1(4)=t4322*t5336 + t5348*t727;
  p_output1(5)=t1582*t4322*t545 + t2823*t545*t727;
  p_output1(6)=t437*t5308 + t5278*t5309;
  p_output1(7)=t5309*t5356 + t437*t5360;
  p_output1(8)=t5309*t5377 + t437*t5385;
}


       
Eigen::Matrix<double,3,3> R_FRfoot(const Eigen::Matrix<double,18,1> &var1)
//void R_FRfoot(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



