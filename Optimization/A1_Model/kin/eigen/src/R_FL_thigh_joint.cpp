/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:59 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_FL_thigh_joint.h"

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
  double t722;
  double t2902;
  double t8270;
  double t3083;
  double t8325;
  double t543;
  double t731;
  double t1740;
  double t2880;
  double t8432;
  double t5474;
  double t8331;
  double t8429;
  double t8447;
  double t8448;
  double t8451;
  double t8464;
  double t8469;
  double t8470;
  double t8472;
  double t8477;
  double t8478;
  double t8431;
  double t8455;
  double t8457;
  double t8471;
  double t8479;
  double t8480;
  double t8487;
  double t8488;
  double t8489;
  t722 = Cos(var1[3]);
  t2902 = Cos(var1[5]);
  t8270 = Sin(var1[3]);
  t3083 = Sin(var1[4]);
  t8325 = Sin(var1[5]);
  t543 = Cos(var1[10]);
  t731 = Cos(var1[4]);
  t1740 = Sin(var1[10]);
  t2880 = Cos(var1[9]);
  t8432 = Sin(var1[9]);
  t5474 = t722*t2902*t3083;
  t8331 = t8270*t8325;
  t8429 = t5474 + t8331;
  t8447 = -1.*t2902*t8270;
  t8448 = t722*t3083*t8325;
  t8451 = t8447 + t8448;
  t8464 = t2902*t8270*t3083;
  t8469 = -1.*t722*t8325;
  t8470 = t8464 + t8469;
  t8472 = t722*t2902;
  t8477 = t8270*t3083*t8325;
  t8478 = t8472 + t8477;
  t8431 = t2880*t8429;
  t8455 = -1.*t8432*t8451;
  t8457 = t8431 + t8455;
  t8471 = t2880*t8470;
  t8479 = -1.*t8432*t8478;
  t8480 = t8471 + t8479;
  t8487 = t2880*t731*t2902;
  t8488 = -1.*t731*t8432*t8325;
  t8489 = t8487 + t8488;

  p_output1(0)=t543*t722*t731 - 1.*t1740*t8457;
  p_output1(1)=t543*t731*t8270 - 1.*t1740*t8480;
  p_output1(2)=-1.*t3083*t543 - 1.*t1740*t8489;
  p_output1(3)=t8429*t8432 + t2880*t8451;
  p_output1(4)=t8432*t8470 + t2880*t8478;
  p_output1(5)=t2880*t731*t8325 + t2902*t731*t8432;
  p_output1(6)=t1740*t722*t731 + t543*t8457;
  p_output1(7)=t1740*t731*t8270 + t543*t8480;
  p_output1(8)=-1.*t1740*t3083 + t543*t8489;
}


       
Eigen::Matrix<double,3,3> R_FL_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void R_FL_thigh_joint(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



