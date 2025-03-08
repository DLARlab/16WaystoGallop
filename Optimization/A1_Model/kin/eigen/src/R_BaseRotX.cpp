/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:41 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_BaseRotX.h"

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
  double t1642;
  double t3359;
  double t1629;
  double t7770;
  double t7773;
  double t7775;
  t1642 = Cos(var1[4]);
  t3359 = Sin(var1[3]);
  t1629 = Cos(var1[3]);
  t7770 = Sin(var1[4]);
  t7773 = Cos(var1[5]);
  t7775 = Sin(var1[5]);

  p_output1(0)=t1629*t1642;
  p_output1(1)=t1642*t3359;
  p_output1(2)=-1.*t7770;
  p_output1(3)=-1.*t3359*t7773 + t1629*t7770*t7775;
  p_output1(4)=t1629*t7773 + t3359*t7770*t7775;
  p_output1(5)=t1642*t7775;
  p_output1(6)=t1629*t7770*t7773 + t3359*t7775;
  p_output1(7)=t3359*t7770*t7773 - 1.*t1629*t7775;
  p_output1(8)=t1642*t7773;
}


       
Eigen::Matrix<double,3,3> R_BaseRotX(const Eigen::Matrix<double,18,1> &var1)
//void R_BaseRotX(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



