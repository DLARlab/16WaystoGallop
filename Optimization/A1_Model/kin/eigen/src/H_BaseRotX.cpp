/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:40 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_BaseRotX.h"

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
  double t2749;
  double t7752;
  double t905;
  double t7762;
  double t7768;
  double t7771;
  t2749 = Cos(var1[4]);
  t7752 = Sin(var1[3]);
  t905 = Cos(var1[3]);
  t7762 = Sin(var1[4]);
  t7768 = Cos(var1[5]);
  t7771 = Sin(var1[5]);

  p_output1(0)=t2749*t905;
  p_output1(1)=t2749*t7752;
  p_output1(2)=-1.*t7762;
  p_output1(3)=0;
  p_output1(4)=-1.*t7752*t7768 + t7762*t7771*t905;
  p_output1(5)=t7752*t7762*t7771 + t7768*t905;
  p_output1(6)=t2749*t7771;
  p_output1(7)=0;
  p_output1(8)=t7752*t7771 + t7762*t7768*t905;
  p_output1(9)=t7752*t7762*t7768 - 1.*t7771*t905;
  p_output1(10)=t2749*t7768;
  p_output1(11)=0;
  p_output1(12)=var1(0);
  p_output1(13)=var1(1);
  p_output1(14)=var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_BaseRotX(const Eigen::Matrix<double,18,1> &var1)
//void H_BaseRotX(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



