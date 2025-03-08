/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:33 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_FLfoot.h"

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
  double t1557;
  double t3379;
  double t3797;
  double t3607;
  double t3837;
  double t2849;
  double t1563;
  double t2686;
  double t3073;
  double t3633;
  double t3838;
  double t3839;
  double t4049;
  double t4171;
  double t4316;
  double t4328;
  double t4377;
  double t4399;
  double t4498;
  double t1290;
  double t4871;
  double t5155;
  double t5272;
  double t5362;
  double t5391;
  double t5392;
  double t5409;
  double t5471;
  double t5475;
  double t5477;
  double t5822;
  double t5861;
  double t5863;
  double t2787;
  double t4535;
  double t4690;
  double t4956;
  double t4960;
  double t4966;
  double t5143;
  double t5481;
  double t5492;
  double t5653;
  double t5672;
  double t5685;
  double t5808;
  double t5870;
  double t5874;
  double t6057;
  double t6079;
  double t6137;
  t1557 = Cos(var1[3]);
  t3379 = Cos(var1[5]);
  t3797 = Sin(var1[3]);
  t3607 = Sin(var1[4]);
  t3837 = Sin(var1[5]);
  t2849 = Cos(var1[10]);
  t1563 = Cos(var1[4]);
  t2686 = Sin(var1[10]);
  t3073 = Cos(var1[9]);
  t3633 = t1557*t3379*t3607;
  t3838 = t3797*t3837;
  t3839 = t3633 + t3838;
  t4049 = t3073*t3839;
  t4171 = Sin(var1[9]);
  t4316 = -1.*t3379*t3797;
  t4328 = t1557*t3607*t3837;
  t4377 = t4316 + t4328;
  t4399 = -1.*t4171*t4377;
  t4498 = t4049 + t4399;
  t1290 = Sin(var1[11]);
  t4871 = Cos(var1[11]);
  t5155 = t3379*t3797*t3607;
  t5272 = -1.*t1557*t3837;
  t5362 = t5155 + t5272;
  t5391 = t3073*t5362;
  t5392 = t1557*t3379;
  t5409 = t3797*t3607*t3837;
  t5471 = t5392 + t5409;
  t5475 = -1.*t4171*t5471;
  t5477 = t5391 + t5475;
  t5822 = t3073*t1563*t3379;
  t5861 = -1.*t1563*t4171*t3837;
  t5863 = t5822 + t5861;
  t2787 = t1557*t1563*t2686;
  t4535 = t2849*t4498;
  t4690 = t2787 + t4535;
  t4956 = t2849*t1557*t1563;
  t4960 = -1.*t2686*t4498;
  t4966 = t4956 + t4960;
  t5143 = t1563*t2686*t3797;
  t5481 = t2849*t5477;
  t5492 = t5143 + t5481;
  t5653 = t2849*t1563*t3797;
  t5672 = -1.*t2686*t5477;
  t5685 = t5653 + t5672;
  t5808 = -1.*t2686*t3607;
  t5870 = t2849*t5863;
  t5874 = t5808 + t5870;
  t6057 = -1.*t2849*t3607;
  t6079 = -1.*t2686*t5863;
  t6137 = t6057 + t6079;

  p_output1(0)=-1.*t1290*t4690 + t4871*t4966;
  p_output1(1)=-1.*t1290*t5492 + t4871*t5685;
  p_output1(2)=-1.*t1290*t5874 + t4871*t6137;
  p_output1(3)=t3839*t4171 + t3073*t4377;
  p_output1(4)=t4171*t5362 + t3073*t5471;
  p_output1(5)=t1563*t3073*t3837 + t1563*t3379*t4171;
  p_output1(6)=t4690*t4871 + t1290*t4966;
  p_output1(7)=t4871*t5492 + t1290*t5685;
  p_output1(8)=t4871*t5874 + t1290*t6137;
}


       
Eigen::Matrix<double,3,3> R_FLfoot(const Eigen::Matrix<double,18,1> &var1)
//void R_FLfoot(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



