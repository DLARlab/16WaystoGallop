/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:37 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RRfoot.h"

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
  double t1062;
  double t3834;
  double t4291;
  double t3890;
  double t4294;
  double t3442;
  double t1072;
  double t1270;
  double t3678;
  double t3937;
  double t4304;
  double t4528;
  double t4639;
  double t4660;
  double t4661;
  double t4668;
  double t4678;
  double t4679;
  double t4685;
  double t154;
  double t4754;
  double t5086;
  double t5093;
  double t5116;
  double t5171;
  double t5173;
  double t5218;
  double t5303;
  double t5330;
  double t5367;
  double t5781;
  double t5805;
  double t5806;
  double t3421;
  double t4686;
  double t4716;
  double t4759;
  double t4804;
  double t4906;
  double t5019;
  double t5399;
  double t5400;
  double t5618;
  double t5674;
  double t5710;
  double t5745;
  double t5813;
  double t5869;
  double t5883;
  double t5895;
  double t5912;
  t1062 = Cos(var1[3]);
  t3834 = Cos(var1[5]);
  t4291 = Sin(var1[3]);
  t3890 = Sin(var1[4]);
  t4294 = Sin(var1[5]);
  t3442 = Cos(var1[13]);
  t1072 = Cos(var1[4]);
  t1270 = Sin(var1[13]);
  t3678 = Cos(var1[12]);
  t3937 = t1062*t3834*t3890;
  t4304 = t4291*t4294;
  t4528 = t3937 + t4304;
  t4639 = t3678*t4528;
  t4660 = Sin(var1[12]);
  t4661 = -1.*t3834*t4291;
  t4668 = t1062*t3890*t4294;
  t4678 = t4661 + t4668;
  t4679 = -1.*t4660*t4678;
  t4685 = t4639 + t4679;
  t154 = Sin(var1[14]);
  t4754 = Cos(var1[14]);
  t5086 = t3834*t4291*t3890;
  t5093 = -1.*t1062*t4294;
  t5116 = t5086 + t5093;
  t5171 = t3678*t5116;
  t5173 = t1062*t3834;
  t5218 = t4291*t3890*t4294;
  t5303 = t5173 + t5218;
  t5330 = -1.*t4660*t5303;
  t5367 = t5171 + t5330;
  t5781 = t3678*t1072*t3834;
  t5805 = -1.*t1072*t4660*t4294;
  t5806 = t5781 + t5805;
  t3421 = t1062*t1072*t1270;
  t4686 = t3442*t4685;
  t4716 = t3421 + t4686;
  t4759 = t3442*t1062*t1072;
  t4804 = -1.*t1270*t4685;
  t4906 = t4759 + t4804;
  t5019 = t1072*t1270*t4291;
  t5399 = t3442*t5367;
  t5400 = t5019 + t5399;
  t5618 = t3442*t1072*t4291;
  t5674 = -1.*t1270*t5367;
  t5710 = t5618 + t5674;
  t5745 = -1.*t1270*t3890;
  t5813 = t3442*t5806;
  t5869 = t5745 + t5813;
  t5883 = -1.*t3442*t3890;
  t5895 = -1.*t1270*t5806;
  t5912 = t5883 + t5895;

  p_output1(0)=-1.*t154*t4716 + t4754*t4906;
  p_output1(1)=-1.*t154*t5400 + t4754*t5710;
  p_output1(2)=-1.*t154*t5869 + t4754*t5912;
  p_output1(3)=t4528*t4660 + t3678*t4678;
  p_output1(4)=t4660*t5116 + t3678*t5303;
  p_output1(5)=t1072*t3678*t4294 + t1072*t3834*t4660;
  p_output1(6)=t4716*t4754 + t154*t4906;
  p_output1(7)=t4754*t5400 + t154*t5710;
  p_output1(8)=t4754*t5869 + t154*t5912;
}


       
Eigen::Matrix<double,3,3> R_RRfoot(const Eigen::Matrix<double,18,1> &var1)
//void R_RRfoot(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



