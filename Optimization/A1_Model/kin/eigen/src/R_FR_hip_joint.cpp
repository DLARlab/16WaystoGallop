/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:44 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_FR_hip_joint.h"

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
  double t632;
  double t1474;
  double t554;
  double t4839;
  double t7834;
  double t7844;
  double t7833;
  double t7853;
  double t7848;
  double t7851;
  double t7852;
  double t7840;
  double t7845;
  double t7846;
  double t7862;
  double t7863;
  double t7864;
  double t7857;
  double t7858;
  double t7860;
  t632 = Cos(var1[4]);
  t1474 = Sin(var1[3]);
  t554 = Cos(var1[3]);
  t4839 = Sin(var1[4]);
  t7834 = Cos(var1[5]);
  t7844 = Sin(var1[5]);
  t7833 = Cos(var1[6]);
  t7853 = Sin(var1[6]);
  t7848 = t554*t7834*t4839;
  t7851 = t1474*t7844;
  t7852 = t7848 + t7851;
  t7840 = -1.*t7834*t1474;
  t7845 = t554*t4839*t7844;
  t7846 = t7840 + t7845;
  t7862 = t7834*t1474*t4839;
  t7863 = -1.*t554*t7844;
  t7864 = t7862 + t7863;
  t7857 = t554*t7834;
  t7858 = t1474*t4839*t7844;
  t7860 = t7857 + t7858;

  p_output1(0)=t554*t632;
  p_output1(1)=t1474*t632;
  p_output1(2)=-1.*t4839;
  p_output1(3)=t7833*t7846 + t7852*t7853;
  p_output1(4)=t7833*t7860 + t7853*t7864;
  p_output1(5)=t632*t7833*t7844 + t632*t7834*t7853;
  p_output1(6)=t7833*t7852 - 1.*t7846*t7853;
  p_output1(7)=-1.*t7853*t7860 + t7833*t7864;
  p_output1(8)=t632*t7833*t7834 - 1.*t632*t7844*t7853;
}


       
Eigen::Matrix<double,3,3> R_FR_hip_joint(const Eigen::Matrix<double,18,1> &var1)
//void R_FR_hip_joint(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



