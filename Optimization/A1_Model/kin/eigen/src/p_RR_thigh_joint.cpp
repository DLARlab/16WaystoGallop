/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:08 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RR_thigh_joint.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t135;
  double t2578;
  double t3274;
  double t2682;
  double t3275;
  double t1581;
  double t3101;
  double t3279;
  double t7280;
  double t8663;
  double t8698;
  double t8706;
  double t8708;
  double t49;
  double t1208;
  double t8724;
  double t8740;
  double t8744;
  double t8745;
  double t53;
  double t57;
  double t62;
  double t64;
  double t2210;
  double t2569;
  double t8672;
  double t8684;
  double t8686;
  double t8690;
  double t8759;
  double t8760;
  double t8762;
  double t8764;
  double t8775;
  double t8781;
  double t8726;
  double t8728;
  double t8806;
  double t8808;
  double t8810;
  double t8836;
  double t8837;
  double t8840;
  t135 = Cos(var1[3]);
  t2578 = Cos(var1[5]);
  t3274 = Sin(var1[3]);
  t2682 = Sin(var1[4]);
  t3275 = Sin(var1[5]);
  t1581 = Sin(var1[12]);
  t3101 = t135*t2578*t2682;
  t3279 = t3274*t3275;
  t7280 = t3101 + t3279;
  t8663 = Cos(var1[12]);
  t8698 = -1.*t2578*t3274;
  t8706 = t135*t2682*t3275;
  t8708 = t8698 + t8706;
  t49 = Cos(var1[13]);
  t1208 = Cos(var1[4]);
  t8724 = Sin(var1[13]);
  t8740 = t8663*t7280;
  t8744 = -1.*t1581*t8708;
  t8745 = t8740 + t8744;
  t53 = -1.*t49;
  t57 = 1. + t53;
  t62 = -0.183*t57;
  t64 = 0. + t62;
  t2210 = 0.047*t1581;
  t2569 = 0. + t2210;
  t8672 = -1.*t8663;
  t8684 = 1. + t8672;
  t8686 = -0.047*t8684;
  t8690 = 0. + t8686;
  t8759 = t2578*t3274*t2682;
  t8760 = -1.*t135*t3275;
  t8762 = t8759 + t8760;
  t8764 = t135*t2578;
  t8775 = t3274*t2682*t3275;
  t8781 = t8764 + t8775;
  t8726 = -0.183*t8724;
  t8728 = 0. + t8726;
  t8806 = t8663*t8762;
  t8808 = -1.*t1581*t8781;
  t8810 = t8806 + t8808;
  t8836 = t8663*t1208*t2578;
  t8837 = -1.*t1208*t1581*t3275;
  t8840 = t8836 + t8837;

  p_output1(0)=0. + t1208*t135*t64 + t2569*t7280 + t8690*t8708 - 0.13205*(t1581*t7280 + t8663*t8708) + t8728*t8745 - 0.183*(t1208*t135*t49 - 1.*t8724*t8745) + var1(0);
  p_output1(1)=0. + t1208*t3274*t64 + t2569*t8762 + t8690*t8781 - 0.13205*(t1581*t8762 + t8663*t8781) + t8728*t8810 - 0.183*(t1208*t3274*t49 - 1.*t8724*t8810) + var1(1);
  p_output1(2)=0. + t1208*t2569*t2578 - 1.*t2682*t64 - 0.13205*(t1208*t1581*t2578 + t1208*t3275*t8663) + t1208*t3275*t8690 + t8728*t8840 - 0.183*(-1.*t2682*t49 - 1.*t8724*t8840) + var1(2);
}


       
Eigen::Matrix<double,3,1> p_RR_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void p_RR_thigh_joint(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



