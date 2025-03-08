/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:02 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FL_calf_joint.h"

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
  double t472;
  double t620;
  double t858;
  double t732;
  double t869;
  double t501;
  double t473;
  double t492;
  double t563;
  double t821;
  double t1030;
  double t1236;
  double t1543;
  double t1564;
  double t1694;
  double t1848;
  double t1849;
  double t1933;
  double t1934;
  double t458;
  double t2141;
  double t2691;
  double t2745;
  double t2845;
  double t2850;
  double t2865;
  double t2929;
  double t2942;
  double t2944;
  double t2951;
  double t3032;
  double t3060;
  double t3072;
  double t499;
  double t2027;
  double t2063;
  double t2237;
  double t2242;
  double t2261;
  double t2672;
  double t2952;
  double t2954;
  double t2956;
  double t2987;
  double t2995;
  double t3019;
  double t3103;
  double t3104;
  double t3195;
  double t3198;
  double t3598;
  double t3689;
  double t3826;
  double t3840;
  double t5984;
  double t6149;
  double t2080;
  double t2449;
  double t2591;
  double t4137;
  double t4181;
  double t4368;
  double t4703;
  double t4777;
  double t4786;
  double t4800;
  double t4857;
  double t4987;
  double t5164;
  double t5304;
  double t5341;
  double t5404;
  double t3852;
  double t3853;
  double t3967;
  double t5771;
  double t5818;
  double t6318;
  double t6341;
  double t6573;
  double t7046;
  double t7061;
  double t7355;
  double t2955;
  double t3007;
  double t3018;
  double t4404;
  double t4465;
  double t4551;
  double t4083;
  double t4084;
  double t4093;
  double t3169;
  double t3654;
  double t3680;
  double t4568;
  double t4573;
  double t4586;
  t472 = Cos(var1[3]);
  t620 = Cos(var1[5]);
  t858 = Sin(var1[3]);
  t732 = Sin(var1[4]);
  t869 = Sin(var1[5]);
  t501 = Cos(var1[10]);
  t473 = Cos(var1[4]);
  t492 = Sin(var1[10]);
  t563 = Cos(var1[9]);
  t821 = t472*t620*t732;
  t1030 = t858*t869;
  t1236 = t821 + t1030;
  t1543 = t563*t1236;
  t1564 = Sin(var1[9]);
  t1694 = -1.*t620*t858;
  t1848 = t472*t732*t869;
  t1849 = t1694 + t1848;
  t1933 = -1.*t1564*t1849;
  t1934 = t1543 + t1933;
  t458 = Sin(var1[11]);
  t2141 = Cos(var1[11]);
  t2691 = t620*t858*t732;
  t2745 = -1.*t472*t869;
  t2845 = t2691 + t2745;
  t2850 = t563*t2845;
  t2865 = t472*t620;
  t2929 = t858*t732*t869;
  t2942 = t2865 + t2929;
  t2944 = -1.*t1564*t2942;
  t2951 = t2850 + t2944;
  t3032 = t563*t473*t620;
  t3060 = -1.*t473*t1564*t869;
  t3072 = t3032 + t3060;
  t499 = t472*t473*t492;
  t2027 = t501*t1934;
  t2063 = t499 + t2027;
  t2237 = t501*t472*t473;
  t2242 = -1.*t492*t1934;
  t2261 = t2237 + t2242;
  t2672 = t473*t492*t858;
  t2952 = t501*t2951;
  t2954 = t2672 + t2952;
  t2956 = t501*t473*t858;
  t2987 = -1.*t492*t2951;
  t2995 = t2956 + t2987;
  t3019 = -1.*t492*t732;
  t3103 = t501*t3072;
  t3104 = t3019 + t3103;
  t3195 = -1.*t501*t732;
  t3198 = -1.*t492*t3072;
  t3598 = t3195 + t3198;
  t3689 = t1564*t1236;
  t3826 = t563*t1849;
  t3840 = t3689 + t3826;
  t5984 = -1.*t2141;
  t6149 = 1. + t5984;
  t2080 = -1.*t458*t2063;
  t2449 = t2141*t2261;
  t2591 = t2080 + t2449;
  t4137 = t2141*t2063;
  t4181 = t458*t2261;
  t4368 = t4137 + t4181;
  t4703 = -1.*t501;
  t4777 = 1. + t4703;
  t4786 = 0.183*t4777;
  t4800 = 0. + t4786;
  t4857 = -0.047*t1564;
  t4987 = 0. + t4857;
  t5164 = -1.*t563;
  t5304 = 1. + t5164;
  t5341 = 0.047*t5304;
  t5404 = 0. + t5341;
  t3852 = t1564*t2845;
  t3853 = t563*t2942;
  t3967 = t3852 + t3853;
  t5771 = 0.183*t492;
  t5818 = 0. + t5771;
  t6318 = -0.2*t6149;
  t6341 = 0.183*t458;
  t6573 = 0. + t6318 + t6341;
  t7046 = 0.183*t6149;
  t7061 = 0.2*t458;
  t7355 = 0. + t7046 + t7061;
  t2955 = -1.*t458*t2954;
  t3007 = t2141*t2995;
  t3018 = t2955 + t3007;
  t4404 = t2141*t2954;
  t4465 = t458*t2995;
  t4551 = t4404 + t4465;
  t4083 = t473*t620*t1564;
  t4084 = t563*t473*t869;
  t4093 = t4083 + t4084;
  t3169 = -1.*t458*t3104;
  t3654 = t2141*t3598;
  t3680 = t3169 + t3654;
  t4568 = t2141*t3104;
  t4573 = t458*t3598;
  t4586 = t4568 + t4573;

  p_output1(0)=t2591;
  p_output1(1)=t3018;
  p_output1(2)=t3680;
  p_output1(3)=0.;
  p_output1(4)=t3840;
  p_output1(5)=t3967;
  p_output1(6)=t4093;
  p_output1(7)=0.;
  p_output1(8)=t4368;
  p_output1(9)=t4551;
  p_output1(10)=t4586;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.183*t2591 + 0.13205*t3840 - 0.2*t4368 + t472*t473*t4800 + t1236*t4987 + t1849*t5404 + t1934*t5818 + t2063*t6573 + t2261*t7355 + var1(0);
  p_output1(13)=0. + 0.183*t3018 + 0.13205*t3967 - 0.2*t4551 + t2845*t4987 + t2942*t5404 + t2951*t5818 + t2954*t6573 + t2995*t7355 + t473*t4800*t858 + var1(1);
  p_output1(14)=0. + 0.183*t3680 + 0.13205*t4093 - 0.2*t4586 + t3072*t5818 + t473*t4987*t620 + t3104*t6573 - 1.*t4800*t732 + t3598*t7355 + t473*t5404*t869 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_FL_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void H_FL_calf_joint(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



