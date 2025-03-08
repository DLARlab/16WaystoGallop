/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:14 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RR_calf_joint.h"

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
  double t816;
  double t3158;
  double t4129;
  double t3184;
  double t4175;
  double t2567;
  double t2301;
  double t2472;
  double t2841;
  double t3851;
  double t4211;
  double t4220;
  double t4226;
  double t4228;
  double t4235;
  double t4236;
  double t4237;
  double t4241;
  double t4308;
  double t794;
  double t4407;
  double t4556;
  double t4558;
  double t4567;
  double t4569;
  double t4570;
  double t4577;
  double t4623;
  double t4672;
  double t4688;
  double t4785;
  double t4794;
  double t4802;
  double t2566;
  double t4380;
  double t4390;
  double t4451;
  double t4501;
  double t4517;
  double t4544;
  double t4730;
  double t4735;
  double t4742;
  double t4752;
  double t4756;
  double t4775;
  double t4823;
  double t4827;
  double t4901;
  double t4925;
  double t4932;
  t816 = Cos(var1[3]);
  t3158 = Cos(var1[5]);
  t4129 = Sin(var1[3]);
  t3184 = Sin(var1[4]);
  t4175 = Sin(var1[5]);
  t2567 = Cos(var1[13]);
  t2301 = Cos(var1[4]);
  t2472 = Sin(var1[13]);
  t2841 = Cos(var1[12]);
  t3851 = t816*t3158*t3184;
  t4211 = t4129*t4175;
  t4220 = t3851 + t4211;
  t4226 = t2841*t4220;
  t4228 = Sin(var1[12]);
  t4235 = -1.*t3158*t4129;
  t4236 = t816*t3184*t4175;
  t4237 = t4235 + t4236;
  t4241 = -1.*t4228*t4237;
  t4308 = t4226 + t4241;
  t794 = Sin(var1[14]);
  t4407 = Cos(var1[14]);
  t4556 = t3158*t4129*t3184;
  t4558 = -1.*t816*t4175;
  t4567 = t4556 + t4558;
  t4569 = t2841*t4567;
  t4570 = t816*t3158;
  t4577 = t4129*t3184*t4175;
  t4623 = t4570 + t4577;
  t4672 = -1.*t4228*t4623;
  t4688 = t4569 + t4672;
  t4785 = t2841*t2301*t3158;
  t4794 = -1.*t2301*t4228*t4175;
  t4802 = t4785 + t4794;
  t2566 = t816*t2301*t2472;
  t4380 = t2567*t4308;
  t4390 = t2566 + t4380;
  t4451 = t2567*t816*t2301;
  t4501 = -1.*t2472*t4308;
  t4517 = t4451 + t4501;
  t4544 = t2301*t2472*t4129;
  t4730 = t2567*t4688;
  t4735 = t4544 + t4730;
  t4742 = t2567*t2301*t4129;
  t4752 = -1.*t2472*t4688;
  t4756 = t4742 + t4752;
  t4775 = -1.*t2472*t3184;
  t4823 = t2567*t4802;
  t4827 = t4775 + t4823;
  t4901 = -1.*t2567*t3184;
  t4925 = -1.*t2472*t4802;
  t4932 = t4901 + t4925;

  p_output1(0)=t4407*t4517 - 1.*t4390*t794;
  p_output1(1)=t4407*t4756 - 1.*t4735*t794;
  p_output1(2)=t4407*t4932 - 1.*t4827*t794;
  p_output1(3)=t4220*t4228 + t2841*t4237;
  p_output1(4)=t4228*t4567 + t2841*t4623;
  p_output1(5)=t2301*t2841*t4175 + t2301*t3158*t4228;
  p_output1(6)=t4390*t4407 + t4517*t794;
  p_output1(7)=t4407*t4735 + t4756*t794;
  p_output1(8)=t4407*t4827 + t4932*t794;
}


       
Eigen::Matrix<double,3,3> R_RR_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void R_RR_calf_joint(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



