/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:38 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RR_hip_joint_src.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE static __inline__        /* use standard inline */
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
static void output1(double *p_output1,const double *var1)
{
  double t2815;
  double t2813;
  double t2832;
  double t2828;
  double t2842;
  double t2846;
  double t2839;
  double t2853;
  double t2843;
  double t2848;
  double t2849;
  double t2854;
  double t2855;
  double t2856;
  double t2862;
  double t2863;
  double t2865;
  double t2867;
  double t2868;
  double t2869;
  double t2852;
  double t2857;
  double t2858;
  double t2895;
  double t2897;
  double t2900;
  double t2901;
  double t2902;
  double t2903;
  double t2866;
  double t2871;
  double t2873;
  double t2874;
  double t2876;
  double t2879;
  t2815 = Cos(var1[4]);
  t2813 = Cos(var1[3]);
  t2832 = Sin(var1[4]);
  t2828 = Sin(var1[3]);
  t2842 = Cos(var1[5]);
  t2846 = Sin(var1[5]);
  t2839 = Sin(var1[12]);
  t2853 = Cos(var1[12]);
  t2843 = t2813*t2842*t2832;
  t2848 = t2828*t2846;
  t2849 = t2843 + t2848;
  t2854 = -1.*t2842*t2828;
  t2855 = t2813*t2832*t2846;
  t2856 = t2854 + t2855;
  t2862 = t2842*t2828*t2832;
  t2863 = -1.*t2813*t2846;
  t2865 = t2862 + t2863;
  t2867 = t2813*t2842;
  t2868 = t2828*t2832*t2846;
  t2869 = t2867 + t2868;
  t2852 = t2839*t2849;
  t2857 = t2853*t2856;
  t2858 = t2852 + t2857;
  t2895 = 0.047*t2839;
  t2897 = 0. + t2895;
  t2900 = -1.*t2853;
  t2901 = 1. + t2900;
  t2902 = -0.047*t2901;
  t2903 = 0. + t2902;
  t2866 = t2839*t2865;
  t2871 = t2853*t2869;
  t2873 = t2866 + t2871;
  t2874 = t2815*t2842*t2839;
  t2876 = t2853*t2815*t2846;
  t2879 = t2874 + t2876;
  p_output1[0]=t2813*t2815;
  p_output1[1]=t2815*t2828;
  p_output1[2]=-1.*t2832;
  p_output1[3]=0.;
  p_output1[4]=t2858;
  p_output1[5]=t2873;
  p_output1[6]=t2879;
  p_output1[7]=0.;
  p_output1[8]=t2849*t2853 - 1.*t2839*t2856;
  p_output1[9]=t2853*t2865 - 1.*t2839*t2869;
  p_output1[10]=-1.*t2815*t2839*t2846 + t2815*t2842*t2853;
  p_output1[11]=0.;
  p_output1[12]=0. - 0.183*t2813*t2815 - 0.047*t2858 + t2849*t2897 + t2856*t2903 + var1[0];
  p_output1[13]=0. - 0.183*t2815*t2828 - 0.047*t2873 + t2865*t2897 + t2869*t2903 + var1[1];
  p_output1[14]=0. + 0.183*t2832 - 0.047*t2879 + t2815*t2842*t2897 + t2815*t2846*t2903 + var1[2];
  p_output1[15]=1.;
}



void H_RR_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
