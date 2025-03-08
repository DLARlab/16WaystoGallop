/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:37 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RR_hip_joint_src.h"

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
  double t2754;
  double t2794;
  double t2790;
  double t2792;
  double t2795;
  double t2780;
  double t2793;
  double t2796;
  double t2797;
  double t2799;
  double t2806;
  double t2807;
  double t2808;
  double t2739;
  double t2781;
  double t2786;
  double t2800;
  double t2801;
  double t2802;
  double t2804;
  double t2816;
  double t2817;
  double t2818;
  double t2820;
  double t2821;
  double t2822;
  double t2858;
  double t2860;
  double t2862;
  double t2869;
  double t2870;
  double t2871;
  double t2864;
  double t2875;
  double t2894;
  double t2895;
  double t2896;
  double t2882;
  double t2883;
  double t2884;
  double t2885;
  t2754 = Sin(var1[3]);
  t2794 = Cos(var1[3]);
  t2790 = Cos(var1[5]);
  t2792 = Sin(var1[4]);
  t2795 = Sin(var1[5]);
  t2780 = Sin(var1[12]);
  t2793 = -1.*t2790*t2754*t2792;
  t2796 = t2794*t2795;
  t2797 = t2793 + t2796;
  t2799 = Cos(var1[12]);
  t2806 = -1.*t2794*t2790;
  t2807 = -1.*t2754*t2792*t2795;
  t2808 = t2806 + t2807;
  t2739 = Cos(var1[4]);
  t2781 = 0.047*t2780;
  t2786 = 0. + t2781;
  t2800 = -1.*t2799;
  t2801 = 1. + t2800;
  t2802 = -0.047*t2801;
  t2804 = 0. + t2802;
  t2816 = t2794*t2790*t2792;
  t2817 = t2754*t2795;
  t2818 = t2816 + t2817;
  t2820 = -1.*t2790*t2754;
  t2821 = t2794*t2792*t2795;
  t2822 = t2820 + t2821;
  t2858 = t2790*t2754;
  t2860 = -1.*t2794*t2792*t2795;
  t2862 = t2858 + t2860;
  t2869 = t2790*t2754*t2792;
  t2870 = -1.*t2794*t2795;
  t2871 = t2869 + t2870;
  t2864 = t2799*t2818;
  t2875 = t2799*t2871;
  t2894 = t2794*t2790;
  t2895 = t2754*t2792*t2795;
  t2896 = t2894 + t2895;
  t2882 = t2799*t2739*t2790;
  t2883 = -1.*t2739*t2780*t2795;
  t2884 = t2882 + t2883;
  t2885 = -0.047*t2884;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=0.183*t2739*t2754 + t2786*t2797 + t2804*t2808 - 0.047*(t2780*t2797 + t2799*t2808);
  p_output1[10]=-0.183*t2739*t2794 + t2786*t2818 + t2804*t2822 - 0.047*(t2780*t2818 + t2799*t2822);
  p_output1[11]=0;
  p_output1[12]=t2739*t2786*t2790*t2794 + 0.183*t2792*t2794 - 0.047*(t2739*t2780*t2790*t2794 + t2739*t2794*t2795*t2799) + t2739*t2794*t2795*t2804;
  p_output1[13]=t2739*t2754*t2786*t2790 + 0.183*t2754*t2792 - 0.047*(t2739*t2754*t2780*t2790 + t2739*t2754*t2795*t2799) + t2739*t2754*t2795*t2804;
  p_output1[14]=0.183*t2739 - 1.*t2786*t2790*t2792 - 0.047*(-1.*t2780*t2790*t2792 - 1.*t2792*t2795*t2799) - 1.*t2792*t2795*t2804;
  p_output1[15]=t2804*t2818 + t2786*t2862 - 0.047*(t2780*t2862 + t2864);
  p_output1[16]=t2786*t2808 + t2804*t2871 - 0.047*(t2780*t2808 + t2875);
  p_output1[17]=-1.*t2739*t2786*t2795 + t2739*t2790*t2804 + t2885;
  p_output1[18]=0;
  p_output1[19]=0;
  p_output1[20]=0;
  p_output1[21]=0;
  p_output1[22]=0;
  p_output1[23]=0;
  p_output1[24]=0;
  p_output1[25]=0;
  p_output1[26]=0;
  p_output1[27]=0;
  p_output1[28]=0;
  p_output1[29]=0;
  p_output1[30]=0;
  p_output1[31]=0;
  p_output1[32]=0;
  p_output1[33]=0;
  p_output1[34]=0;
  p_output1[35]=0;
  p_output1[36]=0.047*t2799*t2818 - 0.047*t2780*t2822 - 0.047*(-1.*t2780*t2822 + t2864);
  p_output1[37]=0.047*t2799*t2871 - 0.047*t2780*t2896 - 0.047*(t2875 - 1.*t2780*t2896);
  p_output1[38]=-0.047*t2739*t2780*t2795 + 0.047*t2739*t2790*t2799 + t2885;
  p_output1[39]=0;
  p_output1[40]=0;
  p_output1[41]=0;
  p_output1[42]=0;
  p_output1[43]=0;
  p_output1[44]=0;
  p_output1[45]=0;
  p_output1[46]=0;
  p_output1[47]=0;
  p_output1[48]=0;
  p_output1[49]=0;
  p_output1[50]=0;
  p_output1[51]=0;
  p_output1[52]=0;
  p_output1[53]=0;
}



void Jp_RR_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
