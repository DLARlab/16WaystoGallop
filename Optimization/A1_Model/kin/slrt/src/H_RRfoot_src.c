/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:09 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RRfoot_src.h"

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
  double t411;
  double t1061;
  double t1558;
  double t1250;
  double t1579;
  double t959;
  double t412;
  double t484;
  double t962;
  double t1461;
  double t1758;
  double t1910;
  double t1919;
  double t1958;
  double t1959;
  double t2194;
  double t2404;
  double t2882;
  double t2892;
  double t141;
  double t3372;
  double t5635;
  double t6798;
  double t6800;
  double t6804;
  double t6812;
  double t6815;
  double t6816;
  double t6817;
  double t6818;
  double t6851;
  double t6854;
  double t6857;
  double t843;
  double t2906;
  double t2990;
  double t3643;
  double t3645;
  double t3650;
  double t4627;
  double t6819;
  double t6823;
  double t6828;
  double t6829;
  double t6830;
  double t6848;
  double t6859;
  double t6862;
  double t6870;
  double t6876;
  double t6877;
  double t6892;
  double t6893;
  double t6896;
  double t6978;
  double t6979;
  double t3124;
  double t3758;
  double t4151;
  double t6910;
  double t6911;
  double t6914;
  double t6940;
  double t6941;
  double t6942;
  double t6943;
  double t6946;
  double t6948;
  double t6951;
  double t6952;
  double t6953;
  double t6955;
  double t6898;
  double t6900;
  double t6906;
  double t6968;
  double t6972;
  double t6980;
  double t6982;
  double t6983;
  double t6987;
  double t6988;
  double t6989;
  double t6826;
  double t6835;
  double t6837;
  double t6916;
  double t6922;
  double t6936;
  double t6907;
  double t6908;
  double t6909;
  double t6868;
  double t6886;
  double t6888;
  double t6937;
  double t6938;
  double t6939;
  t411 = Cos(var1[3]);
  t1061 = Cos(var1[5]);
  t1558 = Sin(var1[3]);
  t1250 = Sin(var1[4]);
  t1579 = Sin(var1[5]);
  t959 = Cos(var1[13]);
  t412 = Cos(var1[4]);
  t484 = Sin(var1[13]);
  t962 = Cos(var1[12]);
  t1461 = t411*t1061*t1250;
  t1758 = t1558*t1579;
  t1910 = t1461 + t1758;
  t1919 = t962*t1910;
  t1958 = Sin(var1[12]);
  t1959 = -1.*t1061*t1558;
  t2194 = t411*t1250*t1579;
  t2404 = t1959 + t2194;
  t2882 = -1.*t1958*t2404;
  t2892 = t1919 + t2882;
  t141 = Sin(var1[14]);
  t3372 = Cos(var1[14]);
  t5635 = t1061*t1558*t1250;
  t6798 = -1.*t411*t1579;
  t6800 = t5635 + t6798;
  t6804 = t962*t6800;
  t6812 = t411*t1061;
  t6815 = t1558*t1250*t1579;
  t6816 = t6812 + t6815;
  t6817 = -1.*t1958*t6816;
  t6818 = t6804 + t6817;
  t6851 = t962*t412*t1061;
  t6854 = -1.*t412*t1958*t1579;
  t6857 = t6851 + t6854;
  t843 = t411*t412*t484;
  t2906 = t959*t2892;
  t2990 = t843 + t2906;
  t3643 = t959*t411*t412;
  t3645 = -1.*t484*t2892;
  t3650 = t3643 + t3645;
  t4627 = t412*t484*t1558;
  t6819 = t959*t6818;
  t6823 = t4627 + t6819;
  t6828 = t959*t412*t1558;
  t6829 = -1.*t484*t6818;
  t6830 = t6828 + t6829;
  t6848 = -1.*t484*t1250;
  t6859 = t959*t6857;
  t6862 = t6848 + t6859;
  t6870 = -1.*t959*t1250;
  t6876 = -1.*t484*t6857;
  t6877 = t6870 + t6876;
  t6892 = t1958*t1910;
  t6893 = t962*t2404;
  t6896 = t6892 + t6893;
  t6978 = -1.*t3372;
  t6979 = 1. + t6978;
  t3124 = -1.*t141*t2990;
  t3758 = t3372*t3650;
  t4151 = t3124 + t3758;
  t6910 = t3372*t2990;
  t6911 = t141*t3650;
  t6914 = t6910 + t6911;
  t6940 = -1.*t959;
  t6941 = 1. + t6940;
  t6942 = -0.183*t6941;
  t6943 = 0. + t6942;
  t6946 = 0.047*t1958;
  t6948 = 0. + t6946;
  t6951 = -1.*t962;
  t6952 = 1. + t6951;
  t6953 = -0.047*t6952;
  t6955 = 0. + t6953;
  t6898 = t1958*t6800;
  t6900 = t962*t6816;
  t6906 = t6898 + t6900;
  t6968 = -0.183*t484;
  t6972 = 0. + t6968;
  t6980 = -0.2*t6979;
  t6982 = -0.183*t141;
  t6983 = 0. + t6980 + t6982;
  t6987 = -0.183*t6979;
  t6988 = 0.2*t141;
  t6989 = 0. + t6987 + t6988;
  t6826 = -1.*t141*t6823;
  t6835 = t3372*t6830;
  t6837 = t6826 + t6835;
  t6916 = t3372*t6823;
  t6922 = t141*t6830;
  t6936 = t6916 + t6922;
  t6907 = t412*t1061*t1958;
  t6908 = t962*t412*t1579;
  t6909 = t6907 + t6908;
  t6868 = -1.*t141*t6862;
  t6886 = t3372*t6877;
  t6888 = t6868 + t6886;
  t6937 = t3372*t6862;
  t6938 = t141*t6877;
  t6939 = t6937 + t6938;
  p_output1[0]=t4151;
  p_output1[1]=t6837;
  p_output1[2]=t6888;
  p_output1[3]=0.;
  p_output1[4]=t6896;
  p_output1[5]=t6906;
  p_output1[6]=t6909;
  p_output1[7]=0.;
  p_output1[8]=t6914;
  p_output1[9]=t6936;
  p_output1[10]=t6939;
  p_output1[11]=0.;
  p_output1[12]=0. - 0.183*t4151 - 0.13205*t6896 - 0.4*t6914 + t411*t412*t6943 + t1910*t6948 + t2404*t6955 + t2892*t6972 + t2990*t6983 + t3650*t6989 + var1[0];
  p_output1[13]=0. - 0.183*t6837 - 0.13205*t6906 - 0.4*t6936 + t1558*t412*t6943 + t6800*t6948 + t6816*t6955 + t6818*t6972 + t6823*t6983 + t6830*t6989 + var1[1];
  p_output1[14]=0. - 0.183*t6888 - 0.13205*t6909 - 0.4*t6939 - 1.*t1250*t6943 + t1061*t412*t6948 + t1579*t412*t6955 + t6857*t6972 + t6862*t6983 + t6877*t6989 + var1[2];
  p_output1[15]=1.;
}



void H_RRfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
