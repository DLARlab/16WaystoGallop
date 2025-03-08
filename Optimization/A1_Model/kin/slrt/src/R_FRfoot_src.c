/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:02 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_FRfoot_src.h"

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
  double t5795;
  double t5885;
  double t5923;
  double t5899;
  double t5925;
  double t5853;
  double t5882;
  double t5904;
  double t5931;
  double t5933;
  double t5935;
  double t5936;
  double t5937;
  double t5941;
  double t5942;
  double t5943;
  double t5944;
  double t5848;
  double t5945;
  double t105;
  double t5958;
  double t5961;
  double t5964;
  double t5965;
  double t5966;
  double t5967;
  double t5970;
  double t5971;
  double t5972;
  double t5952;
  double t5982;
  double t5983;
  double t5986;
  double t5949;
  double t5950;
  double t5951;
  double t5876;
  double t5946;
  double t5947;
  double t5976;
  double t5977;
  double t5978;
  double t5955;
  double t5973;
  double t5974;
  double t5990;
  double t5991;
  double t5993;
  double t5981;
  double t5987;
  double t5988;
  t5795 = Cos(var1[3]);
  t5885 = Cos(var1[5]);
  t5923 = Sin(var1[3]);
  t5899 = Sin(var1[4]);
  t5925 = Sin(var1[5]);
  t5853 = Cos(var1[7]);
  t5882 = Cos(var1[6]);
  t5904 = t5795*t5885*t5899;
  t5931 = t5923*t5925;
  t5933 = t5904 + t5931;
  t5935 = t5882*t5933;
  t5936 = -1.*t5885*t5923;
  t5937 = t5795*t5899*t5925;
  t5941 = t5936 + t5937;
  t5942 = Sin(var1[6]);
  t5943 = -1.*t5941*t5942;
  t5944 = t5935 + t5943;
  t5848 = Cos(var1[4]);
  t5945 = Sin(var1[7]);
  t105 = Cos(var1[8]);
  t5958 = t5885*t5923*t5899;
  t5961 = -1.*t5795*t5925;
  t5964 = t5958 + t5961;
  t5965 = t5882*t5964;
  t5966 = t5795*t5885;
  t5967 = t5923*t5899*t5925;
  t5970 = t5966 + t5967;
  t5971 = -1.*t5970*t5942;
  t5972 = t5965 + t5971;
  t5952 = Sin(var1[8]);
  t5982 = t5848*t5885*t5882;
  t5983 = -1.*t5848*t5925*t5942;
  t5986 = t5982 + t5983;
  t5949 = t5853*t5944;
  t5950 = t5795*t5848*t5945;
  t5951 = t5949 + t5950;
  t5876 = t5795*t5848*t5853;
  t5946 = -1.*t5944*t5945;
  t5947 = t5876 + t5946;
  t5976 = t5853*t5972;
  t5977 = t5848*t5923*t5945;
  t5978 = t5976 + t5977;
  t5955 = t5848*t5853*t5923;
  t5973 = -1.*t5972*t5945;
  t5974 = t5955 + t5973;
  t5990 = t5853*t5986;
  t5991 = -1.*t5899*t5945;
  t5993 = t5990 + t5991;
  t5981 = -1.*t5853*t5899;
  t5987 = -1.*t5986*t5945;
  t5988 = t5981 + t5987;
  p_output1[0]=t105*t5947 - 1.*t5951*t5952;
  p_output1[1]=t105*t5974 - 1.*t5952*t5978;
  p_output1[2]=t105*t5988 - 1.*t5952*t5993;
  p_output1[3]=t5882*t5941 + t5933*t5942;
  p_output1[4]=t5942*t5964 + t5882*t5970;
  p_output1[5]=t5848*t5882*t5925 + t5848*t5885*t5942;
  p_output1[6]=t105*t5951 + t5947*t5952;
  p_output1[7]=t5952*t5974 + t105*t5978;
  p_output1[8]=t5952*t5988 + t105*t5993;
}



void R_FRfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
