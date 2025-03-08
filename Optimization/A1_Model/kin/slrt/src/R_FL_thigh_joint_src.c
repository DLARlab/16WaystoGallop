/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:32 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_FL_thigh_joint_src.h"

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
  double t1936;
  double t1961;
  double t1973;
  double t1969;
  double t1974;
  double t1857;
  double t1937;
  double t1956;
  double t1957;
  double t1978;
  double t1972;
  double t1975;
  double t1976;
  double t1979;
  double t1980;
  double t1981;
  double t1987;
  double t1988;
  double t1989;
  double t1991;
  double t1992;
  double t1993;
  double t1977;
  double t1982;
  double t1983;
  double t1990;
  double t1994;
  double t1995;
  double t1999;
  double t2000;
  double t2001;
  t1936 = Cos(var1[3]);
  t1961 = Cos(var1[5]);
  t1973 = Sin(var1[3]);
  t1969 = Sin(var1[4]);
  t1974 = Sin(var1[5]);
  t1857 = Cos(var1[10]);
  t1937 = Cos(var1[4]);
  t1956 = Sin(var1[10]);
  t1957 = Cos(var1[9]);
  t1978 = Sin(var1[9]);
  t1972 = t1936*t1961*t1969;
  t1975 = t1973*t1974;
  t1976 = t1972 + t1975;
  t1979 = -1.*t1961*t1973;
  t1980 = t1936*t1969*t1974;
  t1981 = t1979 + t1980;
  t1987 = t1961*t1973*t1969;
  t1988 = -1.*t1936*t1974;
  t1989 = t1987 + t1988;
  t1991 = t1936*t1961;
  t1992 = t1973*t1969*t1974;
  t1993 = t1991 + t1992;
  t1977 = t1957*t1976;
  t1982 = -1.*t1978*t1981;
  t1983 = t1977 + t1982;
  t1990 = t1957*t1989;
  t1994 = -1.*t1978*t1993;
  t1995 = t1990 + t1994;
  t1999 = t1957*t1937*t1961;
  t2000 = -1.*t1937*t1978*t1974;
  t2001 = t1999 + t2000;
  p_output1[0]=t1857*t1936*t1937 - 1.*t1956*t1983;
  p_output1[1]=t1857*t1937*t1973 - 1.*t1956*t1995;
  p_output1[2]=-1.*t1857*t1969 - 1.*t1956*t2001;
  p_output1[3]=t1976*t1978 + t1957*t1981;
  p_output1[4]=t1978*t1989 + t1957*t1993;
  p_output1[5]=t1937*t1957*t1974 + t1937*t1961*t1978;
  p_output1[6]=t1936*t1937*t1956 + t1857*t1983;
  p_output1[7]=t1937*t1956*t1973 + t1857*t1995;
  p_output1[8]=-1.*t1956*t1969 + t1857*t2001;
}



void R_FL_thigh_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
