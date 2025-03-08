/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:33 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FL_calf_joint_src.h"

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
  double t2042;
  double t2053;
  double t2056;
  double t2054;
  double t2057;
  double t2048;
  double t2055;
  double t2059;
  double t2060;
  double t2064;
  double t2073;
  double t2075;
  double t2076;
  double t2043;
  double t2084;
  double t2017;
  double t2087;
  double t2088;
  double t2089;
  double t2091;
  double t2092;
  double t2093;
  double t2096;
  double t2100;
  double t2101;
  double t2102;
  double t2109;
  double t2110;
  double t2112;
  double t2033;
  double t2035;
  double t2036;
  double t2039;
  double t2049;
  double t2051;
  double t2068;
  double t2070;
  double t2071;
  double t2072;
  double t2126;
  double t2127;
  double t2128;
  double t2130;
  double t2131;
  double t2132;
  double t2085;
  double t2086;
  double t2095;
  double t2097;
  double t2098;
  double t2145;
  double t2146;
  double t2147;
  double t2104;
  double t2105;
  double t2108;
  double t2149;
  double t2150;
  double t2151;
  double t2153;
  double t2154;
  double t2155;
  double t2173;
  double t2174;
  double t2175;
  double t2177;
  double t2178;
  double t2179;
  double t2181;
  double t2182;
  double t2183;
  t2042 = Cos(var1[3]);
  t2053 = Cos(var1[5]);
  t2056 = Sin(var1[3]);
  t2054 = Sin(var1[4]);
  t2057 = Sin(var1[5]);
  t2048 = Sin(var1[9]);
  t2055 = t2042*t2053*t2054;
  t2059 = t2056*t2057;
  t2060 = t2055 + t2059;
  t2064 = Cos(var1[9]);
  t2073 = -1.*t2053*t2056;
  t2075 = t2042*t2054*t2057;
  t2076 = t2073 + t2075;
  t2043 = Cos(var1[4]);
  t2084 = Sin(var1[10]);
  t2017 = Cos(var1[10]);
  t2087 = t2064*t2060;
  t2088 = -1.*t2048*t2076;
  t2089 = t2087 + t2088;
  t2091 = Cos(var1[11]);
  t2092 = -1.*t2091;
  t2093 = 1. + t2092;
  t2096 = Sin(var1[11]);
  t2100 = t2042*t2043*t2084;
  t2101 = t2017*t2089;
  t2102 = t2100 + t2101;
  t2109 = t2017*t2042*t2043;
  t2110 = -1.*t2084*t2089;
  t2112 = t2109 + t2110;
  t2033 = -1.*t2017;
  t2035 = 1. + t2033;
  t2036 = 0.183*t2035;
  t2039 = 0. + t2036;
  t2049 = -0.047*t2048;
  t2051 = 0. + t2049;
  t2068 = -1.*t2064;
  t2070 = 1. + t2068;
  t2071 = 0.047*t2070;
  t2072 = 0. + t2071;
  t2126 = t2053*t2056*t2054;
  t2127 = -1.*t2042*t2057;
  t2128 = t2126 + t2127;
  t2130 = t2042*t2053;
  t2131 = t2056*t2054*t2057;
  t2132 = t2130 + t2131;
  t2085 = 0.183*t2084;
  t2086 = 0. + t2085;
  t2095 = -0.2*t2093;
  t2097 = 0.183*t2096;
  t2098 = 0. + t2095 + t2097;
  t2145 = t2064*t2128;
  t2146 = -1.*t2048*t2132;
  t2147 = t2145 + t2146;
  t2104 = 0.183*t2093;
  t2105 = 0.2*t2096;
  t2108 = 0. + t2104 + t2105;
  t2149 = t2043*t2084*t2056;
  t2150 = t2017*t2147;
  t2151 = t2149 + t2150;
  t2153 = t2017*t2043*t2056;
  t2154 = -1.*t2084*t2147;
  t2155 = t2153 + t2154;
  t2173 = t2064*t2043*t2053;
  t2174 = -1.*t2043*t2048*t2057;
  t2175 = t2173 + t2174;
  t2177 = -1.*t2084*t2054;
  t2178 = t2017*t2175;
  t2179 = t2177 + t2178;
  t2181 = -1.*t2017*t2054;
  t2182 = -1.*t2084*t2175;
  t2183 = t2181 + t2182;
  p_output1[0]=0. + t2039*t2042*t2043 + t2051*t2060 + t2072*t2076 + 0.13205*(t2048*t2060 + t2064*t2076) + t2086*t2089 + t2098*t2102 + t2108*t2112 + 0.183*(-1.*t2096*t2102 + t2091*t2112) - 0.2*(t2091*t2102 + t2096*t2112) + var1[0];
  p_output1[1]=0. + t2039*t2043*t2056 + t2051*t2128 + t2072*t2132 + 0.13205*(t2048*t2128 + t2064*t2132) + t2086*t2147 + t2098*t2151 + t2108*t2155 + 0.183*(-1.*t2096*t2151 + t2091*t2155) - 0.2*(t2091*t2151 + t2096*t2155) + var1[1];
  p_output1[2]=0. + t2043*t2051*t2053 - 1.*t2039*t2054 + 0.13205*(t2043*t2048*t2053 + t2043*t2057*t2064) + t2043*t2057*t2072 + t2086*t2175 + t2098*t2179 + t2108*t2183 + 0.183*(-1.*t2096*t2179 + t2091*t2183) - 0.2*(t2091*t2179 + t2096*t2183) + var1[2];
}



void p_FL_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
