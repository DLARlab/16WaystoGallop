/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:03 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FLfoot_src.h"

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
  double t6060;
  double t6073;
  double t6078;
  double t6075;
  double t6079;
  double t6067;
  double t6077;
  double t6080;
  double t6081;
  double t6084;
  double t6095;
  double t6096;
  double t6097;
  double t6063;
  double t6106;
  double t289;
  double t6109;
  double t6110;
  double t6111;
  double t6113;
  double t6114;
  double t6115;
  double t6117;
  double t6121;
  double t6122;
  double t6124;
  double t6129;
  double t6132;
  double t6133;
  double t359;
  double t6038;
  double t6055;
  double t6057;
  double t6068;
  double t6072;
  double t6085;
  double t6088;
  double t6092;
  double t6094;
  double t6148;
  double t6149;
  double t6150;
  double t6152;
  double t6153;
  double t6154;
  double t6107;
  double t6108;
  double t6116;
  double t6119;
  double t6120;
  double t6164;
  double t6168;
  double t6169;
  double t6126;
  double t6127;
  double t6128;
  double t6171;
  double t6172;
  double t6173;
  double t6175;
  double t6176;
  double t6177;
  double t6195;
  double t6196;
  double t6197;
  double t6199;
  double t6200;
  double t6201;
  double t6203;
  double t6204;
  double t6205;
  t6060 = Cos(var1[3]);
  t6073 = Cos(var1[5]);
  t6078 = Sin(var1[3]);
  t6075 = Sin(var1[4]);
  t6079 = Sin(var1[5]);
  t6067 = Sin(var1[9]);
  t6077 = t6060*t6073*t6075;
  t6080 = t6078*t6079;
  t6081 = t6077 + t6080;
  t6084 = Cos(var1[9]);
  t6095 = -1.*t6073*t6078;
  t6096 = t6060*t6075*t6079;
  t6097 = t6095 + t6096;
  t6063 = Cos(var1[4]);
  t6106 = Sin(var1[10]);
  t289 = Cos(var1[10]);
  t6109 = t6084*t6081;
  t6110 = -1.*t6067*t6097;
  t6111 = t6109 + t6110;
  t6113 = Cos(var1[11]);
  t6114 = -1.*t6113;
  t6115 = 1. + t6114;
  t6117 = Sin(var1[11]);
  t6121 = t6060*t6063*t6106;
  t6122 = t289*t6111;
  t6124 = t6121 + t6122;
  t6129 = t289*t6060*t6063;
  t6132 = -1.*t6106*t6111;
  t6133 = t6129 + t6132;
  t359 = -1.*t289;
  t6038 = 1. + t359;
  t6055 = 0.183*t6038;
  t6057 = 0. + t6055;
  t6068 = -0.047*t6067;
  t6072 = 0. + t6068;
  t6085 = -1.*t6084;
  t6088 = 1. + t6085;
  t6092 = 0.047*t6088;
  t6094 = 0. + t6092;
  t6148 = t6073*t6078*t6075;
  t6149 = -1.*t6060*t6079;
  t6150 = t6148 + t6149;
  t6152 = t6060*t6073;
  t6153 = t6078*t6075*t6079;
  t6154 = t6152 + t6153;
  t6107 = 0.183*t6106;
  t6108 = 0. + t6107;
  t6116 = -0.2*t6115;
  t6119 = 0.183*t6117;
  t6120 = 0. + t6116 + t6119;
  t6164 = t6084*t6150;
  t6168 = -1.*t6067*t6154;
  t6169 = t6164 + t6168;
  t6126 = 0.183*t6115;
  t6127 = 0.2*t6117;
  t6128 = 0. + t6126 + t6127;
  t6171 = t6063*t6106*t6078;
  t6172 = t289*t6169;
  t6173 = t6171 + t6172;
  t6175 = t289*t6063*t6078;
  t6176 = -1.*t6106*t6169;
  t6177 = t6175 + t6176;
  t6195 = t6084*t6063*t6073;
  t6196 = -1.*t6063*t6067*t6079;
  t6197 = t6195 + t6196;
  t6199 = -1.*t6106*t6075;
  t6200 = t289*t6197;
  t6201 = t6199 + t6200;
  t6203 = -1.*t289*t6075;
  t6204 = -1.*t6106*t6197;
  t6205 = t6203 + t6204;
  p_output1[0]=0. + t6057*t6060*t6063 + t6072*t6081 + t6094*t6097 + 0.13205*(t6067*t6081 + t6084*t6097) + t6108*t6111 + t6120*t6124 + t6128*t6133 + 0.183*(-1.*t6117*t6124 + t6113*t6133) - 0.4*(t6113*t6124 + t6117*t6133) + var1[0];
  p_output1[1]=0. + t6057*t6063*t6078 + t6072*t6150 + t6094*t6154 + 0.13205*(t6067*t6150 + t6084*t6154) + t6108*t6169 + t6120*t6173 + t6128*t6177 + 0.183*(-1.*t6117*t6173 + t6113*t6177) - 0.4*(t6113*t6173 + t6117*t6177) + var1[1];
  p_output1[2]=0. + t6063*t6072*t6073 - 1.*t6057*t6075 + 0.13205*(t6063*t6067*t6073 + t6063*t6079*t6084) + t6063*t6079*t6094 + t6108*t6197 + t6120*t6201 + t6128*t6205 + 0.183*(-1.*t6117*t6201 + t6113*t6205) - 0.4*(t6113*t6201 + t6117*t6205) + var1[2];
}



void p_FLfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
