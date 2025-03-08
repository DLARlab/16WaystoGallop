/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:58 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RL_calf_joint_src.h"

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
  double t850;
  double t4902;
  double t4930;
  double t4922;
  double t4931;
  double t4894;
  double t4825;
  double t4859;
  double t4897;
  double t4928;
  double t4946;
  double t4947;
  double t4949;
  double t4950;
  double t4951;
  double t4952;
  double t4956;
  double t4966;
  double t4978;
  double t631;
  double t4990;
  double t5016;
  double t5017;
  double t5018;
  double t5019;
  double t5020;
  double t5021;
  double t5022;
  double t5023;
  double t5024;
  double t5068;
  double t5070;
  double t5079;
  double t4880;
  double t4979;
  double t4980;
  double t5003;
  double t5009;
  double t5012;
  double t5015;
  double t5025;
  double t5029;
  double t5055;
  double t5059;
  double t5060;
  double t5066;
  double t5080;
  double t5081;
  double t5083;
  double t5084;
  double t5087;
  double t5104;
  double t5105;
  double t5106;
  double t5184;
  double t5185;
  double t4982;
  double t5013;
  double t5014;
  double t5117;
  double t5118;
  double t5119;
  double t5151;
  double t5158;
  double t5159;
  double t5160;
  double t5162;
  double t5163;
  double t5165;
  double t5166;
  double t5168;
  double t5170;
  double t5107;
  double t5109;
  double t5110;
  double t5175;
  double t5179;
  double t5186;
  double t5187;
  double t5188;
  double t5190;
  double t5191;
  double t5192;
  double t5046;
  double t5063;
  double t5065;
  double t5121;
  double t5122;
  double t5123;
  double t5111;
  double t5114;
  double t5115;
  double t5082;
  double t5099;
  double t5103;
  double t5125;
  double t5144;
  double t5148;
  t850 = Cos(var1[3]);
  t4902 = Cos(var1[5]);
  t4930 = Sin(var1[3]);
  t4922 = Sin(var1[4]);
  t4931 = Sin(var1[5]);
  t4894 = Cos(var1[16]);
  t4825 = Cos(var1[4]);
  t4859 = Sin(var1[16]);
  t4897 = Cos(var1[15]);
  t4928 = t850*t4902*t4922;
  t4946 = t4930*t4931;
  t4947 = t4928 + t4946;
  t4949 = t4897*t4947;
  t4950 = Sin(var1[15]);
  t4951 = -1.*t4902*t4930;
  t4952 = t850*t4922*t4931;
  t4956 = t4951 + t4952;
  t4966 = -1.*t4950*t4956;
  t4978 = t4949 + t4966;
  t631 = Sin(var1[17]);
  t4990 = Cos(var1[17]);
  t5016 = t4902*t4930*t4922;
  t5017 = -1.*t850*t4931;
  t5018 = t5016 + t5017;
  t5019 = t4897*t5018;
  t5020 = t850*t4902;
  t5021 = t4930*t4922*t4931;
  t5022 = t5020 + t5021;
  t5023 = -1.*t4950*t5022;
  t5024 = t5019 + t5023;
  t5068 = t4897*t4825*t4902;
  t5070 = -1.*t4825*t4950*t4931;
  t5079 = t5068 + t5070;
  t4880 = t850*t4825*t4859;
  t4979 = t4894*t4978;
  t4980 = t4880 + t4979;
  t5003 = t4894*t850*t4825;
  t5009 = -1.*t4859*t4978;
  t5012 = t5003 + t5009;
  t5015 = t4825*t4859*t4930;
  t5025 = t4894*t5024;
  t5029 = t5015 + t5025;
  t5055 = t4894*t4825*t4930;
  t5059 = -1.*t4859*t5024;
  t5060 = t5055 + t5059;
  t5066 = -1.*t4859*t4922;
  t5080 = t4894*t5079;
  t5081 = t5066 + t5080;
  t5083 = -1.*t4894*t4922;
  t5084 = -1.*t4859*t5079;
  t5087 = t5083 + t5084;
  t5104 = t4950*t4947;
  t5105 = t4897*t4956;
  t5106 = t5104 + t5105;
  t5184 = -1.*t4990;
  t5185 = 1. + t5184;
  t4982 = -1.*t631*t4980;
  t5013 = t4990*t5012;
  t5014 = t4982 + t5013;
  t5117 = t4990*t4980;
  t5118 = t631*t5012;
  t5119 = t5117 + t5118;
  t5151 = -1.*t4894;
  t5158 = 1. + t5151;
  t5159 = -0.183*t5158;
  t5160 = 0. + t5159;
  t5162 = -0.047*t4950;
  t5163 = 0. + t5162;
  t5165 = -1.*t4897;
  t5166 = 1. + t5165;
  t5168 = 0.047*t5166;
  t5170 = 0. + t5168;
  t5107 = t4950*t5018;
  t5109 = t4897*t5022;
  t5110 = t5107 + t5109;
  t5175 = -0.183*t4859;
  t5179 = 0. + t5175;
  t5186 = -0.2*t5185;
  t5187 = -0.183*t631;
  t5188 = 0. + t5186 + t5187;
  t5190 = -0.183*t5185;
  t5191 = 0.2*t631;
  t5192 = 0. + t5190 + t5191;
  t5046 = -1.*t631*t5029;
  t5063 = t4990*t5060;
  t5065 = t5046 + t5063;
  t5121 = t4990*t5029;
  t5122 = t631*t5060;
  t5123 = t5121 + t5122;
  t5111 = t4825*t4902*t4950;
  t5114 = t4897*t4825*t4931;
  t5115 = t5111 + t5114;
  t5082 = -1.*t631*t5081;
  t5099 = t4990*t5087;
  t5103 = t5082 + t5099;
  t5125 = t4990*t5081;
  t5144 = t631*t5087;
  t5148 = t5125 + t5144;
  p_output1[0]=t5014;
  p_output1[1]=t5065;
  p_output1[2]=t5103;
  p_output1[3]=0.;
  p_output1[4]=t5106;
  p_output1[5]=t5110;
  p_output1[6]=t5115;
  p_output1[7]=0.;
  p_output1[8]=t5119;
  p_output1[9]=t5123;
  p_output1[10]=t5148;
  p_output1[11]=0.;
  p_output1[12]=0. - 0.183*t5014 + 0.13205*t5106 - 0.2*t5119 + t4947*t5163 + t4956*t5170 + t4978*t5179 + t4980*t5188 + t5012*t5192 + t4825*t5160*t850 + var1[0];
  p_output1[13]=0. - 0.183*t5065 + 0.13205*t5110 - 0.2*t5123 + t4825*t4930*t5160 + t5018*t5163 + t5022*t5170 + t5024*t5179 + t5029*t5188 + t5060*t5192 + var1[1];
  p_output1[14]=0. - 0.183*t5103 + 0.13205*t5115 - 0.2*t5148 - 1.*t4922*t5160 + t4825*t4902*t5163 + t4825*t4931*t5170 + t5079*t5179 + t5081*t5188 + t5087*t5192 + var1[2];
  p_output1[15]=1.;
}



void H_RL_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
