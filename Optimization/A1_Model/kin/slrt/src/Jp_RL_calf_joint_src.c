/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:57 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RL_calf_joint_src.h"

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
  double t4767;
  double t4803;
  double t4795;
  double t4799;
  double t4810;
  double t4772;
  double t4802;
  double t4811;
  double t4812;
  double t4819;
  double t4832;
  double t4834;
  double t4844;
  double t4741;
  double t4853;
  double t177;
  double t4856;
  double t4857;
  double t4858;
  double t4861;
  double t4863;
  double t4865;
  double t4867;
  double t4874;
  double t4880;
  double t4881;
  double t4890;
  double t4891;
  double t4892;
  double t403;
  double t792;
  double t4715;
  double t4739;
  double t4782;
  double t4793;
  double t4822;
  double t4823;
  double t4825;
  double t4830;
  double t4904;
  double t4905;
  double t4908;
  double t4912;
  double t4916;
  double t4917;
  double t4854;
  double t4855;
  double t4866;
  double t4870;
  double t4872;
  double t4923;
  double t4924;
  double t4925;
  double t4884;
  double t4885;
  double t4888;
  double t4927;
  double t4928;
  double t4929;
  double t4931;
  double t4932;
  double t4933;
  double t4958;
  double t4959;
  double t4960;
  double t4964;
  double t4966;
  double t4968;
  double t4973;
  double t4974;
  double t4975;
  double t4996;
  double t4997;
  double t4998;
  double t5000;
  double t5001;
  double t5002;
  double t5004;
  double t5005;
  double t5008;
  double t5028;
  double t5029;
  double t5036;
  double t5044;
  double t5046;
  double t5048;
  double t5050;
  double t5053;
  double t5054;
  double t5066;
  double t5067;
  double t5068;
  double t5073;
  double t5074;
  double t5075;
  double t5095;
  double t5096;
  double t5097;
  double t4850;
  double t5100;
  double t5101;
  double t5126;
  double t5128;
  double t5129;
  double t5153;
  double t5154;
  double t5172;
  double t5173;
  double t5174;
  double t5176;
  double t5177;
  double t5106;
  double t5133;
  double t5134;
  double t5136;
  double t5137;
  double t5138;
  double t5140;
  double t5141;
  double t5142;
  double t5143;
  double t5144;
  double t5145;
  double t5146;
  double t5147;
  double t5148;
  double t5149;
  double t5198;
  double t5199;
  double t5200;
  double t4940;
  double t5181;
  double t5182;
  double t5213;
  double t5214;
  double t5216;
  double t5217;
  double t5218;
  double t5231;
  double t5232;
  double t5233;
  double t5235;
  double t5236;
  double t5237;
  double t4938;
  double t4944;
  double t5207;
  double t5248;
  double t5249;
  double t5250;
  double t5252;
  double t5253;
  double t5260;
  double t5261;
  double t5262;
  double t5221;
  double t5225;
  double t5272;
  double t5273;
  double t5274;
  double t5240;
  double t5244;
  t4767 = Sin(var1[3]);
  t4803 = Cos(var1[3]);
  t4795 = Cos(var1[5]);
  t4799 = Sin(var1[4]);
  t4810 = Sin(var1[5]);
  t4772 = Sin(var1[15]);
  t4802 = -1.*t4795*t4767*t4799;
  t4811 = t4803*t4810;
  t4812 = t4802 + t4811;
  t4819 = Cos(var1[15]);
  t4832 = -1.*t4803*t4795;
  t4834 = -1.*t4767*t4799*t4810;
  t4844 = t4832 + t4834;
  t4741 = Cos(var1[4]);
  t4853 = Sin(var1[16]);
  t177 = Cos(var1[16]);
  t4856 = t4819*t4812;
  t4857 = -1.*t4772*t4844;
  t4858 = t4856 + t4857;
  t4861 = Cos(var1[17]);
  t4863 = -1.*t4861;
  t4865 = 1. + t4863;
  t4867 = Sin(var1[17]);
  t4874 = -1.*t4741*t4853*t4767;
  t4880 = t177*t4858;
  t4881 = t4874 + t4880;
  t4890 = -1.*t177*t4741*t4767;
  t4891 = -1.*t4853*t4858;
  t4892 = t4890 + t4891;
  t403 = -1.*t177;
  t792 = 1. + t403;
  t4715 = -0.183*t792;
  t4739 = 0. + t4715;
  t4782 = -0.047*t4772;
  t4793 = 0. + t4782;
  t4822 = -1.*t4819;
  t4823 = 1. + t4822;
  t4825 = 0.047*t4823;
  t4830 = 0. + t4825;
  t4904 = t4803*t4795*t4799;
  t4905 = t4767*t4810;
  t4908 = t4904 + t4905;
  t4912 = -1.*t4795*t4767;
  t4916 = t4803*t4799*t4810;
  t4917 = t4912 + t4916;
  t4854 = -0.183*t4853;
  t4855 = 0. + t4854;
  t4866 = -0.2*t4865;
  t4870 = -0.183*t4867;
  t4872 = 0. + t4866 + t4870;
  t4923 = t4819*t4908;
  t4924 = -1.*t4772*t4917;
  t4925 = t4923 + t4924;
  t4884 = -0.183*t4865;
  t4885 = 0.2*t4867;
  t4888 = 0. + t4884 + t4885;
  t4927 = t4803*t4741*t4853;
  t4928 = t177*t4925;
  t4929 = t4927 + t4928;
  t4931 = t177*t4803*t4741;
  t4932 = -1.*t4853*t4925;
  t4933 = t4931 + t4932;
  t4958 = t4819*t4803*t4741*t4795;
  t4959 = -1.*t4803*t4741*t4772*t4810;
  t4960 = t4958 + t4959;
  t4964 = -1.*t4803*t4853*t4799;
  t4966 = t177*t4960;
  t4968 = t4964 + t4966;
  t4973 = -1.*t177*t4803*t4799;
  t4974 = -1.*t4853*t4960;
  t4975 = t4973 + t4974;
  t4996 = t4819*t4741*t4795*t4767;
  t4997 = -1.*t4741*t4772*t4767*t4810;
  t4998 = t4996 + t4997;
  t5000 = -1.*t4853*t4767*t4799;
  t5001 = t177*t4998;
  t5002 = t5000 + t5001;
  t5004 = -1.*t177*t4767*t4799;
  t5005 = -1.*t4853*t4998;
  t5008 = t5004 + t5005;
  t5028 = -1.*t4819*t4795*t4799;
  t5029 = t4772*t4799*t4810;
  t5036 = t5028 + t5029;
  t5044 = -1.*t4741*t4853;
  t5046 = t177*t5036;
  t5048 = t5044 + t5046;
  t5050 = -1.*t177*t4741;
  t5053 = -1.*t4853*t5036;
  t5054 = t5050 + t5053;
  t5066 = t4795*t4767;
  t5067 = -1.*t4803*t4799*t4810;
  t5068 = t5066 + t5067;
  t5073 = -1.*t4772*t4908;
  t5074 = t4819*t5068;
  t5075 = t5073 + t5074;
  t5095 = t4795*t4767*t4799;
  t5096 = -1.*t4803*t4810;
  t5097 = t5095 + t5096;
  t4850 = t4819*t4844;
  t5100 = -1.*t4772*t5097;
  t5101 = t5100 + t4850;
  t5126 = -1.*t4741*t4795*t4772;
  t5128 = -1.*t4819*t4741*t4810;
  t5129 = t5126 + t5128;
  t5153 = -1.*t4819*t4917;
  t5154 = t5073 + t5153;
  t5172 = t4803*t4795;
  t5173 = t4767*t4799*t4810;
  t5174 = t5172 + t5173;
  t5176 = -1.*t4819*t5174;
  t5177 = t5100 + t5176;
  t5106 = t4819*t5097;
  t5133 = t4855*t5129;
  t5134 = t177*t4872*t5129;
  t5136 = -1.*t4853*t4888*t5129;
  t5137 = t4819*t4741*t4795;
  t5138 = -1.*t4741*t4772*t4810;
  t5140 = t5137 + t5138;
  t5141 = 0.13205*t5140;
  t5142 = -1.*t4861*t4853*t5129;
  t5143 = -1.*t177*t4867*t5129;
  t5144 = t5142 + t5143;
  t5145 = -0.183*t5144;
  t5146 = t177*t4861*t5129;
  t5147 = -1.*t4853*t4867*t5129;
  t5148 = t5146 + t5147;
  t5149 = -0.2*t5148;
  t5198 = -1.*t4803*t4741*t4853;
  t5199 = -1.*t177*t4925;
  t5200 = t5198 + t5199;
  t4940 = t4861*t4933;
  t5181 = -1.*t4772*t5174;
  t5182 = t5106 + t5181;
  t5213 = -1.*t177*t5182;
  t5214 = t4874 + t5213;
  t5216 = t177*t4741*t4767;
  t5217 = -1.*t4853*t5182;
  t5218 = t5216 + t5217;
  t5231 = t4853*t4799;
  t5232 = -1.*t177*t5140;
  t5233 = t5231 + t5232;
  t5235 = -1.*t177*t4799;
  t5236 = -1.*t4853*t5140;
  t5237 = t5235 + t5236;
  t4938 = -1.*t4867*t4929;
  t4944 = t4938 + t4940;
  t5207 = -1.*t4867*t4933;
  t5248 = -0.183*t4861;
  t5249 = -0.2*t4867;
  t5250 = t5248 + t5249;
  t5252 = 0.2*t4861;
  t5253 = t5252 + t4870;
  t5260 = t4741*t4853*t4767;
  t5261 = t177*t5182;
  t5262 = t5260 + t5261;
  t5221 = t4861*t5218;
  t5225 = -1.*t4867*t5218;
  t5272 = -1.*t4853*t4799;
  t5273 = t177*t5140;
  t5274 = t5272 + t5273;
  t5240 = t4861*t5237;
  t5244 = -1.*t4867*t5237;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=-1.*t4739*t4741*t4767 + t4793*t4812 + t4830*t4844 + 0.13205*(t4772*t4812 + t4850) + t4855*t4858 + t4872*t4881 + t4888*t4892 - 0.183*(-1.*t4867*t4881 + t4861*t4892) - 0.2*(t4861*t4881 + t4867*t4892);
  p_output1[10]=t4739*t4741*t4803 + t4793*t4908 + t4830*t4917 + 0.13205*(t4772*t4908 + t4819*t4917) + t4855*t4925 + t4872*t4929 + t4888*t4933 - 0.2*(t4861*t4929 + t4867*t4933) - 0.183*t4944;
  p_output1[11]=0;
  p_output1[12]=t4741*t4793*t4795*t4803 - 1.*t4739*t4799*t4803 + 0.13205*(t4741*t4772*t4795*t4803 + t4741*t4803*t4810*t4819) + t4741*t4803*t4810*t4830 + t4855*t4960 + t4872*t4968 + t4888*t4975 - 0.183*(-1.*t4867*t4968 + t4861*t4975) - 0.2*(t4861*t4968 + t4867*t4975);
  p_output1[13]=t4741*t4767*t4793*t4795 - 1.*t4739*t4767*t4799 + 0.13205*(t4741*t4767*t4772*t4795 + t4741*t4767*t4810*t4819) + t4741*t4767*t4810*t4830 + t4855*t4998 + t4872*t5002 + t4888*t5008 - 0.183*(-1.*t4867*t5002 + t4861*t5008) - 0.2*(t4861*t5002 + t4867*t5008);
  p_output1[14]=-1.*t4739*t4741 - 1.*t4793*t4795*t4799 + 0.13205*(-1.*t4772*t4795*t4799 - 1.*t4799*t4810*t4819) - 1.*t4799*t4810*t4830 + t4855*t5036 + t4872*t5048 + t4888*t5054 - 0.183*(-1.*t4867*t5048 + t4861*t5054) - 0.2*(t4861*t5048 + t4867*t5054);
  p_output1[15]=t4830*t4908 + t4793*t5068 + 0.13205*(t4923 + t4772*t5068) + t4855*t5075 + t177*t4872*t5075 - 1.*t4853*t4888*t5075 - 0.183*(-1.*t4853*t4861*t5075 - 1.*t177*t4867*t5075) - 0.2*(t177*t4861*t5075 - 1.*t4853*t4867*t5075);
  p_output1[16]=t4793*t4844 + t4830*t5097 + t4855*t5101 + t177*t4872*t5101 - 1.*t4853*t4888*t5101 - 0.183*(-1.*t4853*t4861*t5101 - 1.*t177*t4867*t5101) - 0.2*(t177*t4861*t5101 - 1.*t4853*t4867*t5101) + 0.13205*(t4772*t4844 + t5106);
  p_output1[17]=-1.*t4741*t4793*t4810 + t4741*t4795*t4830 + t5133 + t5134 + t5136 + t5141 + t5145 + t5149;
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
  p_output1[36]=0;
  p_output1[37]=0;
  p_output1[38]=0;
  p_output1[39]=0;
  p_output1[40]=0;
  p_output1[41]=0;
  p_output1[42]=0;
  p_output1[43]=0;
  p_output1[44]=0;
  p_output1[45]=-0.047*t4819*t4908 + 0.047*t4772*t4917 + 0.13205*t4925 + t4855*t5154 + t177*t4872*t5154 - 1.*t4853*t4888*t5154 - 0.183*(-1.*t4853*t4861*t5154 - 1.*t177*t4867*t5154) - 0.2*(t177*t4861*t5154 - 1.*t4853*t4867*t5154);
  p_output1[46]=-0.047*t4819*t5097 + 0.047*t4772*t5174 + t4855*t5177 + t177*t4872*t5177 - 1.*t4853*t4888*t5177 - 0.183*(-1.*t4853*t4861*t5177 - 1.*t177*t4867*t5177) - 0.2*(t177*t4861*t5177 - 1.*t4853*t4867*t5177) + 0.13205*t5182;
  p_output1[47]=0.047*t4741*t4772*t4810 - 0.047*t4741*t4795*t4819 + t5133 + t5134 + t5136 + t5141 + t5145 + t5149;
  p_output1[48]=-0.183*t4741*t4803*t4853 - 0.183*t177*t4925 + t4872*t4933 + t4888*t5200 - 0.2*(t4940 + t4867*t5200) - 0.183*(t4861*t5200 + t5207);
  p_output1[49]=-0.183*t4741*t4767*t4853 - 0.183*t177*t5182 + t4888*t5214 + t4872*t5218 - 0.2*(t4867*t5214 + t5221) - 0.183*(t4861*t5214 + t5225);
  p_output1[50]=0.183*t4799*t4853 - 0.183*t177*t5140 + t4888*t5233 + t4872*t5237 - 0.2*(t4867*t5233 + t5240) - 0.183*(t4861*t5233 + t5244);
  p_output1[51]=-0.2*t4944 - 0.183*(-1.*t4861*t4929 + t5207) + t4929*t5250 + t4933*t5253;
  p_output1[52]=t5218*t5253 + t5250*t5262 - 0.183*(t5225 - 1.*t4861*t5262) - 0.2*(t5221 - 1.*t4867*t5262);
  p_output1[53]=t5237*t5253 + t5250*t5274 - 0.183*(t5244 - 1.*t4861*t5274) - 0.2*(t5240 - 1.*t4867*t5274);
}



void Jp_RL_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
