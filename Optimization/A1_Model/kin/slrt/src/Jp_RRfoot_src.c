/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:08 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RRfoot_src.h"

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
  double t542;
  double t1083;
  double t881;
  double t935;
  double t1138;
  double t715;
  double t1082;
  double t1428;
  double t1456;
  double t2287;
  double t2890;
  double t2898;
  double t2905;
  double t506;
  double t3667;
  double t33;
  double t6684;
  double t6718;
  double t6720;
  double t6732;
  double t6746;
  double t6749;
  double t6751;
  double t6759;
  double t6761;
  double t6773;
  double t6787;
  double t6788;
  double t6789;
  double t35;
  double t38;
  double t40;
  double t306;
  double t749;
  double t750;
  double t2291;
  double t2350;
  double t2882;
  double t2887;
  double t6818;
  double t6819;
  double t6821;
  double t6825;
  double t6826;
  double t6827;
  double t3774;
  double t6653;
  double t6750;
  double t6753;
  double t6756;
  double t6839;
  double t6844;
  double t6845;
  double t6781;
  double t6782;
  double t6784;
  double t6850;
  double t6851;
  double t6852;
  double t6854;
  double t6855;
  double t6856;
  double t6881;
  double t6882;
  double t6883;
  double t6885;
  double t6886;
  double t6887;
  double t6889;
  double t6890;
  double t6891;
  double t6916;
  double t6917;
  double t6918;
  double t6920;
  double t6922;
  double t6924;
  double t6928;
  double t6932;
  double t6933;
  double t6954;
  double t6955;
  double t6956;
  double t6958;
  double t6959;
  double t6960;
  double t6962;
  double t6963;
  double t6964;
  double t6980;
  double t6981;
  double t6982;
  double t6984;
  double t6985;
  double t6986;
  double t7018;
  double t7019;
  double t7020;
  double t2912;
  double t7023;
  double t7024;
  double t7048;
  double t7049;
  double t7050;
  double t7075;
  double t7077;
  double t7098;
  double t7100;
  double t7101;
  double t7103;
  double t7104;
  double t7028;
  double t7051;
  double t7052;
  double t7055;
  double t7056;
  double t7057;
  double t7058;
  double t7059;
  double t7060;
  double t7061;
  double t7063;
  double t7064;
  double t7065;
  double t7066;
  double t7067;
  double t7069;
  double t7126;
  double t7128;
  double t7130;
  double t6859;
  double t7108;
  double t7109;
  double t7143;
  double t7144;
  double t7146;
  double t7147;
  double t7148;
  double t7161;
  double t7162;
  double t7163;
  double t7165;
  double t7166;
  double t7167;
  double t6858;
  double t6860;
  double t7137;
  double t7178;
  double t7179;
  double t7180;
  double t7182;
  double t7183;
  double t7190;
  double t7191;
  double t7192;
  double t7151;
  double t7155;
  double t7202;
  double t7203;
  double t7204;
  double t7170;
  double t7174;
  t542 = Sin(var1[3]);
  t1083 = Cos(var1[3]);
  t881 = Cos(var1[5]);
  t935 = Sin(var1[4]);
  t1138 = Sin(var1[5]);
  t715 = Sin(var1[12]);
  t1082 = -1.*t881*t542*t935;
  t1428 = t1083*t1138;
  t1456 = t1082 + t1428;
  t2287 = Cos(var1[12]);
  t2890 = -1.*t1083*t881;
  t2898 = -1.*t542*t935*t1138;
  t2905 = t2890 + t2898;
  t506 = Cos(var1[4]);
  t3667 = Sin(var1[13]);
  t33 = Cos(var1[13]);
  t6684 = t2287*t1456;
  t6718 = -1.*t715*t2905;
  t6720 = t6684 + t6718;
  t6732 = Cos(var1[14]);
  t6746 = -1.*t6732;
  t6749 = 1. + t6746;
  t6751 = Sin(var1[14]);
  t6759 = -1.*t506*t3667*t542;
  t6761 = t33*t6720;
  t6773 = t6759 + t6761;
  t6787 = -1.*t33*t506*t542;
  t6788 = -1.*t3667*t6720;
  t6789 = t6787 + t6788;
  t35 = -1.*t33;
  t38 = 1. + t35;
  t40 = -0.183*t38;
  t306 = 0. + t40;
  t749 = 0.047*t715;
  t750 = 0. + t749;
  t2291 = -1.*t2287;
  t2350 = 1. + t2291;
  t2882 = -0.047*t2350;
  t2887 = 0. + t2882;
  t6818 = t1083*t881*t935;
  t6819 = t542*t1138;
  t6821 = t6818 + t6819;
  t6825 = -1.*t881*t542;
  t6826 = t1083*t935*t1138;
  t6827 = t6825 + t6826;
  t3774 = -0.183*t3667;
  t6653 = 0. + t3774;
  t6750 = -0.2*t6749;
  t6753 = -0.183*t6751;
  t6756 = 0. + t6750 + t6753;
  t6839 = t2287*t6821;
  t6844 = -1.*t715*t6827;
  t6845 = t6839 + t6844;
  t6781 = -0.183*t6749;
  t6782 = 0.2*t6751;
  t6784 = 0. + t6781 + t6782;
  t6850 = t1083*t506*t3667;
  t6851 = t33*t6845;
  t6852 = t6850 + t6851;
  t6854 = t33*t1083*t506;
  t6855 = -1.*t3667*t6845;
  t6856 = t6854 + t6855;
  t6881 = t2287*t1083*t506*t881;
  t6882 = -1.*t1083*t506*t715*t1138;
  t6883 = t6881 + t6882;
  t6885 = -1.*t1083*t3667*t935;
  t6886 = t33*t6883;
  t6887 = t6885 + t6886;
  t6889 = -1.*t33*t1083*t935;
  t6890 = -1.*t3667*t6883;
  t6891 = t6889 + t6890;
  t6916 = t2287*t506*t881*t542;
  t6917 = -1.*t506*t715*t542*t1138;
  t6918 = t6916 + t6917;
  t6920 = -1.*t3667*t542*t935;
  t6922 = t33*t6918;
  t6924 = t6920 + t6922;
  t6928 = -1.*t33*t542*t935;
  t6932 = -1.*t3667*t6918;
  t6933 = t6928 + t6932;
  t6954 = -1.*t2287*t881*t935;
  t6955 = t715*t935*t1138;
  t6956 = t6954 + t6955;
  t6958 = -1.*t506*t3667;
  t6959 = t33*t6956;
  t6960 = t6958 + t6959;
  t6962 = -1.*t33*t506;
  t6963 = -1.*t3667*t6956;
  t6964 = t6962 + t6963;
  t6980 = t881*t542;
  t6981 = -1.*t1083*t935*t1138;
  t6982 = t6980 + t6981;
  t6984 = -1.*t715*t6821;
  t6985 = t2287*t6982;
  t6986 = t6984 + t6985;
  t7018 = t881*t542*t935;
  t7019 = -1.*t1083*t1138;
  t7020 = t7018 + t7019;
  t2912 = t2287*t2905;
  t7023 = -1.*t715*t7020;
  t7024 = t7023 + t2912;
  t7048 = -1.*t506*t881*t715;
  t7049 = -1.*t2287*t506*t1138;
  t7050 = t7048 + t7049;
  t7075 = -1.*t2287*t6827;
  t7077 = t6984 + t7075;
  t7098 = t1083*t881;
  t7100 = t542*t935*t1138;
  t7101 = t7098 + t7100;
  t7103 = -1.*t2287*t7101;
  t7104 = t7023 + t7103;
  t7028 = t2287*t7020;
  t7051 = t6653*t7050;
  t7052 = t33*t6756*t7050;
  t7055 = -1.*t3667*t6784*t7050;
  t7056 = t2287*t506*t881;
  t7057 = -1.*t506*t715*t1138;
  t7058 = t7056 + t7057;
  t7059 = -0.13205*t7058;
  t7060 = -1.*t6732*t3667*t7050;
  t7061 = -1.*t33*t6751*t7050;
  t7063 = t7060 + t7061;
  t7064 = -0.183*t7063;
  t7065 = t33*t6732*t7050;
  t7066 = -1.*t3667*t6751*t7050;
  t7067 = t7065 + t7066;
  t7069 = -0.4*t7067;
  t7126 = -1.*t1083*t506*t3667;
  t7128 = -1.*t33*t6845;
  t7130 = t7126 + t7128;
  t6859 = t6732*t6856;
  t7108 = -1.*t715*t7101;
  t7109 = t7028 + t7108;
  t7143 = -1.*t33*t7109;
  t7144 = t6759 + t7143;
  t7146 = t33*t506*t542;
  t7147 = -1.*t3667*t7109;
  t7148 = t7146 + t7147;
  t7161 = t3667*t935;
  t7162 = -1.*t33*t7058;
  t7163 = t7161 + t7162;
  t7165 = -1.*t33*t935;
  t7166 = -1.*t3667*t7058;
  t7167 = t7165 + t7166;
  t6858 = -1.*t6751*t6852;
  t6860 = t6858 + t6859;
  t7137 = -1.*t6751*t6856;
  t7178 = -0.183*t6732;
  t7179 = -0.2*t6751;
  t7180 = t7178 + t7179;
  t7182 = 0.2*t6732;
  t7183 = t7182 + t6753;
  t7190 = t506*t3667*t542;
  t7191 = t33*t7109;
  t7192 = t7190 + t7191;
  t7151 = t6732*t7148;
  t7155 = -1.*t6751*t7148;
  t7202 = -1.*t3667*t935;
  t7203 = t33*t7058;
  t7204 = t7202 + t7203;
  t7170 = t6732*t7167;
  t7174 = -1.*t6751*t7167;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=t2887*t2905 - 1.*t306*t506*t542 + t6653*t6720 + t6756*t6773 + t6784*t6789 - 0.183*(-1.*t6751*t6773 + t6732*t6789) - 0.4*(t6732*t6773 + t6751*t6789) - 0.13205*(t2912 + t1456*t715) + t1456*t750;
  p_output1[10]=t1083*t306*t506 + t2887*t6827 + t6653*t6845 + t6756*t6852 + t6784*t6856 - 0.4*(t6732*t6852 + t6751*t6856) - 0.183*t6860 - 0.13205*(t2287*t6827 + t6821*t715) + t6821*t750;
  p_output1[11]=0;
  p_output1[12]=t1083*t1138*t2887*t506 + t6653*t6883 + t6756*t6887 + t6784*t6891 - 0.183*(-1.*t6751*t6887 + t6732*t6891) - 0.4*(t6732*t6887 + t6751*t6891) + t1083*t506*t750*t881 - 0.13205*(t1083*t1138*t2287*t506 + t1083*t506*t715*t881) - 1.*t1083*t306*t935;
  p_output1[13]=t1138*t2887*t506*t542 + t6653*t6918 + t6756*t6924 + t6784*t6933 - 0.183*(-1.*t6751*t6924 + t6732*t6933) - 0.4*(t6732*t6924 + t6751*t6933) + t506*t542*t750*t881 - 0.13205*(t1138*t2287*t506*t542 + t506*t542*t715*t881) - 1.*t306*t542*t935;
  p_output1[14]=-1.*t306*t506 + t6653*t6956 + t6756*t6960 + t6784*t6964 - 0.183*(-1.*t6751*t6960 + t6732*t6964) - 0.4*(t6732*t6960 + t6751*t6964) - 1.*t1138*t2887*t935 - 1.*t750*t881*t935 - 0.13205*(-1.*t1138*t2287*t935 - 1.*t715*t881*t935);
  p_output1[15]=t2887*t6821 + t6653*t6986 + t33*t6756*t6986 - 1.*t3667*t6784*t6986 - 0.183*(-1.*t3667*t6732*t6986 - 1.*t33*t6751*t6986) - 0.4*(t33*t6732*t6986 - 1.*t3667*t6751*t6986) - 0.13205*(t6839 + t6982*t715) + t6982*t750;
  p_output1[16]=t2887*t7020 + t6653*t7024 + t33*t6756*t7024 - 1.*t3667*t6784*t7024 - 0.183*(-1.*t3667*t6732*t7024 - 1.*t33*t6751*t7024) - 0.4*(t33*t6732*t7024 - 1.*t3667*t6751*t7024) - 0.13205*(t7028 + t2905*t715) + t2905*t750;
  p_output1[17]=t7051 + t7052 + t7055 + t7059 + t7064 + t7069 - 1.*t1138*t506*t750 + t2887*t506*t881;
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
  p_output1[36]=0.047*t2287*t6821 - 0.13205*t6845 + t6653*t7077 + t33*t6756*t7077 - 1.*t3667*t6784*t7077 - 0.183*(-1.*t3667*t6732*t7077 - 1.*t33*t6751*t7077) - 0.4*(t33*t6732*t7077 - 1.*t3667*t6751*t7077) - 0.047*t6827*t715;
  p_output1[37]=0.047*t2287*t7020 + t6653*t7104 + t33*t6756*t7104 - 1.*t3667*t6784*t7104 - 0.183*(-1.*t3667*t6732*t7104 - 1.*t33*t6751*t7104) - 0.4*(t33*t6732*t7104 - 1.*t3667*t6751*t7104) - 0.13205*t7109 - 0.047*t7101*t715;
  p_output1[38]=t7051 + t7052 + t7055 + t7059 + t7064 + t7069 - 0.047*t1138*t506*t715 + 0.047*t2287*t506*t881;
  p_output1[39]=-0.183*t1083*t3667*t506 - 0.183*t33*t6845 + t6756*t6856 + t6784*t7130 - 0.4*(t6859 + t6751*t7130) - 0.183*(t6732*t7130 + t7137);
  p_output1[40]=-0.183*t3667*t506*t542 - 0.183*t33*t7109 + t6784*t7144 + t6756*t7148 - 0.4*(t6751*t7144 + t7151) - 0.183*(t6732*t7144 + t7155);
  p_output1[41]=-0.183*t33*t7058 + t6784*t7163 + t6756*t7167 - 0.4*(t6751*t7163 + t7170) - 0.183*(t6732*t7163 + t7174) + 0.183*t3667*t935;
  p_output1[42]=-0.4*t6860 - 0.183*(-1.*t6732*t6852 + t7137) + t6852*t7180 + t6856*t7183;
  p_output1[43]=t7148*t7183 + t7180*t7192 - 0.183*(t7155 - 1.*t6732*t7192) - 0.4*(t7151 - 1.*t6751*t7192);
  p_output1[44]=t7167*t7183 + t7180*t7204 - 0.183*(t7174 - 1.*t6732*t7204) - 0.4*(t7170 - 1.*t6751*t7204);
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



void Jp_RRfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
