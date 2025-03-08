/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:23 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FR_calf_joint_src.h"

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
  double t917;
  double t966;
  double t970;
  double t968;
  double t974;
  double t921;
  double t937;
  double t969;
  double t975;
  double t976;
  double t984;
  double t991;
  double t996;
  double t997;
  double t1004;
  double t1005;
  double t1006;
  double t920;
  double t1011;
  double t910;
  double t1046;
  double t1060;
  double t1070;
  double t1074;
  double t1077;
  double t1081;
  double t1082;
  double t1083;
  double t1084;
  double t1042;
  double t1109;
  double t1112;
  double t1117;
  double t1036;
  double t1037;
  double t1041;
  double t934;
  double t1015;
  double t1028;
  double t1094;
  double t1095;
  double t1100;
  double t1045;
  double t1086;
  double t1092;
  double t1130;
  double t1132;
  double t1135;
  double t1106;
  double t1122;
  double t1123;
  double t1139;
  double t1140;
  double t1141;
  double t1214;
  double t1215;
  double t1035;
  double t1043;
  double t1044;
  double t1170;
  double t1176;
  double t1180;
  double t1189;
  double t1190;
  double t1191;
  double t1192;
  double t1200;
  double t1201;
  double t1202;
  double t1203;
  double t1205;
  double t1208;
  double t1145;
  double t1147;
  double t1148;
  double t1211;
  double t1212;
  double t1216;
  double t1217;
  double t1220;
  double t1222;
  double t1225;
  double t1227;
  double t1093;
  double t1102;
  double t1105;
  double t1181;
  double t1182;
  double t1183;
  double t1149;
  double t1165;
  double t1169;
  double t1129;
  double t1137;
  double t1138;
  double t1186;
  double t1187;
  double t1188;
  t917 = Cos(var1[3]);
  t966 = Cos(var1[5]);
  t970 = Sin(var1[3]);
  t968 = Sin(var1[4]);
  t974 = Sin(var1[5]);
  t921 = Cos(var1[7]);
  t937 = Cos(var1[6]);
  t969 = t917*t966*t968;
  t975 = t970*t974;
  t976 = t969 + t975;
  t984 = t937*t976;
  t991 = -1.*t966*t970;
  t996 = t917*t968*t974;
  t997 = t991 + t996;
  t1004 = Sin(var1[6]);
  t1005 = -1.*t997*t1004;
  t1006 = t984 + t1005;
  t920 = Cos(var1[4]);
  t1011 = Sin(var1[7]);
  t910 = Cos(var1[8]);
  t1046 = t966*t970*t968;
  t1060 = -1.*t917*t974;
  t1070 = t1046 + t1060;
  t1074 = t937*t1070;
  t1077 = t917*t966;
  t1081 = t970*t968*t974;
  t1082 = t1077 + t1081;
  t1083 = -1.*t1082*t1004;
  t1084 = t1074 + t1083;
  t1042 = Sin(var1[8]);
  t1109 = t920*t966*t937;
  t1112 = -1.*t920*t974*t1004;
  t1117 = t1109 + t1112;
  t1036 = t921*t1006;
  t1037 = t917*t920*t1011;
  t1041 = t1036 + t1037;
  t934 = t917*t920*t921;
  t1015 = -1.*t1006*t1011;
  t1028 = t934 + t1015;
  t1094 = t921*t1084;
  t1095 = t920*t970*t1011;
  t1100 = t1094 + t1095;
  t1045 = t920*t921*t970;
  t1086 = -1.*t1084*t1011;
  t1092 = t1045 + t1086;
  t1130 = t921*t1117;
  t1132 = -1.*t968*t1011;
  t1135 = t1130 + t1132;
  t1106 = -1.*t921*t968;
  t1122 = -1.*t1117*t1011;
  t1123 = t1106 + t1122;
  t1139 = t937*t997;
  t1140 = t976*t1004;
  t1141 = t1139 + t1140;
  t1214 = -1.*t910;
  t1215 = 1. + t1214;
  t1035 = t910*t1028;
  t1043 = -1.*t1041*t1042;
  t1044 = t1035 + t1043;
  t1170 = t910*t1041;
  t1176 = t1028*t1042;
  t1180 = t1170 + t1176;
  t1189 = -1.*t921;
  t1190 = 1. + t1189;
  t1191 = 0.183*t1190;
  t1192 = 0. + t1191;
  t1200 = -1.*t937;
  t1201 = 1. + t1200;
  t1202 = -0.047*t1201;
  t1203 = 0. + t1202;
  t1205 = 0.047*t1004;
  t1208 = 0. + t1205;
  t1145 = t937*t1082;
  t1147 = t1070*t1004;
  t1148 = t1145 + t1147;
  t1211 = 0.183*t1011;
  t1212 = 0. + t1211;
  t1216 = -0.2*t1215;
  t1217 = 0.183*t1042;
  t1220 = 0. + t1216 + t1217;
  t1222 = 0.183*t1215;
  t1225 = 0.2*t1042;
  t1227 = 0. + t1222 + t1225;
  t1093 = t910*t1092;
  t1102 = -1.*t1100*t1042;
  t1105 = t1093 + t1102;
  t1181 = t910*t1100;
  t1182 = t1092*t1042;
  t1183 = t1181 + t1182;
  t1149 = t920*t937*t974;
  t1165 = t920*t966*t1004;
  t1169 = t1149 + t1165;
  t1129 = t910*t1123;
  t1137 = -1.*t1135*t1042;
  t1138 = t1129 + t1137;
  t1186 = t910*t1135;
  t1187 = t1123*t1042;
  t1188 = t1186 + t1187;
  p_output1[0]=t1044;
  p_output1[1]=t1105;
  p_output1[2]=t1138;
  p_output1[3]=0.;
  p_output1[4]=t1141;
  p_output1[5]=t1148;
  p_output1[6]=t1169;
  p_output1[7]=0.;
  p_output1[8]=t1180;
  p_output1[9]=t1183;
  p_output1[10]=t1188;
  p_output1[11]=0.;
  p_output1[12]=0. + 0.183*t1044 - 0.13205*t1141 - 0.2*t1180 + t1006*t1212 + t1041*t1220 + t1028*t1227 + t1192*t917*t920 + t1208*t976 + t1203*t997 + var1[0];
  p_output1[13]=0. + 0.183*t1105 - 0.13205*t1148 - 0.2*t1183 + t1082*t1203 + t1070*t1208 + t1084*t1212 + t1100*t1220 + t1092*t1227 + t1192*t920*t970 + var1[1];
  p_output1[14]=0. + 0.183*t1138 - 0.13205*t1169 - 0.2*t1188 + t1117*t1212 + t1135*t1220 + t1123*t1227 + t1208*t920*t966 - 1.*t1192*t968 + t1203*t920*t974 + var1[2];
  p_output1[15]=1.;
}



void H_FR_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
