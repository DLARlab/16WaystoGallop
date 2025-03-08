/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:22 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FR_calf_joint_src.h"

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
  double t808;
  double t830;
  double t832;
  double t829;
  double t840;
  double t818;
  double t831;
  double t844;
  double t845;
  double t857;
  double t858;
  double t861;
  double t864;
  double t767;
  double t872;
  double t873;
  double t875;
  double t757;
  double t877;
  double t892;
  double t893;
  double t896;
  double t898;
  double t905;
  double t907;
  double t908;
  double t884;
  double t885;
  double t888;
  double t777;
  double t792;
  double t800;
  double t804;
  double t820;
  double t821;
  double t824;
  double t825;
  double t865;
  double t866;
  double t925;
  double t928;
  double t932;
  double t934;
  double t935;
  double t936;
  double t878;
  double t879;
  double t942;
  double t943;
  double t944;
  double t897;
  double t900;
  double t901;
  double t909;
  double t910;
  double t911;
  double t952;
  double t953;
  double t956;
  double t946;
  double t947;
  double t948;
  double t978;
  double t980;
  double t981;
  double t992;
  double t993;
  double t994;
  double t988;
  double t989;
  double t990;
  double t1015;
  double t1016;
  double t1017;
  double t1025;
  double t1026;
  double t1027;
  double t1019;
  double t1020;
  double t1021;
  double t1058;
  double t1060;
  double t1062;
  double t1071;
  double t1072;
  double t1073;
  double t1065;
  double t1066;
  double t1067;
  double t1086;
  double t1089;
  double t1092;
  double t1097;
  double t1098;
  double t1099;
  double t1114;
  double t1115;
  double t1116;
  double t868;
  double t1124;
  double t1127;
  double t1154;
  double t1156;
  double t1157;
  double t1174;
  double t1175;
  double t1120;
  double t1191;
  double t1192;
  double t1193;
  double t1198;
  double t1199;
  double t1149;
  double t1151;
  double t1152;
  double t1153;
  double t1158;
  double t1159;
  double t1160;
  double t1161;
  double t1162;
  double t1163;
  double t1164;
  double t1165;
  double t1166;
  double t1167;
  double t1168;
  double t961;
  double t1218;
  double t1219;
  double t1220;
  double t1195;
  double t1196;
  double t1232;
  double t1233;
  double t1234;
  double t1236;
  double t1237;
  double t1250;
  double t1251;
  double t1252;
  double t1254;
  double t1255;
  double t1256;
  double t962;
  double t963;
  double t1226;
  double t1267;
  double t1268;
  double t1269;
  double t1271;
  double t1272;
  double t1239;
  double t1279;
  double t1280;
  double t1281;
  double t1244;
  double t1258;
  double t1291;
  double t1292;
  double t1293;
  double t1263;
  t808 = Sin(var1[3]);
  t830 = Cos(var1[5]);
  t832 = Sin(var1[4]);
  t829 = Cos(var1[3]);
  t840 = Sin(var1[5]);
  t818 = Cos(var1[6]);
  t831 = -1.*t829*t830;
  t844 = -1.*t808*t832*t840;
  t845 = t831 + t844;
  t857 = -1.*t830*t808*t832;
  t858 = t829*t840;
  t861 = t857 + t858;
  t864 = Sin(var1[6]);
  t767 = Cos(var1[7]);
  t872 = t818*t861;
  t873 = -1.*t845*t864;
  t875 = t872 + t873;
  t757 = Cos(var1[4]);
  t877 = Sin(var1[7]);
  t892 = Cos(var1[8]);
  t893 = -1.*t892;
  t896 = 1. + t893;
  t898 = Sin(var1[8]);
  t905 = -1.*t757*t767*t808;
  t907 = -1.*t875*t877;
  t908 = t905 + t907;
  t884 = t767*t875;
  t885 = -1.*t757*t808*t877;
  t888 = t884 + t885;
  t777 = -1.*t767;
  t792 = 1. + t777;
  t800 = 0.183*t792;
  t804 = 0. + t800;
  t820 = -1.*t818;
  t821 = 1. + t820;
  t824 = -0.047*t821;
  t825 = 0. + t824;
  t865 = 0.047*t864;
  t866 = 0. + t865;
  t925 = -1.*t830*t808;
  t928 = t829*t832*t840;
  t932 = t925 + t928;
  t934 = t829*t830*t832;
  t935 = t808*t840;
  t936 = t934 + t935;
  t878 = 0.183*t877;
  t879 = 0. + t878;
  t942 = t818*t936;
  t943 = -1.*t932*t864;
  t944 = t942 + t943;
  t897 = -0.2*t896;
  t900 = 0.183*t898;
  t901 = 0. + t897 + t900;
  t909 = 0.183*t896;
  t910 = 0.2*t898;
  t911 = 0. + t909 + t910;
  t952 = t829*t757*t767;
  t953 = -1.*t944*t877;
  t956 = t952 + t953;
  t946 = t767*t944;
  t947 = t829*t757*t877;
  t948 = t946 + t947;
  t978 = t829*t757*t830*t818;
  t980 = -1.*t829*t757*t840*t864;
  t981 = t978 + t980;
  t992 = -1.*t829*t767*t832;
  t993 = -1.*t981*t877;
  t994 = t992 + t993;
  t988 = t767*t981;
  t989 = -1.*t829*t832*t877;
  t990 = t988 + t989;
  t1015 = t757*t830*t818*t808;
  t1016 = -1.*t757*t808*t840*t864;
  t1017 = t1015 + t1016;
  t1025 = -1.*t767*t808*t832;
  t1026 = -1.*t1017*t877;
  t1027 = t1025 + t1026;
  t1019 = t767*t1017;
  t1020 = -1.*t808*t832*t877;
  t1021 = t1019 + t1020;
  t1058 = -1.*t830*t818*t832;
  t1060 = t832*t840*t864;
  t1062 = t1058 + t1060;
  t1071 = -1.*t757*t767;
  t1072 = -1.*t1062*t877;
  t1073 = t1071 + t1072;
  t1065 = t767*t1062;
  t1066 = -1.*t757*t877;
  t1067 = t1065 + t1066;
  t1086 = t830*t808;
  t1089 = -1.*t829*t832*t840;
  t1092 = t1086 + t1089;
  t1097 = t818*t1092;
  t1098 = -1.*t936*t864;
  t1099 = t1097 + t1098;
  t1114 = t830*t808*t832;
  t1115 = -1.*t829*t840;
  t1116 = t1114 + t1115;
  t868 = t818*t845;
  t1124 = -1.*t1116*t864;
  t1127 = t868 + t1124;
  t1154 = -1.*t757*t818*t840;
  t1156 = -1.*t757*t830*t864;
  t1157 = t1154 + t1156;
  t1174 = -1.*t818*t932;
  t1175 = t1174 + t1098;
  t1120 = t818*t1116;
  t1191 = t829*t830;
  t1192 = t808*t832*t840;
  t1193 = t1191 + t1192;
  t1198 = -1.*t818*t1193;
  t1199 = t1198 + t1124;
  t1149 = t757*t830*t818;
  t1151 = -1.*t757*t840*t864;
  t1152 = t1149 + t1151;
  t1153 = -0.13205*t1152;
  t1158 = t1157*t879;
  t1159 = t767*t1157*t901;
  t1160 = -1.*t1157*t877*t911;
  t1161 = -1.*t892*t1157*t877;
  t1162 = -1.*t767*t1157*t898;
  t1163 = t1161 + t1162;
  t1164 = 0.183*t1163;
  t1165 = t767*t892*t1157;
  t1166 = -1.*t1157*t877*t898;
  t1167 = t1165 + t1166;
  t1168 = -0.2*t1167;
  t961 = t892*t956;
  t1218 = -1.*t767*t944;
  t1219 = -1.*t829*t757*t877;
  t1220 = t1218 + t1219;
  t1195 = -1.*t1193*t864;
  t1196 = t1120 + t1195;
  t1232 = t757*t767*t808;
  t1233 = -1.*t1196*t877;
  t1234 = t1232 + t1233;
  t1236 = -1.*t767*t1196;
  t1237 = t1236 + t885;
  t1250 = -1.*t767*t832;
  t1251 = -1.*t1152*t877;
  t1252 = t1250 + t1251;
  t1254 = -1.*t767*t1152;
  t1255 = t832*t877;
  t1256 = t1254 + t1255;
  t962 = -1.*t948*t898;
  t963 = t961 + t962;
  t1226 = -1.*t956*t898;
  t1267 = 0.183*t892;
  t1268 = -0.2*t898;
  t1269 = t1267 + t1268;
  t1271 = 0.2*t892;
  t1272 = t1271 + t900;
  t1239 = t892*t1234;
  t1279 = t767*t1196;
  t1280 = t757*t808*t877;
  t1281 = t1279 + t1280;
  t1244 = -1.*t1234*t898;
  t1258 = t892*t1252;
  t1291 = t767*t1152;
  t1292 = -1.*t832*t877;
  t1293 = t1291 + t1292;
  t1263 = -1.*t1252*t898;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=-1.*t757*t804*t808 + t825*t845 + t861*t866 - 0.13205*(t861*t864 + t868) + t875*t879 + t888*t901 + 0.183*(-1.*t888*t898 + t892*t908) - 0.2*(t888*t892 + t898*t908) + t908*t911;
  p_output1[10]=t757*t804*t829 + t825*t932 + t866*t936 - 0.13205*(t818*t932 + t864*t936) + t879*t944 + t901*t948 + t911*t956 - 0.2*(t892*t948 + t898*t956) + 0.183*t963;
  p_output1[11]=0;
  p_output1[12]=-1.*t804*t829*t832 + t757*t825*t829*t840 - 0.13205*(t757*t818*t829*t840 + t757*t829*t830*t864) + t757*t829*t830*t866 + t879*t981 + t901*t990 + t911*t994 + 0.183*(-1.*t898*t990 + t892*t994) - 0.2*(t892*t990 + t898*t994);
  p_output1[13]=-1.*t804*t808*t832 + t757*t808*t825*t840 - 0.13205*(t757*t808*t818*t840 + t757*t808*t830*t864) + t757*t808*t830*t866 + t1017*t879 + 0.183*(t1027*t892 - 1.*t1021*t898) - 0.2*(t1021*t892 + t1027*t898) + t1021*t901 + t1027*t911;
  p_output1[14]=-1.*t757*t804 - 1.*t825*t832*t840 - 0.13205*(-1.*t818*t832*t840 - 1.*t830*t832*t864) - 1.*t830*t832*t866 + t1062*t879 + 0.183*(t1073*t892 - 1.*t1067*t898) - 0.2*(t1067*t892 + t1073*t898) + t1067*t901 + t1073*t911;
  p_output1[15]=t1092*t866 + t1099*t879 + 0.183*(-1.*t1099*t877*t892 - 1.*t1099*t767*t898) - 0.2*(t1099*t767*t892 - 1.*t1099*t877*t898) + t1099*t767*t901 - 1.*t1099*t877*t911 + t825*t936 - 0.13205*(t1092*t864 + t942);
  p_output1[16]=t1116*t825 - 0.13205*(t1120 + t845*t864) + t845*t866 + t1127*t879 + 0.183*(-1.*t1127*t877*t892 - 1.*t1127*t767*t898) - 0.2*(t1127*t767*t892 - 1.*t1127*t877*t898) + t1127*t767*t901 - 1.*t1127*t877*t911;
  p_output1[17]=t1153 + t1158 + t1159 + t1160 + t1164 + t1168 + t757*t825*t830 - 1.*t757*t840*t866;
  p_output1[18]=t1175*t879 + 0.183*(-1.*t1175*t877*t892 - 1.*t1175*t767*t898) - 0.2*(t1175*t767*t892 - 1.*t1175*t877*t898) + t1175*t767*t901 - 1.*t1175*t877*t911 - 0.047*t864*t932 + 0.047*t818*t936 - 0.13205*t944;
  p_output1[19]=-0.13205*t1196 + 0.047*t1116*t818 - 0.047*t1193*t864 + t1199*t879 + 0.183*(-1.*t1199*t877*t892 - 1.*t1199*t767*t898) - 0.2*(t1199*t767*t892 - 1.*t1199*t877*t898) + t1199*t767*t901 - 1.*t1199*t877*t911;
  p_output1[20]=t1153 + t1158 + t1159 + t1160 + t1164 + t1168 + 0.047*t757*t818*t830 - 0.047*t757*t840*t864;
  p_output1[21]=0.183*t757*t829*t877 + 0.183*(t1226 + t1220*t892) + t1220*t911 + 0.183*t767*t944 + t901*t956 - 0.2*(t1220*t898 + t961);
  p_output1[22]=0.183*t1196*t767 + 0.183*t757*t808*t877 + 0.183*(t1244 + t1237*t892) - 0.2*(t1239 + t1237*t898) + t1234*t901 + t1237*t911;
  p_output1[23]=0.183*t1152*t767 - 0.183*t832*t877 + 0.183*(t1263 + t1256*t892) - 0.2*(t1258 + t1256*t898) + t1252*t901 + t1256*t911;
  p_output1[24]=t1269*t948 + 0.183*(t1226 - 1.*t892*t948) + t1272*t956 - 0.2*t963;
  p_output1[25]=t1234*t1272 + t1269*t1281 + 0.183*(t1244 - 1.*t1281*t892) - 0.2*(t1239 - 1.*t1281*t898);
  p_output1[26]=t1252*t1272 + t1269*t1293 + 0.183*(t1263 - 1.*t1293*t892) - 0.2*(t1258 - 1.*t1293*t898);
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



void Jp_FR_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
