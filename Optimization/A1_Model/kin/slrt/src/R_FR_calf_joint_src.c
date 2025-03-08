/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:24 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_FR_calf_joint_src.h"

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
  double t1222;
  double t1286;
  double t1296;
  double t1294;
  double t1297;
  double t1246;
  double t1275;
  double t1295;
  double t1298;
  double t1299;
  double t1300;
  double t1302;
  double t1305;
  double t1308;
  double t1309;
  double t1310;
  double t1311;
  double t1225;
  double t1312;
  double t1210;
  double t1323;
  double t1324;
  double t1325;
  double t1328;
  double t1329;
  double t1330;
  double t1331;
  double t1332;
  double t1333;
  double t1319;
  double t1348;
  double t1349;
  double t1350;
  double t1316;
  double t1317;
  double t1318;
  double t1257;
  double t1313;
  double t1314;
  double t1338;
  double t1339;
  double t1340;
  double t1322;
  double t1335;
  double t1336;
  double t1355;
  double t1356;
  double t1357;
  double t1346;
  double t1351;
  double t1353;
  t1222 = Cos(var1[3]);
  t1286 = Cos(var1[5]);
  t1296 = Sin(var1[3]);
  t1294 = Sin(var1[4]);
  t1297 = Sin(var1[5]);
  t1246 = Cos(var1[7]);
  t1275 = Cos(var1[6]);
  t1295 = t1222*t1286*t1294;
  t1298 = t1296*t1297;
  t1299 = t1295 + t1298;
  t1300 = t1275*t1299;
  t1302 = -1.*t1286*t1296;
  t1305 = t1222*t1294*t1297;
  t1308 = t1302 + t1305;
  t1309 = Sin(var1[6]);
  t1310 = -1.*t1308*t1309;
  t1311 = t1300 + t1310;
  t1225 = Cos(var1[4]);
  t1312 = Sin(var1[7]);
  t1210 = Cos(var1[8]);
  t1323 = t1286*t1296*t1294;
  t1324 = -1.*t1222*t1297;
  t1325 = t1323 + t1324;
  t1328 = t1275*t1325;
  t1329 = t1222*t1286;
  t1330 = t1296*t1294*t1297;
  t1331 = t1329 + t1330;
  t1332 = -1.*t1331*t1309;
  t1333 = t1328 + t1332;
  t1319 = Sin(var1[8]);
  t1348 = t1225*t1286*t1275;
  t1349 = -1.*t1225*t1297*t1309;
  t1350 = t1348 + t1349;
  t1316 = t1246*t1311;
  t1317 = t1222*t1225*t1312;
  t1318 = t1316 + t1317;
  t1257 = t1222*t1225*t1246;
  t1313 = -1.*t1311*t1312;
  t1314 = t1257 + t1313;
  t1338 = t1246*t1333;
  t1339 = t1225*t1296*t1312;
  t1340 = t1338 + t1339;
  t1322 = t1225*t1246*t1296;
  t1335 = -1.*t1333*t1312;
  t1336 = t1322 + t1335;
  t1355 = t1246*t1350;
  t1356 = -1.*t1294*t1312;
  t1357 = t1355 + t1356;
  t1346 = -1.*t1246*t1294;
  t1351 = -1.*t1350*t1312;
  t1353 = t1346 + t1351;
  p_output1[0]=t1210*t1314 - 1.*t1318*t1319;
  p_output1[1]=t1210*t1336 - 1.*t1319*t1340;
  p_output1[2]=t1210*t1353 - 1.*t1319*t1357;
  p_output1[3]=t1275*t1308 + t1299*t1309;
  p_output1[4]=t1309*t1325 + t1275*t1331;
  p_output1[5]=t1225*t1275*t1297 + t1225*t1286*t1309;
  p_output1[6]=t1210*t1318 + t1314*t1319;
  p_output1[7]=t1319*t1336 + t1210*t1340;
  p_output1[8]=t1319*t1353 + t1210*t1357;
}



void R_FR_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
