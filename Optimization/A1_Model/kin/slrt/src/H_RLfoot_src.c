/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:13 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RLfoot_src.h"

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
  double t91;
  double t887;
  double t1251;
  double t891;
  double t1268;
  double t703;
  double t514;
  double t544;
  double t833;
  double t1047;
  double t1337;
  double t1361;
  double t1533;
  double t1681;
  double t2047;
  double t2102;
  double t2174;
  double t2289;
  double t2325;
  double t9;
  double t2702;
  double t2787;
  double t2861;
  double t2897;
  double t3212;
  double t4162;
  double t4736;
  double t7250;
  double t7251;
  double t7259;
  double t7363;
  double t7365;
  double t7366;
  double t568;
  double t2686;
  double t2693;
  double t2709;
  double t2710;
  double t2730;
  double t2776;
  double t7273;
  double t7288;
  double t7300;
  double t7306;
  double t7335;
  double t7349;
  double t7369;
  double t7374;
  double t7377;
  double t7379;
  double t7380;
  double t7405;
  double t7408;
  double t7409;
  double t7502;
  double t7503;
  double t2701;
  double t2744;
  double t2775;
  double t7429;
  double t7436;
  double t7440;
  double t7461;
  double t7462;
  double t7463;
  double t7464;
  double t7466;
  double t7467;
  double t7469;
  double t7470;
  double t7471;
  double t7477;
  double t7413;
  double t7416;
  double t7421;
  double t7497;
  double t7499;
  double t7504;
  double t7505;
  double t7506;
  double t7508;
  double t7509;
  double t7510;
  double t7293;
  double t7339;
  double t7348;
  double t7443;
  double t7447;
  double t7451;
  double t7423;
  double t7427;
  double t7428;
  double t7376;
  double t7402;
  double t7404;
  double t7458;
  double t7459;
  double t7460;
  t91 = Cos(var1[3]);
  t887 = Cos(var1[5]);
  t1251 = Sin(var1[3]);
  t891 = Sin(var1[4]);
  t1268 = Sin(var1[5]);
  t703 = Cos(var1[16]);
  t514 = Cos(var1[4]);
  t544 = Sin(var1[16]);
  t833 = Cos(var1[15]);
  t1047 = t91*t887*t891;
  t1337 = t1251*t1268;
  t1361 = t1047 + t1337;
  t1533 = t833*t1361;
  t1681 = Sin(var1[15]);
  t2047 = -1.*t887*t1251;
  t2102 = t91*t891*t1268;
  t2174 = t2047 + t2102;
  t2289 = -1.*t1681*t2174;
  t2325 = t1533 + t2289;
  t9 = Sin(var1[17]);
  t2702 = Cos(var1[17]);
  t2787 = t887*t1251*t891;
  t2861 = -1.*t91*t1268;
  t2897 = t2787 + t2861;
  t3212 = t833*t2897;
  t4162 = t91*t887;
  t4736 = t1251*t891*t1268;
  t7250 = t4162 + t4736;
  t7251 = -1.*t1681*t7250;
  t7259 = t3212 + t7251;
  t7363 = t833*t514*t887;
  t7365 = -1.*t514*t1681*t1268;
  t7366 = t7363 + t7365;
  t568 = t91*t514*t544;
  t2686 = t703*t2325;
  t2693 = t568 + t2686;
  t2709 = t703*t91*t514;
  t2710 = -1.*t544*t2325;
  t2730 = t2709 + t2710;
  t2776 = t514*t544*t1251;
  t7273 = t703*t7259;
  t7288 = t2776 + t7273;
  t7300 = t703*t514*t1251;
  t7306 = -1.*t544*t7259;
  t7335 = t7300 + t7306;
  t7349 = -1.*t544*t891;
  t7369 = t703*t7366;
  t7374 = t7349 + t7369;
  t7377 = -1.*t703*t891;
  t7379 = -1.*t544*t7366;
  t7380 = t7377 + t7379;
  t7405 = t1681*t1361;
  t7408 = t833*t2174;
  t7409 = t7405 + t7408;
  t7502 = -1.*t2702;
  t7503 = 1. + t7502;
  t2701 = -1.*t9*t2693;
  t2744 = t2702*t2730;
  t2775 = t2701 + t2744;
  t7429 = t2702*t2693;
  t7436 = t9*t2730;
  t7440 = t7429 + t7436;
  t7461 = -1.*t703;
  t7462 = 1. + t7461;
  t7463 = -0.183*t7462;
  t7464 = 0. + t7463;
  t7466 = -0.047*t1681;
  t7467 = 0. + t7466;
  t7469 = -1.*t833;
  t7470 = 1. + t7469;
  t7471 = 0.047*t7470;
  t7477 = 0. + t7471;
  t7413 = t1681*t2897;
  t7416 = t833*t7250;
  t7421 = t7413 + t7416;
  t7497 = -0.183*t544;
  t7499 = 0. + t7497;
  t7504 = -0.2*t7503;
  t7505 = -0.183*t9;
  t7506 = 0. + t7504 + t7505;
  t7508 = -0.183*t7503;
  t7509 = 0.2*t9;
  t7510 = 0. + t7508 + t7509;
  t7293 = -1.*t9*t7288;
  t7339 = t2702*t7335;
  t7348 = t7293 + t7339;
  t7443 = t2702*t7288;
  t7447 = t9*t7335;
  t7451 = t7443 + t7447;
  t7423 = t514*t887*t1681;
  t7427 = t833*t514*t1268;
  t7428 = t7423 + t7427;
  t7376 = -1.*t9*t7374;
  t7402 = t2702*t7380;
  t7404 = t7376 + t7402;
  t7458 = t2702*t7374;
  t7459 = t9*t7380;
  t7460 = t7458 + t7459;
  p_output1[0]=t2775;
  p_output1[1]=t7348;
  p_output1[2]=t7404;
  p_output1[3]=0.;
  p_output1[4]=t7409;
  p_output1[5]=t7421;
  p_output1[6]=t7428;
  p_output1[7]=0.;
  p_output1[8]=t7440;
  p_output1[9]=t7451;
  p_output1[10]=t7460;
  p_output1[11]=0.;
  p_output1[12]=0. - 0.183*t2775 + 0.13205*t7409 - 0.4*t7440 + t1361*t7467 + t2174*t7477 + t2325*t7499 + t2693*t7506 + t2730*t7510 + t514*t7464*t91 + var1[0];
  p_output1[13]=0. - 0.183*t7348 + 0.13205*t7421 - 0.4*t7451 + t1251*t514*t7464 + t2897*t7467 + t7250*t7477 + t7259*t7499 + t7288*t7506 + t7335*t7510 + var1[1];
  p_output1[14]=0. - 0.183*t7404 + 0.13205*t7428 - 0.4*t7460 + t1268*t514*t7477 + t7366*t7499 + t7374*t7506 + t7380*t7510 + t514*t7467*t887 - 1.*t7464*t891 + var1[2];
  p_output1[15]=1.;
}



void H_RLfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
