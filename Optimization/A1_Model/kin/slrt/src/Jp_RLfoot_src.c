/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:12 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RLfoot_src.h"

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
  double t929;
  double t1334;
  double t979;
  double t986;
  double t1412;
  double t945;
  double t1021;
  double t1681;
  double t1741;
  double t2357;
  double t3607;
  double t3780;
  double t3787;
  double t700;
  double t7287;
  double t86;
  double t7300;
  double t7302;
  double t7305;
  double t7308;
  double t7312;
  double t7327;
  double t7333;
  double t7338;
  double t7339;
  double t7346;
  double t7353;
  double t7361;
  double t7362;
  double t122;
  double t195;
  double t436;
  double t516;
  double t971;
  double t973;
  double t2361;
  double t2385;
  double t2494;
  double t2560;
  double t7380;
  double t7381;
  double t7384;
  double t7386;
  double t7391;
  double t7393;
  double t7288;
  double t7293;
  double t7330;
  double t7335;
  double t7337;
  double t7405;
  double t7406;
  double t7407;
  double t7348;
  double t7349;
  double t7350;
  double t7409;
  double t7410;
  double t7411;
  double t7413;
  double t7414;
  double t7415;
  double t7440;
  double t7441;
  double t7442;
  double t7444;
  double t7447;
  double t7449;
  double t7455;
  double t7456;
  double t7457;
  double t7479;
  double t7483;
  double t7484;
  double t7486;
  double t7487;
  double t7488;
  double t7490;
  double t7491;
  double t7492;
  double t7513;
  double t7514;
  double t7515;
  double t7519;
  double t7520;
  double t7523;
  double t7525;
  double t7526;
  double t7527;
  double t7539;
  double t7540;
  double t7547;
  double t7555;
  double t7557;
  double t7559;
  double t7577;
  double t7578;
  double t7579;
  double t7251;
  double t7585;
  double t7586;
  double t7609;
  double t7610;
  double t7611;
  double t7636;
  double t7637;
  double t7658;
  double t7659;
  double t7660;
  double t7662;
  double t7663;
  double t7592;
  double t7612;
  double t7614;
  double t7615;
  double t7616;
  double t7617;
  double t7618;
  double t7620;
  double t7621;
  double t7622;
  double t7625;
  double t7626;
  double t7628;
  double t7629;
  double t7630;
  double t7632;
  double t7687;
  double t7688;
  double t7689;
  double t7421;
  double t7667;
  double t7669;
  double t7702;
  double t7703;
  double t7705;
  double t7706;
  double t7707;
  double t7720;
  double t7721;
  double t7722;
  double t7724;
  double t7725;
  double t7726;
  double t7419;
  double t7423;
  double t7696;
  double t7737;
  double t7738;
  double t7739;
  double t7741;
  double t7742;
  double t7749;
  double t7750;
  double t7751;
  double t7710;
  double t7714;
  double t7761;
  double t7762;
  double t7763;
  double t7729;
  double t7733;
  t929 = Sin(var1[3]);
  t1334 = Cos(var1[3]);
  t979 = Cos(var1[5]);
  t986 = Sin(var1[4]);
  t1412 = Sin(var1[5]);
  t945 = Sin(var1[15]);
  t1021 = -1.*t979*t929*t986;
  t1681 = t1334*t1412;
  t1741 = t1021 + t1681;
  t2357 = Cos(var1[15]);
  t3607 = -1.*t1334*t979;
  t3780 = -1.*t929*t986*t1412;
  t3787 = t3607 + t3780;
  t700 = Cos(var1[4]);
  t7287 = Sin(var1[16]);
  t86 = Cos(var1[16]);
  t7300 = t2357*t1741;
  t7302 = -1.*t945*t3787;
  t7305 = t7300 + t7302;
  t7308 = Cos(var1[17]);
  t7312 = -1.*t7308;
  t7327 = 1. + t7312;
  t7333 = Sin(var1[17]);
  t7338 = -1.*t700*t7287*t929;
  t7339 = t86*t7305;
  t7346 = t7338 + t7339;
  t7353 = -1.*t86*t700*t929;
  t7361 = -1.*t7287*t7305;
  t7362 = t7353 + t7361;
  t122 = -1.*t86;
  t195 = 1. + t122;
  t436 = -0.183*t195;
  t516 = 0. + t436;
  t971 = -0.047*t945;
  t973 = 0. + t971;
  t2361 = -1.*t2357;
  t2385 = 1. + t2361;
  t2494 = 0.047*t2385;
  t2560 = 0. + t2494;
  t7380 = t1334*t979*t986;
  t7381 = t929*t1412;
  t7384 = t7380 + t7381;
  t7386 = -1.*t979*t929;
  t7391 = t1334*t986*t1412;
  t7393 = t7386 + t7391;
  t7288 = -0.183*t7287;
  t7293 = 0. + t7288;
  t7330 = -0.2*t7327;
  t7335 = -0.183*t7333;
  t7337 = 0. + t7330 + t7335;
  t7405 = t2357*t7384;
  t7406 = -1.*t945*t7393;
  t7407 = t7405 + t7406;
  t7348 = -0.183*t7327;
  t7349 = 0.2*t7333;
  t7350 = 0. + t7348 + t7349;
  t7409 = t1334*t700*t7287;
  t7410 = t86*t7407;
  t7411 = t7409 + t7410;
  t7413 = t86*t1334*t700;
  t7414 = -1.*t7287*t7407;
  t7415 = t7413 + t7414;
  t7440 = t2357*t1334*t700*t979;
  t7441 = -1.*t1334*t700*t945*t1412;
  t7442 = t7440 + t7441;
  t7444 = -1.*t1334*t7287*t986;
  t7447 = t86*t7442;
  t7449 = t7444 + t7447;
  t7455 = -1.*t86*t1334*t986;
  t7456 = -1.*t7287*t7442;
  t7457 = t7455 + t7456;
  t7479 = t2357*t700*t979*t929;
  t7483 = -1.*t700*t945*t929*t1412;
  t7484 = t7479 + t7483;
  t7486 = -1.*t7287*t929*t986;
  t7487 = t86*t7484;
  t7488 = t7486 + t7487;
  t7490 = -1.*t86*t929*t986;
  t7491 = -1.*t7287*t7484;
  t7492 = t7490 + t7491;
  t7513 = -1.*t2357*t979*t986;
  t7514 = t945*t986*t1412;
  t7515 = t7513 + t7514;
  t7519 = -1.*t700*t7287;
  t7520 = t86*t7515;
  t7523 = t7519 + t7520;
  t7525 = -1.*t86*t700;
  t7526 = -1.*t7287*t7515;
  t7527 = t7525 + t7526;
  t7539 = t979*t929;
  t7540 = -1.*t1334*t986*t1412;
  t7547 = t7539 + t7540;
  t7555 = -1.*t945*t7384;
  t7557 = t2357*t7547;
  t7559 = t7555 + t7557;
  t7577 = t979*t929*t986;
  t7578 = -1.*t1334*t1412;
  t7579 = t7577 + t7578;
  t7251 = t2357*t3787;
  t7585 = -1.*t945*t7579;
  t7586 = t7585 + t7251;
  t7609 = -1.*t700*t979*t945;
  t7610 = -1.*t2357*t700*t1412;
  t7611 = t7609 + t7610;
  t7636 = -1.*t2357*t7393;
  t7637 = t7555 + t7636;
  t7658 = t1334*t979;
  t7659 = t929*t986*t1412;
  t7660 = t7658 + t7659;
  t7662 = -1.*t2357*t7660;
  t7663 = t7585 + t7662;
  t7592 = t2357*t7579;
  t7612 = t7293*t7611;
  t7614 = t86*t7337*t7611;
  t7615 = -1.*t7287*t7350*t7611;
  t7616 = t2357*t700*t979;
  t7617 = -1.*t700*t945*t1412;
  t7618 = t7616 + t7617;
  t7620 = 0.13205*t7618;
  t7621 = -1.*t7308*t7287*t7611;
  t7622 = -1.*t86*t7333*t7611;
  t7625 = t7621 + t7622;
  t7626 = -0.183*t7625;
  t7628 = t86*t7308*t7611;
  t7629 = -1.*t7287*t7333*t7611;
  t7630 = t7628 + t7629;
  t7632 = -0.4*t7630;
  t7687 = -1.*t1334*t700*t7287;
  t7688 = -1.*t86*t7407;
  t7689 = t7687 + t7688;
  t7421 = t7308*t7415;
  t7667 = -1.*t945*t7660;
  t7669 = t7592 + t7667;
  t7702 = -1.*t86*t7669;
  t7703 = t7338 + t7702;
  t7705 = t86*t700*t929;
  t7706 = -1.*t7287*t7669;
  t7707 = t7705 + t7706;
  t7720 = t7287*t986;
  t7721 = -1.*t86*t7618;
  t7722 = t7720 + t7721;
  t7724 = -1.*t86*t986;
  t7725 = -1.*t7287*t7618;
  t7726 = t7724 + t7725;
  t7419 = -1.*t7333*t7411;
  t7423 = t7419 + t7421;
  t7696 = -1.*t7333*t7415;
  t7737 = -0.183*t7308;
  t7738 = -0.2*t7333;
  t7739 = t7737 + t7738;
  t7741 = 0.2*t7308;
  t7742 = t7741 + t7335;
  t7749 = t700*t7287*t929;
  t7750 = t86*t7669;
  t7751 = t7749 + t7750;
  t7710 = t7308*t7707;
  t7714 = -1.*t7333*t7707;
  t7761 = -1.*t7287*t986;
  t7762 = t86*t7618;
  t7763 = t7761 + t7762;
  t7729 = t7308*t7726;
  t7733 = -1.*t7333*t7726;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=t2560*t3787 + t7293*t7305 + t7337*t7346 + t7350*t7362 - 0.183*(-1.*t7333*t7346 + t7308*t7362) - 0.4*(t7308*t7346 + t7333*t7362) - 1.*t516*t700*t929 + 0.13205*(t7251 + t1741*t945) + t1741*t973;
  p_output1[10]=t1334*t516*t700 + t2560*t7393 + t7293*t7407 + t7337*t7411 + t7350*t7415 - 0.4*(t7308*t7411 + t7333*t7415) - 0.183*t7423 + 0.13205*(t2357*t7393 + t7384*t945) + t7384*t973;
  p_output1[11]=0;
  p_output1[12]=t1334*t1412*t2560*t700 + t7293*t7442 + t7337*t7449 + t7350*t7457 - 0.183*(-1.*t7333*t7449 + t7308*t7457) - 0.4*(t7308*t7449 + t7333*t7457) + t1334*t700*t973*t979 + 0.13205*(t1334*t1412*t2357*t700 + t1334*t700*t945*t979) - 1.*t1334*t516*t986;
  p_output1[13]=t7293*t7484 + t7337*t7488 + t7350*t7492 - 0.183*(-1.*t7333*t7488 + t7308*t7492) - 0.4*(t7308*t7488 + t7333*t7492) + t1412*t2560*t700*t929 + t700*t929*t973*t979 + 0.13205*(t1412*t2357*t700*t929 + t700*t929*t945*t979) - 1.*t516*t929*t986;
  p_output1[14]=-1.*t516*t700 + t7293*t7515 + t7337*t7523 + t7350*t7527 - 0.183*(-1.*t7333*t7523 + t7308*t7527) - 0.4*(t7308*t7523 + t7333*t7527) - 1.*t1412*t2560*t986 - 1.*t973*t979*t986 + 0.13205*(-1.*t1412*t2357*t986 - 1.*t945*t979*t986);
  p_output1[15]=t2560*t7384 + t7293*t7559 - 1.*t7287*t7350*t7559 + t7337*t7559*t86 - 0.4*(-1.*t7287*t7333*t7559 + t7308*t7559*t86) - 0.183*(-1.*t7287*t7308*t7559 - 1.*t7333*t7559*t86) + 0.13205*(t7405 + t7547*t945) + t7547*t973;
  p_output1[16]=t2560*t7579 + t7293*t7586 - 1.*t7287*t7350*t7586 + t7337*t7586*t86 - 0.4*(-1.*t7287*t7333*t7586 + t7308*t7586*t86) - 0.183*(-1.*t7287*t7308*t7586 - 1.*t7333*t7586*t86) + 0.13205*(t7592 + t3787*t945) + t3787*t973;
  p_output1[17]=t7612 + t7614 + t7615 + t7620 + t7626 + t7632 - 1.*t1412*t700*t973 + t2560*t700*t979;
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
  p_output1[45]=-0.047*t2357*t7384 + 0.13205*t7407 + t7293*t7637 - 1.*t7287*t7350*t7637 + t7337*t7637*t86 - 0.4*(-1.*t7287*t7333*t7637 + t7308*t7637*t86) - 0.183*(-1.*t7287*t7308*t7637 - 1.*t7333*t7637*t86) + 0.047*t7393*t945;
  p_output1[46]=-0.047*t2357*t7579 + t7293*t7663 - 1.*t7287*t7350*t7663 + 0.13205*t7669 + t7337*t7663*t86 - 0.4*(-1.*t7287*t7333*t7663 + t7308*t7663*t86) - 0.183*(-1.*t7287*t7308*t7663 - 1.*t7333*t7663*t86) + 0.047*t7660*t945;
  p_output1[47]=t7612 + t7614 + t7615 + t7620 + t7626 + t7632 + 0.047*t1412*t700*t945 - 0.047*t2357*t700*t979;
  p_output1[48]=-0.183*t1334*t700*t7287 + t7337*t7415 + t7350*t7689 - 0.4*(t7421 + t7333*t7689) - 0.183*(t7308*t7689 + t7696) - 0.183*t7407*t86;
  p_output1[49]=t7350*t7703 + t7337*t7707 - 0.4*(t7333*t7703 + t7710) - 0.183*(t7308*t7703 + t7714) - 0.183*t7669*t86 - 0.183*t700*t7287*t929;
  p_output1[50]=t7350*t7722 + t7337*t7726 - 0.4*(t7333*t7722 + t7729) - 0.183*(t7308*t7722 + t7733) - 0.183*t7618*t86 + 0.183*t7287*t986;
  p_output1[51]=-0.4*t7423 - 0.183*(-1.*t7308*t7411 + t7696) + t7411*t7739 + t7415*t7742;
  p_output1[52]=t7707*t7742 + t7739*t7751 - 0.183*(t7714 - 1.*t7308*t7751) - 0.4*(t7710 - 1.*t7333*t7751);
  p_output1[53]=t7726*t7742 + t7739*t7763 - 0.183*(t7733 - 1.*t7308*t7763) - 0.4*(t7729 - 1.*t7333*t7763);
}



void Jp_RLfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
