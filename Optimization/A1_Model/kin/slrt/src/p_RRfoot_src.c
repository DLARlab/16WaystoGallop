/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:07 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RRfoot_src.h"

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
  double t6640;
  double t6661;
  double t6667;
  double t6662;
  double t6668;
  double t6652;
  double t6665;
  double t6669;
  double t6671;
  double t6677;
  double t6690;
  double t6691;
  double t6692;
  double t6644;
  double t6699;
  double t120;
  double t6705;
  double t6706;
  double t6707;
  double t6709;
  double t6711;
  double t6713;
  double t6715;
  double t6719;
  double t6720;
  double t6721;
  double t6727;
  double t6728;
  double t6731;
  double t1400;
  double t1656;
  double t6599;
  double t6630;
  double t6653;
  double t6654;
  double t6679;
  double t6680;
  double t6684;
  double t6688;
  double t6746;
  double t6747;
  double t6748;
  double t6750;
  double t6751;
  double t6752;
  double t6703;
  double t6704;
  double t6714;
  double t6716;
  double t6718;
  double t6761;
  double t6763;
  double t6767;
  double t6724;
  double t6725;
  double t6726;
  double t6769;
  double t6770;
  double t6771;
  double t6773;
  double t6774;
  double t6775;
  double t6793;
  double t6794;
  double t6795;
  double t6797;
  double t6798;
  double t6799;
  double t6801;
  double t6802;
  double t6803;
  t6640 = Cos(var1[3]);
  t6661 = Cos(var1[5]);
  t6667 = Sin(var1[3]);
  t6662 = Sin(var1[4]);
  t6668 = Sin(var1[5]);
  t6652 = Sin(var1[12]);
  t6665 = t6640*t6661*t6662;
  t6669 = t6667*t6668;
  t6671 = t6665 + t6669;
  t6677 = Cos(var1[12]);
  t6690 = -1.*t6661*t6667;
  t6691 = t6640*t6662*t6668;
  t6692 = t6690 + t6691;
  t6644 = Cos(var1[4]);
  t6699 = Sin(var1[13]);
  t120 = Cos(var1[13]);
  t6705 = t6677*t6671;
  t6706 = -1.*t6652*t6692;
  t6707 = t6705 + t6706;
  t6709 = Cos(var1[14]);
  t6711 = -1.*t6709;
  t6713 = 1. + t6711;
  t6715 = Sin(var1[14]);
  t6719 = t6640*t6644*t6699;
  t6720 = t120*t6707;
  t6721 = t6719 + t6720;
  t6727 = t120*t6640*t6644;
  t6728 = -1.*t6699*t6707;
  t6731 = t6727 + t6728;
  t1400 = -1.*t120;
  t1656 = 1. + t1400;
  t6599 = -0.183*t1656;
  t6630 = 0. + t6599;
  t6653 = 0.047*t6652;
  t6654 = 0. + t6653;
  t6679 = -1.*t6677;
  t6680 = 1. + t6679;
  t6684 = -0.047*t6680;
  t6688 = 0. + t6684;
  t6746 = t6661*t6667*t6662;
  t6747 = -1.*t6640*t6668;
  t6748 = t6746 + t6747;
  t6750 = t6640*t6661;
  t6751 = t6667*t6662*t6668;
  t6752 = t6750 + t6751;
  t6703 = -0.183*t6699;
  t6704 = 0. + t6703;
  t6714 = -0.2*t6713;
  t6716 = -0.183*t6715;
  t6718 = 0. + t6714 + t6716;
  t6761 = t6677*t6748;
  t6763 = -1.*t6652*t6752;
  t6767 = t6761 + t6763;
  t6724 = -0.183*t6713;
  t6725 = 0.2*t6715;
  t6726 = 0. + t6724 + t6725;
  t6769 = t6644*t6699*t6667;
  t6770 = t120*t6767;
  t6771 = t6769 + t6770;
  t6773 = t120*t6644*t6667;
  t6774 = -1.*t6699*t6767;
  t6775 = t6773 + t6774;
  t6793 = t6677*t6644*t6661;
  t6794 = -1.*t6644*t6652*t6668;
  t6795 = t6793 + t6794;
  t6797 = -1.*t6699*t6662;
  t6798 = t120*t6795;
  t6799 = t6797 + t6798;
  t6801 = -1.*t120*t6662;
  t6802 = -1.*t6699*t6795;
  t6803 = t6801 + t6802;
  p_output1[0]=0. + t6630*t6640*t6644 + t6654*t6671 + t6688*t6692 - 0.13205*(t6652*t6671 + t6677*t6692) + t6704*t6707 + t6718*t6721 + t6726*t6731 - 0.183*(-1.*t6715*t6721 + t6709*t6731) - 0.4*(t6709*t6721 + t6715*t6731) + var1[0];
  p_output1[1]=0. + t6630*t6644*t6667 + t6654*t6748 + t6688*t6752 - 0.13205*(t6652*t6748 + t6677*t6752) + t6704*t6767 + t6718*t6771 + t6726*t6775 - 0.183*(-1.*t6715*t6771 + t6709*t6775) - 0.4*(t6709*t6771 + t6715*t6775) + var1[1];
  p_output1[2]=0. + t6644*t6654*t6661 - 1.*t6630*t6662 - 0.13205*(t6644*t6652*t6661 + t6644*t6668*t6677) + t6644*t6668*t6688 + t6704*t6795 + t6718*t6799 + t6726*t6803 - 0.183*(-1.*t6715*t6799 + t6709*t6803) - 0.4*(t6709*t6799 + t6715*t6803) + var1[2];
}



void p_RRfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
