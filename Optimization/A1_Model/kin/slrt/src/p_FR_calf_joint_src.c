/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:21 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FR_calf_joint_src.h"

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
  double t696;
  double t723;
  double t726;
  double t724;
  double t730;
  double t717;
  double t725;
  double t731;
  double t732;
  double t734;
  double t739;
  double t741;
  double t742;
  double t705;
  double t752;
  double t753;
  double t754;
  double t701;
  double t755;
  double t762;
  double t764;
  double t765;
  double t768;
  double t772;
  double t773;
  double t776;
  double t759;
  double t760;
  double t761;
  double t706;
  double t712;
  double t714;
  double t715;
  double t719;
  double t720;
  double t721;
  double t722;
  double t743;
  double t744;
  double t794;
  double t795;
  double t796;
  double t798;
  double t799;
  double t800;
  double t756;
  double t757;
  double t813;
  double t814;
  double t815;
  double t767;
  double t769;
  double t770;
  double t777;
  double t778;
  double t780;
  double t821;
  double t822;
  double t823;
  double t817;
  double t818;
  double t819;
  double t841;
  double t842;
  double t843;
  double t849;
  double t850;
  double t851;
  double t845;
  double t846;
  double t847;
  t696 = Cos(var1[3]);
  t723 = Cos(var1[5]);
  t726 = Sin(var1[4]);
  t724 = Sin(var1[3]);
  t730 = Sin(var1[5]);
  t717 = Cos(var1[6]);
  t725 = -1.*t723*t724;
  t731 = t696*t726*t730;
  t732 = t725 + t731;
  t734 = t696*t723*t726;
  t739 = t724*t730;
  t741 = t734 + t739;
  t742 = Sin(var1[6]);
  t705 = Cos(var1[7]);
  t752 = t717*t741;
  t753 = -1.*t732*t742;
  t754 = t752 + t753;
  t701 = Cos(var1[4]);
  t755 = Sin(var1[7]);
  t762 = Cos(var1[8]);
  t764 = -1.*t762;
  t765 = 1. + t764;
  t768 = Sin(var1[8]);
  t772 = t696*t701*t705;
  t773 = -1.*t754*t755;
  t776 = t772 + t773;
  t759 = t705*t754;
  t760 = t696*t701*t755;
  t761 = t759 + t760;
  t706 = -1.*t705;
  t712 = 1. + t706;
  t714 = 0.183*t712;
  t715 = 0. + t714;
  t719 = -1.*t717;
  t720 = 1. + t719;
  t721 = -0.047*t720;
  t722 = 0. + t721;
  t743 = 0.047*t742;
  t744 = 0. + t743;
  t794 = t696*t723;
  t795 = t724*t726*t730;
  t796 = t794 + t795;
  t798 = t723*t724*t726;
  t799 = -1.*t696*t730;
  t800 = t798 + t799;
  t756 = 0.183*t755;
  t757 = 0. + t756;
  t813 = t717*t800;
  t814 = -1.*t796*t742;
  t815 = t813 + t814;
  t767 = -0.2*t765;
  t769 = 0.183*t768;
  t770 = 0. + t767 + t769;
  t777 = 0.183*t765;
  t778 = 0.2*t768;
  t780 = 0. + t777 + t778;
  t821 = t701*t705*t724;
  t822 = -1.*t815*t755;
  t823 = t821 + t822;
  t817 = t705*t815;
  t818 = t701*t724*t755;
  t819 = t817 + t818;
  t841 = t701*t723*t717;
  t842 = -1.*t701*t730*t742;
  t843 = t841 + t842;
  t849 = -1.*t705*t726;
  t850 = -1.*t843*t755;
  t851 = t849 + t850;
  t845 = t705*t843;
  t846 = -1.*t726*t755;
  t847 = t845 + t846;
  p_output1[0]=0. + t696*t701*t715 + t722*t732 - 0.13205*(t717*t732 + t741*t742) + t741*t744 + t754*t757 + t761*t770 + 0.183*(-1.*t761*t768 + t762*t776) - 0.2*(t761*t762 + t768*t776) + t776*t780 + var1[0];
  p_output1[1]=0. + t701*t715*t724 + t722*t796 + t744*t800 - 0.13205*(t717*t796 + t742*t800) + t757*t815 + t770*t819 + t780*t823 + 0.183*(-1.*t768*t819 + t762*t823) - 0.2*(t762*t819 + t768*t823) + var1[1];
  p_output1[2]=0. - 1.*t715*t726 + t701*t722*t730 - 0.13205*(t701*t717*t730 + t701*t723*t742) + t701*t723*t744 + t757*t843 + t770*t847 + t780*t851 + 0.183*(-1.*t768*t847 + t762*t851) - 0.2*(t762*t847 + t768*t851) + var1[2];
}



void p_FR_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
