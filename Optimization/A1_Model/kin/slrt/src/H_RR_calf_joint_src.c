/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:46 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RR_calf_joint_src.h"

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
  double t3560;
  double t3626;
  double t3637;
  double t3634;
  double t3638;
  double t3592;
  double t3582;
  double t3588;
  double t3609;
  double t3636;
  double t3639;
  double t3640;
  double t3648;
  double t3653;
  double t3655;
  double t3660;
  double t3668;
  double t3672;
  double t3676;
  double t509;
  double t3698;
  double t3709;
  double t3716;
  double t3724;
  double t3734;
  double t3739;
  double t3741;
  double t3745;
  double t3746;
  double t3747;
  double t3768;
  double t3769;
  double t3777;
  double t3590;
  double t3677;
  double t3679;
  double t3703;
  double t3704;
  double t3705;
  double t3708;
  double t3748;
  double t3750;
  double t3755;
  double t3759;
  double t3761;
  double t3767;
  double t3781;
  double t3789;
  double t3791;
  double t3794;
  double t3795;
  double t3802;
  double t3803;
  double t3804;
  double t3877;
  double t3878;
  double t3694;
  double t3706;
  double t3707;
  double t3833;
  double t3835;
  double t3840;
  double t3852;
  double t3853;
  double t3854;
  double t3855;
  double t3858;
  double t3862;
  double t3867;
  double t3869;
  double t3870;
  double t3871;
  double t3805;
  double t3806;
  double t3808;
  double t3874;
  double t3875;
  double t3879;
  double t3880;
  double t3883;
  double t3885;
  double t3886;
  double t3889;
  double t3754;
  double t3764;
  double t3766;
  double t3844;
  double t3845;
  double t3846;
  double t3809;
  double t3821;
  double t3829;
  double t3790;
  double t3799;
  double t3801;
  double t3848;
  double t3850;
  double t3851;
  t3560 = Cos(var1[3]);
  t3626 = Cos(var1[5]);
  t3637 = Sin(var1[3]);
  t3634 = Sin(var1[4]);
  t3638 = Sin(var1[5]);
  t3592 = Cos(var1[13]);
  t3582 = Cos(var1[4]);
  t3588 = Sin(var1[13]);
  t3609 = Cos(var1[12]);
  t3636 = t3560*t3626*t3634;
  t3639 = t3637*t3638;
  t3640 = t3636 + t3639;
  t3648 = t3609*t3640;
  t3653 = Sin(var1[12]);
  t3655 = -1.*t3626*t3637;
  t3660 = t3560*t3634*t3638;
  t3668 = t3655 + t3660;
  t3672 = -1.*t3653*t3668;
  t3676 = t3648 + t3672;
  t509 = Sin(var1[14]);
  t3698 = Cos(var1[14]);
  t3709 = t3626*t3637*t3634;
  t3716 = -1.*t3560*t3638;
  t3724 = t3709 + t3716;
  t3734 = t3609*t3724;
  t3739 = t3560*t3626;
  t3741 = t3637*t3634*t3638;
  t3745 = t3739 + t3741;
  t3746 = -1.*t3653*t3745;
  t3747 = t3734 + t3746;
  t3768 = t3609*t3582*t3626;
  t3769 = -1.*t3582*t3653*t3638;
  t3777 = t3768 + t3769;
  t3590 = t3560*t3582*t3588;
  t3677 = t3592*t3676;
  t3679 = t3590 + t3677;
  t3703 = t3592*t3560*t3582;
  t3704 = -1.*t3588*t3676;
  t3705 = t3703 + t3704;
  t3708 = t3582*t3588*t3637;
  t3748 = t3592*t3747;
  t3750 = t3708 + t3748;
  t3755 = t3592*t3582*t3637;
  t3759 = -1.*t3588*t3747;
  t3761 = t3755 + t3759;
  t3767 = -1.*t3588*t3634;
  t3781 = t3592*t3777;
  t3789 = t3767 + t3781;
  t3791 = -1.*t3592*t3634;
  t3794 = -1.*t3588*t3777;
  t3795 = t3791 + t3794;
  t3802 = t3653*t3640;
  t3803 = t3609*t3668;
  t3804 = t3802 + t3803;
  t3877 = -1.*t3698;
  t3878 = 1. + t3877;
  t3694 = -1.*t509*t3679;
  t3706 = t3698*t3705;
  t3707 = t3694 + t3706;
  t3833 = t3698*t3679;
  t3835 = t509*t3705;
  t3840 = t3833 + t3835;
  t3852 = -1.*t3592;
  t3853 = 1. + t3852;
  t3854 = -0.183*t3853;
  t3855 = 0. + t3854;
  t3858 = 0.047*t3653;
  t3862 = 0. + t3858;
  t3867 = -1.*t3609;
  t3869 = 1. + t3867;
  t3870 = -0.047*t3869;
  t3871 = 0. + t3870;
  t3805 = t3653*t3724;
  t3806 = t3609*t3745;
  t3808 = t3805 + t3806;
  t3874 = -0.183*t3588;
  t3875 = 0. + t3874;
  t3879 = -0.2*t3878;
  t3880 = -0.183*t509;
  t3883 = 0. + t3879 + t3880;
  t3885 = -0.183*t3878;
  t3886 = 0.2*t509;
  t3889 = 0. + t3885 + t3886;
  t3754 = -1.*t509*t3750;
  t3764 = t3698*t3761;
  t3766 = t3754 + t3764;
  t3844 = t3698*t3750;
  t3845 = t509*t3761;
  t3846 = t3844 + t3845;
  t3809 = t3582*t3626*t3653;
  t3821 = t3609*t3582*t3638;
  t3829 = t3809 + t3821;
  t3790 = -1.*t509*t3789;
  t3799 = t3698*t3795;
  t3801 = t3790 + t3799;
  t3848 = t3698*t3789;
  t3850 = t509*t3795;
  t3851 = t3848 + t3850;
  p_output1[0]=t3707;
  p_output1[1]=t3766;
  p_output1[2]=t3801;
  p_output1[3]=0.;
  p_output1[4]=t3804;
  p_output1[5]=t3808;
  p_output1[6]=t3829;
  p_output1[7]=0.;
  p_output1[8]=t3840;
  p_output1[9]=t3846;
  p_output1[10]=t3851;
  p_output1[11]=0.;
  p_output1[12]=0. - 0.183*t3707 - 0.13205*t3804 - 0.2*t3840 + t3560*t3582*t3855 + t3640*t3862 + t3668*t3871 + t3676*t3875 + t3679*t3883 + t3705*t3889 + var1[0];
  p_output1[13]=0. - 0.183*t3766 - 0.13205*t3808 - 0.2*t3846 + t3582*t3637*t3855 + t3724*t3862 + t3745*t3871 + t3747*t3875 + t3750*t3883 + t3761*t3889 + var1[1];
  p_output1[14]=0. - 0.183*t3801 - 0.13205*t3829 - 0.2*t3851 - 1.*t3634*t3855 + t3582*t3626*t3862 + t3582*t3638*t3871 + t3777*t3875 + t3789*t3883 + t3795*t3889 + var1[2];
  p_output1[15]=1.;
}



void H_RR_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
