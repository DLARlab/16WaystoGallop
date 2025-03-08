/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:45 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RR_calf_joint_src.h"

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
  double t3463;
  double t3492;
  double t3488;
  double t3490;
  double t3495;
  double t3480;
  double t3491;
  double t3500;
  double t3502;
  double t3508;
  double t3528;
  double t3529;
  double t3530;
  double t3452;
  double t3536;
  double t3407;
  double t3539;
  double t3541;
  double t3543;
  double t3548;
  double t3550;
  double t3552;
  double t3557;
  double t3562;
  double t3564;
  double t3565;
  double t3573;
  double t3574;
  double t3575;
  double t3419;
  double t3432;
  double t3450;
  double t3451;
  double t3481;
  double t3485;
  double t3512;
  double t3516;
  double t3517;
  double t3525;
  double t3590;
  double t3592;
  double t3596;
  double t3598;
  double t3599;
  double t3600;
  double t3537;
  double t3538;
  double t3556;
  double t3560;
  double t3561;
  double t3606;
  double t3607;
  double t3608;
  double t3570;
  double t3571;
  double t3572;
  double t3610;
  double t3611;
  double t3612;
  double t3616;
  double t3618;
  double t3620;
  double t3642;
  double t3644;
  double t3646;
  double t3652;
  double t3653;
  double t3654;
  double t3656;
  double t3657;
  double t3658;
  double t3679;
  double t3680;
  double t3681;
  double t3683;
  double t3684;
  double t3685;
  double t3689;
  double t3692;
  double t3693;
  double t3722;
  double t3724;
  double t3726;
  double t3729;
  double t3730;
  double t3733;
  double t3735;
  double t3736;
  double t3737;
  double t3750;
  double t3753;
  double t3754;
  double t3758;
  double t3759;
  double t3760;
  double t3778;
  double t3779;
  double t3780;
  double t3533;
  double t3784;
  double t3785;
  double t3813;
  double t3814;
  double t3816;
  double t3836;
  double t3838;
  double t3855;
  double t3856;
  double t3857;
  double t3859;
  double t3860;
  double t3790;
  double t3817;
  double t3818;
  double t3820;
  double t3821;
  double t3822;
  double t3823;
  double t3824;
  double t3825;
  double t3826;
  double t3827;
  double t3828;
  double t3829;
  double t3830;
  double t3831;
  double t3832;
  double t3881;
  double t3882;
  double t3883;
  double t3626;
  double t3864;
  double t3865;
  double t3896;
  double t3897;
  double t3899;
  double t3900;
  double t3901;
  double t3914;
  double t3915;
  double t3916;
  double t3918;
  double t3919;
  double t3920;
  double t3625;
  double t3627;
  double t3890;
  double t3931;
  double t3932;
  double t3933;
  double t3935;
  double t3936;
  double t3943;
  double t3944;
  double t3945;
  double t3904;
  double t3908;
  double t3955;
  double t3956;
  double t3957;
  double t3923;
  double t3927;
  t3463 = Sin(var1[3]);
  t3492 = Cos(var1[3]);
  t3488 = Cos(var1[5]);
  t3490 = Sin(var1[4]);
  t3495 = Sin(var1[5]);
  t3480 = Sin(var1[12]);
  t3491 = -1.*t3488*t3463*t3490;
  t3500 = t3492*t3495;
  t3502 = t3491 + t3500;
  t3508 = Cos(var1[12]);
  t3528 = -1.*t3492*t3488;
  t3529 = -1.*t3463*t3490*t3495;
  t3530 = t3528 + t3529;
  t3452 = Cos(var1[4]);
  t3536 = Sin(var1[13]);
  t3407 = Cos(var1[13]);
  t3539 = t3508*t3502;
  t3541 = -1.*t3480*t3530;
  t3543 = t3539 + t3541;
  t3548 = Cos(var1[14]);
  t3550 = -1.*t3548;
  t3552 = 1. + t3550;
  t3557 = Sin(var1[14]);
  t3562 = -1.*t3452*t3536*t3463;
  t3564 = t3407*t3543;
  t3565 = t3562 + t3564;
  t3573 = -1.*t3407*t3452*t3463;
  t3574 = -1.*t3536*t3543;
  t3575 = t3573 + t3574;
  t3419 = -1.*t3407;
  t3432 = 1. + t3419;
  t3450 = -0.183*t3432;
  t3451 = 0. + t3450;
  t3481 = 0.047*t3480;
  t3485 = 0. + t3481;
  t3512 = -1.*t3508;
  t3516 = 1. + t3512;
  t3517 = -0.047*t3516;
  t3525 = 0. + t3517;
  t3590 = t3492*t3488*t3490;
  t3592 = t3463*t3495;
  t3596 = t3590 + t3592;
  t3598 = -1.*t3488*t3463;
  t3599 = t3492*t3490*t3495;
  t3600 = t3598 + t3599;
  t3537 = -0.183*t3536;
  t3538 = 0. + t3537;
  t3556 = -0.2*t3552;
  t3560 = -0.183*t3557;
  t3561 = 0. + t3556 + t3560;
  t3606 = t3508*t3596;
  t3607 = -1.*t3480*t3600;
  t3608 = t3606 + t3607;
  t3570 = -0.183*t3552;
  t3571 = 0.2*t3557;
  t3572 = 0. + t3570 + t3571;
  t3610 = t3492*t3452*t3536;
  t3611 = t3407*t3608;
  t3612 = t3610 + t3611;
  t3616 = t3407*t3492*t3452;
  t3618 = -1.*t3536*t3608;
  t3620 = t3616 + t3618;
  t3642 = t3508*t3492*t3452*t3488;
  t3644 = -1.*t3492*t3452*t3480*t3495;
  t3646 = t3642 + t3644;
  t3652 = -1.*t3492*t3536*t3490;
  t3653 = t3407*t3646;
  t3654 = t3652 + t3653;
  t3656 = -1.*t3407*t3492*t3490;
  t3657 = -1.*t3536*t3646;
  t3658 = t3656 + t3657;
  t3679 = t3508*t3452*t3488*t3463;
  t3680 = -1.*t3452*t3480*t3463*t3495;
  t3681 = t3679 + t3680;
  t3683 = -1.*t3536*t3463*t3490;
  t3684 = t3407*t3681;
  t3685 = t3683 + t3684;
  t3689 = -1.*t3407*t3463*t3490;
  t3692 = -1.*t3536*t3681;
  t3693 = t3689 + t3692;
  t3722 = -1.*t3508*t3488*t3490;
  t3724 = t3480*t3490*t3495;
  t3726 = t3722 + t3724;
  t3729 = -1.*t3452*t3536;
  t3730 = t3407*t3726;
  t3733 = t3729 + t3730;
  t3735 = -1.*t3407*t3452;
  t3736 = -1.*t3536*t3726;
  t3737 = t3735 + t3736;
  t3750 = t3488*t3463;
  t3753 = -1.*t3492*t3490*t3495;
  t3754 = t3750 + t3753;
  t3758 = -1.*t3480*t3596;
  t3759 = t3508*t3754;
  t3760 = t3758 + t3759;
  t3778 = t3488*t3463*t3490;
  t3779 = -1.*t3492*t3495;
  t3780 = t3778 + t3779;
  t3533 = t3508*t3530;
  t3784 = -1.*t3480*t3780;
  t3785 = t3784 + t3533;
  t3813 = -1.*t3452*t3488*t3480;
  t3814 = -1.*t3508*t3452*t3495;
  t3816 = t3813 + t3814;
  t3836 = -1.*t3508*t3600;
  t3838 = t3758 + t3836;
  t3855 = t3492*t3488;
  t3856 = t3463*t3490*t3495;
  t3857 = t3855 + t3856;
  t3859 = -1.*t3508*t3857;
  t3860 = t3784 + t3859;
  t3790 = t3508*t3780;
  t3817 = t3538*t3816;
  t3818 = t3407*t3561*t3816;
  t3820 = -1.*t3536*t3572*t3816;
  t3821 = t3508*t3452*t3488;
  t3822 = -1.*t3452*t3480*t3495;
  t3823 = t3821 + t3822;
  t3824 = -0.13205*t3823;
  t3825 = -1.*t3548*t3536*t3816;
  t3826 = -1.*t3407*t3557*t3816;
  t3827 = t3825 + t3826;
  t3828 = -0.183*t3827;
  t3829 = t3407*t3548*t3816;
  t3830 = -1.*t3536*t3557*t3816;
  t3831 = t3829 + t3830;
  t3832 = -0.2*t3831;
  t3881 = -1.*t3492*t3452*t3536;
  t3882 = -1.*t3407*t3608;
  t3883 = t3881 + t3882;
  t3626 = t3548*t3620;
  t3864 = -1.*t3480*t3857;
  t3865 = t3790 + t3864;
  t3896 = -1.*t3407*t3865;
  t3897 = t3562 + t3896;
  t3899 = t3407*t3452*t3463;
  t3900 = -1.*t3536*t3865;
  t3901 = t3899 + t3900;
  t3914 = t3536*t3490;
  t3915 = -1.*t3407*t3823;
  t3916 = t3914 + t3915;
  t3918 = -1.*t3407*t3490;
  t3919 = -1.*t3536*t3823;
  t3920 = t3918 + t3919;
  t3625 = -1.*t3557*t3612;
  t3627 = t3625 + t3626;
  t3890 = -1.*t3557*t3620;
  t3931 = -0.183*t3548;
  t3932 = -0.2*t3557;
  t3933 = t3931 + t3932;
  t3935 = 0.2*t3548;
  t3936 = t3935 + t3560;
  t3943 = t3452*t3536*t3463;
  t3944 = t3407*t3865;
  t3945 = t3943 + t3944;
  t3904 = t3548*t3901;
  t3908 = -1.*t3557*t3901;
  t3955 = -1.*t3536*t3490;
  t3956 = t3407*t3823;
  t3957 = t3955 + t3956;
  t3923 = t3548*t3920;
  t3927 = -1.*t3557*t3920;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=-1.*t3451*t3452*t3463 + t3485*t3502 + t3525*t3530 - 0.13205*(t3480*t3502 + t3533) + t3538*t3543 + t3561*t3565 + t3572*t3575 - 0.183*(-1.*t3557*t3565 + t3548*t3575) - 0.2*(t3548*t3565 + t3557*t3575);
  p_output1[10]=t3451*t3452*t3492 + t3485*t3596 + t3525*t3600 - 0.13205*(t3480*t3596 + t3508*t3600) + t3538*t3608 + t3561*t3612 + t3572*t3620 - 0.2*(t3548*t3612 + t3557*t3620) - 0.183*t3627;
  p_output1[11]=0;
  p_output1[12]=t3452*t3485*t3488*t3492 - 1.*t3451*t3490*t3492 - 0.13205*(t3452*t3480*t3488*t3492 + t3452*t3492*t3495*t3508) + t3452*t3492*t3495*t3525 + t3538*t3646 + t3561*t3654 + t3572*t3658 - 0.183*(-1.*t3557*t3654 + t3548*t3658) - 0.2*(t3548*t3654 + t3557*t3658);
  p_output1[13]=t3452*t3463*t3485*t3488 - 1.*t3451*t3463*t3490 - 0.13205*(t3452*t3463*t3480*t3488 + t3452*t3463*t3495*t3508) + t3452*t3463*t3495*t3525 + t3538*t3681 + t3561*t3685 + t3572*t3693 - 0.183*(-1.*t3557*t3685 + t3548*t3693) - 0.2*(t3548*t3685 + t3557*t3693);
  p_output1[14]=-1.*t3451*t3452 - 1.*t3485*t3488*t3490 - 0.13205*(-1.*t3480*t3488*t3490 - 1.*t3490*t3495*t3508) - 1.*t3490*t3495*t3525 + t3538*t3726 + t3561*t3733 + t3572*t3737 - 0.183*(-1.*t3557*t3733 + t3548*t3737) - 0.2*(t3548*t3733 + t3557*t3737);
  p_output1[15]=t3525*t3596 + t3485*t3754 - 0.13205*(t3606 + t3480*t3754) + t3538*t3760 + t3407*t3561*t3760 - 1.*t3536*t3572*t3760 - 0.183*(-1.*t3536*t3548*t3760 - 1.*t3407*t3557*t3760) - 0.2*(t3407*t3548*t3760 - 1.*t3536*t3557*t3760);
  p_output1[16]=t3485*t3530 + t3525*t3780 + t3538*t3785 + t3407*t3561*t3785 - 1.*t3536*t3572*t3785 - 0.183*(-1.*t3536*t3548*t3785 - 1.*t3407*t3557*t3785) - 0.2*(t3407*t3548*t3785 - 1.*t3536*t3557*t3785) - 0.13205*(t3480*t3530 + t3790);
  p_output1[17]=-1.*t3452*t3485*t3495 + t3452*t3488*t3525 + t3817 + t3818 + t3820 + t3824 + t3828 + t3832;
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
  p_output1[36]=0.047*t3508*t3596 - 0.047*t3480*t3600 - 0.13205*t3608 + t3538*t3838 + t3407*t3561*t3838 - 1.*t3536*t3572*t3838 - 0.183*(-1.*t3536*t3548*t3838 - 1.*t3407*t3557*t3838) - 0.2*(t3407*t3548*t3838 - 1.*t3536*t3557*t3838);
  p_output1[37]=0.047*t3508*t3780 - 0.047*t3480*t3857 + t3538*t3860 + t3407*t3561*t3860 - 1.*t3536*t3572*t3860 - 0.183*(-1.*t3536*t3548*t3860 - 1.*t3407*t3557*t3860) - 0.2*(t3407*t3548*t3860 - 1.*t3536*t3557*t3860) - 0.13205*t3865;
  p_output1[38]=-0.047*t3452*t3480*t3495 + 0.047*t3452*t3488*t3508 + t3817 + t3818 + t3820 + t3824 + t3828 + t3832;
  p_output1[39]=-0.183*t3452*t3492*t3536 - 0.183*t3407*t3608 + t3561*t3620 + t3572*t3883 - 0.2*(t3626 + t3557*t3883) - 0.183*(t3548*t3883 + t3890);
  p_output1[40]=-0.183*t3452*t3463*t3536 - 0.183*t3407*t3865 + t3572*t3897 + t3561*t3901 - 0.2*(t3557*t3897 + t3904) - 0.183*(t3548*t3897 + t3908);
  p_output1[41]=0.183*t3490*t3536 - 0.183*t3407*t3823 + t3572*t3916 + t3561*t3920 - 0.2*(t3557*t3916 + t3923) - 0.183*(t3548*t3916 + t3927);
  p_output1[42]=-0.2*t3627 - 0.183*(-1.*t3548*t3612 + t3890) + t3612*t3933 + t3620*t3936;
  p_output1[43]=t3901*t3936 + t3933*t3945 - 0.183*(t3908 - 1.*t3548*t3945) - 0.2*(t3904 - 1.*t3557*t3945);
  p_output1[44]=t3920*t3936 + t3933*t3957 - 0.183*(t3927 - 1.*t3548*t3957) - 0.2*(t3923 - 1.*t3557*t3957);
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



void Jp_RR_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
