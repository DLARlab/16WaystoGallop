/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:39 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RLfoot.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE static inline        /* use standard inline */
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
static void output1(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t617;
  double t1084;
  double t789;
  double t912;
  double t1086;
  double t744;
  double t1000;
  double t1103;
  double t1342;
  double t2354;
  double t2410;
  double t2411;
  double t2416;
  double t571;
  double t2693;
  double t6;
  double t3035;
  double t3066;
  double t3067;
  double t3481;
  double t3502;
  double t3559;
  double t3569;
  double t3836;
  double t4023;
  double t4082;
  double t4116;
  double t4207;
  double t4229;
  double t21;
  double t104;
  double t105;
  double t529;
  double t771;
  double t774;
  double t2362;
  double t2366;
  double t2368;
  double t2398;
  double t5001;
  double t5088;
  double t5108;
  double t5158;
  double t5325;
  double t5345;
  double t2701;
  double t2983;
  double t3566;
  double t3729;
  double t3767;
  double t5610;
  double t5613;
  double t5679;
  double t4101;
  double t4113;
  double t4115;
  double t5828;
  double t5833;
  double t5834;
  double t5844;
  double t5853;
  double t5866;
  double t5951;
  double t5952;
  double t5955;
  double t5961;
  double t5964;
  double t5965;
  double t5967;
  double t5970;
  double t5971;
  double t6151;
  double t6154;
  double t6157;
  double t6191;
  double t6216;
  double t6281;
  double t6334;
  double t6339;
  double t6357;
  double t6622;
  double t6629;
  double t6632;
  double t6634;
  double t6638;
  double t6643;
  double t6693;
  double t6742;
  double t6767;
  double t6932;
  double t6954;
  double t6962;
  double t7159;
  double t7183;
  double t7211;
  double t7554;
  double t7586;
  double t7623;
  double t2547;
  double t7736;
  double t7772;
  double t8644;
  double t8651;
  double t8653;
  double t9466;
  double t9467;
  double t9626;
  double t9628;
  double t9638;
  double t9641;
  double t9642;
  double t8026;
  double t8657;
  double t8681;
  double t8764;
  double t8776;
  double t8799;
  double t8889;
  double t8910;
  double t8918;
  double t8951;
  double t8974;
  double t9072;
  double t9170;
  double t9253;
  double t9353;
  double t9359;
  double t9733;
  double t9738;
  double t9739;
  double t5885;
  double t9652;
  double t9653;
  double t9787;
  double t9790;
  double t9792;
  double t9795;
  double t9798;
  double t9845;
  double t9847;
  double t9848;
  double t9852;
  double t9853;
  double t9856;
  double t5882;
  double t5899;
  double t9765;
  double t9876;
  double t9879;
  double t9883;
  double t9885;
  double t9886;
  double t9895;
  double t9896;
  double t9897;
  double t9801;
  double t9827;
  double t9913;
  double t9914;
  double t9915;
  double t9860;
  double t9865;
  t617 = Sin(var1[3]);
  t1084 = Cos(var1[3]);
  t789 = Cos(var1[5]);
  t912 = Sin(var1[4]);
  t1086 = Sin(var1[5]);
  t744 = Sin(var1[15]);
  t1000 = -1.*t789*t617*t912;
  t1103 = t1084*t1086;
  t1342 = t1000 + t1103;
  t2354 = Cos(var1[15]);
  t2410 = -1.*t1084*t789;
  t2411 = -1.*t617*t912*t1086;
  t2416 = t2410 + t2411;
  t571 = Cos(var1[4]);
  t2693 = Sin(var1[16]);
  t6 = Cos(var1[16]);
  t3035 = t2354*t1342;
  t3066 = -1.*t744*t2416;
  t3067 = t3035 + t3066;
  t3481 = Cos(var1[17]);
  t3502 = -1.*t3481;
  t3559 = 1. + t3502;
  t3569 = Sin(var1[17]);
  t3836 = -1.*t571*t2693*t617;
  t4023 = t6*t3067;
  t4082 = t3836 + t4023;
  t4116 = -1.*t6*t571*t617;
  t4207 = -1.*t2693*t3067;
  t4229 = t4116 + t4207;
  t21 = -1.*t6;
  t104 = 1. + t21;
  t105 = -0.183*t104;
  t529 = 0. + t105;
  t771 = -0.047*t744;
  t774 = 0. + t771;
  t2362 = -1.*t2354;
  t2366 = 1. + t2362;
  t2368 = 0.047*t2366;
  t2398 = 0. + t2368;
  t5001 = t1084*t789*t912;
  t5088 = t617*t1086;
  t5108 = t5001 + t5088;
  t5158 = -1.*t789*t617;
  t5325 = t1084*t912*t1086;
  t5345 = t5158 + t5325;
  t2701 = -0.183*t2693;
  t2983 = 0. + t2701;
  t3566 = -0.2*t3559;
  t3729 = -0.183*t3569;
  t3767 = 0. + t3566 + t3729;
  t5610 = t2354*t5108;
  t5613 = -1.*t744*t5345;
  t5679 = t5610 + t5613;
  t4101 = -0.183*t3559;
  t4113 = 0.2*t3569;
  t4115 = 0. + t4101 + t4113;
  t5828 = t1084*t571*t2693;
  t5833 = t6*t5679;
  t5834 = t5828 + t5833;
  t5844 = t6*t1084*t571;
  t5853 = -1.*t2693*t5679;
  t5866 = t5844 + t5853;
  t5951 = t2354*t1084*t571*t789;
  t5952 = -1.*t1084*t571*t744*t1086;
  t5955 = t5951 + t5952;
  t5961 = -1.*t1084*t2693*t912;
  t5964 = t6*t5955;
  t5965 = t5961 + t5964;
  t5967 = -1.*t6*t1084*t912;
  t5970 = -1.*t2693*t5955;
  t5971 = t5967 + t5970;
  t6151 = t2354*t571*t789*t617;
  t6154 = -1.*t571*t744*t617*t1086;
  t6157 = t6151 + t6154;
  t6191 = -1.*t2693*t617*t912;
  t6216 = t6*t6157;
  t6281 = t6191 + t6216;
  t6334 = -1.*t6*t617*t912;
  t6339 = -1.*t2693*t6157;
  t6357 = t6334 + t6339;
  t6622 = -1.*t2354*t789*t912;
  t6629 = t744*t912*t1086;
  t6632 = t6622 + t6629;
  t6634 = -1.*t571*t2693;
  t6638 = t6*t6632;
  t6643 = t6634 + t6638;
  t6693 = -1.*t6*t571;
  t6742 = -1.*t2693*t6632;
  t6767 = t6693 + t6742;
  t6932 = t789*t617;
  t6954 = -1.*t1084*t912*t1086;
  t6962 = t6932 + t6954;
  t7159 = -1.*t744*t5108;
  t7183 = t2354*t6962;
  t7211 = t7159 + t7183;
  t7554 = t789*t617*t912;
  t7586 = -1.*t1084*t1086;
  t7623 = t7554 + t7586;
  t2547 = t2354*t2416;
  t7736 = -1.*t744*t7623;
  t7772 = t7736 + t2547;
  t8644 = -1.*t571*t789*t744;
  t8651 = -1.*t2354*t571*t1086;
  t8653 = t8644 + t8651;
  t9466 = -1.*t2354*t5345;
  t9467 = t7159 + t9466;
  t9626 = t1084*t789;
  t9628 = t617*t912*t1086;
  t9638 = t9626 + t9628;
  t9641 = -1.*t2354*t9638;
  t9642 = t7736 + t9641;
  t8026 = t2354*t7623;
  t8657 = t2983*t8653;
  t8681 = t6*t3767*t8653;
  t8764 = -1.*t2693*t4115*t8653;
  t8776 = t2354*t571*t789;
  t8799 = -1.*t571*t744*t1086;
  t8889 = t8776 + t8799;
  t8910 = 0.13205*t8889;
  t8918 = -1.*t3481*t2693*t8653;
  t8951 = -1.*t6*t3569*t8653;
  t8974 = t8918 + t8951;
  t9072 = -0.183*t8974;
  t9170 = t6*t3481*t8653;
  t9253 = -1.*t2693*t3569*t8653;
  t9353 = t9170 + t9253;
  t9359 = -0.4*t9353;
  t9733 = -1.*t1084*t571*t2693;
  t9738 = -1.*t6*t5679;
  t9739 = t9733 + t9738;
  t5885 = t3481*t5866;
  t9652 = -1.*t744*t9638;
  t9653 = t8026 + t9652;
  t9787 = -1.*t6*t9653;
  t9790 = t3836 + t9787;
  t9792 = t6*t571*t617;
  t9795 = -1.*t2693*t9653;
  t9798 = t9792 + t9795;
  t9845 = t2693*t912;
  t9847 = -1.*t6*t8889;
  t9848 = t9845 + t9847;
  t9852 = -1.*t6*t912;
  t9853 = -1.*t2693*t8889;
  t9856 = t9852 + t9853;
  t5882 = -1.*t3569*t5834;
  t5899 = t5882 + t5885;
  t9765 = -1.*t3569*t5866;
  t9876 = -0.183*t3481;
  t9879 = -0.2*t3569;
  t9883 = t9876 + t9879;
  t9885 = 0.2*t3481;
  t9886 = t9885 + t3729;
  t9895 = t571*t2693*t617;
  t9896 = t6*t9653;
  t9897 = t9895 + t9896;
  t9801 = t3481*t9798;
  t9827 = -1.*t3569*t9798;
  t9913 = -1.*t2693*t912;
  t9914 = t6*t8889;
  t9915 = t9913 + t9914;
  t9860 = t3481*t9856;
  t9865 = -1.*t3569*t9856;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t2398*t2416 + t2983*t3067 + t3767*t4082 + t4115*t4229 - 0.183*(-1.*t3569*t4082 + t3481*t4229) - 0.4*(t3481*t4082 + t3569*t4229) - 1.*t529*t571*t617 + 0.13205*(t2547 + t1342*t744) + t1342*t774;
  p_output1(10)=t2398*t5345 + t2983*t5679 + t1084*t529*t571 + t3767*t5834 + t4115*t5866 - 0.4*(t3481*t5834 + t3569*t5866) - 0.183*t5899 + 0.13205*(t2354*t5345 + t5108*t744) + t5108*t774;
  p_output1(11)=0;
  p_output1(12)=t1084*t1086*t2398*t571 + t2983*t5955 + t3767*t5965 + t4115*t5971 - 0.183*(-1.*t3569*t5965 + t3481*t5971) - 0.4*(t3481*t5965 + t3569*t5971) + t1084*t571*t774*t789 + 0.13205*(t1084*t1086*t2354*t571 + t1084*t571*t744*t789) - 1.*t1084*t529*t912;
  p_output1(13)=t2983*t6157 + t1086*t2398*t571*t617 + t3767*t6281 + t4115*t6357 - 0.183*(-1.*t3569*t6281 + t3481*t6357) - 0.4*(t3481*t6281 + t3569*t6357) + t571*t617*t774*t789 + 0.13205*(t1086*t2354*t571*t617 + t571*t617*t744*t789) - 1.*t529*t617*t912;
  p_output1(14)=-1.*t529*t571 + t2983*t6632 + t3767*t6643 + t4115*t6767 - 0.183*(-1.*t3569*t6643 + t3481*t6767) - 0.4*(t3481*t6643 + t3569*t6767) - 1.*t1086*t2398*t912 - 1.*t774*t789*t912 + 0.13205*(-1.*t1086*t2354*t912 - 1.*t744*t789*t912);
  p_output1(15)=t2398*t5108 + t2983*t7211 - 1.*t2693*t4115*t7211 + t3767*t6*t7211 - 0.4*(-1.*t2693*t3569*t7211 + t3481*t6*t7211) - 0.183*(-1.*t2693*t3481*t7211 - 1.*t3569*t6*t7211) + 0.13205*(t5610 + t6962*t744) + t6962*t774;
  p_output1(16)=t2398*t7623 + t2416*t774 + t2983*t7772 - 1.*t2693*t4115*t7772 + t3767*t6*t7772 - 0.4*(-1.*t2693*t3569*t7772 + t3481*t6*t7772) - 0.183*(-1.*t2693*t3481*t7772 - 1.*t3569*t6*t7772) + 0.13205*(t2416*t744 + t8026);
  p_output1(17)=-1.*t1086*t571*t774 + t2398*t571*t789 + t8657 + t8681 + t8764 + t8910 + t9072 + t9359;
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=0;
  p_output1(34)=0;
  p_output1(35)=0;
  p_output1(36)=0;
  p_output1(37)=0;
  p_output1(38)=0;
  p_output1(39)=0;
  p_output1(40)=0;
  p_output1(41)=0;
  p_output1(42)=0;
  p_output1(43)=0;
  p_output1(44)=0;
  p_output1(45)=-0.047*t2354*t5108 + 0.13205*t5679 + 0.047*t5345*t744 + t2983*t9467 - 1.*t2693*t4115*t9467 + t3767*t6*t9467 - 0.4*(-1.*t2693*t3569*t9467 + t3481*t6*t9467) - 0.183*(-1.*t2693*t3481*t9467 - 1.*t3569*t6*t9467);
  p_output1(46)=-0.047*t2354*t7623 + 0.047*t744*t9638 + t2983*t9642 - 1.*t2693*t4115*t9642 + t3767*t6*t9642 - 0.4*(-1.*t2693*t3569*t9642 + t3481*t6*t9642) - 0.183*(-1.*t2693*t3481*t9642 - 1.*t3569*t6*t9642) + 0.13205*t9653;
  p_output1(47)=0.047*t1086*t571*t744 - 0.047*t2354*t571*t789 + t8657 + t8681 + t8764 + t8910 + t9072 + t9359;
  p_output1(48)=-0.183*t1084*t2693*t571 + t3767*t5866 - 0.183*t5679*t6 + t4115*t9739 - 0.4*(t5885 + t3569*t9739) - 0.183*(t3481*t9739 + t9765);
  p_output1(49)=-0.183*t2693*t571*t617 - 0.183*t6*t9653 + t4115*t9790 + t3767*t9798 - 0.4*(t3569*t9790 + t9801) - 0.183*(t3481*t9790 + t9827);
  p_output1(50)=-0.183*t6*t8889 + 0.183*t2693*t912 + t4115*t9848 + t3767*t9856 - 0.4*(t3569*t9848 + t9860) - 0.183*(t3481*t9848 + t9865);
  p_output1(51)=-0.4*t5899 - 0.183*(-1.*t3481*t5834 + t9765) + t5834*t9883 + t5866*t9886;
  p_output1(52)=t9798*t9886 + t9883*t9897 - 0.183*(t9827 - 1.*t3481*t9897) - 0.4*(t9801 - 1.*t3569*t9897);
  p_output1(53)=t9856*t9886 + t9883*t9915 - 0.183*(t9865 - 1.*t3481*t9915) - 0.4*(t9860 - 1.*t3569*t9915);
}


       
Eigen::Matrix<double,3,18> Jp_RLfoot(const Eigen::Matrix<double,18,1> &var1)
//void Jp_RLfoot(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



