/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:31 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FLfoot.h"

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
  double t919;
  double t1135;
  double t1100;
  double t1102;
  double t1137;
  double t1008;
  double t1119;
  double t1214;
  double t1215;
  double t1358;
  double t1769;
  double t1942;
  double t1943;
  double t867;
  double t2223;
  double t442;
  double t2479;
  double t2484;
  double t2488;
  double t2658;
  double t2814;
  double t2834;
  double t2856;
  double t3621;
  double t3622;
  double t4218;
  double t4579;
  double t4691;
  double t4948;
  double t451;
  double t487;
  double t540;
  double t738;
  double t1015;
  double t1042;
  double t1519;
  double t1540;
  double t1656;
  double t1705;
  double t5409;
  double t5477;
  double t5479;
  double t5483;
  double t5524;
  double t5547;
  double t2252;
  double t2310;
  double t2849;
  double t3554;
  double t3615;
  double t5759;
  double t5777;
  double t5798;
  double t4377;
  double t4399;
  double t4456;
  double t5861;
  double t5863;
  double t5867;
  double t5897;
  double t5900;
  double t5901;
  double t6559;
  double t6636;
  double t6655;
  double t6677;
  double t6687;
  double t6703;
  double t6869;
  double t6905;
  double t6991;
  double t7544;
  double t7545;
  double t7550;
  double t7557;
  double t7558;
  double t7677;
  double t7747;
  double t7789;
  double t7812;
  double t9207;
  double t9215;
  double t9224;
  double t9267;
  double t9286;
  double t9302;
  double t9307;
  double t9309;
  double t9316;
  double t9363;
  double t9366;
  double t9368;
  double t9381;
  double t9382;
  double t9411;
  double t9486;
  double t9490;
  double t9492;
  double t2128;
  double t9505;
  double t9506;
  double t9578;
  double t9579;
  double t9581;
  double t9631;
  double t9634;
  double t9661;
  double t9662;
  double t9664;
  double t9668;
  double t9669;
  double t9515;
  double t9582;
  double t9583;
  double t9584;
  double t9586;
  double t9587;
  double t9588;
  double t9590;
  double t9593;
  double t9599;
  double t9601;
  double t9602;
  double t9603;
  double t9605;
  double t9613;
  double t9621;
  double t9697;
  double t9698;
  double t9700;
  double t6079;
  double t9675;
  double t9676;
  double t9718;
  double t9719;
  double t9721;
  double t9722;
  double t9723;
  double t9742;
  double t9743;
  double t9744;
  double t9746;
  double t9747;
  double t9748;
  double t5921;
  double t6137;
  double t9708;
  double t9760;
  double t9761;
  double t9762;
  double t9765;
  double t9767;
  double t9774;
  double t9775;
  double t9776;
  double t9730;
  double t9735;
  double t9786;
  double t9787;
  double t9788;
  double t9751;
  double t9756;
  t919 = Sin(var1[3]);
  t1135 = Cos(var1[3]);
  t1100 = Cos(var1[5]);
  t1102 = Sin(var1[4]);
  t1137 = Sin(var1[5]);
  t1008 = Sin(var1[9]);
  t1119 = -1.*t1100*t919*t1102;
  t1214 = t1135*t1137;
  t1215 = t1119 + t1214;
  t1358 = Cos(var1[9]);
  t1769 = -1.*t1135*t1100;
  t1942 = -1.*t919*t1102*t1137;
  t1943 = t1769 + t1942;
  t867 = Cos(var1[4]);
  t2223 = Sin(var1[10]);
  t442 = Cos(var1[10]);
  t2479 = t1358*t1215;
  t2484 = -1.*t1008*t1943;
  t2488 = t2479 + t2484;
  t2658 = Cos(var1[11]);
  t2814 = -1.*t2658;
  t2834 = 1. + t2814;
  t2856 = Sin(var1[11]);
  t3621 = -1.*t867*t2223*t919;
  t3622 = t442*t2488;
  t4218 = t3621 + t3622;
  t4579 = -1.*t442*t867*t919;
  t4691 = -1.*t2223*t2488;
  t4948 = t4579 + t4691;
  t451 = -1.*t442;
  t487 = 1. + t451;
  t540 = 0.183*t487;
  t738 = 0. + t540;
  t1015 = -0.047*t1008;
  t1042 = 0. + t1015;
  t1519 = -1.*t1358;
  t1540 = 1. + t1519;
  t1656 = 0.047*t1540;
  t1705 = 0. + t1656;
  t5409 = t1135*t1100*t1102;
  t5477 = t919*t1137;
  t5479 = t5409 + t5477;
  t5483 = -1.*t1100*t919;
  t5524 = t1135*t1102*t1137;
  t5547 = t5483 + t5524;
  t2252 = 0.183*t2223;
  t2310 = 0. + t2252;
  t2849 = -0.2*t2834;
  t3554 = 0.183*t2856;
  t3615 = 0. + t2849 + t3554;
  t5759 = t1358*t5479;
  t5777 = -1.*t1008*t5547;
  t5798 = t5759 + t5777;
  t4377 = 0.183*t2834;
  t4399 = 0.2*t2856;
  t4456 = 0. + t4377 + t4399;
  t5861 = t1135*t867*t2223;
  t5863 = t442*t5798;
  t5867 = t5861 + t5863;
  t5897 = t442*t1135*t867;
  t5900 = -1.*t2223*t5798;
  t5901 = t5897 + t5900;
  t6559 = t1358*t1135*t867*t1100;
  t6636 = -1.*t1135*t867*t1008*t1137;
  t6655 = t6559 + t6636;
  t6677 = -1.*t1135*t2223*t1102;
  t6687 = t442*t6655;
  t6703 = t6677 + t6687;
  t6869 = -1.*t442*t1135*t1102;
  t6905 = -1.*t2223*t6655;
  t6991 = t6869 + t6905;
  t7544 = t1358*t867*t1100*t919;
  t7545 = -1.*t867*t1008*t919*t1137;
  t7550 = t7544 + t7545;
  t7557 = -1.*t2223*t919*t1102;
  t7558 = t442*t7550;
  t7677 = t7557 + t7558;
  t7747 = -1.*t442*t919*t1102;
  t7789 = -1.*t2223*t7550;
  t7812 = t7747 + t7789;
  t9207 = -1.*t1358*t1100*t1102;
  t9215 = t1008*t1102*t1137;
  t9224 = t9207 + t9215;
  t9267 = -1.*t867*t2223;
  t9286 = t442*t9224;
  t9302 = t9267 + t9286;
  t9307 = -1.*t442*t867;
  t9309 = -1.*t2223*t9224;
  t9316 = t9307 + t9309;
  t9363 = t1100*t919;
  t9366 = -1.*t1135*t1102*t1137;
  t9368 = t9363 + t9366;
  t9381 = -1.*t1008*t5479;
  t9382 = t1358*t9368;
  t9411 = t9381 + t9382;
  t9486 = t1100*t919*t1102;
  t9490 = -1.*t1135*t1137;
  t9492 = t9486 + t9490;
  t2128 = t1358*t1943;
  t9505 = -1.*t1008*t9492;
  t9506 = t9505 + t2128;
  t9578 = -1.*t867*t1100*t1008;
  t9579 = -1.*t1358*t867*t1137;
  t9581 = t9578 + t9579;
  t9631 = -1.*t1358*t5547;
  t9634 = t9381 + t9631;
  t9661 = t1135*t1100;
  t9662 = t919*t1102*t1137;
  t9664 = t9661 + t9662;
  t9668 = -1.*t1358*t9664;
  t9669 = t9505 + t9668;
  t9515 = t1358*t9492;
  t9582 = t2310*t9581;
  t9583 = t442*t3615*t9581;
  t9584 = -1.*t2223*t4456*t9581;
  t9586 = t1358*t867*t1100;
  t9587 = -1.*t867*t1008*t1137;
  t9588 = t9586 + t9587;
  t9590 = 0.13205*t9588;
  t9593 = -1.*t2658*t2223*t9581;
  t9599 = -1.*t442*t2856*t9581;
  t9601 = t9593 + t9599;
  t9602 = 0.183*t9601;
  t9603 = t442*t2658*t9581;
  t9605 = -1.*t2223*t2856*t9581;
  t9613 = t9603 + t9605;
  t9621 = -0.4*t9613;
  t9697 = -1.*t1135*t867*t2223;
  t9698 = -1.*t442*t5798;
  t9700 = t9697 + t9698;
  t6079 = t2658*t5901;
  t9675 = -1.*t1008*t9664;
  t9676 = t9515 + t9675;
  t9718 = -1.*t442*t9676;
  t9719 = t3621 + t9718;
  t9721 = t442*t867*t919;
  t9722 = -1.*t2223*t9676;
  t9723 = t9721 + t9722;
  t9742 = t2223*t1102;
  t9743 = -1.*t442*t9588;
  t9744 = t9742 + t9743;
  t9746 = -1.*t442*t1102;
  t9747 = -1.*t2223*t9588;
  t9748 = t9746 + t9747;
  t5921 = -1.*t2856*t5867;
  t6137 = t5921 + t6079;
  t9708 = -1.*t2856*t5901;
  t9760 = 0.183*t2658;
  t9761 = -0.2*t2856;
  t9762 = t9760 + t9761;
  t9765 = 0.2*t2658;
  t9767 = t9765 + t3554;
  t9774 = t867*t2223*t919;
  t9775 = t442*t9676;
  t9776 = t9774 + t9775;
  t9730 = t2658*t9723;
  t9735 = -1.*t2856*t9723;
  t9786 = -1.*t2223*t1102;
  t9787 = t442*t9588;
  t9788 = t9786 + t9787;
  t9751 = t2658*t9748;
  t9756 = -1.*t2856*t9748;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1042*t1215 + t1705*t1943 + 0.13205*(t1008*t1215 + t2128) + t2310*t2488 + t3615*t4218 + t4456*t4948 + 0.183*(-1.*t2856*t4218 + t2658*t4948) - 0.4*(t2658*t4218 + t2856*t4948) - 1.*t738*t867*t919;
  p_output1(10)=t1042*t5479 + t1705*t5547 + 0.13205*(t1008*t5479 + t1358*t5547) + t2310*t5798 + t3615*t5867 + t4456*t5901 - 0.4*(t2658*t5867 + t2856*t5901) + 0.183*t6137 + t1135*t738*t867;
  p_output1(11)=0;
  p_output1(12)=t2310*t6655 + t3615*t6703 + t4456*t6991 + 0.183*(-1.*t2856*t6703 + t2658*t6991) - 0.4*(t2658*t6703 + t2856*t6991) - 1.*t1102*t1135*t738 + t1042*t1100*t1135*t867 + t1135*t1137*t1705*t867 + 0.13205*(t1008*t1100*t1135*t867 + t1135*t1137*t1358*t867);
  p_output1(13)=t2310*t7550 + t3615*t7677 + t4456*t7812 + 0.183*(-1.*t2856*t7677 + t2658*t7812) - 0.4*(t2658*t7677 + t2856*t7812) - 1.*t1102*t738*t919 + t1042*t1100*t867*t919 + t1137*t1705*t867*t919 + 0.13205*(t1008*t1100*t867*t919 + t1137*t1358*t867*t919);
  p_output1(14)=-1.*t1042*t1100*t1102 + 0.13205*(-1.*t1008*t1100*t1102 - 1.*t1102*t1137*t1358) - 1.*t1102*t1137*t1705 - 1.*t738*t867 + t2310*t9224 + t3615*t9302 + t4456*t9316 + 0.183*(-1.*t2856*t9302 + t2658*t9316) - 0.4*(t2658*t9302 + t2856*t9316);
  p_output1(15)=t1705*t5479 + t1042*t9368 + 0.13205*(t5759 + t1008*t9368) + t2310*t9411 + t3615*t442*t9411 - 1.*t2223*t4456*t9411 - 0.4*(-1.*t2223*t2856*t9411 + t2658*t442*t9411) + 0.183*(-1.*t2223*t2658*t9411 - 1.*t2856*t442*t9411);
  p_output1(16)=t1042*t1943 + t1705*t9492 + t2310*t9506 + t3615*t442*t9506 - 1.*t2223*t4456*t9506 - 0.4*(-1.*t2223*t2856*t9506 + t2658*t442*t9506) + 0.183*(-1.*t2223*t2658*t9506 - 1.*t2856*t442*t9506) + 0.13205*(t1008*t1943 + t9515);
  p_output1(17)=-1.*t1042*t1137*t867 + t1100*t1705*t867 + t9582 + t9583 + t9584 + t9590 + t9602 + t9621;
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=-0.047*t1358*t5479 + 0.047*t1008*t5547 + 0.13205*t5798 + t2310*t9634 + t3615*t442*t9634 - 1.*t2223*t4456*t9634 - 0.4*(-1.*t2223*t2856*t9634 + t2658*t442*t9634) + 0.183*(-1.*t2223*t2658*t9634 - 1.*t2856*t442*t9634);
  p_output1(28)=-0.047*t1358*t9492 + 0.047*t1008*t9664 + t2310*t9669 + t3615*t442*t9669 - 1.*t2223*t4456*t9669 - 0.4*(-1.*t2223*t2856*t9669 + t2658*t442*t9669) + 0.183*(-1.*t2223*t2658*t9669 - 1.*t2856*t442*t9669) + 0.13205*t9676;
  p_output1(29)=0.047*t1008*t1137*t867 - 0.047*t1100*t1358*t867 + t9582 + t9583 + t9584 + t9590 + t9602 + t9621;
  p_output1(30)=0.183*t442*t5798 + t3615*t5901 + 0.183*t1135*t2223*t867 + t4456*t9700 - 0.4*(t6079 + t2856*t9700) + 0.183*(t2658*t9700 + t9708);
  p_output1(31)=0.183*t2223*t867*t919 + 0.183*t442*t9676 + t4456*t9719 + t3615*t9723 - 0.4*(t2856*t9719 + t9730) + 0.183*(t2658*t9719 + t9735);
  p_output1(32)=-0.183*t1102*t2223 + 0.183*t442*t9588 + t4456*t9744 + t3615*t9748 - 0.4*(t2856*t9744 + t9751) + 0.183*(t2658*t9744 + t9756);
  p_output1(33)=-0.4*t6137 + 0.183*(-1.*t2658*t5867 + t9708) + t5867*t9762 + t5901*t9767;
  p_output1(34)=t9723*t9767 + t9762*t9776 + 0.183*(t9735 - 1.*t2658*t9776) - 0.4*(t9730 - 1.*t2856*t9776);
  p_output1(35)=t9748*t9767 + t9762*t9788 + 0.183*(t9756 - 1.*t2658*t9788) - 0.4*(t9751 - 1.*t2856*t9788);
  p_output1(36)=0;
  p_output1(37)=0;
  p_output1(38)=0;
  p_output1(39)=0;
  p_output1(40)=0;
  p_output1(41)=0;
  p_output1(42)=0;
  p_output1(43)=0;
  p_output1(44)=0;
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
  p_output1(48)=0;
  p_output1(49)=0;
  p_output1(50)=0;
  p_output1(51)=0;
  p_output1(52)=0;
  p_output1(53)=0;
}


       
Eigen::Matrix<double,3,18> Jp_FLfoot(const Eigen::Matrix<double,18,1> &var1)
//void Jp_FLfoot(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



