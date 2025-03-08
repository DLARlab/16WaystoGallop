/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:35 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RRfoot.h"

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
  double t520;
  double t1074;
  double t1028;
  double t1037;
  double t1121;
  double t804;
  double t1049;
  double t1343;
  double t1367;
  double t1375;
  double t1394;
  double t1403;
  double t1415;
  double t249;
  double t1772;
  double t83;
  double t1809;
  double t1817;
  double t1881;
  double t1890;
  double t1913;
  double t2054;
  double t2181;
  double t2369;
  double t2387;
  double t2391;
  double t2851;
  double t2857;
  double t2858;
  double t84;
  double t216;
  double t238;
  double t244;
  double t861;
  double t992;
  double t1376;
  double t1382;
  double t1388;
  double t1392;
  double t3524;
  double t3561;
  double t3612;
  double t3637;
  double t3655;
  double t3860;
  double t1794;
  double t1808;
  double t2099;
  double t2249;
  double t2297;
  double t4245;
  double t4255;
  double t4348;
  double t2750;
  double t2830;
  double t2835;
  double t4607;
  double t4759;
  double t4776;
  double t4994;
  double t5031;
  double t5038;
  double t6364;
  double t6391;
  double t6423;
  double t6470;
  double t6493;
  double t6514;
  double t6524;
  double t6530;
  double t6532;
  double t6993;
  double t7004;
  double t7095;
  double t7144;
  double t7145;
  double t7158;
  double t7190;
  double t7218;
  double t7258;
  double t8186;
  double t8223;
  double t8232;
  double t8302;
  double t8358;
  double t8383;
  double t8394;
  double t8463;
  double t8496;
  double t9337;
  double t9351;
  double t9353;
  double t9360;
  double t9363;
  double t9382;
  double t9513;
  double t9515;
  double t9516;
  double t1711;
  double t9532;
  double t9539;
  double t9656;
  double t9658;
  double t9661;
  double t9706;
  double t9707;
  double t9742;
  double t9744;
  double t9745;
  double t9750;
  double t9752;
  double t9601;
  double t9667;
  double t9678;
  double t9679;
  double t9680;
  double t9681;
  double t9682;
  double t9683;
  double t9684;
  double t9685;
  double t9686;
  double t9687;
  double t9692;
  double t9693;
  double t9695;
  double t9696;
  double t9796;
  double t9797;
  double t9798;
  double t5303;
  double t9758;
  double t9759;
  double t9812;
  double t9815;
  double t9817;
  double t9820;
  double t9821;
  double t9834;
  double t9837;
  double t9838;
  double t9840;
  double t9841;
  double t9842;
  double t5127;
  double t5330;
  double t9805;
  double t9857;
  double t9859;
  double t9860;
  double t9863;
  double t9864;
  double t9875;
  double t9876;
  double t9878;
  double t9824;
  double t9828;
  double t9889;
  double t9890;
  double t9891;
  double t9846;
  double t9851;
  t520 = Sin(var1[3]);
  t1074 = Cos(var1[3]);
  t1028 = Cos(var1[5]);
  t1037 = Sin(var1[4]);
  t1121 = Sin(var1[5]);
  t804 = Sin(var1[12]);
  t1049 = -1.*t1028*t520*t1037;
  t1343 = t1074*t1121;
  t1367 = t1049 + t1343;
  t1375 = Cos(var1[12]);
  t1394 = -1.*t1074*t1028;
  t1403 = -1.*t520*t1037*t1121;
  t1415 = t1394 + t1403;
  t249 = Cos(var1[4]);
  t1772 = Sin(var1[13]);
  t83 = Cos(var1[13]);
  t1809 = t1375*t1367;
  t1817 = -1.*t804*t1415;
  t1881 = t1809 + t1817;
  t1890 = Cos(var1[14]);
  t1913 = -1.*t1890;
  t2054 = 1. + t1913;
  t2181 = Sin(var1[14]);
  t2369 = -1.*t249*t1772*t520;
  t2387 = t83*t1881;
  t2391 = t2369 + t2387;
  t2851 = -1.*t83*t249*t520;
  t2857 = -1.*t1772*t1881;
  t2858 = t2851 + t2857;
  t84 = -1.*t83;
  t216 = 1. + t84;
  t238 = -0.183*t216;
  t244 = 0. + t238;
  t861 = 0.047*t804;
  t992 = 0. + t861;
  t1376 = -1.*t1375;
  t1382 = 1. + t1376;
  t1388 = -0.047*t1382;
  t1392 = 0. + t1388;
  t3524 = t1074*t1028*t1037;
  t3561 = t520*t1121;
  t3612 = t3524 + t3561;
  t3637 = -1.*t1028*t520;
  t3655 = t1074*t1037*t1121;
  t3860 = t3637 + t3655;
  t1794 = -0.183*t1772;
  t1808 = 0. + t1794;
  t2099 = -0.2*t2054;
  t2249 = -0.183*t2181;
  t2297 = 0. + t2099 + t2249;
  t4245 = t1375*t3612;
  t4255 = -1.*t804*t3860;
  t4348 = t4245 + t4255;
  t2750 = -0.183*t2054;
  t2830 = 0.2*t2181;
  t2835 = 0. + t2750 + t2830;
  t4607 = t1074*t249*t1772;
  t4759 = t83*t4348;
  t4776 = t4607 + t4759;
  t4994 = t83*t1074*t249;
  t5031 = -1.*t1772*t4348;
  t5038 = t4994 + t5031;
  t6364 = t1375*t1074*t249*t1028;
  t6391 = -1.*t1074*t249*t804*t1121;
  t6423 = t6364 + t6391;
  t6470 = -1.*t1074*t1772*t1037;
  t6493 = t83*t6423;
  t6514 = t6470 + t6493;
  t6524 = -1.*t83*t1074*t1037;
  t6530 = -1.*t1772*t6423;
  t6532 = t6524 + t6530;
  t6993 = t1375*t249*t1028*t520;
  t7004 = -1.*t249*t804*t520*t1121;
  t7095 = t6993 + t7004;
  t7144 = -1.*t1772*t520*t1037;
  t7145 = t83*t7095;
  t7158 = t7144 + t7145;
  t7190 = -1.*t83*t520*t1037;
  t7218 = -1.*t1772*t7095;
  t7258 = t7190 + t7218;
  t8186 = -1.*t1375*t1028*t1037;
  t8223 = t804*t1037*t1121;
  t8232 = t8186 + t8223;
  t8302 = -1.*t249*t1772;
  t8358 = t83*t8232;
  t8383 = t8302 + t8358;
  t8394 = -1.*t83*t249;
  t8463 = -1.*t1772*t8232;
  t8496 = t8394 + t8463;
  t9337 = t1028*t520;
  t9351 = -1.*t1074*t1037*t1121;
  t9353 = t9337 + t9351;
  t9360 = -1.*t804*t3612;
  t9363 = t1375*t9353;
  t9382 = t9360 + t9363;
  t9513 = t1028*t520*t1037;
  t9515 = -1.*t1074*t1121;
  t9516 = t9513 + t9515;
  t1711 = t1375*t1415;
  t9532 = -1.*t804*t9516;
  t9539 = t9532 + t1711;
  t9656 = -1.*t249*t1028*t804;
  t9658 = -1.*t1375*t249*t1121;
  t9661 = t9656 + t9658;
  t9706 = -1.*t1375*t3860;
  t9707 = t9360 + t9706;
  t9742 = t1074*t1028;
  t9744 = t520*t1037*t1121;
  t9745 = t9742 + t9744;
  t9750 = -1.*t1375*t9745;
  t9752 = t9532 + t9750;
  t9601 = t1375*t9516;
  t9667 = t1808*t9661;
  t9678 = t83*t2297*t9661;
  t9679 = -1.*t1772*t2835*t9661;
  t9680 = t1375*t249*t1028;
  t9681 = -1.*t249*t804*t1121;
  t9682 = t9680 + t9681;
  t9683 = -0.13205*t9682;
  t9684 = -1.*t1890*t1772*t9661;
  t9685 = -1.*t83*t2181*t9661;
  t9686 = t9684 + t9685;
  t9687 = -0.183*t9686;
  t9692 = t83*t1890*t9661;
  t9693 = -1.*t1772*t2181*t9661;
  t9695 = t9692 + t9693;
  t9696 = -0.4*t9695;
  t9796 = -1.*t1074*t249*t1772;
  t9797 = -1.*t83*t4348;
  t9798 = t9796 + t9797;
  t5303 = t1890*t5038;
  t9758 = -1.*t804*t9745;
  t9759 = t9601 + t9758;
  t9812 = -1.*t83*t9759;
  t9815 = t2369 + t9812;
  t9817 = t83*t249*t520;
  t9820 = -1.*t1772*t9759;
  t9821 = t9817 + t9820;
  t9834 = t1772*t1037;
  t9837 = -1.*t83*t9682;
  t9838 = t9834 + t9837;
  t9840 = -1.*t83*t1037;
  t9841 = -1.*t1772*t9682;
  t9842 = t9840 + t9841;
  t5127 = -1.*t2181*t4776;
  t5330 = t5127 + t5303;
  t9805 = -1.*t2181*t5038;
  t9857 = -0.183*t1890;
  t9859 = -0.2*t2181;
  t9860 = t9857 + t9859;
  t9863 = 0.2*t1890;
  t9864 = t9863 + t2249;
  t9875 = t249*t1772*t520;
  t9876 = t83*t9759;
  t9878 = t9875 + t9876;
  t9824 = t1890*t9821;
  t9828 = -1.*t2181*t9821;
  t9889 = -1.*t1772*t1037;
  t9890 = t83*t9682;
  t9891 = t9889 + t9890;
  t9846 = t1890*t9842;
  t9851 = -1.*t2181*t9842;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1392*t1415 + t1808*t1881 + t2297*t2391 + t2835*t2858 - 0.183*(-1.*t2181*t2391 + t1890*t2858) - 0.4*(t1890*t2391 + t2181*t2858) - 1.*t244*t249*t520 - 0.13205*(t1711 + t1367*t804) + t1367*t992;
  p_output1(10)=t1074*t244*t249 + t1392*t3860 + t1808*t4348 + t2297*t4776 + t2835*t5038 - 0.4*(t1890*t4776 + t2181*t5038) - 0.183*t5330 - 0.13205*(t1375*t3860 + t3612*t804) + t3612*t992;
  p_output1(11)=0;
  p_output1(12)=-1.*t1037*t1074*t244 + t1074*t1121*t1392*t249 + t1808*t6423 + t2297*t6514 + t2835*t6532 - 0.183*(-1.*t2181*t6514 + t1890*t6532) - 0.4*(t1890*t6514 + t2181*t6532) - 0.13205*(t1074*t1121*t1375*t249 + t1028*t1074*t249*t804) + t1028*t1074*t249*t992;
  p_output1(13)=-1.*t1037*t244*t520 + t1121*t1392*t249*t520 + t1808*t7095 + t2297*t7158 + t2835*t7258 - 0.183*(-1.*t2181*t7158 + t1890*t7258) - 0.4*(t1890*t7158 + t2181*t7258) - 0.13205*(t1121*t1375*t249*t520 + t1028*t249*t520*t804) + t1028*t249*t520*t992;
  p_output1(14)=-1.*t1037*t1121*t1392 - 1.*t244*t249 - 0.13205*(-1.*t1037*t1121*t1375 - 1.*t1028*t1037*t804) + t1808*t8232 + t2297*t8383 + t2835*t8496 - 0.183*(-1.*t2181*t8383 + t1890*t8496) - 0.4*(t1890*t8383 + t2181*t8496) - 1.*t1028*t1037*t992;
  p_output1(15)=t1392*t3612 - 0.13205*(t4245 + t804*t9353) + t1808*t9382 - 1.*t1772*t2835*t9382 + t2297*t83*t9382 - 0.4*(-1.*t1772*t2181*t9382 + t1890*t83*t9382) - 0.183*(-1.*t1772*t1890*t9382 - 1.*t2181*t83*t9382) + t9353*t992;
  p_output1(16)=t1392*t9516 + t1808*t9539 - 1.*t1772*t2835*t9539 + t2297*t83*t9539 - 0.4*(-1.*t1772*t2181*t9539 + t1890*t83*t9539) - 0.183*(-1.*t1772*t1890*t9539 - 1.*t2181*t83*t9539) - 0.13205*(t1415*t804 + t9601) + t1415*t992;
  p_output1(17)=t1028*t1392*t249 + t9667 + t9678 + t9679 + t9683 + t9687 + t9696 - 1.*t1121*t249*t992;
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
  p_output1(36)=0.047*t1375*t3612 - 0.13205*t4348 - 0.047*t3860*t804 + t1808*t9707 - 1.*t1772*t2835*t9707 + t2297*t83*t9707 - 0.4*(-1.*t1772*t2181*t9707 + t1890*t83*t9707) - 0.183*(-1.*t1772*t1890*t9707 - 1.*t2181*t83*t9707);
  p_output1(37)=0.047*t1375*t9516 - 0.047*t804*t9745 + t1808*t9752 - 1.*t1772*t2835*t9752 + t2297*t83*t9752 - 0.4*(-1.*t1772*t2181*t9752 + t1890*t83*t9752) - 0.183*(-1.*t1772*t1890*t9752 - 1.*t2181*t83*t9752) - 0.13205*t9759;
  p_output1(38)=0.047*t1028*t1375*t249 - 0.047*t1121*t249*t804 + t9667 + t9678 + t9679 + t9683 + t9687 + t9696;
  p_output1(39)=-0.183*t1074*t1772*t249 + t2297*t5038 - 0.183*t4348*t83 + t2835*t9798 - 0.4*(t5303 + t2181*t9798) - 0.183*(t1890*t9798 + t9805);
  p_output1(40)=-0.183*t1772*t249*t520 - 0.183*t83*t9759 + t2835*t9815 + t2297*t9821 - 0.4*(t2181*t9815 + t9824) - 0.183*(t1890*t9815 + t9828);
  p_output1(41)=0.183*t1037*t1772 - 0.183*t83*t9682 + t2835*t9838 + t2297*t9842 - 0.4*(t2181*t9838 + t9846) - 0.183*(t1890*t9838 + t9851);
  p_output1(42)=-0.4*t5330 - 0.183*(-1.*t1890*t4776 + t9805) + t4776*t9860 + t5038*t9864;
  p_output1(43)=t9821*t9864 + t9860*t9878 - 0.183*(t9828 - 1.*t1890*t9878) - 0.4*(t9824 - 1.*t2181*t9878);
  p_output1(44)=t9842*t9864 + t9860*t9891 - 0.183*(t9851 - 1.*t1890*t9891) - 0.4*(t9846 - 1.*t2181*t9891);
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


       
Eigen::Matrix<double,3,18> Jp_RRfoot(const Eigen::Matrix<double,18,1> &var1)
//void Jp_RRfoot(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



