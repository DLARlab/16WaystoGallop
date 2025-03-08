/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:04:10 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Ge_a1.h"

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
static void output1(Eigen::Matrix<double,18,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t104;
  double t55;
  double t103;
  double t159;
  double t193;
  double t266;
  double t471;
  double t488;
  double t468;
  double t655;
  double t666;
  double t261;
  double t269;
  double t300;
  double t310;
  double t328;
  double t345;
  double t350;
  double t381;
  double t412;
  double t419;
  double t859;
  double t810;
  double t479;
  double t503;
  double t523;
  double t532;
  double t559;
  double t583;
  double t584;
  double t588;
  double t618;
  double t635;
  double t1025;
  double t936;
  double t656;
  double t667;
  double t670;
  double t706;
  double t720;
  double t729;
  double t756;
  double t770;
  double t772;
  double t773;
  double t1154;
  double t1071;
  double t811;
  double t817;
  double t836;
  double t865;
  double t873;
  double t879;
  double t890;
  double t1202;
  double t1237;
  double t896;
  double t897;
  double t905;
  double t945;
  double t978;
  double t979;
  double t1026;
  double t1032;
  double t1035;
  double t1043;
  double t1522;
  double t1528;
  double t1048;
  double t1049;
  double t1053;
  double t1072;
  double t1097;
  double t1118;
  double t1157;
  double t1175;
  double t1176;
  double t1180;
  double t1688;
  double t1704;
  double t1187;
  double t1188;
  double t1189;
  double t175;
  double t2039;
  double t2076;
  double t2040;
  double t2051;
  double t2065;
  double t2077;
  double t2082;
  double t2084;
  double t2089;
  double t2097;
  double t2109;
  double t2155;
  double t2188;
  double t2259;
  double t2191;
  double t2214;
  double t2245;
  double t2265;
  double t2289;
  double t2290;
  double t2292;
  double t2311;
  double t2312;
  double t2313;
  double t2355;
  double t2373;
  double t2508;
  double t2509;
  double t2512;
  double t2513;
  double t2519;
  double t2523;
  double t2530;
  double t2534;
  double t2565;
  double t2567;
  double t2570;
  double t2563;
  double t2564;
  double t2591;
  double t2595;
  double t2603;
  double t2624;
  double t2627;
  double t2628;
  double t2631;
  double t2633;
  double t2648;
  double t2655;
  double t2656;
  double t1203;
  double t1208;
  double t1244;
  double t1269;
  double t1275;
  double t1307;
  double t1310;
  double t1316;
  double t1337;
  double t1525;
  double t1526;
  double t1534;
  double t1538;
  double t1542;
  double t1549;
  double t1550;
  double t1555;
  double t1567;
  double t1689;
  double t1700;
  double t1716;
  double t1722;
  double t1777;
  double t1793;
  double t1795;
  double t1809;
  double t1817;
  double t2886;
  double t2892;
  double t2907;
  double t2908;
  double t2912;
  double t2932;
  double t2934;
  double t2940;
  double t2358;
  double t2365;
  double t2375;
  double t2382;
  double t2387;
  double t2392;
  double t2393;
  double t2398;
  double t2403;
  double t2929;
  double t2942;
  double t3065;
  double t2964;
  double t2969;
  double t2970;
  double t2992;
  double t2996;
  double t3003;
  double t3011;
  double t3013;
  double t3023;
  double t3025;
  double t3034;
  double t3047;
  double t3048;
  double t3049;
  double t3051;
  double t3113;
  double t3116;
  double t3163;
  double t3188;
  double t3189;
  double t3124;
  double t3132;
  double t3140;
  double t3224;
  double t3275;
  double t3284;
  double t3287;
  double t3253;
  double t2520;
  double t2535;
  double t3358;
  double t2554;
  double t2557;
  double t2558;
  double t2684;
  double t2695;
  double t2699;
  double t2704;
  double t2720;
  double t2735;
  double t2737;
  double t2739;
  double t2745;
  double t2751;
  double t2754;
  double t2761;
  double t3376;
  double t3377;
  double t3378;
  double t3380;
  double t3384;
  double t3393;
  double t3395;
  double t3397;
  double t3461;
  double t3465;
  double t3469;
  double t3421;
  double t3437;
  double t2583;
  double t2590;
  double t2605;
  double t3501;
  double t2610;
  double t2618;
  double t2789;
  double t2791;
  double t2793;
  double t2794;
  double t2796;
  double t2798;
  double t2804;
  double t2805;
  double t2809;
  double t2820;
  double t2822;
  double t2823;
  double t3523;
  double t3525;
  double t3527;
  double t3530;
  double t3541;
  double t3547;
  double t3553;
  double t3554;
  double t3638;
  double t3661;
  double t3663;
  double t3579;
  double t3590;
  double t2634;
  double t2665;
  double t3691;
  double t2674;
  double t2675;
  double t2676;
  double t2830;
  double t2837;
  double t2838;
  double t2842;
  double t2843;
  double t2844;
  double t2846;
  double t2850;
  double t2864;
  double t2869;
  double t2874;
  double t2876;
  double t3716;
  double t3722;
  double t3723;
  double t3724;
  double t3725;
  double t3727;
  double t3736;
  double t3737;
  double t3807;
  double t3813;
  double t3817;
  double t3771;
  double t3785;
  t104 = Sin(var1[4]);
  t55 = Cos(var1[4]);
  t103 = Cos(var1[5]);
  t159 = Sin(var1[5]);
  t193 = Sin(var1[9]);
  t266 = Cos(var1[9]);
  t471 = Sin(var1[12]);
  t488 = Cos(var1[12]);
  t468 = 0.17969*t55;
  t655 = Sin(var1[15]);
  t666 = Cos(var1[15]);
  t261 = 0.047*t103*t193*t104;
  t269 = -1.*t266;
  t300 = 1. + t269;
  t310 = -0.047*t300*t104*t159;
  t328 = -1.*t103*t193*t104;
  t345 = -1.*t266*t104*t159;
  t350 = t328 + t345;
  t381 = -1.*t266*t103*t104;
  t412 = t193*t104*t159;
  t419 = t381 + t412;
  t859 = Sin(var1[10]);
  t810 = Cos(var1[10]);
  t479 = -0.047*t103*t471*t104;
  t503 = -1.*t488;
  t523 = 1. + t503;
  t532 = 0.047*t523*t104*t159;
  t559 = -1.*t103*t471*t104;
  t583 = -1.*t488*t104*t159;
  t584 = t559 + t583;
  t588 = -1.*t488*t103*t104;
  t618 = t471*t104*t159;
  t635 = t588 + t618;
  t1025 = Sin(var1[13]);
  t936 = Cos(var1[13]);
  t656 = 0.047*t103*t655*t104;
  t667 = -1.*t666;
  t670 = 1. + t667;
  t706 = -0.047*t670*t104*t159;
  t720 = -1.*t103*t655*t104;
  t729 = -1.*t666*t104*t159;
  t756 = t720 + t729;
  t770 = -1.*t666*t103*t104;
  t772 = t655*t104*t159;
  t773 = t770 + t772;
  t1154 = Sin(var1[16]);
  t1071 = Cos(var1[16]);
  t811 = -1.*t810;
  t817 = 1. + t811;
  t836 = -0.183*t817*t55;
  t865 = 0.183*t859*t419;
  t873 = -1.*t55*t859;
  t879 = t810*t419;
  t890 = t873 + t879;
  t1202 = Cos(var1[11]);
  t1237 = Sin(var1[11]);
  t896 = -1.*t810*t55;
  t897 = -1.*t859*t419;
  t905 = t896 + t897;
  t945 = -1.*t936;
  t978 = 1. + t945;
  t979 = 0.183*t978*t55;
  t1026 = -0.183*t1025*t635;
  t1032 = -1.*t55*t1025;
  t1035 = t936*t635;
  t1043 = t1032 + t1035;
  t1522 = Cos(var1[14]);
  t1528 = Sin(var1[14]);
  t1048 = -1.*t936*t55;
  t1049 = -1.*t1025*t635;
  t1053 = t1048 + t1049;
  t1072 = -1.*t1071;
  t1097 = 1. + t1072;
  t1118 = 0.183*t1097*t55;
  t1157 = -0.183*t1154*t773;
  t1175 = -1.*t55*t1154;
  t1176 = t1071*t773;
  t1180 = t1175 + t1176;
  t1688 = Cos(var1[17]);
  t1704 = Sin(var1[17]);
  t1187 = -1.*t1071*t55;
  t1188 = -1.*t1154*t773;
  t1189 = t1187 + t1188;
  t175 = -0.17969*t55;
  t2039 = Cos(var1[6]);
  t2076 = Sin(var1[6]);
  t2040 = -1.*t2039;
  t2051 = 1. + t2040;
  t2065 = 0.047*t2051*t104*t159;
  t2077 = -0.047*t103*t104*t2076;
  t2082 = -1.*t2039*t104*t159;
  t2084 = -1.*t103*t104*t2076;
  t2089 = t2082 + t2084;
  t2097 = -1.*t103*t2039*t104;
  t2109 = t104*t159*t2076;
  t2155 = t2097 + t2109;
  t2188 = Cos(var1[7]);
  t2259 = Sin(var1[7]);
  t2191 = -1.*t2188;
  t2214 = 1. + t2191;
  t2245 = -0.183*t55*t2214;
  t2265 = 0.183*t2155*t2259;
  t2289 = t2188*t2155;
  t2290 = -1.*t55*t2259;
  t2292 = t2289 + t2290;
  t2311 = -1.*t55*t2188;
  t2312 = -1.*t2155*t2259;
  t2313 = t2311 + t2312;
  t2355 = Cos(var1[8]);
  t2373 = Sin(var1[8]);
  t2508 = 0.047*t300*t55*t103;
  t2509 = 0.047*t55*t193*t159;
  t2512 = -1.*t55*t103*t193;
  t2513 = -1.*t266*t55*t159;
  t2519 = t2512 + t2513;
  t2523 = t266*t55*t103;
  t2530 = -1.*t55*t193*t159;
  t2534 = t2523 + t2530;
  t2565 = -1.*t55*t103*t471;
  t2567 = -1.*t488*t55*t159;
  t2570 = t2565 + t2567;
  t2563 = -0.047*t523*t55*t103;
  t2564 = -0.047*t55*t471*t159;
  t2591 = t488*t55*t103;
  t2595 = -1.*t55*t471*t159;
  t2603 = t2591 + t2595;
  t2624 = 0.047*t670*t55*t103;
  t2627 = 0.047*t55*t655*t159;
  t2628 = -1.*t55*t103*t655;
  t2631 = -1.*t666*t55*t159;
  t2633 = t2628 + t2631;
  t2648 = t666*t55*t103;
  t2655 = -1.*t55*t655*t159;
  t2656 = t2648 + t2655;
  t1203 = -1. + t1202;
  t1208 = 0.2*t1203;
  t1244 = 0.183*t1237;
  t1269 = t1208 + t1244;
  t1275 = -1.*t1202;
  t1307 = 1. + t1275;
  t1310 = 0.183*t1307;
  t1316 = 0.2*t1237;
  t1337 = t1310 + t1316;
  t1525 = -1. + t1522;
  t1526 = 0.2*t1525;
  t1534 = -0.183*t1528;
  t1538 = t1526 + t1534;
  t1542 = -1.*t1522;
  t1549 = 1. + t1542;
  t1550 = -0.183*t1549;
  t1555 = 0.2*t1528;
  t1567 = t1550 + t1555;
  t1689 = -1. + t1688;
  t1700 = 0.2*t1689;
  t1716 = -0.183*t1704;
  t1722 = t1700 + t1716;
  t1777 = -1.*t1688;
  t1793 = 1. + t1777;
  t1795 = -0.183*t1793;
  t1809 = 0.2*t1704;
  t1817 = t1795 + t1809;
  t2886 = -0.047*t55*t103*t2051;
  t2892 = -0.047*t55*t159*t2076;
  t2907 = -1.*t55*t2039*t159;
  t2908 = -1.*t55*t103*t2076;
  t2912 = t2907 + t2908;
  t2932 = t55*t103*t2039;
  t2934 = -1.*t55*t159*t2076;
  t2940 = t2932 + t2934;
  t2358 = -1. + t2355;
  t2365 = 0.2*t2358;
  t2375 = 0.183*t2373;
  t2382 = t2365 + t2375;
  t2387 = -1.*t2355;
  t2392 = 1. + t2387;
  t2393 = 0.183*t2392;
  t2398 = 0.2*t2373;
  t2403 = t2393 + t2398;
  t2929 = 0.00003*t2912;
  t2942 = -0.04764*t2940;
  t3065 = 0.047*t55*t103*t2039;
  t2964 = -0.02733*t2188*t2912;
  t2969 = -0.10972*t2940;
  t2970 = 0.00324*t2912*t2259;
  t2992 = -0.13205*t2940;
  t2996 = 0.183*t2912*t2259;
  t3003 = t2188*t2912*t2382;
  t3011 = -1.*t2912*t2259*t2403;
  t3013 = -1.*t2355*t2912*t2259;
  t3023 = -1.*t2188*t2912*t2373;
  t3025 = t3013 + t3023;
  t3034 = 0.18773*t3025;
  t3047 = t2188*t2355*t2912;
  t3048 = -1.*t2912*t2259*t2373;
  t3049 = t3047 + t3048;
  t3051 = -0.278879*t3049;
  t3113 = 0.183*t2188*t2940;
  t3116 = -0.183*t104*t2259;
  t3163 = -1.*t2188*t104;
  t3188 = -1.*t2940*t2259;
  t3189 = t3163 + t3188;
  t3124 = -1.*t2188*t2940;
  t3132 = t104*t2259;
  t3140 = t3124 + t3132;
  t3224 = t2355*t3189;
  t3275 = t2188*t2940;
  t3284 = -1.*t104*t2259;
  t3287 = t3275 + t3284;
  t3253 = -1.*t3189*t2373;
  t2520 = 0.00003*t2519;
  t2535 = 0.04764*t2534;
  t3358 = -0.047*t266*t55*t103;
  t2554 = -0.02733*t810*t2519;
  t2557 = 0.00324*t859*t2519;
  t2558 = 0.10972*t2534;
  t2684 = 0.183*t859*t2519;
  t2695 = t810*t1269*t2519;
  t2699 = -1.*t859*t1337*t2519;
  t2704 = 0.13205*t2534;
  t2720 = -1.*t1202*t859*t2519;
  t2735 = -1.*t810*t1237*t2519;
  t2737 = t2720 + t2735;
  t2739 = 0.18773*t2737;
  t2745 = t810*t1202*t2519;
  t2751 = -1.*t859*t1237*t2519;
  t2754 = t2745 + t2751;
  t2761 = -0.278879*t2754;
  t3376 = -0.183*t859*t104;
  t3377 = 0.183*t810*t2534;
  t3378 = t859*t104;
  t3380 = -1.*t810*t2534;
  t3384 = t3378 + t3380;
  t3393 = -1.*t810*t104;
  t3395 = -1.*t859*t2534;
  t3397 = t3393 + t3395;
  t3461 = -1.*t859*t104;
  t3465 = t810*t2534;
  t3469 = t3461 + t3465;
  t3421 = t1202*t3397;
  t3437 = -1.*t1237*t3397;
  t2583 = -0.02733*t936*t2570;
  t2590 = 0.00324*t1025*t2570;
  t2605 = -0.10972*t2603;
  t3501 = 0.047*t488*t55*t103;
  t2610 = 0.00003*t2570;
  t2618 = -0.04764*t2603;
  t2789 = -0.183*t1025*t2570;
  t2791 = t936*t1538*t2570;
  t2793 = -1.*t1025*t1567*t2570;
  t2794 = -0.13205*t2603;
  t2796 = -1.*t1522*t1025*t2570;
  t2798 = -1.*t936*t1528*t2570;
  t2804 = t2796 + t2798;
  t2805 = -0.17827*t2804;
  t2809 = t936*t1522*t2570;
  t2820 = -1.*t1025*t1528*t2570;
  t2822 = t2809 + t2820;
  t2823 = -0.278879*t2822;
  t3523 = 0.183*t1025*t104;
  t3525 = -0.183*t936*t2603;
  t3527 = t1025*t104;
  t3530 = -1.*t936*t2603;
  t3541 = t3527 + t3530;
  t3547 = -1.*t936*t104;
  t3553 = -1.*t1025*t2603;
  t3554 = t3547 + t3553;
  t3638 = -1.*t1025*t104;
  t3661 = t936*t2603;
  t3663 = t3638 + t3661;
  t3579 = t1522*t3554;
  t3590 = -1.*t1528*t3554;
  t2634 = 0.00003*t2633;
  t2665 = 0.04764*t2656;
  t3691 = -0.047*t666*t55*t103;
  t2674 = -0.02733*t1071*t2633;
  t2675 = 0.00324*t1154*t2633;
  t2676 = 0.10972*t2656;
  t2830 = -0.183*t1154*t2633;
  t2837 = t1071*t1722*t2633;
  t2838 = -1.*t1154*t1817*t2633;
  t2842 = 0.13205*t2656;
  t2843 = -1.*t1688*t1154*t2633;
  t2844 = -1.*t1071*t1704*t2633;
  t2846 = t2843 + t2844;
  t2850 = -0.17827*t2846;
  t2864 = t1071*t1688*t2633;
  t2869 = -1.*t1154*t1704*t2633;
  t2874 = t2864 + t2869;
  t2876 = -0.278879*t2874;
  t3716 = 0.183*t1154*t104;
  t3722 = -0.183*t1071*t2656;
  t3723 = t1154*t104;
  t3724 = -1.*t1071*t2656;
  t3725 = t3723 + t3724;
  t3727 = -1.*t1071*t104;
  t3736 = -1.*t1154*t2656;
  t3737 = t3727 + t3736;
  t3807 = -1.*t1154*t104;
  t3813 = t1071*t2656;
  t3817 = t3807 + t3813;
  t3771 = t1688*t3737;
  t3785 = -1.*t1704*t3737;

  p_output1(0)=0;
  p_output1(1)=0;
  p_output1(2)=122.17374000000001;
  p_output1(3)=0;
  p_output1(4)=6.82776*(t175 + t2065 + t2077 - 0.04764*t2089 + 0.00003*t2155) + 9.937529999999999*(t2065 + t2077 - 0.10972*t2089 + t2245 + t2265 - 0.02733*t2292 + 0.17976*t2313) + 2.21706*(t2065 + t2077 - 0.13205*t2089 + t2245 + t2265 + 0.18773*(t2313*t2355 - 1.*t2292*t2373) - 0.278879*(t2292*t2355 + t2313*t2373) + t2292*t2382 + t2313*t2403) + 6.82776*(t175 + t261 + t310 + 0.04764*t350 + 0.00003*t419) + 46.24434000000001*(-0.00052*t103*t104 - 0.00219*t104*t159 - 0.012727*t55) + 6.82776*(t468 + t479 + t532 - 0.04764*t584 + 0.00003*t635) + 9.937529999999999*(t1118 + t1157 - 0.02733*t1180 - 0.18624*t1189 + t656 + t706 + 0.10972*t756) + 2.21706*(t1118 + t1157 - 0.17827*(t1189*t1688 - 1.*t1180*t1704) - 0.278879*(t1180*t1688 + t1189*t1704) + t1180*t1722 + t1189*t1817 + t656 + t706 + 0.13205*t756) + 6.82776*(t468 + t656 + t706 + 0.04764*t756 + 0.00003*t773) + 9.937529999999999*(t261 + t310 + 0.10972*t350 + t836 + t865 - 0.02733*t890 + 0.17976*t905) + 2.21706*(t261 + t310 + 0.13205*t350 + t836 + t865 + t1269*t890 + t1337*t905 + 0.18773*(-1.*t1237*t890 + t1202*t905) - 0.278879*(t1202*t890 + t1237*t905)) + 2.21706*(t1026 - 0.17827*(t1053*t1522 - 1.*t1043*t1528) - 0.278879*(t1043*t1522 + t1053*t1528) + t1043*t1538 + t1053*t1567 + t479 + t532 - 0.13205*t584 + t979) + 9.937529999999999*(t1026 - 0.02733*t1043 - 0.18624*t1053 + t479 + t532 - 0.10972*t584 + t979);
  p_output1(5)=6.82776*(t2508 + t2509 + t2520 + t2535) + 9.937529999999999*(t2508 + t2509 + t2554 + t2557 + t2558) + 9.937529999999999*(t2563 + t2564 + t2583 + t2590 + t2605) + 6.82776*(t2563 + t2564 + t2610 + t2618) + 6.82776*(t2624 + t2627 + t2634 + t2665) + 9.937529999999999*(t2624 + t2627 + t2674 + t2675 + t2676) + 2.21706*(t2508 + t2509 + t2684 + t2695 + t2699 + t2704 + t2739 + t2761) + 2.21706*(t2563 + t2564 + t2789 + t2791 + t2793 + t2794 + t2805 + t2823) + 2.21706*(t2624 + t2627 + t2830 + t2837 + t2838 + t2842 + t2850 + t2876) + 6.82776*(t2886 + t2892 + t2929 + t2942) + 9.937529999999999*(t2886 + t2892 + t2964 + t2969 + t2970) + 2.21706*(t2886 + t2892 + t2992 + t2996 + t3003 + t3011 + t3034 + t3051) + 46.24434000000001*(0.00219*t103*t55 - 0.00052*t159*t55);
  p_output1(6)=6.82776*(t2892 + t2929 + t2942 + t3065) + 9.937529999999999*(t2892 + t2964 + t2969 + t2970 + t3065) + 2.21706*(t2892 + t2992 + t2996 + t3003 + t3011 + t3034 + t3051 + t3065);
  p_output1(7)=9.937529999999999*(t3113 + t3116 + 0.17976*t3140 - 0.02733*t3189) + 2.21706*(t3113 + t3116 + t2403*t3140 + t2382*t3189 - 0.278879*(t2373*t3140 + t3224) + 0.18773*(t2355*t3140 + t3253));
  p_output1(8)=2.21706*((0.2*t2355 + t2375)*t3189 + (0.183*t2355 - 0.2*t2373)*t3287 + 0.18773*(t3253 - 1.*t2355*t3287) - 0.278879*(t3224 - 1.*t2373*t3287));
  p_output1(9)=6.82776*(t2509 + t2520 + t2535 + t3358) + 9.937529999999999*(t2509 + t2554 + t2557 + t2558 + t3358) + 2.21706*(t2509 + t2684 + t2695 + t2699 + t2704 + t2739 + t2761 + t3358);
  p_output1(10)=9.937529999999999*(t3376 + t3377 + 0.17976*t3384 - 0.02733*t3397) + 2.21706*(t3376 + t3377 + t1337*t3384 + t1269*t3397 - 0.278879*(t1237*t3384 + t3421) + 0.18773*(t1202*t3384 + t3437));
  p_output1(11)=2.21706*((0.2*t1202 + t1244)*t3397 + (0.183*t1202 - 0.2*t1237)*t3469 + 0.18773*(t3437 - 1.*t1202*t3469) - 0.278879*(t3421 - 1.*t1237*t3469));
  p_output1(12)=9.937529999999999*(t2564 + t2583 + t2590 + t2605 + t3501) + 6.82776*(t2564 + t2610 + t2618 + t3501) + 2.21706*(t2564 + t2789 + t2791 + t2793 + t2794 + t2805 + t2823 + t3501);
  p_output1(13)=9.937529999999999*(t3523 + t3525 - 0.18624*t3541 - 0.02733*t3554) + 2.21706*(t3523 + t3525 + t1567*t3541 + t1538*t3554 - 0.278879*(t1528*t3541 + t3579) - 0.17827*(t1522*t3541 + t3590));
  p_output1(14)=2.21706*((0.2*t1522 + t1534)*t3554 + (-0.183*t1522 - 0.2*t1528)*t3663 - 0.17827*(t3590 - 1.*t1522*t3663) - 0.278879*(t3579 - 1.*t1528*t3663));
  p_output1(15)=6.82776*(t2627 + t2634 + t2665 + t3691) + 9.937529999999999*(t2627 + t2674 + t2675 + t2676 + t3691) + 2.21706*(t2627 + t2830 + t2837 + t2838 + t2842 + t2850 + t2876 + t3691);
  p_output1(16)=9.937529999999999*(t3716 + t3722 - 0.18624*t3725 - 0.02733*t3737) + 2.21706*(t3716 + t3722 + t1817*t3725 + t1722*t3737 - 0.278879*(t1704*t3725 + t3771) - 0.17827*(t1688*t3725 + t3785));
  p_output1(17)=2.21706*((0.2*t1688 + t1716)*t3737 + (-0.183*t1688 - 0.2*t1704)*t3817 - 0.17827*(t3785 - 1.*t1688*t3817) - 0.278879*(t3771 - 1.*t1704*t3817));
}


       
Eigen::Matrix<double,18,1> Ge_a1(const Eigen::Matrix<double,18,1> &var1)
//void Ge_a1(Eigen::Matrix<double,18,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,18,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



