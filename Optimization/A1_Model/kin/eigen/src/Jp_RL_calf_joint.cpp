/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:24 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RL_calf_joint.h"

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
  double t360;
  double t1467;
  double t711;
  double t766;
  double t1490;
  double t515;
  double t1115;
  double t1527;
  double t1617;
  double t1762;
  double t1851;
  double t1855;
  double t1867;
  double t323;
  double t2193;
  double t111;
  double t2485;
  double t2486;
  double t2492;
  double t2500;
  double t2598;
  double t2675;
  double t2747;
  double t2807;
  double t2831;
  double t2836;
  double t3227;
  double t3472;
  double t3521;
  double t264;
  double t303;
  double t310;
  double t314;
  double t609;
  double t654;
  double t1801;
  double t1806;
  double t1813;
  double t1818;
  double t4257;
  double t4260;
  double t4262;
  double t4265;
  double t4266;
  double t4295;
  double t2293;
  double t2317;
  double t2692;
  double t2754;
  double t2761;
  double t4445;
  double t4460;
  double t4537;
  double t3013;
  double t3026;
  double t3213;
  double t4881;
  double t4892;
  double t4963;
  double t4982;
  double t5030;
  double t5037;
  double t6513;
  double t6637;
  double t6656;
  double t6667;
  double t6687;
  double t6690;
  double t6945;
  double t6955;
  double t7284;
  double t9107;
  double t9113;
  double t9117;
  double t9123;
  double t9125;
  double t9127;
  double t9130;
  double t9139;
  double t9140;
  double t9289;
  double t9291;
  double t9297;
  double t9299;
  double t9300;
  double t9301;
  double t9303;
  double t9304;
  double t9305;
  double t9327;
  double t9328;
  double t9329;
  double t9331;
  double t9332;
  double t9333;
  double t9356;
  double t9357;
  double t9358;
  double t1928;
  double t9361;
  double t9362;
  double t9388;
  double t9389;
  double t9391;
  double t9414;
  double t9415;
  double t9432;
  double t9433;
  double t9434;
  double t9442;
  double t9448;
  double t9366;
  double t9392;
  double t9394;
  double t9396;
  double t9397;
  double t9398;
  double t9399;
  double t9400;
  double t9401;
  double t9402;
  double t9403;
  double t9404;
  double t9405;
  double t9406;
  double t9407;
  double t9408;
  double t9474;
  double t9476;
  double t9479;
  double t5082;
  double t9455;
  double t9456;
  double t9494;
  double t9495;
  double t9497;
  double t9498;
  double t9501;
  double t9516;
  double t9517;
  double t9520;
  double t9523;
  double t9524;
  double t9525;
  double t5081;
  double t5172;
  double t9488;
  double t9542;
  double t9543;
  double t9544;
  double t9547;
  double t9548;
  double t9555;
  double t9556;
  double t9557;
  double t9504;
  double t9509;
  double t9568;
  double t9569;
  double t9570;
  double t9529;
  double t9534;
  t360 = Sin(var1[3]);
  t1467 = Cos(var1[3]);
  t711 = Cos(var1[5]);
  t766 = Sin(var1[4]);
  t1490 = Sin(var1[5]);
  t515 = Sin(var1[15]);
  t1115 = -1.*t711*t360*t766;
  t1527 = t1467*t1490;
  t1617 = t1115 + t1527;
  t1762 = Cos(var1[15]);
  t1851 = -1.*t1467*t711;
  t1855 = -1.*t360*t766*t1490;
  t1867 = t1851 + t1855;
  t323 = Cos(var1[4]);
  t2193 = Sin(var1[16]);
  t111 = Cos(var1[16]);
  t2485 = t1762*t1617;
  t2486 = -1.*t515*t1867;
  t2492 = t2485 + t2486;
  t2500 = Cos(var1[17]);
  t2598 = -1.*t2500;
  t2675 = 1. + t2598;
  t2747 = Sin(var1[17]);
  t2807 = -1.*t323*t2193*t360;
  t2831 = t111*t2492;
  t2836 = t2807 + t2831;
  t3227 = -1.*t111*t323*t360;
  t3472 = -1.*t2193*t2492;
  t3521 = t3227 + t3472;
  t264 = -1.*t111;
  t303 = 1. + t264;
  t310 = -0.183*t303;
  t314 = 0. + t310;
  t609 = -0.047*t515;
  t654 = 0. + t609;
  t1801 = -1.*t1762;
  t1806 = 1. + t1801;
  t1813 = 0.047*t1806;
  t1818 = 0. + t1813;
  t4257 = t1467*t711*t766;
  t4260 = t360*t1490;
  t4262 = t4257 + t4260;
  t4265 = -1.*t711*t360;
  t4266 = t1467*t766*t1490;
  t4295 = t4265 + t4266;
  t2293 = -0.183*t2193;
  t2317 = 0. + t2293;
  t2692 = -0.2*t2675;
  t2754 = -0.183*t2747;
  t2761 = 0. + t2692 + t2754;
  t4445 = t1762*t4262;
  t4460 = -1.*t515*t4295;
  t4537 = t4445 + t4460;
  t3013 = -0.183*t2675;
  t3026 = 0.2*t2747;
  t3213 = 0. + t3013 + t3026;
  t4881 = t1467*t323*t2193;
  t4892 = t111*t4537;
  t4963 = t4881 + t4892;
  t4982 = t111*t1467*t323;
  t5030 = -1.*t2193*t4537;
  t5037 = t4982 + t5030;
  t6513 = t1762*t1467*t323*t711;
  t6637 = -1.*t1467*t323*t515*t1490;
  t6656 = t6513 + t6637;
  t6667 = -1.*t1467*t2193*t766;
  t6687 = t111*t6656;
  t6690 = t6667 + t6687;
  t6945 = -1.*t111*t1467*t766;
  t6955 = -1.*t2193*t6656;
  t7284 = t6945 + t6955;
  t9107 = t1762*t323*t711*t360;
  t9113 = -1.*t323*t515*t360*t1490;
  t9117 = t9107 + t9113;
  t9123 = -1.*t2193*t360*t766;
  t9125 = t111*t9117;
  t9127 = t9123 + t9125;
  t9130 = -1.*t111*t360*t766;
  t9139 = -1.*t2193*t9117;
  t9140 = t9130 + t9139;
  t9289 = -1.*t1762*t711*t766;
  t9291 = t515*t766*t1490;
  t9297 = t9289 + t9291;
  t9299 = -1.*t323*t2193;
  t9300 = t111*t9297;
  t9301 = t9299 + t9300;
  t9303 = -1.*t111*t323;
  t9304 = -1.*t2193*t9297;
  t9305 = t9303 + t9304;
  t9327 = t711*t360;
  t9328 = -1.*t1467*t766*t1490;
  t9329 = t9327 + t9328;
  t9331 = -1.*t515*t4262;
  t9332 = t1762*t9329;
  t9333 = t9331 + t9332;
  t9356 = t711*t360*t766;
  t9357 = -1.*t1467*t1490;
  t9358 = t9356 + t9357;
  t1928 = t1762*t1867;
  t9361 = -1.*t515*t9358;
  t9362 = t9361 + t1928;
  t9388 = -1.*t323*t711*t515;
  t9389 = -1.*t1762*t323*t1490;
  t9391 = t9388 + t9389;
  t9414 = -1.*t1762*t4295;
  t9415 = t9331 + t9414;
  t9432 = t1467*t711;
  t9433 = t360*t766*t1490;
  t9434 = t9432 + t9433;
  t9442 = -1.*t1762*t9434;
  t9448 = t9361 + t9442;
  t9366 = t1762*t9358;
  t9392 = t2317*t9391;
  t9394 = t111*t2761*t9391;
  t9396 = -1.*t2193*t3213*t9391;
  t9397 = t1762*t323*t711;
  t9398 = -1.*t323*t515*t1490;
  t9399 = t9397 + t9398;
  t9400 = 0.13205*t9399;
  t9401 = -1.*t2500*t2193*t9391;
  t9402 = -1.*t111*t2747*t9391;
  t9403 = t9401 + t9402;
  t9404 = -0.183*t9403;
  t9405 = t111*t2500*t9391;
  t9406 = -1.*t2193*t2747*t9391;
  t9407 = t9405 + t9406;
  t9408 = -0.2*t9407;
  t9474 = -1.*t1467*t323*t2193;
  t9476 = -1.*t111*t4537;
  t9479 = t9474 + t9476;
  t5082 = t2500*t5037;
  t9455 = -1.*t515*t9434;
  t9456 = t9366 + t9455;
  t9494 = -1.*t111*t9456;
  t9495 = t2807 + t9494;
  t9497 = t111*t323*t360;
  t9498 = -1.*t2193*t9456;
  t9501 = t9497 + t9498;
  t9516 = t2193*t766;
  t9517 = -1.*t111*t9399;
  t9520 = t9516 + t9517;
  t9523 = -1.*t111*t766;
  t9524 = -1.*t2193*t9399;
  t9525 = t9523 + t9524;
  t5081 = -1.*t2747*t4963;
  t5172 = t5081 + t5082;
  t9488 = -1.*t2747*t5037;
  t9542 = -0.183*t2500;
  t9543 = -0.2*t2747;
  t9544 = t9542 + t9543;
  t9547 = 0.2*t2500;
  t9548 = t9547 + t2754;
  t9555 = t323*t2193*t360;
  t9556 = t111*t9456;
  t9557 = t9555 + t9556;
  t9504 = t2500*t9501;
  t9509 = -1.*t2747*t9501;
  t9568 = -1.*t2193*t766;
  t9569 = t111*t9399;
  t9570 = t9568 + t9569;
  t9529 = t2500*t9525;
  t9534 = -1.*t2747*t9525;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1818*t1867 + t2317*t2492 + t2761*t2836 + t3213*t3521 - 0.183*(-1.*t2747*t2836 + t2500*t3521) - 0.2*(t2500*t2836 + t2747*t3521) - 1.*t314*t323*t360 + 0.13205*(t1928 + t1617*t515) + t1617*t654;
  p_output1(10)=t1467*t314*t323 + t1818*t4295 + t2317*t4537 + t2761*t4963 + t3213*t5037 - 0.2*(t2500*t4963 + t2747*t5037) + 0.13205*(t1762*t4295 + t4262*t515) - 0.183*t5172 + t4262*t654;
  p_output1(11)=0;
  p_output1(12)=t1467*t1490*t1818*t323 + t2317*t6656 + t2761*t6690 + t1467*t323*t654*t711 + 0.13205*(t1467*t1490*t1762*t323 + t1467*t323*t515*t711) + t3213*t7284 - 0.183*(-1.*t2747*t6690 + t2500*t7284) - 0.2*(t2500*t6690 + t2747*t7284) - 1.*t1467*t314*t766;
  p_output1(13)=t1490*t1818*t323*t360 + t323*t360*t654*t711 + 0.13205*(t1490*t1762*t323*t360 + t323*t360*t515*t711) - 1.*t314*t360*t766 + t2317*t9117 + t2761*t9127 + t3213*t9140 - 0.183*(-1.*t2747*t9127 + t2500*t9140) - 0.2*(t2500*t9127 + t2747*t9140);
  p_output1(14)=-1.*t314*t323 - 1.*t1490*t1818*t766 - 1.*t654*t711*t766 + 0.13205*(-1.*t1490*t1762*t766 - 1.*t515*t711*t766) + t2317*t9297 + t2761*t9301 + t3213*t9305 - 0.183*(-1.*t2747*t9301 + t2500*t9305) - 0.2*(t2500*t9301 + t2747*t9305);
  p_output1(15)=t1818*t4262 + t654*t9329 + 0.13205*(t4445 + t515*t9329) + t2317*t9333 + t111*t2761*t9333 - 1.*t2193*t3213*t9333 - 0.183*(-1.*t2193*t2500*t9333 - 1.*t111*t2747*t9333) - 0.2*(t111*t2500*t9333 - 1.*t2193*t2747*t9333);
  p_output1(16)=t1867*t654 + t1818*t9358 + t2317*t9362 + t111*t2761*t9362 - 1.*t2193*t3213*t9362 - 0.183*(-1.*t2193*t2500*t9362 - 1.*t111*t2747*t9362) - 0.2*(t111*t2500*t9362 - 1.*t2193*t2747*t9362) + 0.13205*(t1867*t515 + t9366);
  p_output1(17)=-1.*t1490*t323*t654 + t1818*t323*t711 + t9392 + t9394 + t9396 + t9400 + t9404 + t9408;
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
  p_output1(45)=-0.047*t1762*t4262 + 0.13205*t4537 + 0.047*t4295*t515 + t2317*t9415 + t111*t2761*t9415 - 1.*t2193*t3213*t9415 - 0.183*(-1.*t2193*t2500*t9415 - 1.*t111*t2747*t9415) - 0.2*(t111*t2500*t9415 - 1.*t2193*t2747*t9415);
  p_output1(46)=-0.047*t1762*t9358 + 0.047*t515*t9434 + t2317*t9448 + t111*t2761*t9448 - 1.*t2193*t3213*t9448 - 0.183*(-1.*t2193*t2500*t9448 - 1.*t111*t2747*t9448) - 0.2*(t111*t2500*t9448 - 1.*t2193*t2747*t9448) + 0.13205*t9456;
  p_output1(47)=0.047*t1490*t323*t515 - 0.047*t1762*t323*t711 + t9392 + t9394 + t9396 + t9400 + t9404 + t9408;
  p_output1(48)=-0.183*t1467*t2193*t323 - 0.183*t111*t4537 + t2761*t5037 + t3213*t9479 - 0.2*(t5082 + t2747*t9479) - 0.183*(t2500*t9479 + t9488);
  p_output1(49)=-0.183*t2193*t323*t360 - 0.183*t111*t9456 + t3213*t9495 + t2761*t9501 - 0.2*(t2747*t9495 + t9504) - 0.183*(t2500*t9495 + t9509);
  p_output1(50)=0.183*t2193*t766 - 0.183*t111*t9399 + t3213*t9520 + t2761*t9525 - 0.2*(t2747*t9520 + t9529) - 0.183*(t2500*t9520 + t9534);
  p_output1(51)=-0.2*t5172 - 0.183*(-1.*t2500*t4963 + t9488) + t4963*t9544 + t5037*t9548;
  p_output1(52)=t9501*t9548 + t9544*t9557 - 0.183*(t9509 - 1.*t2500*t9557) - 0.2*(t9504 - 1.*t2747*t9557);
  p_output1(53)=t9525*t9548 + t9544*t9570 - 0.183*(t9534 - 1.*t2500*t9570) - 0.2*(t9529 - 1.*t2747*t9570);
}


       
Eigen::Matrix<double,3,18> Jp_RL_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void Jp_RL_calf_joint(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



