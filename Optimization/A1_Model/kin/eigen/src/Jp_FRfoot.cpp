/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:28 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FRfoot.h"

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
  double t676;
  double t855;
  double t1099;
  double t842;
  double t1148;
  double t730;
  double t862;
  double t1264;
  double t1570;
  double t1723;
  double t1843;
  double t2230;
  double t2264;
  double t101;
  double t2877;
  double t2894;
  double t3278;
  double t38;
  double t3511;
  double t3701;
  double t3738;
  double t3757;
  double t3864;
  double t4259;
  double t4267;
  double t4268;
  double t3668;
  double t3672;
  double t3688;
  double t145;
  double t219;
  double t345;
  double t347;
  double t742;
  double t779;
  double t783;
  double t788;
  double t2324;
  double t2426;
  double t4341;
  double t4787;
  double t4844;
  double t4902;
  double t4930;
  double t4962;
  double t3515;
  double t3627;
  double t5309;
  double t5317;
  double t5324;
  double t3792;
  double t4202;
  double t4233;
  double t4269;
  double t4270;
  double t4271;
  double t5332;
  double t5333;
  double t5334;
  double t5327;
  double t5328;
  double t5329;
  double t5363;
  double t5364;
  double t5365;
  double t5376;
  double t5378;
  double t5380;
  double t5370;
  double t5371;
  double t5374;
  double t5761;
  double t5768;
  double t5776;
  double t5887;
  double t6124;
  double t6133;
  double t5811;
  double t5843;
  double t5846;
  double t7215;
  double t7268;
  double t7300;
  double t7456;
  double t7463;
  double t7468;
  double t7316;
  double t7396;
  double t7429;
  double t9006;
  double t9050;
  double t9072;
  double t9125;
  double t9184;
  double t9186;
  double t9324;
  double t9325;
  double t9329;
  double t2443;
  double t9344;
  double t9346;
  double t9386;
  double t9387;
  double t9397;
  double t9435;
  double t9450;
  double t9337;
  double t9471;
  double t9479;
  double t9480;
  double t9487;
  double t9489;
  double t9382;
  double t9383;
  double t9384;
  double t9385;
  double t9403;
  double t9405;
  double t9407;
  double t9409;
  double t9410;
  double t9411;
  double t9418;
  double t9419;
  double t9420;
  double t9422;
  double t9424;
  double t5336;
  double t9530;
  double t9531;
  double t9532;
  double t9484;
  double t9485;
  double t9567;
  double t9572;
  double t9574;
  double t9577;
  double t9578;
  double t9596;
  double t9597;
  double t9598;
  double t9600;
  double t9601;
  double t9602;
  double t5338;
  double t5339;
  double t9556;
  double t9613;
  double t9616;
  double t9617;
  double t9619;
  double t9620;
  double t9580;
  double t9628;
  double t9631;
  double t9633;
  double t9585;
  double t9604;
  double t9645;
  double t9649;
  double t9651;
  double t9609;
  t676 = Sin(var1[3]);
  t855 = Cos(var1[5]);
  t1099 = Sin(var1[4]);
  t842 = Cos(var1[3]);
  t1148 = Sin(var1[5]);
  t730 = Cos(var1[6]);
  t862 = -1.*t842*t855;
  t1264 = -1.*t676*t1099*t1148;
  t1570 = t862 + t1264;
  t1723 = -1.*t855*t676*t1099;
  t1843 = t842*t1148;
  t2230 = t1723 + t1843;
  t2264 = Sin(var1[6]);
  t101 = Cos(var1[7]);
  t2877 = t730*t2230;
  t2894 = -1.*t1570*t2264;
  t3278 = t2877 + t2894;
  t38 = Cos(var1[4]);
  t3511 = Sin(var1[7]);
  t3701 = Cos(var1[8]);
  t3738 = -1.*t3701;
  t3757 = 1. + t3738;
  t3864 = Sin(var1[8]);
  t4259 = -1.*t38*t101*t676;
  t4267 = -1.*t3278*t3511;
  t4268 = t4259 + t4267;
  t3668 = t101*t3278;
  t3672 = -1.*t38*t676*t3511;
  t3688 = t3668 + t3672;
  t145 = -1.*t101;
  t219 = 1. + t145;
  t345 = 0.183*t219;
  t347 = 0. + t345;
  t742 = -1.*t730;
  t779 = 1. + t742;
  t783 = -0.047*t779;
  t788 = 0. + t783;
  t2324 = 0.047*t2264;
  t2426 = 0. + t2324;
  t4341 = -1.*t855*t676;
  t4787 = t842*t1099*t1148;
  t4844 = t4341 + t4787;
  t4902 = t842*t855*t1099;
  t4930 = t676*t1148;
  t4962 = t4902 + t4930;
  t3515 = 0.183*t3511;
  t3627 = 0. + t3515;
  t5309 = t730*t4962;
  t5317 = -1.*t4844*t2264;
  t5324 = t5309 + t5317;
  t3792 = -0.2*t3757;
  t4202 = 0.183*t3864;
  t4233 = 0. + t3792 + t4202;
  t4269 = 0.183*t3757;
  t4270 = 0.2*t3864;
  t4271 = 0. + t4269 + t4270;
  t5332 = t842*t38*t101;
  t5333 = -1.*t5324*t3511;
  t5334 = t5332 + t5333;
  t5327 = t101*t5324;
  t5328 = t842*t38*t3511;
  t5329 = t5327 + t5328;
  t5363 = t842*t38*t855*t730;
  t5364 = -1.*t842*t38*t1148*t2264;
  t5365 = t5363 + t5364;
  t5376 = -1.*t842*t101*t1099;
  t5378 = -1.*t5365*t3511;
  t5380 = t5376 + t5378;
  t5370 = t101*t5365;
  t5371 = -1.*t842*t1099*t3511;
  t5374 = t5370 + t5371;
  t5761 = t38*t855*t730*t676;
  t5768 = -1.*t38*t676*t1148*t2264;
  t5776 = t5761 + t5768;
  t5887 = -1.*t101*t676*t1099;
  t6124 = -1.*t5776*t3511;
  t6133 = t5887 + t6124;
  t5811 = t101*t5776;
  t5843 = -1.*t676*t1099*t3511;
  t5846 = t5811 + t5843;
  t7215 = -1.*t855*t730*t1099;
  t7268 = t1099*t1148*t2264;
  t7300 = t7215 + t7268;
  t7456 = -1.*t38*t101;
  t7463 = -1.*t7300*t3511;
  t7468 = t7456 + t7463;
  t7316 = t101*t7300;
  t7396 = -1.*t38*t3511;
  t7429 = t7316 + t7396;
  t9006 = t855*t676;
  t9050 = -1.*t842*t1099*t1148;
  t9072 = t9006 + t9050;
  t9125 = t730*t9072;
  t9184 = -1.*t4962*t2264;
  t9186 = t9125 + t9184;
  t9324 = t855*t676*t1099;
  t9325 = -1.*t842*t1148;
  t9329 = t9324 + t9325;
  t2443 = t730*t1570;
  t9344 = -1.*t9329*t2264;
  t9346 = t2443 + t9344;
  t9386 = -1.*t38*t730*t1148;
  t9387 = -1.*t38*t855*t2264;
  t9397 = t9386 + t9387;
  t9435 = -1.*t730*t4844;
  t9450 = t9435 + t9184;
  t9337 = t730*t9329;
  t9471 = t842*t855;
  t9479 = t676*t1099*t1148;
  t9480 = t9471 + t9479;
  t9487 = -1.*t730*t9480;
  t9489 = t9487 + t9344;
  t9382 = t38*t855*t730;
  t9383 = -1.*t38*t1148*t2264;
  t9384 = t9382 + t9383;
  t9385 = -0.13205*t9384;
  t9403 = t9397*t3627;
  t9405 = t101*t9397*t4233;
  t9407 = -1.*t9397*t3511*t4271;
  t9409 = -1.*t3701*t9397*t3511;
  t9410 = -1.*t101*t9397*t3864;
  t9411 = t9409 + t9410;
  t9418 = 0.183*t9411;
  t9419 = t101*t3701*t9397;
  t9420 = -1.*t9397*t3511*t3864;
  t9422 = t9419 + t9420;
  t9424 = -0.4*t9422;
  t5336 = t3701*t5334;
  t9530 = -1.*t101*t5324;
  t9531 = -1.*t842*t38*t3511;
  t9532 = t9530 + t9531;
  t9484 = -1.*t9480*t2264;
  t9485 = t9337 + t9484;
  t9567 = t38*t101*t676;
  t9572 = -1.*t9485*t3511;
  t9574 = t9567 + t9572;
  t9577 = -1.*t101*t9485;
  t9578 = t9577 + t3672;
  t9596 = -1.*t101*t1099;
  t9597 = -1.*t9384*t3511;
  t9598 = t9596 + t9597;
  t9600 = -1.*t101*t9384;
  t9601 = t1099*t3511;
  t9602 = t9600 + t9601;
  t5338 = -1.*t5329*t3864;
  t5339 = t5336 + t5338;
  t9556 = -1.*t5334*t3864;
  t9613 = 0.183*t3701;
  t9616 = -0.2*t3864;
  t9617 = t9613 + t9616;
  t9619 = 0.2*t3701;
  t9620 = t9619 + t4202;
  t9580 = t3701*t9574;
  t9628 = t101*t9485;
  t9631 = t38*t676*t3511;
  t9633 = t9628 + t9631;
  t9585 = -1.*t9574*t3864;
  t9604 = t3701*t9598;
  t9645 = t101*t9384;
  t9649 = -1.*t1099*t3511;
  t9651 = t9645 + t9649;
  t9609 = -1.*t9598*t3864;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t2230*t2426 - 0.13205*(t2230*t2264 + t2443) + t3278*t3627 + t3688*t4233 + 0.183*(-1.*t3688*t3864 + t3701*t4268) - 0.4*(t3688*t3701 + t3864*t4268) + t4268*t4271 - 1.*t347*t38*t676 + t1570*t788;
  p_output1(10)=t2426*t4962 + t3627*t5324 + t4233*t5329 + t4271*t5334 - 0.4*(t3701*t5329 + t3864*t5334) + 0.183*t5339 - 0.13205*(t2264*t4962 + t4844*t730) + t4844*t788 + t347*t38*t842;
  p_output1(11)=0;
  p_output1(12)=t3627*t5365 + t4233*t5374 + t4271*t5380 + 0.183*(-1.*t3864*t5374 + t3701*t5380) - 0.4*(t3701*t5374 + t3864*t5380) - 1.*t1099*t347*t842 + t1148*t38*t788*t842 + t2426*t38*t842*t855 - 0.13205*(t1148*t38*t730*t842 + t2264*t38*t842*t855);
  p_output1(13)=t3627*t5776 + t4233*t5846 + t4271*t6133 + 0.183*(-1.*t3864*t5846 + t3701*t6133) - 0.4*(t3701*t5846 + t3864*t6133) - 1.*t1099*t347*t676 + t1148*t38*t676*t788 + t2426*t38*t676*t855 - 0.13205*(t1148*t38*t676*t730 + t2264*t38*t676*t855);
  p_output1(14)=-1.*t347*t38 + t3627*t7300 + t4233*t7429 + t4271*t7468 + 0.183*(-1.*t3864*t7429 + t3701*t7468) - 0.4*(t3701*t7429 + t3864*t7468) - 1.*t1099*t1148*t788 - 1.*t1099*t2426*t855 - 0.13205*(-1.*t1099*t1148*t730 - 1.*t1099*t2264*t855);
  p_output1(15)=t4962*t788 + t2426*t9072 - 0.13205*(t5309 + t2264*t9072) + t3627*t9186 + t101*t4233*t9186 - 1.*t3511*t4271*t9186 + 0.183*(-1.*t3511*t3701*t9186 - 1.*t101*t3864*t9186) - 0.4*(t101*t3701*t9186 - 1.*t3511*t3864*t9186);
  p_output1(16)=t1570*t2426 + t788*t9329 - 0.13205*(t1570*t2264 + t9337) + t3627*t9346 + t101*t4233*t9346 - 1.*t3511*t4271*t9346 + 0.183*(-1.*t3511*t3701*t9346 - 1.*t101*t3864*t9346) - 0.4*(t101*t3701*t9346 - 1.*t3511*t3864*t9346);
  p_output1(17)=-1.*t1148*t2426*t38 + t38*t788*t855 + t9385 + t9403 + t9405 + t9407 + t9418 + t9424;
  p_output1(18)=-0.047*t2264*t4844 - 0.13205*t5324 + 0.047*t4962*t730 + t3627*t9450 + t101*t4233*t9450 - 1.*t3511*t4271*t9450 + 0.183*(-1.*t3511*t3701*t9450 - 1.*t101*t3864*t9450) - 0.4*(t101*t3701*t9450 - 1.*t3511*t3864*t9450);
  p_output1(19)=0.047*t730*t9329 - 0.047*t2264*t9480 - 0.13205*t9485 + t3627*t9489 + t101*t4233*t9489 - 1.*t3511*t4271*t9489 + 0.183*(-1.*t3511*t3701*t9489 - 1.*t101*t3864*t9489) - 0.4*(t101*t3701*t9489 - 1.*t3511*t3864*t9489);
  p_output1(20)=-0.047*t1148*t2264*t38 + 0.047*t38*t730*t855 + t9385 + t9403 + t9405 + t9407 + t9418 + t9424;
  p_output1(21)=0.183*t101*t5324 + t4233*t5334 + 0.183*t3511*t38*t842 + t4271*t9532 - 0.4*(t5336 + t3864*t9532) + 0.183*(t3701*t9532 + t9556);
  p_output1(22)=0.183*t3511*t38*t676 + 0.183*t101*t9485 + t4233*t9574 + t4271*t9578 - 0.4*(t3864*t9578 + t9580) + 0.183*(t3701*t9578 + t9585);
  p_output1(23)=-0.183*t1099*t3511 + 0.183*t101*t9384 + t4233*t9598 + t4271*t9602 - 0.4*(t3864*t9602 + t9604) + 0.183*(t3701*t9602 + t9609);
  p_output1(24)=-0.4*t5339 + 0.183*(-1.*t3701*t5329 + t9556) + t5329*t9617 + t5334*t9620;
  p_output1(25)=t9574*t9620 + t9617*t9633 + 0.183*(t9585 - 1.*t3701*t9633) - 0.4*(t9580 - 1.*t3864*t9633);
  p_output1(26)=t9598*t9620 + t9617*t9651 + 0.183*(t9609 - 1.*t3701*t9651) - 0.4*(t9604 - 1.*t3864*t9651);
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


       
Eigen::Matrix<double,3,18> Jp_FRfoot(const Eigen::Matrix<double,18,1> &var1)
//void Jp_FRfoot(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



