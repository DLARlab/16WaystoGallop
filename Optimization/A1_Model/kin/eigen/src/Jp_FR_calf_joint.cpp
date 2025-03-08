/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:50 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FR_calf_joint.h"

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
  double t605;
  double t644;
  double t646;
  double t643;
  double t650;
  double t625;
  double t645;
  double t651;
  double t655;
  double t658;
  double t659;
  double t660;
  double t662;
  double t309;
  double t1001;
  double t1025;
  double t1125;
  double t65;
  double t1154;
  double t1872;
  double t2509;
  double t3191;
  double t3505;
  double t4462;
  double t4743;
  double t4764;
  double t1498;
  double t1538;
  double t1641;
  double t326;
  double t339;
  double t455;
  double t604;
  double t629;
  double t639;
  double t640;
  double t642;
  double t663;
  double t667;
  double t8058;
  double t8061;
  double t8063;
  double t8085;
  double t8086;
  double t8088;
  double t1379;
  double t1383;
  double t8109;
  double t8110;
  double t8113;
  double t3441;
  double t3578;
  double t4293;
  double t4862;
  double t5199;
  double t5633;
  double t8125;
  double t8129;
  double t8133;
  double t8115;
  double t8117;
  double t8118;
  double t8165;
  double t8166;
  double t8167;
  double t8173;
  double t8174;
  double t8175;
  double t8169;
  double t8170;
  double t8171;
  double t8200;
  double t8201;
  double t8202;
  double t8215;
  double t8216;
  double t8217;
  double t8204;
  double t8207;
  double t8208;
  double t8239;
  double t8243;
  double t8244;
  double t8250;
  double t8251;
  double t8252;
  double t8246;
  double t8247;
  double t8248;
  double t8267;
  double t8268;
  double t8269;
  double t8274;
  double t8275;
  double t8276;
  double t8293;
  double t8294;
  double t8295;
  double t669;
  double t8313;
  double t8315;
  double t8338;
  double t8339;
  double t8341;
  double t8361;
  double t8362;
  double t8298;
  double t8379;
  double t8382;
  double t8384;
  double t8390;
  double t8392;
  double t8334;
  double t8335;
  double t8336;
  double t8337;
  double t8344;
  double t8347;
  double t8348;
  double t8349;
  double t8350;
  double t8351;
  double t8352;
  double t8353;
  double t8354;
  double t8355;
  double t8356;
  double t8136;
  double t8417;
  double t8418;
  double t8419;
  double t8387;
  double t8388;
  double t8432;
  double t8433;
  double t8434;
  double t8436;
  double t8437;
  double t8452;
  double t8453;
  double t8454;
  double t8456;
  double t8457;
  double t8458;
  double t8137;
  double t8138;
  double t8425;
  double t8469;
  double t8470;
  double t8471;
  double t8473;
  double t8474;
  double t8441;
  double t8481;
  double t8482;
  double t8483;
  double t8446;
  double t8460;
  double t8493;
  double t8494;
  double t8495;
  double t8465;
  t605 = Sin(var1[3]);
  t644 = Cos(var1[5]);
  t646 = Sin(var1[4]);
  t643 = Cos(var1[3]);
  t650 = Sin(var1[5]);
  t625 = Cos(var1[6]);
  t645 = -1.*t643*t644;
  t651 = -1.*t605*t646*t650;
  t655 = t645 + t651;
  t658 = -1.*t644*t605*t646;
  t659 = t643*t650;
  t660 = t658 + t659;
  t662 = Sin(var1[6]);
  t309 = Cos(var1[7]);
  t1001 = t625*t660;
  t1025 = -1.*t655*t662;
  t1125 = t1001 + t1025;
  t65 = Cos(var1[4]);
  t1154 = Sin(var1[7]);
  t1872 = Cos(var1[8]);
  t2509 = -1.*t1872;
  t3191 = 1. + t2509;
  t3505 = Sin(var1[8]);
  t4462 = -1.*t65*t309*t605;
  t4743 = -1.*t1125*t1154;
  t4764 = t4462 + t4743;
  t1498 = t309*t1125;
  t1538 = -1.*t65*t605*t1154;
  t1641 = t1498 + t1538;
  t326 = -1.*t309;
  t339 = 1. + t326;
  t455 = 0.183*t339;
  t604 = 0. + t455;
  t629 = -1.*t625;
  t639 = 1. + t629;
  t640 = -0.047*t639;
  t642 = 0. + t640;
  t663 = 0.047*t662;
  t667 = 0. + t663;
  t8058 = -1.*t644*t605;
  t8061 = t643*t646*t650;
  t8063 = t8058 + t8061;
  t8085 = t643*t644*t646;
  t8086 = t605*t650;
  t8088 = t8085 + t8086;
  t1379 = 0.183*t1154;
  t1383 = 0. + t1379;
  t8109 = t625*t8088;
  t8110 = -1.*t8063*t662;
  t8113 = t8109 + t8110;
  t3441 = -0.2*t3191;
  t3578 = 0.183*t3505;
  t4293 = 0. + t3441 + t3578;
  t4862 = 0.183*t3191;
  t5199 = 0.2*t3505;
  t5633 = 0. + t4862 + t5199;
  t8125 = t643*t65*t309;
  t8129 = -1.*t8113*t1154;
  t8133 = t8125 + t8129;
  t8115 = t309*t8113;
  t8117 = t643*t65*t1154;
  t8118 = t8115 + t8117;
  t8165 = t643*t65*t644*t625;
  t8166 = -1.*t643*t65*t650*t662;
  t8167 = t8165 + t8166;
  t8173 = -1.*t643*t309*t646;
  t8174 = -1.*t8167*t1154;
  t8175 = t8173 + t8174;
  t8169 = t309*t8167;
  t8170 = -1.*t643*t646*t1154;
  t8171 = t8169 + t8170;
  t8200 = t65*t644*t625*t605;
  t8201 = -1.*t65*t605*t650*t662;
  t8202 = t8200 + t8201;
  t8215 = -1.*t309*t605*t646;
  t8216 = -1.*t8202*t1154;
  t8217 = t8215 + t8216;
  t8204 = t309*t8202;
  t8207 = -1.*t605*t646*t1154;
  t8208 = t8204 + t8207;
  t8239 = -1.*t644*t625*t646;
  t8243 = t646*t650*t662;
  t8244 = t8239 + t8243;
  t8250 = -1.*t65*t309;
  t8251 = -1.*t8244*t1154;
  t8252 = t8250 + t8251;
  t8246 = t309*t8244;
  t8247 = -1.*t65*t1154;
  t8248 = t8246 + t8247;
  t8267 = t644*t605;
  t8268 = -1.*t643*t646*t650;
  t8269 = t8267 + t8268;
  t8274 = t625*t8269;
  t8275 = -1.*t8088*t662;
  t8276 = t8274 + t8275;
  t8293 = t644*t605*t646;
  t8294 = -1.*t643*t650;
  t8295 = t8293 + t8294;
  t669 = t625*t655;
  t8313 = -1.*t8295*t662;
  t8315 = t669 + t8313;
  t8338 = -1.*t65*t625*t650;
  t8339 = -1.*t65*t644*t662;
  t8341 = t8338 + t8339;
  t8361 = -1.*t625*t8063;
  t8362 = t8361 + t8275;
  t8298 = t625*t8295;
  t8379 = t643*t644;
  t8382 = t605*t646*t650;
  t8384 = t8379 + t8382;
  t8390 = -1.*t625*t8384;
  t8392 = t8390 + t8313;
  t8334 = t65*t644*t625;
  t8335 = -1.*t65*t650*t662;
  t8336 = t8334 + t8335;
  t8337 = -0.13205*t8336;
  t8344 = t8341*t1383;
  t8347 = t309*t8341*t4293;
  t8348 = -1.*t8341*t1154*t5633;
  t8349 = -1.*t1872*t8341*t1154;
  t8350 = -1.*t309*t8341*t3505;
  t8351 = t8349 + t8350;
  t8352 = 0.183*t8351;
  t8353 = t309*t1872*t8341;
  t8354 = -1.*t8341*t1154*t3505;
  t8355 = t8353 + t8354;
  t8356 = -0.2*t8355;
  t8136 = t1872*t8133;
  t8417 = -1.*t309*t8113;
  t8418 = -1.*t643*t65*t1154;
  t8419 = t8417 + t8418;
  t8387 = -1.*t8384*t662;
  t8388 = t8298 + t8387;
  t8432 = t65*t309*t605;
  t8433 = -1.*t8388*t1154;
  t8434 = t8432 + t8433;
  t8436 = -1.*t309*t8388;
  t8437 = t8436 + t1538;
  t8452 = -1.*t309*t646;
  t8453 = -1.*t8336*t1154;
  t8454 = t8452 + t8453;
  t8456 = -1.*t309*t8336;
  t8457 = t646*t1154;
  t8458 = t8456 + t8457;
  t8137 = -1.*t8118*t3505;
  t8138 = t8136 + t8137;
  t8425 = -1.*t8133*t3505;
  t8469 = 0.183*t1872;
  t8470 = -0.2*t3505;
  t8471 = t8469 + t8470;
  t8473 = 0.2*t1872;
  t8474 = t8473 + t3578;
  t8441 = t1872*t8434;
  t8481 = t309*t8388;
  t8482 = t65*t605*t1154;
  t8483 = t8481 + t8482;
  t8446 = -1.*t8434*t3505;
  t8460 = t1872*t8454;
  t8493 = t309*t8336;
  t8494 = -1.*t646*t1154;
  t8495 = t8493 + t8494;
  t8465 = -1.*t8454*t3505;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1125*t1383 + t1641*t4293 + 0.183*(-1.*t1641*t3505 + t1872*t4764) - 0.2*(t1641*t1872 + t3505*t4764) + t4764*t5633 - 1.*t604*t605*t65 + t642*t655 + t660*t667 - 0.13205*(t660*t662 + t669);
  p_output1(10)=t604*t643*t65 + t642*t8063 + t667*t8088 - 0.13205*(t625*t8063 + t662*t8088) + t1383*t8113 + t4293*t8118 + t5633*t8133 - 0.2*(t1872*t8118 + t3505*t8133) + 0.183*t8138;
  p_output1(11)=0;
  p_output1(12)=-1.*t604*t643*t646 + t642*t643*t65*t650 - 0.13205*(t625*t643*t65*t650 + t643*t644*t65*t662) + t643*t644*t65*t667 + t1383*t8167 + t4293*t8171 + t5633*t8175 + 0.183*(-1.*t3505*t8171 + t1872*t8175) - 0.2*(t1872*t8171 + t3505*t8175);
  p_output1(13)=-1.*t604*t605*t646 + t605*t642*t65*t650 - 0.13205*(t605*t625*t65*t650 + t605*t644*t65*t662) + t605*t644*t65*t667 + t1383*t8202 + t4293*t8208 + t5633*t8217 + 0.183*(-1.*t3505*t8208 + t1872*t8217) - 0.2*(t1872*t8208 + t3505*t8217);
  p_output1(14)=-1.*t604*t65 - 1.*t642*t646*t650 - 0.13205*(-1.*t625*t646*t650 - 1.*t644*t646*t662) - 1.*t644*t646*t667 + t1383*t8244 + t4293*t8248 + t5633*t8252 + 0.183*(-1.*t3505*t8248 + t1872*t8252) - 0.2*(t1872*t8248 + t3505*t8252);
  p_output1(15)=t642*t8088 + t667*t8269 - 0.13205*(t8109 + t662*t8269) + t1383*t8276 + t309*t4293*t8276 - 1.*t1154*t5633*t8276 - 0.2*(t1872*t309*t8276 - 1.*t1154*t3505*t8276) + 0.183*(-1.*t1154*t1872*t8276 - 1.*t309*t3505*t8276);
  p_output1(16)=t655*t667 + t642*t8295 - 0.13205*(t655*t662 + t8298) + t1383*t8315 + t309*t4293*t8315 - 1.*t1154*t5633*t8315 - 0.2*(t1872*t309*t8315 - 1.*t1154*t3505*t8315) + 0.183*(-1.*t1154*t1872*t8315 - 1.*t309*t3505*t8315);
  p_output1(17)=t642*t644*t65 - 1.*t65*t650*t667 + t8337 + t8344 + t8347 + t8348 + t8352 + t8356;
  p_output1(18)=-0.047*t662*t8063 + 0.047*t625*t8088 - 0.13205*t8113 + t1383*t8362 + t309*t4293*t8362 - 1.*t1154*t5633*t8362 - 0.2*(t1872*t309*t8362 - 1.*t1154*t3505*t8362) + 0.183*(-1.*t1154*t1872*t8362 - 1.*t309*t3505*t8362);
  p_output1(19)=0.047*t625*t8295 - 0.047*t662*t8384 - 0.13205*t8388 + t1383*t8392 + t309*t4293*t8392 - 1.*t1154*t5633*t8392 - 0.2*(t1872*t309*t8392 - 1.*t1154*t3505*t8392) + 0.183*(-1.*t1154*t1872*t8392 - 1.*t309*t3505*t8392);
  p_output1(20)=0.047*t625*t644*t65 - 0.047*t65*t650*t662 + t8337 + t8344 + t8347 + t8348 + t8352 + t8356;
  p_output1(21)=0.183*t1154*t643*t65 + 0.183*t309*t8113 + t4293*t8133 + t5633*t8419 - 0.2*(t8136 + t3505*t8419) + 0.183*(t1872*t8419 + t8425);
  p_output1(22)=0.183*t1154*t605*t65 + 0.183*t309*t8388 + t4293*t8434 + t5633*t8437 - 0.2*(t3505*t8437 + t8441) + 0.183*(t1872*t8437 + t8446);
  p_output1(23)=-0.183*t1154*t646 + 0.183*t309*t8336 + t4293*t8454 + t5633*t8458 - 0.2*(t3505*t8458 + t8460) + 0.183*(t1872*t8458 + t8465);
  p_output1(24)=-0.2*t8138 + 0.183*(-1.*t1872*t8118 + t8425) + t8118*t8471 + t8133*t8474;
  p_output1(25)=t8434*t8474 + t8471*t8483 + 0.183*(t8446 - 1.*t1872*t8483) - 0.2*(t8441 - 1.*t3505*t8483);
  p_output1(26)=t8454*t8474 + t8471*t8495 + 0.183*(t8465 - 1.*t1872*t8495) - 0.2*(t8460 - 1.*t3505*t8495);
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


       
Eigen::Matrix<double,3,18> Jp_FR_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void Jp_FR_calf_joint(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



