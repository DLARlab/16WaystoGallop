/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:40 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RLfoot.h"

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
static void output1(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t251;
  double t889;
  double t1087;
  double t1009;
  double t1256;
  double t743;
  double t252;
  double t655;
  double t771;
  double t1034;
  double t1304;
  double t1341;
  double t1342;
  double t1362;
  double t1406;
  double t1512;
  double t1635;
  double t1638;
  double t1640;
  double t170;
  double t1717;
  double t2454;
  double t2547;
  double t2592;
  double t2606;
  double t2652;
  double t2736;
  double t2773;
  double t2811;
  double t2983;
  double t3477;
  double t3485;
  double t3523;
  double t690;
  double t1649;
  double t1679;
  double t1806;
  double t1823;
  double t1826;
  double t2442;
  double t3035;
  double t3229;
  double t3336;
  double t3374;
  double t3392;
  double t3459;
  double t3527;
  double t3546;
  double t3642;
  double t3729;
  double t3730;
  double t3817;
  double t3818;
  double t3820;
  double t5466;
  double t5472;
  double t1680;
  double t1926;
  double t2227;
  double t4215;
  double t4261;
  double t4352;
  double t4749;
  double t4784;
  double t4792;
  double t4874;
  double t4938;
  double t4977;
  double t5001;
  double t5088;
  double t5142;
  double t5168;
  double t3832;
  double t3944;
  double t4023;
  double t5419;
  double t5430;
  double t5507;
  double t5538;
  double t5539;
  double t5610;
  double t5646;
  double t5671;
  double t3330;
  double t3398;
  double t3399;
  double t4422;
  double t4426;
  double t4437;
  double t4087;
  double t4101;
  double t4113;
  double t3566;
  double t3767;
  double t3816;
  double t4448;
  double t4450;
  double t4683;
  t251 = Cos(var1[3]);
  t889 = Cos(var1[5]);
  t1087 = Sin(var1[3]);
  t1009 = Sin(var1[4]);
  t1256 = Sin(var1[5]);
  t743 = Cos(var1[16]);
  t252 = Cos(var1[4]);
  t655 = Sin(var1[16]);
  t771 = Cos(var1[15]);
  t1034 = t251*t889*t1009;
  t1304 = t1087*t1256;
  t1341 = t1034 + t1304;
  t1342 = t771*t1341;
  t1362 = Sin(var1[15]);
  t1406 = -1.*t889*t1087;
  t1512 = t251*t1009*t1256;
  t1635 = t1406 + t1512;
  t1638 = -1.*t1362*t1635;
  t1640 = t1342 + t1638;
  t170 = Sin(var1[17]);
  t1717 = Cos(var1[17]);
  t2454 = t889*t1087*t1009;
  t2547 = -1.*t251*t1256;
  t2592 = t2454 + t2547;
  t2606 = t771*t2592;
  t2652 = t251*t889;
  t2736 = t1087*t1009*t1256;
  t2773 = t2652 + t2736;
  t2811 = -1.*t1362*t2773;
  t2983 = t2606 + t2811;
  t3477 = t771*t252*t889;
  t3485 = -1.*t252*t1362*t1256;
  t3523 = t3477 + t3485;
  t690 = t251*t252*t655;
  t1649 = t743*t1640;
  t1679 = t690 + t1649;
  t1806 = t743*t251*t252;
  t1823 = -1.*t655*t1640;
  t1826 = t1806 + t1823;
  t2442 = t252*t655*t1087;
  t3035 = t743*t2983;
  t3229 = t2442 + t3035;
  t3336 = t743*t252*t1087;
  t3374 = -1.*t655*t2983;
  t3392 = t3336 + t3374;
  t3459 = -1.*t655*t1009;
  t3527 = t743*t3523;
  t3546 = t3459 + t3527;
  t3642 = -1.*t743*t1009;
  t3729 = -1.*t655*t3523;
  t3730 = t3642 + t3729;
  t3817 = t1362*t1341;
  t3818 = t771*t1635;
  t3820 = t3817 + t3818;
  t5466 = -1.*t1717;
  t5472 = 1. + t5466;
  t1680 = -1.*t170*t1679;
  t1926 = t1717*t1826;
  t2227 = t1680 + t1926;
  t4215 = t1717*t1679;
  t4261 = t170*t1826;
  t4352 = t4215 + t4261;
  t4749 = -1.*t743;
  t4784 = 1. + t4749;
  t4792 = -0.183*t4784;
  t4874 = 0. + t4792;
  t4938 = -0.047*t1362;
  t4977 = 0. + t4938;
  t5001 = -1.*t771;
  t5088 = 1. + t5001;
  t5142 = 0.047*t5088;
  t5168 = 0. + t5142;
  t3832 = t1362*t2592;
  t3944 = t771*t2773;
  t4023 = t3832 + t3944;
  t5419 = -0.183*t655;
  t5430 = 0. + t5419;
  t5507 = -0.2*t5472;
  t5538 = -0.183*t170;
  t5539 = 0. + t5507 + t5538;
  t5610 = -0.183*t5472;
  t5646 = 0.2*t170;
  t5671 = 0. + t5610 + t5646;
  t3330 = -1.*t170*t3229;
  t3398 = t1717*t3392;
  t3399 = t3330 + t3398;
  t4422 = t1717*t3229;
  t4426 = t170*t3392;
  t4437 = t4422 + t4426;
  t4087 = t252*t889*t1362;
  t4101 = t771*t252*t1256;
  t4113 = t4087 + t4101;
  t3566 = -1.*t170*t3546;
  t3767 = t1717*t3730;
  t3816 = t3566 + t3767;
  t4448 = t1717*t3546;
  t4450 = t170*t3730;
  t4683 = t4448 + t4450;

  p_output1(0)=t2227;
  p_output1(1)=t3399;
  p_output1(2)=t3816;
  p_output1(3)=0.;
  p_output1(4)=t3820;
  p_output1(5)=t4023;
  p_output1(6)=t4113;
  p_output1(7)=0.;
  p_output1(8)=t4352;
  p_output1(9)=t4437;
  p_output1(10)=t4683;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.183*t2227 + 0.13205*t3820 - 0.4*t4352 + t251*t252*t4874 + t1341*t4977 + t1635*t5168 + t1640*t5430 + t1679*t5539 + t1826*t5671 + var1(0);
  p_output1(13)=0. - 0.183*t3399 + 0.13205*t4023 - 0.4*t4437 + t1087*t252*t4874 + t2592*t4977 + t2773*t5168 + t2983*t5430 + t3229*t5539 + t3392*t5671 + var1(1);
  p_output1(14)=0. - 0.183*t3816 + 0.13205*t4113 - 0.4*t4683 - 1.*t1009*t4874 + t1256*t252*t5168 + t3523*t5430 + t3546*t5539 + t3730*t5671 + t252*t4977*t889 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_RLfoot(const Eigen::Matrix<double,18,1> &var1)
//void H_RLfoot(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



