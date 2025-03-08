/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:36 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RRfoot.h"

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
  double t408;
  double t1031;
  double t1567;
  double t1094;
  double t1633;
  double t861;
  double t459;
  double t557;
  double t1019;
  double t1367;
  double t1648;
  double t1692;
  double t1702;
  double t1711;
  double t1725;
  double t1763;
  double t1792;
  double t1808;
  double t1809;
  double t399;
  double t2750;
  double t3412;
  double t3419;
  double t3463;
  double t3509;
  double t3510;
  double t3524;
  double t3561;
  double t3626;
  double t3655;
  double t3834;
  double t3855;
  double t3856;
  double t635;
  double t1873;
  double t1882;
  double t2828;
  double t2830;
  double t2854;
  double t3314;
  double t3678;
  double t3683;
  double t3756;
  double t3777;
  double t3793;
  double t3830;
  double t3890;
  double t4117;
  double t4160;
  double t4222;
  double t4234;
  double t4346;
  double t4493;
  double t4536;
  double t5002;
  double t5008;
  double t2387;
  double t2866;
  double t2907;
  double t4660;
  double t4661;
  double t4662;
  double t4669;
  double t4673;
  double t4674;
  double t4675;
  double t4677;
  double t4678;
  double t4680;
  double t4681;
  double t4685;
  double t4686;
  double t4550;
  double t4607;
  double t4639;
  double t4759;
  double t4804;
  double t5086;
  double t5093;
  double t5116;
  double t5173;
  double t5218;
  double t5303;
  double t3749;
  double t3794;
  double t3800;
  double t4663;
  double t4664;
  double t4665;
  double t4640;
  double t4652;
  double t4653;
  double t4150;
  double t4327;
  double t4330;
  double t4666;
  double t4667;
  double t4668;
  t408 = Cos(var1[3]);
  t1031 = Cos(var1[5]);
  t1567 = Sin(var1[3]);
  t1094 = Sin(var1[4]);
  t1633 = Sin(var1[5]);
  t861 = Cos(var1[13]);
  t459 = Cos(var1[4]);
  t557 = Sin(var1[13]);
  t1019 = Cos(var1[12]);
  t1367 = t408*t1031*t1094;
  t1648 = t1567*t1633;
  t1692 = t1367 + t1648;
  t1702 = t1019*t1692;
  t1711 = Sin(var1[12]);
  t1725 = -1.*t1031*t1567;
  t1763 = t408*t1094*t1633;
  t1792 = t1725 + t1763;
  t1808 = -1.*t1711*t1792;
  t1809 = t1702 + t1808;
  t399 = Sin(var1[14]);
  t2750 = Cos(var1[14]);
  t3412 = t1031*t1567*t1094;
  t3419 = -1.*t408*t1633;
  t3463 = t3412 + t3419;
  t3509 = t1019*t3463;
  t3510 = t408*t1031;
  t3524 = t1567*t1094*t1633;
  t3561 = t3510 + t3524;
  t3626 = -1.*t1711*t3561;
  t3655 = t3509 + t3626;
  t3834 = t1019*t459*t1031;
  t3855 = -1.*t459*t1711*t1633;
  t3856 = t3834 + t3855;
  t635 = t408*t459*t557;
  t1873 = t861*t1809;
  t1882 = t635 + t1873;
  t2828 = t861*t408*t459;
  t2830 = -1.*t557*t1809;
  t2854 = t2828 + t2830;
  t3314 = t459*t557*t1567;
  t3678 = t861*t3655;
  t3683 = t3314 + t3678;
  t3756 = t861*t459*t1567;
  t3777 = -1.*t557*t3655;
  t3793 = t3756 + t3777;
  t3830 = -1.*t557*t1094;
  t3890 = t861*t3856;
  t4117 = t3830 + t3890;
  t4160 = -1.*t861*t1094;
  t4222 = -1.*t557*t3856;
  t4234 = t4160 + t4222;
  t4346 = t1711*t1692;
  t4493 = t1019*t1792;
  t4536 = t4346 + t4493;
  t5002 = -1.*t2750;
  t5008 = 1. + t5002;
  t2387 = -1.*t399*t1882;
  t2866 = t2750*t2854;
  t2907 = t2387 + t2866;
  t4660 = t2750*t1882;
  t4661 = t399*t2854;
  t4662 = t4660 + t4661;
  t4669 = -1.*t861;
  t4673 = 1. + t4669;
  t4674 = -0.183*t4673;
  t4675 = 0. + t4674;
  t4677 = 0.047*t1711;
  t4678 = 0. + t4677;
  t4680 = -1.*t1019;
  t4681 = 1. + t4680;
  t4685 = -0.047*t4681;
  t4686 = 0. + t4685;
  t4550 = t1711*t3463;
  t4607 = t1019*t3561;
  t4639 = t4550 + t4607;
  t4759 = -0.183*t557;
  t4804 = 0. + t4759;
  t5086 = -0.2*t5008;
  t5093 = -0.183*t399;
  t5116 = 0. + t5086 + t5093;
  t5173 = -0.183*t5008;
  t5218 = 0.2*t399;
  t5303 = 0. + t5173 + t5218;
  t3749 = -1.*t399*t3683;
  t3794 = t2750*t3793;
  t3800 = t3749 + t3794;
  t4663 = t2750*t3683;
  t4664 = t399*t3793;
  t4665 = t4663 + t4664;
  t4640 = t459*t1031*t1711;
  t4652 = t1019*t459*t1633;
  t4653 = t4640 + t4652;
  t4150 = -1.*t399*t4117;
  t4327 = t2750*t4234;
  t4330 = t4150 + t4327;
  t4666 = t2750*t4117;
  t4667 = t399*t4234;
  t4668 = t4666 + t4667;

  p_output1(0)=t2907;
  p_output1(1)=t3800;
  p_output1(2)=t4330;
  p_output1(3)=0.;
  p_output1(4)=t4536;
  p_output1(5)=t4639;
  p_output1(6)=t4653;
  p_output1(7)=0.;
  p_output1(8)=t4662;
  p_output1(9)=t4665;
  p_output1(10)=t4668;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.183*t2907 - 0.13205*t4536 - 0.4*t4662 + t408*t459*t4675 + t1692*t4678 + t1792*t4686 + t1809*t4804 + t1882*t5116 + t2854*t5303 + var1(0);
  p_output1(13)=0. - 0.183*t3800 - 0.13205*t4639 - 0.4*t4665 + t1567*t459*t4675 + t3463*t4678 + t3561*t4686 + t3655*t4804 + t3683*t5116 + t3793*t5303 + var1(1);
  p_output1(14)=0. - 0.183*t4330 - 0.13205*t4653 - 0.4*t4668 - 1.*t1094*t4675 + t1031*t459*t4678 + t1633*t459*t4686 + t3856*t4804 + t4117*t5116 + t4234*t5303 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_RRfoot(const Eigen::Matrix<double,18,1> &var1)
//void H_RRfoot(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



