/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:12 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RR_calf_joint.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t1637;
  double t2843;
  double t4349;
  double t2874;
  double t4370;
  double t1863;
  double t4190;
  double t4522;
  double t5095;
  double t5681;
  double t7133;
  double t7548;
  double t8748;
  double t1773;
  double t8806;
  double t928;
  double t8817;
  double t8819;
  double t8828;
  double t8847;
  double t8848;
  double t8849;
  double t8852;
  double t8860;
  double t8866;
  double t8869;
  double t8883;
  double t8885;
  double t8887;
  double t963;
  double t1366;
  double t1406;
  double t1471;
  double t1866;
  double t2378;
  double t6178;
  double t6294;
  double t6567;
  double t7105;
  double t8922;
  double t8923;
  double t8924;
  double t8930;
  double t8936;
  double t8943;
  double t8811;
  double t8815;
  double t8850;
  double t8857;
  double t8858;
  double t8957;
  double t8958;
  double t8959;
  double t8873;
  double t8876;
  double t8877;
  double t8970;
  double t8971;
  double t8972;
  double t8977;
  double t8978;
  double t8979;
  double t9008;
  double t9009;
  double t9010;
  double t9012;
  double t9013;
  double t9014;
  double t9016;
  double t9017;
  double t9018;
  t1637 = Cos(var1[3]);
  t2843 = Cos(var1[5]);
  t4349 = Sin(var1[3]);
  t2874 = Sin(var1[4]);
  t4370 = Sin(var1[5]);
  t1863 = Sin(var1[12]);
  t4190 = t1637*t2843*t2874;
  t4522 = t4349*t4370;
  t5095 = t4190 + t4522;
  t5681 = Cos(var1[12]);
  t7133 = -1.*t2843*t4349;
  t7548 = t1637*t2874*t4370;
  t8748 = t7133 + t7548;
  t1773 = Cos(var1[4]);
  t8806 = Sin(var1[13]);
  t928 = Cos(var1[13]);
  t8817 = t5681*t5095;
  t8819 = -1.*t1863*t8748;
  t8828 = t8817 + t8819;
  t8847 = Cos(var1[14]);
  t8848 = -1.*t8847;
  t8849 = 1. + t8848;
  t8852 = Sin(var1[14]);
  t8860 = t1637*t1773*t8806;
  t8866 = t928*t8828;
  t8869 = t8860 + t8866;
  t8883 = t928*t1637*t1773;
  t8885 = -1.*t8806*t8828;
  t8887 = t8883 + t8885;
  t963 = -1.*t928;
  t1366 = 1. + t963;
  t1406 = -0.183*t1366;
  t1471 = 0. + t1406;
  t1866 = 0.047*t1863;
  t2378 = 0. + t1866;
  t6178 = -1.*t5681;
  t6294 = 1. + t6178;
  t6567 = -0.047*t6294;
  t7105 = 0. + t6567;
  t8922 = t2843*t4349*t2874;
  t8923 = -1.*t1637*t4370;
  t8924 = t8922 + t8923;
  t8930 = t1637*t2843;
  t8936 = t4349*t2874*t4370;
  t8943 = t8930 + t8936;
  t8811 = -0.183*t8806;
  t8815 = 0. + t8811;
  t8850 = -0.2*t8849;
  t8857 = -0.183*t8852;
  t8858 = 0. + t8850 + t8857;
  t8957 = t5681*t8924;
  t8958 = -1.*t1863*t8943;
  t8959 = t8957 + t8958;
  t8873 = -0.183*t8849;
  t8876 = 0.2*t8852;
  t8877 = 0. + t8873 + t8876;
  t8970 = t1773*t8806*t4349;
  t8971 = t928*t8959;
  t8972 = t8970 + t8971;
  t8977 = t928*t1773*t4349;
  t8978 = -1.*t8806*t8959;
  t8979 = t8977 + t8978;
  t9008 = t5681*t1773*t2843;
  t9009 = -1.*t1773*t1863*t4370;
  t9010 = t9008 + t9009;
  t9012 = -1.*t8806*t2874;
  t9013 = t928*t9010;
  t9014 = t9012 + t9013;
  t9016 = -1.*t928*t2874;
  t9017 = -1.*t8806*t9010;
  t9018 = t9016 + t9017;

  p_output1(0)=0. + t1471*t1637*t1773 + t2378*t5095 + t7105*t8748 - 0.13205*(t1863*t5095 + t5681*t8748) + t8815*t8828 + t8858*t8869 + t8877*t8887 - 0.183*(-1.*t8852*t8869 + t8847*t8887) - 0.2*(t8847*t8869 + t8852*t8887) + var1(0);
  p_output1(1)=0. + t1471*t1773*t4349 + t2378*t8924 + t7105*t8943 - 0.13205*(t1863*t8924 + t5681*t8943) + t8815*t8959 + t8858*t8972 + t8877*t8979 - 0.183*(-1.*t8852*t8972 + t8847*t8979) - 0.2*(t8847*t8972 + t8852*t8979) + var1(1);
  p_output1(2)=0. + t1773*t2378*t2843 - 1.*t1471*t2874 - 0.13205*(t1773*t1863*t2843 + t1773*t4370*t5681) + t1773*t4370*t7105 + t8815*t9010 + t8858*t9014 + t8877*t9018 - 0.183*(-1.*t8852*t9014 + t8847*t9018) - 0.2*(t8847*t9014 + t8852*t9018) + var1(2);
}


       
Eigen::Matrix<double,3,1> p_RR_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void p_RR_calf_joint(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



