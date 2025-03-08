/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:30 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FLfoot.h"

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
  double t1744;
  double t4300;
  double t5078;
  double t4452;
  double t5117;
  double t1960;
  double t4453;
  double t5118;
  double t5326;
  double t5381;
  double t5579;
  double t5585;
  double t5612;
  double t1829;
  double t5706;
  double t1444;
  double t5777;
  double t5780;
  double t5787;
  double t5811;
  double t5812;
  double t5814;
  double t5855;
  double t5863;
  double t5867;
  double t5868;
  double t5889;
  double t5892;
  double t5893;
  double t1551;
  double t1631;
  double t1705;
  double t1706;
  double t1962;
  double t3743;
  double t5386;
  double t5436;
  double t5483;
  double t5524;
  double t6287;
  double t6296;
  double t6391;
  double t6499;
  double t6559;
  double t6566;
  double t5759;
  double t5773;
  double t5821;
  double t5861;
  double t5862;
  double t6703;
  double t6797;
  double t6885;
  double t5877;
  double t5878;
  double t5879;
  double t6912;
  double t6927;
  double t6931;
  double t7051;
  double t7052;
  double t7059;
  double t7558;
  double t7560;
  double t7604;
  double t7733;
  double t7734;
  double t7743;
  double t7761;
  double t7808;
  double t7950;
  t1744 = Cos(var1[3]);
  t4300 = Cos(var1[5]);
  t5078 = Sin(var1[3]);
  t4452 = Sin(var1[4]);
  t5117 = Sin(var1[5]);
  t1960 = Sin(var1[9]);
  t4453 = t1744*t4300*t4452;
  t5118 = t5078*t5117;
  t5326 = t4453 + t5118;
  t5381 = Cos(var1[9]);
  t5579 = -1.*t4300*t5078;
  t5585 = t1744*t4452*t5117;
  t5612 = t5579 + t5585;
  t1829 = Cos(var1[4]);
  t5706 = Sin(var1[10]);
  t1444 = Cos(var1[10]);
  t5777 = t5381*t5326;
  t5780 = -1.*t1960*t5612;
  t5787 = t5777 + t5780;
  t5811 = Cos(var1[11]);
  t5812 = -1.*t5811;
  t5814 = 1. + t5812;
  t5855 = Sin(var1[11]);
  t5863 = t1744*t1829*t5706;
  t5867 = t1444*t5787;
  t5868 = t5863 + t5867;
  t5889 = t1444*t1744*t1829;
  t5892 = -1.*t5706*t5787;
  t5893 = t5889 + t5892;
  t1551 = -1.*t1444;
  t1631 = 1. + t1551;
  t1705 = 0.183*t1631;
  t1706 = 0. + t1705;
  t1962 = -0.047*t1960;
  t3743 = 0. + t1962;
  t5386 = -1.*t5381;
  t5436 = 1. + t5386;
  t5483 = 0.047*t5436;
  t5524 = 0. + t5483;
  t6287 = t4300*t5078*t4452;
  t6296 = -1.*t1744*t5117;
  t6391 = t6287 + t6296;
  t6499 = t1744*t4300;
  t6559 = t5078*t4452*t5117;
  t6566 = t6499 + t6559;
  t5759 = 0.183*t5706;
  t5773 = 0. + t5759;
  t5821 = -0.2*t5814;
  t5861 = 0.183*t5855;
  t5862 = 0. + t5821 + t5861;
  t6703 = t5381*t6391;
  t6797 = -1.*t1960*t6566;
  t6885 = t6703 + t6797;
  t5877 = 0.183*t5814;
  t5878 = 0.2*t5855;
  t5879 = 0. + t5877 + t5878;
  t6912 = t1829*t5706*t5078;
  t6927 = t1444*t6885;
  t6931 = t6912 + t6927;
  t7051 = t1444*t1829*t5078;
  t7052 = -1.*t5706*t6885;
  t7059 = t7051 + t7052;
  t7558 = t5381*t1829*t4300;
  t7560 = -1.*t1829*t1960*t5117;
  t7604 = t7558 + t7560;
  t7733 = -1.*t5706*t4452;
  t7734 = t1444*t7604;
  t7743 = t7733 + t7734;
  t7761 = -1.*t1444*t4452;
  t7808 = -1.*t5706*t7604;
  t7950 = t7761 + t7808;

  p_output1(0)=0. + t1706*t1744*t1829 + t3743*t5326 + t5524*t5612 + 0.13205*(t1960*t5326 + t5381*t5612) + t5773*t5787 + t5862*t5868 + t5879*t5893 + 0.183*(-1.*t5855*t5868 + t5811*t5893) - 0.4*(t5811*t5868 + t5855*t5893) + var1(0);
  p_output1(1)=0. + t1706*t1829*t5078 + t3743*t6391 + t5524*t6566 + 0.13205*(t1960*t6391 + t5381*t6566) + t5773*t6885 + t5862*t6931 + t5879*t7059 + 0.183*(-1.*t5855*t6931 + t5811*t7059) - 0.4*(t5811*t6931 + t5855*t7059) + var1(1);
  p_output1(2)=0. + t1829*t3743*t4300 - 1.*t1706*t4452 + 0.13205*(t1829*t1960*t4300 + t1829*t5117*t5381) + t1829*t5117*t5524 + t5773*t7604 + t5862*t7743 + t5879*t7950 + 0.183*(-1.*t5855*t7743 + t5811*t7950) - 0.4*(t5811*t7743 + t5855*t7950) + var1(2);
}


       
Eigen::Matrix<double,3,1> p_FLfoot(const Eigen::Matrix<double,18,1> &var1)
//void p_FLfoot(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



