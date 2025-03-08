/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:00 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FL_calf_joint.h"

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
  double t3783;
  double t5020;
  double t6127;
  double t5453;
  double t8458;
  double t4790;
  double t5999;
  double t8482;
  double t8490;
  double t8497;
  double t8513;
  double t8514;
  double t8515;
  double t3928;
  double t8538;
  double t468;
  double t8542;
  double t8543;
  double t8549;
  double t8558;
  double t8559;
  double t8560;
  double t8565;
  double t8568;
  double t8569;
  double t8571;
  double t8579;
  double t8581;
  double t8582;
  double t1678;
  double t2665;
  double t3059;
  double t3347;
  double t4797;
  double t4921;
  double t8502;
  double t8503;
  double t8505;
  double t8510;
  double t8598;
  double t8599;
  double t8602;
  double t8604;
  double t8605;
  double t8606;
  double t8540;
  double t8541;
  double t8564;
  double t8566;
  double t8567;
  double t8612;
  double t8613;
  double t8614;
  double t8574;
  double t8575;
  double t8578;
  double t8616;
  double t8617;
  double t8618;
  double t8622;
  double t8624;
  double t8626;
  double t8647;
  double t8648;
  double t8649;
  double t8651;
  double t8652;
  double t8653;
  double t8655;
  double t8656;
  double t8657;
  t3783 = Cos(var1[3]);
  t5020 = Cos(var1[5]);
  t6127 = Sin(var1[3]);
  t5453 = Sin(var1[4]);
  t8458 = Sin(var1[5]);
  t4790 = Sin(var1[9]);
  t5999 = t3783*t5020*t5453;
  t8482 = t6127*t8458;
  t8490 = t5999 + t8482;
  t8497 = Cos(var1[9]);
  t8513 = -1.*t5020*t6127;
  t8514 = t3783*t5453*t8458;
  t8515 = t8513 + t8514;
  t3928 = Cos(var1[4]);
  t8538 = Sin(var1[10]);
  t468 = Cos(var1[10]);
  t8542 = t8497*t8490;
  t8543 = -1.*t4790*t8515;
  t8549 = t8542 + t8543;
  t8558 = Cos(var1[11]);
  t8559 = -1.*t8558;
  t8560 = 1. + t8559;
  t8565 = Sin(var1[11]);
  t8568 = t3783*t3928*t8538;
  t8569 = t468*t8549;
  t8571 = t8568 + t8569;
  t8579 = t468*t3783*t3928;
  t8581 = -1.*t8538*t8549;
  t8582 = t8579 + t8581;
  t1678 = -1.*t468;
  t2665 = 1. + t1678;
  t3059 = 0.183*t2665;
  t3347 = 0. + t3059;
  t4797 = -0.047*t4790;
  t4921 = 0. + t4797;
  t8502 = -1.*t8497;
  t8503 = 1. + t8502;
  t8505 = 0.047*t8503;
  t8510 = 0. + t8505;
  t8598 = t5020*t6127*t5453;
  t8599 = -1.*t3783*t8458;
  t8602 = t8598 + t8599;
  t8604 = t3783*t5020;
  t8605 = t6127*t5453*t8458;
  t8606 = t8604 + t8605;
  t8540 = 0.183*t8538;
  t8541 = 0. + t8540;
  t8564 = -0.2*t8560;
  t8566 = 0.183*t8565;
  t8567 = 0. + t8564 + t8566;
  t8612 = t8497*t8602;
  t8613 = -1.*t4790*t8606;
  t8614 = t8612 + t8613;
  t8574 = 0.183*t8560;
  t8575 = 0.2*t8565;
  t8578 = 0. + t8574 + t8575;
  t8616 = t3928*t8538*t6127;
  t8617 = t468*t8614;
  t8618 = t8616 + t8617;
  t8622 = t468*t3928*t6127;
  t8624 = -1.*t8538*t8614;
  t8626 = t8622 + t8624;
  t8647 = t8497*t3928*t5020;
  t8648 = -1.*t3928*t4790*t8458;
  t8649 = t8647 + t8648;
  t8651 = -1.*t8538*t5453;
  t8652 = t468*t8649;
  t8653 = t8651 + t8652;
  t8655 = -1.*t468*t5453;
  t8656 = -1.*t8538*t8649;
  t8657 = t8655 + t8656;

  p_output1(0)=0. + t3347*t3783*t3928 + t4921*t8490 + t8510*t8515 + 0.13205*(t4790*t8490 + t8497*t8515) + t8541*t8549 + t8567*t8571 + t8578*t8582 + 0.183*(-1.*t8565*t8571 + t8558*t8582) - 0.2*(t8558*t8571 + t8565*t8582) + var1(0);
  p_output1(1)=0. + t3347*t3928*t6127 + t4921*t8602 + t8510*t8606 + 0.13205*(t4790*t8602 + t8497*t8606) + t8541*t8614 + t8567*t8618 + t8578*t8626 + 0.183*(-1.*t8565*t8618 + t8558*t8626) - 0.2*(t8558*t8618 + t8565*t8626) + var1(1);
  p_output1(2)=0. + t3928*t4921*t5020 - 1.*t3347*t5453 + 0.13205*(t3928*t4790*t5020 + t3928*t8458*t8497) + t3928*t8458*t8510 + t8541*t8649 + t8567*t8653 + t8578*t8657 + 0.183*(-1.*t8565*t8653 + t8558*t8657) - 0.2*(t8558*t8653 + t8565*t8657) + var1(2);
}


       
Eigen::Matrix<double,3,1> p_FL_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void p_FL_calf_joint(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



