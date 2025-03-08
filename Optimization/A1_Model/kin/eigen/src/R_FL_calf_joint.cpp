/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:03 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_FL_calf_joint.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t431;
  double t2588;
  double t3090;
  double t2691;
  double t3272;
  double t1880;
  double t454;
  double t1051;
  double t2315;
  double t2845;
  double t3595;
  double t3689;
  double t3740;
  double t3744;
  double t3775;
  double t3826;
  double t3854;
  double t4084;
  double t4368;
  double t232;
  double t4516;
  double t4706;
  double t4710;
  double t4727;
  double t4786;
  double t4849;
  double t4857;
  double t4983;
  double t4987;
  double t5041;
  double t5771;
  double t5818;
  double t5842;
  double t1274;
  double t4404;
  double t4465;
  double t4527;
  double t4551;
  double t4568;
  double t4619;
  double t5083;
  double t5341;
  double t5540;
  double t5544;
  double t5567;
  double t5731;
  double t6318;
  double t6341;
  double t6503;
  double t6573;
  double t6873;
  t431 = Cos(var1[3]);
  t2588 = Cos(var1[5]);
  t3090 = Sin(var1[3]);
  t2691 = Sin(var1[4]);
  t3272 = Sin(var1[5]);
  t1880 = Cos(var1[10]);
  t454 = Cos(var1[4]);
  t1051 = Sin(var1[10]);
  t2315 = Cos(var1[9]);
  t2845 = t431*t2588*t2691;
  t3595 = t3090*t3272;
  t3689 = t2845 + t3595;
  t3740 = t2315*t3689;
  t3744 = Sin(var1[9]);
  t3775 = -1.*t2588*t3090;
  t3826 = t431*t2691*t3272;
  t3854 = t3775 + t3826;
  t4084 = -1.*t3744*t3854;
  t4368 = t3740 + t4084;
  t232 = Sin(var1[11]);
  t4516 = Cos(var1[11]);
  t4706 = t2588*t3090*t2691;
  t4710 = -1.*t431*t3272;
  t4727 = t4706 + t4710;
  t4786 = t2315*t4727;
  t4849 = t431*t2588;
  t4857 = t3090*t2691*t3272;
  t4983 = t4849 + t4857;
  t4987 = -1.*t3744*t4983;
  t5041 = t4786 + t4987;
  t5771 = t2315*t454*t2588;
  t5818 = -1.*t454*t3744*t3272;
  t5842 = t5771 + t5818;
  t1274 = t431*t454*t1051;
  t4404 = t1880*t4368;
  t4465 = t1274 + t4404;
  t4527 = t1880*t431*t454;
  t4551 = -1.*t1051*t4368;
  t4568 = t4527 + t4551;
  t4619 = t454*t1051*t3090;
  t5083 = t1880*t5041;
  t5341 = t4619 + t5083;
  t5540 = t1880*t454*t3090;
  t5544 = -1.*t1051*t5041;
  t5567 = t5540 + t5544;
  t5731 = -1.*t1051*t2691;
  t6318 = t1880*t5842;
  t6341 = t5731 + t6318;
  t6503 = -1.*t1880*t2691;
  t6573 = -1.*t1051*t5842;
  t6873 = t6503 + t6573;

  p_output1(0)=-1.*t232*t4465 + t4516*t4568;
  p_output1(1)=-1.*t232*t5341 + t4516*t5567;
  p_output1(2)=-1.*t232*t6341 + t4516*t6873;
  p_output1(3)=t3689*t3744 + t2315*t3854;
  p_output1(4)=t3744*t4727 + t2315*t4983;
  p_output1(5)=t2315*t3272*t454 + t2588*t3744*t454;
  p_output1(6)=t4465*t4516 + t232*t4568;
  p_output1(7)=t4516*t5341 + t232*t5567;
  p_output1(8)=t4516*t6341 + t232*t6873;
}


       
Eigen::Matrix<double,3,3> R_FL_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void R_FL_calf_joint(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



