/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:47 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RR_calf_joint_src.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE static __inline__        /* use standard inline */
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
static void output1(double *p_output1,const double *var1)
{
  double t3885;
  double t3956;
  double t3960;
  double t3958;
  double t3961;
  double t3929;
  double t3886;
  double t3909;
  double t3949;
  double t3959;
  double t3962;
  double t3963;
  double t3964;
  double t3966;
  double t3969;
  double t3970;
  double t3971;
  double t3974;
  double t3975;
  double t3872;
  double t3979;
  double t3986;
  double t3987;
  double t3988;
  double t3991;
  double t3992;
  double t3993;
  double t3994;
  double t3995;
  double t3996;
  double t4010;
  double t4011;
  double t4013;
  double t3917;
  double t3976;
  double t3977;
  double t3980;
  double t3981;
  double t3982;
  double t3985;
  double t3997;
  double t3999;
  double t4001;
  double t4002;
  double t4003;
  double t4007;
  double t4014;
  double t4015;
  double t4018;
  double t4019;
  double t4020;
  t3885 = Cos(var1[3]);
  t3956 = Cos(var1[5]);
  t3960 = Sin(var1[3]);
  t3958 = Sin(var1[4]);
  t3961 = Sin(var1[5]);
  t3929 = Cos(var1[13]);
  t3886 = Cos(var1[4]);
  t3909 = Sin(var1[13]);
  t3949 = Cos(var1[12]);
  t3959 = t3885*t3956*t3958;
  t3962 = t3960*t3961;
  t3963 = t3959 + t3962;
  t3964 = t3949*t3963;
  t3966 = Sin(var1[12]);
  t3969 = -1.*t3956*t3960;
  t3970 = t3885*t3958*t3961;
  t3971 = t3969 + t3970;
  t3974 = -1.*t3966*t3971;
  t3975 = t3964 + t3974;
  t3872 = Sin(var1[14]);
  t3979 = Cos(var1[14]);
  t3986 = t3956*t3960*t3958;
  t3987 = -1.*t3885*t3961;
  t3988 = t3986 + t3987;
  t3991 = t3949*t3988;
  t3992 = t3885*t3956;
  t3993 = t3960*t3958*t3961;
  t3994 = t3992 + t3993;
  t3995 = -1.*t3966*t3994;
  t3996 = t3991 + t3995;
  t4010 = t3949*t3886*t3956;
  t4011 = -1.*t3886*t3966*t3961;
  t4013 = t4010 + t4011;
  t3917 = t3885*t3886*t3909;
  t3976 = t3929*t3975;
  t3977 = t3917 + t3976;
  t3980 = t3929*t3885*t3886;
  t3981 = -1.*t3909*t3975;
  t3982 = t3980 + t3981;
  t3985 = t3886*t3909*t3960;
  t3997 = t3929*t3996;
  t3999 = t3985 + t3997;
  t4001 = t3929*t3886*t3960;
  t4002 = -1.*t3909*t3996;
  t4003 = t4001 + t4002;
  t4007 = -1.*t3909*t3958;
  t4014 = t3929*t4013;
  t4015 = t4007 + t4014;
  t4018 = -1.*t3929*t3958;
  t4019 = -1.*t3909*t4013;
  t4020 = t4018 + t4019;
  p_output1[0]=-1.*t3872*t3977 + t3979*t3982;
  p_output1[1]=-1.*t3872*t3999 + t3979*t4003;
  p_output1[2]=-1.*t3872*t4015 + t3979*t4020;
  p_output1[3]=t3963*t3966 + t3949*t3971;
  p_output1[4]=t3966*t3988 + t3949*t3994;
  p_output1[5]=t3886*t3949*t3961 + t3886*t3956*t3966;
  p_output1[6]=t3977*t3979 + t3872*t3982;
  p_output1[7]=t3979*t3999 + t3872*t4003;
  p_output1[8]=t3979*t4015 + t3872*t4020;
}



void R_RR_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
