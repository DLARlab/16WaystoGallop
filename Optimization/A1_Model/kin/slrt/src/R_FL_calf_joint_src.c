/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:35 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_FL_calf_joint_src.h"

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
  double t2554;
  double t2626;
  double t2629;
  double t2627;
  double t2630;
  double t2603;
  double t2557;
  double t2578;
  double t2618;
  double t2628;
  double t2631;
  double t2632;
  double t2634;
  double t2637;
  double t2638;
  double t2639;
  double t2642;
  double t2643;
  double t2644;
  double t2541;
  double t2648;
  double t2655;
  double t2656;
  double t2659;
  double t2660;
  double t2661;
  double t2662;
  double t2663;
  double t2664;
  double t2665;
  double t2679;
  double t2681;
  double t2682;
  double t2597;
  double t2645;
  double t2646;
  double t2649;
  double t2650;
  double t2651;
  double t2654;
  double t2667;
  double t2668;
  double t2670;
  double t2671;
  double t2673;
  double t2678;
  double t2683;
  double t2685;
  double t2687;
  double t2688;
  double t2689;
  t2554 = Cos(var1[3]);
  t2626 = Cos(var1[5]);
  t2629 = Sin(var1[3]);
  t2627 = Sin(var1[4]);
  t2630 = Sin(var1[5]);
  t2603 = Cos(var1[10]);
  t2557 = Cos(var1[4]);
  t2578 = Sin(var1[10]);
  t2618 = Cos(var1[9]);
  t2628 = t2554*t2626*t2627;
  t2631 = t2629*t2630;
  t2632 = t2628 + t2631;
  t2634 = t2618*t2632;
  t2637 = Sin(var1[9]);
  t2638 = -1.*t2626*t2629;
  t2639 = t2554*t2627*t2630;
  t2642 = t2638 + t2639;
  t2643 = -1.*t2637*t2642;
  t2644 = t2634 + t2643;
  t2541 = Sin(var1[11]);
  t2648 = Cos(var1[11]);
  t2655 = t2626*t2629*t2627;
  t2656 = -1.*t2554*t2630;
  t2659 = t2655 + t2656;
  t2660 = t2618*t2659;
  t2661 = t2554*t2626;
  t2662 = t2629*t2627*t2630;
  t2663 = t2661 + t2662;
  t2664 = -1.*t2637*t2663;
  t2665 = t2660 + t2664;
  t2679 = t2618*t2557*t2626;
  t2681 = -1.*t2557*t2637*t2630;
  t2682 = t2679 + t2681;
  t2597 = t2554*t2557*t2578;
  t2645 = t2603*t2644;
  t2646 = t2597 + t2645;
  t2649 = t2603*t2554*t2557;
  t2650 = -1.*t2578*t2644;
  t2651 = t2649 + t2650;
  t2654 = t2557*t2578*t2629;
  t2667 = t2603*t2665;
  t2668 = t2654 + t2667;
  t2670 = t2603*t2557*t2629;
  t2671 = -1.*t2578*t2665;
  t2673 = t2670 + t2671;
  t2678 = -1.*t2578*t2627;
  t2683 = t2603*t2682;
  t2685 = t2678 + t2683;
  t2687 = -1.*t2603*t2627;
  t2688 = -1.*t2578*t2682;
  t2689 = t2687 + t2688;
  p_output1[0]=-1.*t2541*t2646 + t2648*t2651;
  p_output1[1]=-1.*t2541*t2668 + t2648*t2673;
  p_output1[2]=-1.*t2541*t2685 + t2648*t2689;
  p_output1[3]=t2632*t2637 + t2618*t2642;
  p_output1[4]=t2637*t2659 + t2618*t2663;
  p_output1[5]=t2557*t2618*t2630 + t2557*t2626*t2637;
  p_output1[6]=t2646*t2648 + t2541*t2651;
  p_output1[7]=t2648*t2668 + t2541*t2673;
  p_output1[8]=t2648*t2685 + t2541*t2689;
}



void R_FL_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
