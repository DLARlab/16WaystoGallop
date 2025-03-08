/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:02 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FRfoot_src.h"

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
  double t300;
  double t5521;
  double t5546;
  double t5523;
  double t5577;
  double t673;
  double t1344;
  double t5539;
  double t5582;
  double t5592;
  double t5594;
  double t5605;
  double t5613;
  double t5614;
  double t5616;
  double t5617;
  double t5618;
  double t301;
  double t5621;
  double t299;
  double t5679;
  double t5680;
  double t5682;
  double t5683;
  double t5684;
  double t5685;
  double t5686;
  double t5687;
  double t5690;
  double t5656;
  double t5720;
  double t5722;
  double t5727;
  double t5642;
  double t5646;
  double t5648;
  double t751;
  double t5630;
  double t5637;
  double t5698;
  double t5706;
  double t5707;
  double t5678;
  double t5691;
  double t5694;
  double t5752;
  double t5753;
  double t5754;
  double t5717;
  double t5743;
  double t5748;
  double t5759;
  double t5773;
  double t5776;
  double t5851;
  double t5852;
  double t5641;
  double t5662;
  double t5671;
  double t5783;
  double t5784;
  double t5785;
  double t5797;
  double t5802;
  double t5803;
  double t5808;
  double t5815;
  double t5816;
  double t5818;
  double t5819;
  double t5821;
  double t5839;
  double t5777;
  double t5778;
  double t5779;
  double t5845;
  double t5846;
  double t5853;
  double t5855;
  double t5856;
  double t5860;
  double t5861;
  double t5862;
  double t5695;
  double t5710;
  double t5716;
  double t5786;
  double t5787;
  double t5788;
  double t5780;
  double t5781;
  double t5782;
  double t5751;
  double t5755;
  double t5756;
  double t5793;
  double t5794;
  double t5795;
  t300 = Cos(var1[3]);
  t5521 = Cos(var1[5]);
  t5546 = Sin(var1[3]);
  t5523 = Sin(var1[4]);
  t5577 = Sin(var1[5]);
  t673 = Cos(var1[7]);
  t1344 = Cos(var1[6]);
  t5539 = t300*t5521*t5523;
  t5582 = t5546*t5577;
  t5592 = t5539 + t5582;
  t5594 = t1344*t5592;
  t5605 = -1.*t5521*t5546;
  t5613 = t300*t5523*t5577;
  t5614 = t5605 + t5613;
  t5616 = Sin(var1[6]);
  t5617 = -1.*t5614*t5616;
  t5618 = t5594 + t5617;
  t301 = Cos(var1[4]);
  t5621 = Sin(var1[7]);
  t299 = Cos(var1[8]);
  t5679 = t5521*t5546*t5523;
  t5680 = -1.*t300*t5577;
  t5682 = t5679 + t5680;
  t5683 = t1344*t5682;
  t5684 = t300*t5521;
  t5685 = t5546*t5523*t5577;
  t5686 = t5684 + t5685;
  t5687 = -1.*t5686*t5616;
  t5690 = t5683 + t5687;
  t5656 = Sin(var1[8]);
  t5720 = t301*t5521*t1344;
  t5722 = -1.*t301*t5577*t5616;
  t5727 = t5720 + t5722;
  t5642 = t673*t5618;
  t5646 = t300*t301*t5621;
  t5648 = t5642 + t5646;
  t751 = t300*t301*t673;
  t5630 = -1.*t5618*t5621;
  t5637 = t751 + t5630;
  t5698 = t673*t5690;
  t5706 = t301*t5546*t5621;
  t5707 = t5698 + t5706;
  t5678 = t301*t673*t5546;
  t5691 = -1.*t5690*t5621;
  t5694 = t5678 + t5691;
  t5752 = t673*t5727;
  t5753 = -1.*t5523*t5621;
  t5754 = t5752 + t5753;
  t5717 = -1.*t673*t5523;
  t5743 = -1.*t5727*t5621;
  t5748 = t5717 + t5743;
  t5759 = t1344*t5614;
  t5773 = t5592*t5616;
  t5776 = t5759 + t5773;
  t5851 = -1.*t299;
  t5852 = 1. + t5851;
  t5641 = t299*t5637;
  t5662 = -1.*t5648*t5656;
  t5671 = t5641 + t5662;
  t5783 = t299*t5648;
  t5784 = t5637*t5656;
  t5785 = t5783 + t5784;
  t5797 = -1.*t673;
  t5802 = 1. + t5797;
  t5803 = 0.183*t5802;
  t5808 = 0. + t5803;
  t5815 = -1.*t1344;
  t5816 = 1. + t5815;
  t5818 = -0.047*t5816;
  t5819 = 0. + t5818;
  t5821 = 0.047*t5616;
  t5839 = 0. + t5821;
  t5777 = t1344*t5686;
  t5778 = t5682*t5616;
  t5779 = t5777 + t5778;
  t5845 = 0.183*t5621;
  t5846 = 0. + t5845;
  t5853 = -0.2*t5852;
  t5855 = 0.183*t5656;
  t5856 = 0. + t5853 + t5855;
  t5860 = 0.183*t5852;
  t5861 = 0.2*t5656;
  t5862 = 0. + t5860 + t5861;
  t5695 = t299*t5694;
  t5710 = -1.*t5707*t5656;
  t5716 = t5695 + t5710;
  t5786 = t299*t5707;
  t5787 = t5694*t5656;
  t5788 = t5786 + t5787;
  t5780 = t301*t1344*t5577;
  t5781 = t301*t5521*t5616;
  t5782 = t5780 + t5781;
  t5751 = t299*t5748;
  t5755 = -1.*t5754*t5656;
  t5756 = t5751 + t5755;
  t5793 = t299*t5754;
  t5794 = t5748*t5656;
  t5795 = t5793 + t5794;
  p_output1[0]=t5671;
  p_output1[1]=t5716;
  p_output1[2]=t5756;
  p_output1[3]=0.;
  p_output1[4]=t5776;
  p_output1[5]=t5779;
  p_output1[6]=t5782;
  p_output1[7]=0.;
  p_output1[8]=t5785;
  p_output1[9]=t5788;
  p_output1[10]=t5795;
  p_output1[11]=0.;
  p_output1[12]=0. + 0.183*t5671 - 0.13205*t5776 - 0.4*t5785 + t300*t301*t5808 + t5614*t5819 + t5592*t5839 + t5618*t5846 + t5648*t5856 + t5637*t5862 + var1[0];
  p_output1[13]=0. + 0.183*t5716 - 0.13205*t5779 - 0.4*t5788 + t301*t5546*t5808 + t5686*t5819 + t5682*t5839 + t5690*t5846 + t5707*t5856 + t5694*t5862 + var1[1];
  p_output1[14]=0. + 0.183*t5756 - 0.13205*t5782 - 0.4*t5795 - 1.*t5523*t5808 + t301*t5577*t5819 + t301*t5521*t5839 + t5727*t5846 + t5754*t5856 + t5748*t5862 + var1[2];
  p_output1[15]=1.;
}



void H_FRfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
