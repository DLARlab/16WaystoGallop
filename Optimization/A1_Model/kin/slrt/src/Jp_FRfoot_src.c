/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:01 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FRfoot_src.h"

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
  double t5442;
  double t5500;
  double t5503;
  double t5499;
  double t5506;
  double t5473;
  double t5502;
  double t5510;
  double t5511;
  double t5513;
  double t5521;
  double t5523;
  double t5526;
  double t1114;
  double t5548;
  double t5549;
  double t5550;
  double t1020;
  double t5551;
  double t5560;
  double t5562;
  double t5565;
  double t5569;
  double t5578;
  double t5579;
  double t5581;
  double t5556;
  double t5558;
  double t5559;
  double t1121;
  double t1125;
  double t2206;
  double t5438;
  double t5480;
  double t5483;
  double t5485;
  double t5498;
  double t5527;
  double t5538;
  double t5599;
  double t5600;
  double t5601;
  double t5605;
  double t5606;
  double t5609;
  double t5552;
  double t5553;
  double t5618;
  double t5619;
  double t5620;
  double t5566;
  double t5573;
  double t5574;
  double t5582;
  double t5585;
  double t5586;
  double t5626;
  double t5627;
  double t5628;
  double t5622;
  double t5623;
  double t5624;
  double t5653;
  double t5654;
  double t5655;
  double t5665;
  double t5669;
  double t5670;
  double t5657;
  double t5659;
  double t5661;
  double t5691;
  double t5692;
  double t5693;
  double t5699;
  double t5700;
  double t5701;
  double t5695;
  double t5696;
  double t5697;
  double t5723;
  double t5724;
  double t5725;
  double t5745;
  double t5746;
  double t5747;
  double t5727;
  double t5739;
  double t5741;
  double t5761;
  double t5762;
  double t5763;
  double t5773;
  double t5774;
  double t5775;
  double t5788;
  double t5789;
  double t5790;
  double t5542;
  double t5800;
  double t5801;
  double t5829;
  double t5830;
  double t5832;
  double t5849;
  double t5850;
  double t5795;
  double t5867;
  double t5868;
  double t5869;
  double t5874;
  double t5875;
  double t5821;
  double t5822;
  double t5826;
  double t5828;
  double t5833;
  double t5834;
  double t5835;
  double t5837;
  double t5838;
  double t5839;
  double t5840;
  double t5841;
  double t5842;
  double t5843;
  double t5844;
  double t5630;
  double t5894;
  double t5895;
  double t5896;
  double t5871;
  double t5872;
  double t5908;
  double t5909;
  double t5910;
  double t5912;
  double t5913;
  double t5926;
  double t5927;
  double t5928;
  double t5930;
  double t5931;
  double t5932;
  double t5633;
  double t5634;
  double t5902;
  double t5943;
  double t5944;
  double t5945;
  double t5947;
  double t5948;
  double t5915;
  double t5955;
  double t5956;
  double t5957;
  double t5920;
  double t5934;
  double t5967;
  double t5968;
  double t5969;
  double t5939;
  t5442 = Sin(var1[3]);
  t5500 = Cos(var1[5]);
  t5503 = Sin(var1[4]);
  t5499 = Cos(var1[3]);
  t5506 = Sin(var1[5]);
  t5473 = Cos(var1[6]);
  t5502 = -1.*t5499*t5500;
  t5510 = -1.*t5442*t5503*t5506;
  t5511 = t5502 + t5510;
  t5513 = -1.*t5500*t5442*t5503;
  t5521 = t5499*t5506;
  t5523 = t5513 + t5521;
  t5526 = Sin(var1[6]);
  t1114 = Cos(var1[7]);
  t5548 = t5473*t5523;
  t5549 = -1.*t5511*t5526;
  t5550 = t5548 + t5549;
  t1020 = Cos(var1[4]);
  t5551 = Sin(var1[7]);
  t5560 = Cos(var1[8]);
  t5562 = -1.*t5560;
  t5565 = 1. + t5562;
  t5569 = Sin(var1[8]);
  t5578 = -1.*t1020*t1114*t5442;
  t5579 = -1.*t5550*t5551;
  t5581 = t5578 + t5579;
  t5556 = t1114*t5550;
  t5558 = -1.*t1020*t5442*t5551;
  t5559 = t5556 + t5558;
  t1121 = -1.*t1114;
  t1125 = 1. + t1121;
  t2206 = 0.183*t1125;
  t5438 = 0. + t2206;
  t5480 = -1.*t5473;
  t5483 = 1. + t5480;
  t5485 = -0.047*t5483;
  t5498 = 0. + t5485;
  t5527 = 0.047*t5526;
  t5538 = 0. + t5527;
  t5599 = -1.*t5500*t5442;
  t5600 = t5499*t5503*t5506;
  t5601 = t5599 + t5600;
  t5605 = t5499*t5500*t5503;
  t5606 = t5442*t5506;
  t5609 = t5605 + t5606;
  t5552 = 0.183*t5551;
  t5553 = 0. + t5552;
  t5618 = t5473*t5609;
  t5619 = -1.*t5601*t5526;
  t5620 = t5618 + t5619;
  t5566 = -0.2*t5565;
  t5573 = 0.183*t5569;
  t5574 = 0. + t5566 + t5573;
  t5582 = 0.183*t5565;
  t5585 = 0.2*t5569;
  t5586 = 0. + t5582 + t5585;
  t5626 = t5499*t1020*t1114;
  t5627 = -1.*t5620*t5551;
  t5628 = t5626 + t5627;
  t5622 = t1114*t5620;
  t5623 = t5499*t1020*t5551;
  t5624 = t5622 + t5623;
  t5653 = t5499*t1020*t5500*t5473;
  t5654 = -1.*t5499*t1020*t5506*t5526;
  t5655 = t5653 + t5654;
  t5665 = -1.*t5499*t1114*t5503;
  t5669 = -1.*t5655*t5551;
  t5670 = t5665 + t5669;
  t5657 = t1114*t5655;
  t5659 = -1.*t5499*t5503*t5551;
  t5661 = t5657 + t5659;
  t5691 = t1020*t5500*t5473*t5442;
  t5692 = -1.*t1020*t5442*t5506*t5526;
  t5693 = t5691 + t5692;
  t5699 = -1.*t1114*t5442*t5503;
  t5700 = -1.*t5693*t5551;
  t5701 = t5699 + t5700;
  t5695 = t1114*t5693;
  t5696 = -1.*t5442*t5503*t5551;
  t5697 = t5695 + t5696;
  t5723 = -1.*t5500*t5473*t5503;
  t5724 = t5503*t5506*t5526;
  t5725 = t5723 + t5724;
  t5745 = -1.*t1020*t1114;
  t5746 = -1.*t5725*t5551;
  t5747 = t5745 + t5746;
  t5727 = t1114*t5725;
  t5739 = -1.*t1020*t5551;
  t5741 = t5727 + t5739;
  t5761 = t5500*t5442;
  t5762 = -1.*t5499*t5503*t5506;
  t5763 = t5761 + t5762;
  t5773 = t5473*t5763;
  t5774 = -1.*t5609*t5526;
  t5775 = t5773 + t5774;
  t5788 = t5500*t5442*t5503;
  t5789 = -1.*t5499*t5506;
  t5790 = t5788 + t5789;
  t5542 = t5473*t5511;
  t5800 = -1.*t5790*t5526;
  t5801 = t5542 + t5800;
  t5829 = -1.*t1020*t5473*t5506;
  t5830 = -1.*t1020*t5500*t5526;
  t5832 = t5829 + t5830;
  t5849 = -1.*t5473*t5601;
  t5850 = t5849 + t5774;
  t5795 = t5473*t5790;
  t5867 = t5499*t5500;
  t5868 = t5442*t5503*t5506;
  t5869 = t5867 + t5868;
  t5874 = -1.*t5473*t5869;
  t5875 = t5874 + t5800;
  t5821 = t1020*t5500*t5473;
  t5822 = -1.*t1020*t5506*t5526;
  t5826 = t5821 + t5822;
  t5828 = -0.13205*t5826;
  t5833 = t5832*t5553;
  t5834 = t1114*t5832*t5574;
  t5835 = -1.*t5832*t5551*t5586;
  t5837 = -1.*t5560*t5832*t5551;
  t5838 = -1.*t1114*t5832*t5569;
  t5839 = t5837 + t5838;
  t5840 = 0.183*t5839;
  t5841 = t1114*t5560*t5832;
  t5842 = -1.*t5832*t5551*t5569;
  t5843 = t5841 + t5842;
  t5844 = -0.4*t5843;
  t5630 = t5560*t5628;
  t5894 = -1.*t1114*t5620;
  t5895 = -1.*t5499*t1020*t5551;
  t5896 = t5894 + t5895;
  t5871 = -1.*t5869*t5526;
  t5872 = t5795 + t5871;
  t5908 = t1020*t1114*t5442;
  t5909 = -1.*t5872*t5551;
  t5910 = t5908 + t5909;
  t5912 = -1.*t1114*t5872;
  t5913 = t5912 + t5558;
  t5926 = -1.*t1114*t5503;
  t5927 = -1.*t5826*t5551;
  t5928 = t5926 + t5927;
  t5930 = -1.*t1114*t5826;
  t5931 = t5503*t5551;
  t5932 = t5930 + t5931;
  t5633 = -1.*t5624*t5569;
  t5634 = t5630 + t5633;
  t5902 = -1.*t5628*t5569;
  t5943 = 0.183*t5560;
  t5944 = -0.2*t5569;
  t5945 = t5943 + t5944;
  t5947 = 0.2*t5560;
  t5948 = t5947 + t5573;
  t5915 = t5560*t5910;
  t5955 = t1114*t5872;
  t5956 = t1020*t5442*t5551;
  t5957 = t5955 + t5956;
  t5920 = -1.*t5910*t5569;
  t5934 = t5560*t5928;
  t5967 = t1114*t5826;
  t5968 = -1.*t5503*t5551;
  t5969 = t5967 + t5968;
  t5939 = -1.*t5928*t5569;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=-1.*t1020*t5438*t5442 + t5498*t5511 + t5523*t5538 - 0.13205*(t5523*t5526 + t5542) + t5550*t5553 + t5559*t5574 + 0.183*(-1.*t5559*t5569 + t5560*t5581) - 0.4*(t5559*t5560 + t5569*t5581) + t5581*t5586;
  p_output1[10]=t1020*t5438*t5499 + t5498*t5601 + t5538*t5609 - 0.13205*(t5473*t5601 + t5526*t5609) + t5553*t5620 + t5574*t5624 + t5586*t5628 - 0.4*(t5560*t5624 + t5569*t5628) + 0.183*t5634;
  p_output1[11]=0;
  p_output1[12]=-1.*t5438*t5499*t5503 + t1020*t5498*t5499*t5506 - 0.13205*(t1020*t5473*t5499*t5506 + t1020*t5499*t5500*t5526) + t1020*t5499*t5500*t5538 + t5553*t5655 + t5574*t5661 + t5586*t5670 + 0.183*(-1.*t5569*t5661 + t5560*t5670) - 0.4*(t5560*t5661 + t5569*t5670);
  p_output1[13]=-1.*t5438*t5442*t5503 + t1020*t5442*t5498*t5506 - 0.13205*(t1020*t5442*t5473*t5506 + t1020*t5442*t5500*t5526) + t1020*t5442*t5500*t5538 + t5553*t5693 + t5574*t5697 + t5586*t5701 + 0.183*(-1.*t5569*t5697 + t5560*t5701) - 0.4*(t5560*t5697 + t5569*t5701);
  p_output1[14]=-1.*t1020*t5438 - 1.*t5498*t5503*t5506 - 0.13205*(-1.*t5473*t5503*t5506 - 1.*t5500*t5503*t5526) - 1.*t5500*t5503*t5538 + t5553*t5725 + t5574*t5741 + t5586*t5747 + 0.183*(-1.*t5569*t5741 + t5560*t5747) - 0.4*(t5560*t5741 + t5569*t5747);
  p_output1[15]=t5498*t5609 + t5538*t5763 - 0.13205*(t5618 + t5526*t5763) + t5553*t5775 + t1114*t5574*t5775 - 1.*t5551*t5586*t5775 + 0.183*(-1.*t5551*t5560*t5775 - 1.*t1114*t5569*t5775) - 0.4*(t1114*t5560*t5775 - 1.*t5551*t5569*t5775);
  p_output1[16]=t5511*t5538 + t5498*t5790 - 0.13205*(t5511*t5526 + t5795) + t5553*t5801 + t1114*t5574*t5801 - 1.*t5551*t5586*t5801 + 0.183*(-1.*t5551*t5560*t5801 - 1.*t1114*t5569*t5801) - 0.4*(t1114*t5560*t5801 - 1.*t5551*t5569*t5801);
  p_output1[17]=t1020*t5498*t5500 - 1.*t1020*t5506*t5538 + t5828 + t5833 + t5834 + t5835 + t5840 + t5844;
  p_output1[18]=-0.047*t5526*t5601 + 0.047*t5473*t5609 - 0.13205*t5620 + t5553*t5850 + t1114*t5574*t5850 - 1.*t5551*t5586*t5850 + 0.183*(-1.*t5551*t5560*t5850 - 1.*t1114*t5569*t5850) - 0.4*(t1114*t5560*t5850 - 1.*t5551*t5569*t5850);
  p_output1[19]=0.047*t5473*t5790 - 0.047*t5526*t5869 - 0.13205*t5872 + t5553*t5875 + t1114*t5574*t5875 - 1.*t5551*t5586*t5875 + 0.183*(-1.*t5551*t5560*t5875 - 1.*t1114*t5569*t5875) - 0.4*(t1114*t5560*t5875 - 1.*t5551*t5569*t5875);
  p_output1[20]=0.047*t1020*t5473*t5500 - 0.047*t1020*t5506*t5526 + t5828 + t5833 + t5834 + t5835 + t5840 + t5844;
  p_output1[21]=0.183*t1020*t5499*t5551 + 0.183*t1114*t5620 + t5574*t5628 + t5586*t5896 - 0.4*(t5630 + t5569*t5896) + 0.183*(t5560*t5896 + t5902);
  p_output1[22]=0.183*t1020*t5442*t5551 + 0.183*t1114*t5872 + t5574*t5910 + t5586*t5913 - 0.4*(t5569*t5913 + t5915) + 0.183*(t5560*t5913 + t5920);
  p_output1[23]=-0.183*t5503*t5551 + 0.183*t1114*t5826 + t5574*t5928 + t5586*t5932 - 0.4*(t5569*t5932 + t5934) + 0.183*(t5560*t5932 + t5939);
  p_output1[24]=-0.4*t5634 + 0.183*(-1.*t5560*t5624 + t5902) + t5624*t5945 + t5628*t5948;
  p_output1[25]=t5910*t5948 + t5945*t5957 + 0.183*(t5920 - 1.*t5560*t5957) - 0.4*(t5915 - 1.*t5569*t5957);
  p_output1[26]=t5928*t5948 + t5945*t5969 + 0.183*(t5939 - 1.*t5560*t5969) - 0.4*(t5934 - 1.*t5569*t5969);
  p_output1[27]=0;
  p_output1[28]=0;
  p_output1[29]=0;
  p_output1[30]=0;
  p_output1[31]=0;
  p_output1[32]=0;
  p_output1[33]=0;
  p_output1[34]=0;
  p_output1[35]=0;
  p_output1[36]=0;
  p_output1[37]=0;
  p_output1[38]=0;
  p_output1[39]=0;
  p_output1[40]=0;
  p_output1[41]=0;
  p_output1[42]=0;
  p_output1[43]=0;
  p_output1[44]=0;
  p_output1[45]=0;
  p_output1[46]=0;
  p_output1[47]=0;
  p_output1[48]=0;
  p_output1[49]=0;
  p_output1[50]=0;
  p_output1[51]=0;
  p_output1[52]=0;
  p_output1[53]=0;
}



void Jp_FRfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
