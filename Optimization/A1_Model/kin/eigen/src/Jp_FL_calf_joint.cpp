/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:01 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FL_calf_joint.h"

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
static void output1(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t224;
  double t482;
  double t321;
  double t452;
  double t610;
  double t311;
  double t481;
  double t614;
  double t615;
  double t724;
  double t1015;
  double t1039;
  double t1087;
  double t204;
  double t2211;
  double t78;
  double t2449;
  double t2454;
  double t2601;
  double t2868;
  double t2876;
  double t2879;
  double t2923;
  double t2938;
  double t2939;
  double t2940;
  double t2946;
  double t2947;
  double t2950;
  double t153;
  double t159;
  double t171;
  double t180;
  double t312;
  double t320;
  double t725;
  double t934;
  double t950;
  double t1006;
  double t3751;
  double t4043;
  double t4070;
  double t4176;
  double t4181;
  double t4183;
  double t2242;
  double t2440;
  double t2903;
  double t2929;
  double t2937;
  double t4586;
  double t4589;
  double t4729;
  double t2942;
  double t2944;
  double t2945;
  double t4786;
  double t4800;
  double t4845;
  double t4857;
  double t4875;
  double t4943;
  double t8532;
  double t8542;
  double t8567;
  double t8573;
  double t8575;
  double t8579;
  double t8584;
  double t8590;
  double t8591;
  double t8631;
  double t8632;
  double t8635;
  double t8637;
  double t8639;
  double t8640;
  double t8645;
  double t8646;
  double t8647;
  double t8679;
  double t8680;
  double t8681;
  double t8683;
  double t8684;
  double t8685;
  double t8687;
  double t8688;
  double t8689;
  double t8708;
  double t8709;
  double t8710;
  double t8712;
  double t8713;
  double t8714;
  double t8737;
  double t8738;
  double t8739;
  double t1849;
  double t8742;
  double t8743;
  double t8765;
  double t8766;
  double t8767;
  double t8790;
  double t8791;
  double t8822;
  double t8823;
  double t8824;
  double t8826;
  double t8827;
  double t8748;
  double t8770;
  double t8771;
  double t8774;
  double t8775;
  double t8776;
  double t8777;
  double t8778;
  double t8779;
  double t8780;
  double t8781;
  double t8782;
  double t8783;
  double t8784;
  double t8785;
  double t8786;
  double t8853;
  double t8854;
  double t8857;
  double t5341;
  double t8832;
  double t8835;
  double t8872;
  double t8873;
  double t8877;
  double t8879;
  double t8880;
  double t8899;
  double t8900;
  double t8902;
  double t8904;
  double t8905;
  double t8906;
  double t5100;
  double t5404;
  double t8865;
  double t8917;
  double t8918;
  double t8920;
  double t8922;
  double t8923;
  double t8932;
  double t8933;
  double t8934;
  double t8884;
  double t8888;
  double t8944;
  double t8945;
  double t8946;
  double t8909;
  double t8913;
  t224 = Sin(var1[3]);
  t482 = Cos(var1[3]);
  t321 = Cos(var1[5]);
  t452 = Sin(var1[4]);
  t610 = Sin(var1[5]);
  t311 = Sin(var1[9]);
  t481 = -1.*t321*t224*t452;
  t614 = t482*t610;
  t615 = t481 + t614;
  t724 = Cos(var1[9]);
  t1015 = -1.*t482*t321;
  t1039 = -1.*t224*t452*t610;
  t1087 = t1015 + t1039;
  t204 = Cos(var1[4]);
  t2211 = Sin(var1[10]);
  t78 = Cos(var1[10]);
  t2449 = t724*t615;
  t2454 = -1.*t311*t1087;
  t2601 = t2449 + t2454;
  t2868 = Cos(var1[11]);
  t2876 = -1.*t2868;
  t2879 = 1. + t2876;
  t2923 = Sin(var1[11]);
  t2938 = -1.*t204*t2211*t224;
  t2939 = t78*t2601;
  t2940 = t2938 + t2939;
  t2946 = -1.*t78*t204*t224;
  t2947 = -1.*t2211*t2601;
  t2950 = t2946 + t2947;
  t153 = -1.*t78;
  t159 = 1. + t153;
  t171 = 0.183*t159;
  t180 = 0. + t171;
  t312 = -0.047*t311;
  t320 = 0. + t312;
  t725 = -1.*t724;
  t934 = 1. + t725;
  t950 = 0.047*t934;
  t1006 = 0. + t950;
  t3751 = t482*t321*t452;
  t4043 = t224*t610;
  t4070 = t3751 + t4043;
  t4176 = -1.*t321*t224;
  t4181 = t482*t452*t610;
  t4183 = t4176 + t4181;
  t2242 = 0.183*t2211;
  t2440 = 0. + t2242;
  t2903 = -0.2*t2879;
  t2929 = 0.183*t2923;
  t2937 = 0. + t2903 + t2929;
  t4586 = t724*t4070;
  t4589 = -1.*t311*t4183;
  t4729 = t4586 + t4589;
  t2942 = 0.183*t2879;
  t2944 = 0.2*t2923;
  t2945 = 0. + t2942 + t2944;
  t4786 = t482*t204*t2211;
  t4800 = t78*t4729;
  t4845 = t4786 + t4800;
  t4857 = t78*t482*t204;
  t4875 = -1.*t2211*t4729;
  t4943 = t4857 + t4875;
  t8532 = t724*t482*t204*t321;
  t8542 = -1.*t482*t204*t311*t610;
  t8567 = t8532 + t8542;
  t8573 = -1.*t482*t2211*t452;
  t8575 = t78*t8567;
  t8579 = t8573 + t8575;
  t8584 = -1.*t78*t482*t452;
  t8590 = -1.*t2211*t8567;
  t8591 = t8584 + t8590;
  t8631 = t724*t204*t321*t224;
  t8632 = -1.*t204*t311*t224*t610;
  t8635 = t8631 + t8632;
  t8637 = -1.*t2211*t224*t452;
  t8639 = t78*t8635;
  t8640 = t8637 + t8639;
  t8645 = -1.*t78*t224*t452;
  t8646 = -1.*t2211*t8635;
  t8647 = t8645 + t8646;
  t8679 = -1.*t724*t321*t452;
  t8680 = t311*t452*t610;
  t8681 = t8679 + t8680;
  t8683 = -1.*t204*t2211;
  t8684 = t78*t8681;
  t8685 = t8683 + t8684;
  t8687 = -1.*t78*t204;
  t8688 = -1.*t2211*t8681;
  t8689 = t8687 + t8688;
  t8708 = t321*t224;
  t8709 = -1.*t482*t452*t610;
  t8710 = t8708 + t8709;
  t8712 = -1.*t311*t4070;
  t8713 = t724*t8710;
  t8714 = t8712 + t8713;
  t8737 = t321*t224*t452;
  t8738 = -1.*t482*t610;
  t8739 = t8737 + t8738;
  t1849 = t724*t1087;
  t8742 = -1.*t311*t8739;
  t8743 = t8742 + t1849;
  t8765 = -1.*t204*t321*t311;
  t8766 = -1.*t724*t204*t610;
  t8767 = t8765 + t8766;
  t8790 = -1.*t724*t4183;
  t8791 = t8712 + t8790;
  t8822 = t482*t321;
  t8823 = t224*t452*t610;
  t8824 = t8822 + t8823;
  t8826 = -1.*t724*t8824;
  t8827 = t8742 + t8826;
  t8748 = t724*t8739;
  t8770 = t2440*t8767;
  t8771 = t78*t2937*t8767;
  t8774 = -1.*t2211*t2945*t8767;
  t8775 = t724*t204*t321;
  t8776 = -1.*t204*t311*t610;
  t8777 = t8775 + t8776;
  t8778 = 0.13205*t8777;
  t8779 = -1.*t2868*t2211*t8767;
  t8780 = -1.*t78*t2923*t8767;
  t8781 = t8779 + t8780;
  t8782 = 0.183*t8781;
  t8783 = t78*t2868*t8767;
  t8784 = -1.*t2211*t2923*t8767;
  t8785 = t8783 + t8784;
  t8786 = -0.2*t8785;
  t8853 = -1.*t482*t204*t2211;
  t8854 = -1.*t78*t4729;
  t8857 = t8853 + t8854;
  t5341 = t2868*t4943;
  t8832 = -1.*t311*t8824;
  t8835 = t8748 + t8832;
  t8872 = -1.*t78*t8835;
  t8873 = t2938 + t8872;
  t8877 = t78*t204*t224;
  t8879 = -1.*t2211*t8835;
  t8880 = t8877 + t8879;
  t8899 = t2211*t452;
  t8900 = -1.*t78*t8777;
  t8902 = t8899 + t8900;
  t8904 = -1.*t78*t452;
  t8905 = -1.*t2211*t8777;
  t8906 = t8904 + t8905;
  t5100 = -1.*t2923*t4845;
  t5404 = t5100 + t5341;
  t8865 = -1.*t2923*t4943;
  t8917 = 0.183*t2868;
  t8918 = -0.2*t2923;
  t8920 = t8917 + t8918;
  t8922 = 0.2*t2868;
  t8923 = t8922 + t2929;
  t8932 = t204*t2211*t224;
  t8933 = t78*t8835;
  t8934 = t8932 + t8933;
  t8884 = t2868*t8880;
  t8888 = -1.*t2923*t8880;
  t8944 = -1.*t2211*t452;
  t8945 = t78*t8777;
  t8946 = t8944 + t8945;
  t8909 = t2868*t8906;
  t8913 = -1.*t2923*t8906;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1006*t1087 - 1.*t180*t204*t224 + t2440*t2601 + t2937*t2940 + t2945*t2950 + 0.183*(-1.*t2923*t2940 + t2868*t2950) - 0.2*(t2868*t2940 + t2923*t2950) + t320*t615 + 0.13205*(t1849 + t311*t615);
  p_output1(10)=t320*t4070 + t1006*t4183 + t2440*t4729 + t180*t204*t482 + t2937*t4845 + t2945*t4943 - 0.2*(t2868*t4845 + t2923*t4943) + 0.183*t5404 + 0.13205*(t311*t4070 + t4183*t724);
  p_output1(11)=0;
  p_output1(12)=t204*t320*t321*t482 - 1.*t180*t452*t482 + t1006*t204*t482*t610 + 0.13205*(t204*t311*t321*t482 + t204*t482*t610*t724) + t2440*t8567 + t2937*t8579 + t2945*t8591 + 0.183*(-1.*t2923*t8579 + t2868*t8591) - 0.2*(t2868*t8579 + t2923*t8591);
  p_output1(13)=t204*t224*t320*t321 - 1.*t180*t224*t452 + t1006*t204*t224*t610 + 0.13205*(t204*t224*t311*t321 + t204*t224*t610*t724) + t2440*t8635 + t2937*t8640 + t2945*t8647 + 0.183*(-1.*t2923*t8640 + t2868*t8647) - 0.2*(t2868*t8640 + t2923*t8647);
  p_output1(14)=-1.*t180*t204 - 1.*t320*t321*t452 - 1.*t1006*t452*t610 + 0.13205*(-1.*t311*t321*t452 - 1.*t452*t610*t724) + t2440*t8681 + t2937*t8685 + t2945*t8689 + 0.183*(-1.*t2923*t8685 + t2868*t8689) - 0.2*(t2868*t8685 + t2923*t8689);
  p_output1(15)=t1006*t4070 + t320*t8710 + 0.13205*(t4586 + t311*t8710) + t2440*t8714 - 1.*t2211*t2945*t8714 + t2937*t78*t8714 - 0.2*(-1.*t2211*t2923*t8714 + t2868*t78*t8714) + 0.183*(-1.*t2211*t2868*t8714 - 1.*t2923*t78*t8714);
  p_output1(16)=t1087*t320 + t1006*t8739 + t2440*t8743 - 1.*t2211*t2945*t8743 + t2937*t78*t8743 - 0.2*(-1.*t2211*t2923*t8743 + t2868*t78*t8743) + 0.183*(-1.*t2211*t2868*t8743 - 1.*t2923*t78*t8743) + 0.13205*(t1087*t311 + t8748);
  p_output1(17)=t1006*t204*t321 - 1.*t204*t320*t610 + t8770 + t8771 + t8774 + t8778 + t8782 + t8786;
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0.047*t311*t4183 + 0.13205*t4729 - 0.047*t4070*t724 + t2440*t8791 - 1.*t2211*t2945*t8791 + t2937*t78*t8791 - 0.2*(-1.*t2211*t2923*t8791 + t2868*t78*t8791) + 0.183*(-1.*t2211*t2868*t8791 - 1.*t2923*t78*t8791);
  p_output1(28)=-0.047*t724*t8739 + 0.047*t311*t8824 + t2440*t8827 - 1.*t2211*t2945*t8827 + t2937*t78*t8827 - 0.2*(-1.*t2211*t2923*t8827 + t2868*t78*t8827) + 0.183*(-1.*t2211*t2868*t8827 - 1.*t2923*t78*t8827) + 0.13205*t8835;
  p_output1(29)=0.047*t204*t311*t610 - 0.047*t204*t321*t724 + t8770 + t8771 + t8774 + t8778 + t8782 + t8786;
  p_output1(30)=0.183*t204*t2211*t482 + t2937*t4943 + 0.183*t4729*t78 + t2945*t8857 - 0.2*(t5341 + t2923*t8857) + 0.183*(t2868*t8857 + t8865);
  p_output1(31)=0.183*t204*t2211*t224 + 0.183*t78*t8835 + t2945*t8873 + t2937*t8880 - 0.2*(t2923*t8873 + t8884) + 0.183*(t2868*t8873 + t8888);
  p_output1(32)=-0.183*t2211*t452 + 0.183*t78*t8777 + t2945*t8902 + t2937*t8906 - 0.2*(t2923*t8902 + t8909) + 0.183*(t2868*t8902 + t8913);
  p_output1(33)=-0.2*t5404 + 0.183*(-1.*t2868*t4845 + t8865) + t4845*t8920 + t4943*t8923;
  p_output1(34)=t8880*t8923 + t8920*t8934 + 0.183*(t8888 - 1.*t2868*t8934) - 0.2*(t8884 - 1.*t2923*t8934);
  p_output1(35)=t8906*t8923 + t8920*t8946 + 0.183*(t8913 - 1.*t2868*t8946) - 0.2*(t8909 - 1.*t2923*t8946);
  p_output1(36)=0;
  p_output1(37)=0;
  p_output1(38)=0;
  p_output1(39)=0;
  p_output1(40)=0;
  p_output1(41)=0;
  p_output1(42)=0;
  p_output1(43)=0;
  p_output1(44)=0;
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
  p_output1(48)=0;
  p_output1(49)=0;
  p_output1(50)=0;
  p_output1(51)=0;
  p_output1(52)=0;
  p_output1(53)=0;
}


       
Eigen::Matrix<double,3,18> Jp_FL_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void Jp_FL_calf_joint(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



