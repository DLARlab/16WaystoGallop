/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:31 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FL_thigh_joint_src.h"

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
  double t1777;
  double t1797;
  double t1807;
  double t1799;
  double t1810;
  double t1760;
  double t1780;
  double t1787;
  double t1792;
  double t1827;
  double t1803;
  double t1811;
  double t1813;
  double t1831;
  double t1832;
  double t1833;
  double t1851;
  double t1852;
  double t1853;
  double t1855;
  double t1857;
  double t1859;
  double t1824;
  double t1834;
  double t1835;
  double t1854;
  double t1862;
  double t1863;
  double t1876;
  double t1878;
  double t1879;
  double t1884;
  double t1885;
  double t1886;
  double t1786;
  double t1836;
  double t1842;
  double t1918;
  double t1919;
  double t1920;
  double t1921;
  double t1923;
  double t1926;
  double t1929;
  double t1930;
  double t1931;
  double t1932;
  double t1887;
  double t1893;
  double t1898;
  double t1935;
  double t1936;
  double t1850;
  double t1866;
  double t1867;
  double t1899;
  double t1901;
  double t1902;
  double t1872;
  double t1880;
  double t1883;
  t1777 = Cos(var1[3]);
  t1797 = Cos(var1[5]);
  t1807 = Sin(var1[3]);
  t1799 = Sin(var1[4]);
  t1810 = Sin(var1[5]);
  t1760 = Cos(var1[10]);
  t1780 = Cos(var1[4]);
  t1787 = Sin(var1[10]);
  t1792 = Cos(var1[9]);
  t1827 = Sin(var1[9]);
  t1803 = t1777*t1797*t1799;
  t1811 = t1807*t1810;
  t1813 = t1803 + t1811;
  t1831 = -1.*t1797*t1807;
  t1832 = t1777*t1799*t1810;
  t1833 = t1831 + t1832;
  t1851 = t1797*t1807*t1799;
  t1852 = -1.*t1777*t1810;
  t1853 = t1851 + t1852;
  t1855 = t1777*t1797;
  t1857 = t1807*t1799*t1810;
  t1859 = t1855 + t1857;
  t1824 = t1792*t1813;
  t1834 = -1.*t1827*t1833;
  t1835 = t1824 + t1834;
  t1854 = t1792*t1853;
  t1862 = -1.*t1827*t1859;
  t1863 = t1854 + t1862;
  t1876 = t1792*t1780*t1797;
  t1878 = -1.*t1780*t1827*t1810;
  t1879 = t1876 + t1878;
  t1884 = t1827*t1813;
  t1885 = t1792*t1833;
  t1886 = t1884 + t1885;
  t1786 = t1760*t1777*t1780;
  t1836 = -1.*t1787*t1835;
  t1842 = t1786 + t1836;
  t1918 = -1.*t1760;
  t1919 = 1. + t1918;
  t1920 = 0.183*t1919;
  t1921 = 0. + t1920;
  t1923 = -0.047*t1827;
  t1926 = 0. + t1923;
  t1929 = -1.*t1792;
  t1930 = 1. + t1929;
  t1931 = 0.047*t1930;
  t1932 = 0. + t1931;
  t1887 = t1827*t1853;
  t1893 = t1792*t1859;
  t1898 = t1887 + t1893;
  t1935 = 0.183*t1787;
  t1936 = 0. + t1935;
  t1850 = t1760*t1780*t1807;
  t1866 = -1.*t1787*t1863;
  t1867 = t1850 + t1866;
  t1899 = t1780*t1797*t1827;
  t1901 = t1792*t1780*t1810;
  t1902 = t1899 + t1901;
  t1872 = -1.*t1760*t1799;
  t1880 = -1.*t1787*t1879;
  t1883 = t1872 + t1880;
  p_output1[0]=t1842;
  p_output1[1]=t1867;
  p_output1[2]=t1883;
  p_output1[3]=0.;
  p_output1[4]=t1886;
  p_output1[5]=t1898;
  p_output1[6]=t1902;
  p_output1[7]=0.;
  p_output1[8]=t1777*t1780*t1787 + t1760*t1835;
  p_output1[9]=t1780*t1787*t1807 + t1760*t1863;
  p_output1[10]=-1.*t1787*t1799 + t1760*t1879;
  p_output1[11]=0.;
  p_output1[12]=0. + 0.183*t1842 + 0.13205*t1886 + t1777*t1780*t1921 + t1813*t1926 + t1833*t1932 + t1835*t1936 + var1[0];
  p_output1[13]=0. + 0.183*t1867 + 0.13205*t1898 + t1780*t1807*t1921 + t1853*t1926 + t1859*t1932 + t1863*t1936 + var1[1];
  p_output1[14]=0. + 0.183*t1883 + 0.13205*t1902 - 1.*t1799*t1921 + t1780*t1797*t1926 + t1780*t1810*t1932 + t1879*t1936 + var1[2];
  p_output1[15]=1.;
}



void H_FL_thigh_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
