/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:30 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FL_thigh_joint_src.h"

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
  double t1707;
  double t1724;
  double t1720;
  double t1722;
  double t1729;
  double t1714;
  double t1723;
  double t1730;
  double t1731;
  double t1733;
  double t1739;
  double t1741;
  double t1745;
  double t1662;
  double t1699;
  double t1753;
  double t1756;
  double t1757;
  double t1758;
  double t1674;
  double t1683;
  double t1697;
  double t1698;
  double t1715;
  double t1716;
  double t1734;
  double t1735;
  double t1736;
  double t1737;
  double t1766;
  double t1769;
  double t1770;
  double t1772;
  double t1773;
  double t1774;
  double t1754;
  double t1755;
  double t1780;
  double t1781;
  double t1782;
  double t1799;
  double t1801;
  double t1802;
  double t1818;
  double t1819;
  double t1820;
  double t1839;
  double t1842;
  double t1846;
  double t1855;
  double t1856;
  double t1857;
  double t1861;
  double t1862;
  double t1863;
  double t1872;
  double t1873;
  double t1874;
  double t1748;
  double t1877;
  double t1878;
  double t1888;
  double t1889;
  double t1890;
  double t1900;
  double t1901;
  double t1907;
  double t1908;
  double t1909;
  double t1911;
  double t1912;
  double t1881;
  double t1891;
  double t1892;
  double t1893;
  double t1894;
  double t1895;
  double t1896;
  double t1915;
  double t1916;
  t1707 = Sin(var1[3]);
  t1724 = Cos(var1[3]);
  t1720 = Cos(var1[5]);
  t1722 = Sin(var1[4]);
  t1729 = Sin(var1[5]);
  t1714 = Sin(var1[9]);
  t1723 = -1.*t1720*t1707*t1722;
  t1730 = t1724*t1729;
  t1731 = t1723 + t1730;
  t1733 = Cos(var1[9]);
  t1739 = -1.*t1724*t1720;
  t1741 = -1.*t1707*t1722*t1729;
  t1745 = t1739 + t1741;
  t1662 = Cos(var1[10]);
  t1699 = Cos(var1[4]);
  t1753 = Sin(var1[10]);
  t1756 = t1733*t1731;
  t1757 = -1.*t1714*t1745;
  t1758 = t1756 + t1757;
  t1674 = -1.*t1662;
  t1683 = 1. + t1674;
  t1697 = 0.183*t1683;
  t1698 = 0. + t1697;
  t1715 = -0.047*t1714;
  t1716 = 0. + t1715;
  t1734 = -1.*t1733;
  t1735 = 1. + t1734;
  t1736 = 0.047*t1735;
  t1737 = 0. + t1736;
  t1766 = t1724*t1720*t1722;
  t1769 = t1707*t1729;
  t1770 = t1766 + t1769;
  t1772 = -1.*t1720*t1707;
  t1773 = t1724*t1722*t1729;
  t1774 = t1772 + t1773;
  t1754 = 0.183*t1753;
  t1755 = 0. + t1754;
  t1780 = t1733*t1770;
  t1781 = -1.*t1714*t1774;
  t1782 = t1780 + t1781;
  t1799 = t1733*t1724*t1699*t1720;
  t1801 = -1.*t1724*t1699*t1714*t1729;
  t1802 = t1799 + t1801;
  t1818 = t1733*t1699*t1720*t1707;
  t1819 = -1.*t1699*t1714*t1707*t1729;
  t1820 = t1818 + t1819;
  t1839 = -1.*t1733*t1720*t1722;
  t1842 = t1714*t1722*t1729;
  t1846 = t1839 + t1842;
  t1855 = t1720*t1707;
  t1856 = -1.*t1724*t1722*t1729;
  t1857 = t1855 + t1856;
  t1861 = -1.*t1714*t1770;
  t1862 = t1733*t1857;
  t1863 = t1861 + t1862;
  t1872 = t1720*t1707*t1722;
  t1873 = -1.*t1724*t1729;
  t1874 = t1872 + t1873;
  t1748 = t1733*t1745;
  t1877 = -1.*t1714*t1874;
  t1878 = t1877 + t1748;
  t1888 = -1.*t1699*t1720*t1714;
  t1889 = -1.*t1733*t1699*t1729;
  t1890 = t1888 + t1889;
  t1900 = -1.*t1733*t1774;
  t1901 = t1861 + t1900;
  t1907 = t1724*t1720;
  t1908 = t1707*t1722*t1729;
  t1909 = t1907 + t1908;
  t1911 = -1.*t1733*t1909;
  t1912 = t1877 + t1911;
  t1881 = t1733*t1874;
  t1891 = t1755*t1890;
  t1892 = -0.183*t1753*t1890;
  t1893 = t1733*t1699*t1720;
  t1894 = -1.*t1699*t1714*t1729;
  t1895 = t1893 + t1894;
  t1896 = 0.13205*t1895;
  t1915 = -1.*t1714*t1909;
  t1916 = t1881 + t1915;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=-1.*t1698*t1699*t1707 + t1716*t1731 + t1737*t1745 + 0.13205*(t1714*t1731 + t1748) + t1755*t1758 + 0.183*(-1.*t1662*t1699*t1707 - 1.*t1753*t1758);
  p_output1[10]=t1698*t1699*t1724 + t1716*t1770 + t1737*t1774 + 0.13205*(t1714*t1770 + t1733*t1774) + t1755*t1782 + 0.183*(t1662*t1699*t1724 - 1.*t1753*t1782);
  p_output1[11]=0;
  p_output1[12]=t1699*t1716*t1720*t1724 - 1.*t1698*t1722*t1724 + 0.13205*(t1699*t1714*t1720*t1724 + t1699*t1724*t1729*t1733) + t1699*t1724*t1729*t1737 + t1755*t1802 + 0.183*(-1.*t1662*t1722*t1724 - 1.*t1753*t1802);
  p_output1[13]=t1699*t1707*t1716*t1720 - 1.*t1698*t1707*t1722 + 0.13205*(t1699*t1707*t1714*t1720 + t1699*t1707*t1729*t1733) + t1699*t1707*t1729*t1737 + t1755*t1820 + 0.183*(-1.*t1662*t1707*t1722 - 1.*t1753*t1820);
  p_output1[14]=-1.*t1698*t1699 - 1.*t1716*t1720*t1722 + 0.13205*(-1.*t1714*t1720*t1722 - 1.*t1722*t1729*t1733) - 1.*t1722*t1729*t1737 + t1755*t1846 + 0.183*(-1.*t1662*t1699 - 1.*t1753*t1846);
  p_output1[15]=t1737*t1770 + t1716*t1857 + 0.13205*(t1780 + t1714*t1857) - 0.183*t1753*t1863 + t1755*t1863;
  p_output1[16]=t1716*t1745 + t1737*t1874 - 0.183*t1753*t1878 + t1755*t1878 + 0.13205*(t1714*t1745 + t1881);
  p_output1[17]=-1.*t1699*t1716*t1729 + t1699*t1720*t1737 + t1891 + t1892 + t1896;
  p_output1[18]=0;
  p_output1[19]=0;
  p_output1[20]=0;
  p_output1[21]=0;
  p_output1[22]=0;
  p_output1[23]=0;
  p_output1[24]=0;
  p_output1[25]=0;
  p_output1[26]=0;
  p_output1[27]=-0.047*t1733*t1770 + 0.047*t1714*t1774 + 0.13205*t1782 - 0.183*t1753*t1901 + t1755*t1901;
  p_output1[28]=-0.047*t1733*t1874 + 0.047*t1714*t1909 - 0.183*t1753*t1912 + t1755*t1912 + 0.13205*t1916;
  p_output1[29]=0.047*t1699*t1714*t1729 - 0.047*t1699*t1720*t1733 + t1891 + t1892 + t1896;
  p_output1[30]=0.183*t1699*t1724*t1753 + 0.183*t1662*t1782 + 0.183*(-1.*t1699*t1724*t1753 - 1.*t1662*t1782);
  p_output1[31]=0.183*t1699*t1707*t1753 + 0.183*t1662*t1916 + 0.183*(-1.*t1699*t1707*t1753 - 1.*t1662*t1916);
  p_output1[32]=-0.183*t1722*t1753 + 0.183*t1662*t1895 + 0.183*(t1722*t1753 - 1.*t1662*t1895);
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



void Jp_FL_thigh_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
