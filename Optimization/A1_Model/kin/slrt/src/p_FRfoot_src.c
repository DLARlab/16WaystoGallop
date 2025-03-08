/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:00 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FRfoot_src.h"

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
  double t487;
  double t5405;
  double t5408;
  double t5406;
  double t5409;
  double t5399;
  double t5407;
  double t5410;
  double t5414;
  double t5416;
  double t5417;
  double t5418;
  double t5423;
  double t5372;
  double t5434;
  double t5435;
  double t5436;
  double t748;
  double t5437;
  double t5444;
  double t5445;
  double t5446;
  double t5449;
  double t5454;
  double t5455;
  double t5456;
  double t5441;
  double t5442;
  double t5443;
  double t5382;
  double t5389;
  double t5390;
  double t5396;
  double t5400;
  double t5401;
  double t5403;
  double t5404;
  double t5425;
  double t5426;
  double t5476;
  double t5477;
  double t5478;
  double t5480;
  double t5481;
  double t5482;
  double t5438;
  double t5439;
  double t5492;
  double t5496;
  double t5497;
  double t5448;
  double t5451;
  double t5452;
  double t5457;
  double t5460;
  double t5461;
  double t5503;
  double t5504;
  double t5505;
  double t5499;
  double t5500;
  double t5501;
  double t5523;
  double t5524;
  double t5525;
  double t5531;
  double t5532;
  double t5533;
  double t5527;
  double t5528;
  double t5529;
  t487 = Cos(var1[3]);
  t5405 = Cos(var1[5]);
  t5408 = Sin(var1[4]);
  t5406 = Sin(var1[3]);
  t5409 = Sin(var1[5]);
  t5399 = Cos(var1[6]);
  t5407 = -1.*t5405*t5406;
  t5410 = t487*t5408*t5409;
  t5414 = t5407 + t5410;
  t5416 = t487*t5405*t5408;
  t5417 = t5406*t5409;
  t5418 = t5416 + t5417;
  t5423 = Sin(var1[6]);
  t5372 = Cos(var1[7]);
  t5434 = t5399*t5418;
  t5435 = -1.*t5414*t5423;
  t5436 = t5434 + t5435;
  t748 = Cos(var1[4]);
  t5437 = Sin(var1[7]);
  t5444 = Cos(var1[8]);
  t5445 = -1.*t5444;
  t5446 = 1. + t5445;
  t5449 = Sin(var1[8]);
  t5454 = t487*t748*t5372;
  t5455 = -1.*t5436*t5437;
  t5456 = t5454 + t5455;
  t5441 = t5372*t5436;
  t5442 = t487*t748*t5437;
  t5443 = t5441 + t5442;
  t5382 = -1.*t5372;
  t5389 = 1. + t5382;
  t5390 = 0.183*t5389;
  t5396 = 0. + t5390;
  t5400 = -1.*t5399;
  t5401 = 1. + t5400;
  t5403 = -0.047*t5401;
  t5404 = 0. + t5403;
  t5425 = 0.047*t5423;
  t5426 = 0. + t5425;
  t5476 = t487*t5405;
  t5477 = t5406*t5408*t5409;
  t5478 = t5476 + t5477;
  t5480 = t5405*t5406*t5408;
  t5481 = -1.*t487*t5409;
  t5482 = t5480 + t5481;
  t5438 = 0.183*t5437;
  t5439 = 0. + t5438;
  t5492 = t5399*t5482;
  t5496 = -1.*t5478*t5423;
  t5497 = t5492 + t5496;
  t5448 = -0.2*t5446;
  t5451 = 0.183*t5449;
  t5452 = 0. + t5448 + t5451;
  t5457 = 0.183*t5446;
  t5460 = 0.2*t5449;
  t5461 = 0. + t5457 + t5460;
  t5503 = t748*t5372*t5406;
  t5504 = -1.*t5497*t5437;
  t5505 = t5503 + t5504;
  t5499 = t5372*t5497;
  t5500 = t748*t5406*t5437;
  t5501 = t5499 + t5500;
  t5523 = t748*t5405*t5399;
  t5524 = -1.*t748*t5409*t5423;
  t5525 = t5523 + t5524;
  t5531 = -1.*t5372*t5408;
  t5532 = -1.*t5525*t5437;
  t5533 = t5531 + t5532;
  t5527 = t5372*t5525;
  t5528 = -1.*t5408*t5437;
  t5529 = t5527 + t5528;
  p_output1[0]=0. + t5404*t5414 - 0.13205*(t5399*t5414 + t5418*t5423) + t5418*t5426 + t5436*t5439 + t5443*t5452 + 0.183*(-1.*t5443*t5449 + t5444*t5456) - 0.4*(t5443*t5444 + t5449*t5456) + t5456*t5461 + t487*t5396*t748 + var1[0];
  p_output1[1]=0. + t5404*t5478 + t5426*t5482 - 0.13205*(t5399*t5478 + t5423*t5482) + t5439*t5497 + t5452*t5501 + t5461*t5505 + 0.183*(-1.*t5449*t5501 + t5444*t5505) - 0.4*(t5444*t5501 + t5449*t5505) + t5396*t5406*t748 + var1[1];
  p_output1[2]=0. - 1.*t5396*t5408 + t5439*t5525 + t5452*t5529 + t5461*t5533 + 0.183*(-1.*t5449*t5529 + t5444*t5533) - 0.4*(t5444*t5529 + t5449*t5533) + t5404*t5409*t748 + t5405*t5426*t748 - 0.13205*(t5399*t5409*t748 + t5405*t5423*t748) + var1[2];
}



void p_FRfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
