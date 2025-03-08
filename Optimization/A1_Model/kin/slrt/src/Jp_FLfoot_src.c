/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:04 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FLfoot_src.h"

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
  double t2817;
  double t6096;
  double t3788;
  double t4934;
  double t6102;
  double t3001;
  double t6055;
  double t6126;
  double t6127;
  double t6142;
  double t6160;
  double t6171;
  double t6172;
  double t2239;
  double t6189;
  double t5;
  double t6194;
  double t6208;
  double t6209;
  double t6212;
  double t6215;
  double t6216;
  double t6218;
  double t6221;
  double t6222;
  double t6223;
  double t6231;
  double t6232;
  double t6235;
  double t734;
  double t909;
  double t1864;
  double t1870;
  double t3008;
  double t3710;
  double t6152;
  double t6155;
  double t6156;
  double t6157;
  double t6258;
  double t6259;
  double t6260;
  double t6262;
  double t6263;
  double t6264;
  double t6192;
  double t6193;
  double t6217;
  double t6219;
  double t6220;
  double t6272;
  double t6274;
  double t6276;
  double t6227;
  double t6229;
  double t6230;
  double t6281;
  double t6282;
  double t6283;
  double t6285;
  double t6286;
  double t6287;
  double t6312;
  double t6313;
  double t6314;
  double t6316;
  double t6317;
  double t6318;
  double t6320;
  double t6321;
  double t6322;
  double t6347;
  double t6349;
  double t6350;
  double t6354;
  double t6355;
  double t6356;
  double t6358;
  double t6359;
  double t6360;
  double t6382;
  double t6383;
  double t6384;
  double t6386;
  double t6387;
  double t6388;
  double t6390;
  double t6391;
  double t6392;
  double t6420;
  double t6421;
  double t6422;
  double t6424;
  double t6425;
  double t6426;
  double t6447;
  double t6448;
  double t6449;
  double t6183;
  double t6452;
  double t6453;
  double t6478;
  double t6479;
  double t6481;
  double t6507;
  double t6508;
  double t6524;
  double t6525;
  double t6526;
  double t6528;
  double t6529;
  double t6459;
  double t6482;
  double t6483;
  double t6485;
  double t6486;
  double t6487;
  double t6488;
  double t6489;
  double t6490;
  double t6492;
  double t6493;
  double t6497;
  double t6498;
  double t6500;
  double t6501;
  double t6502;
  double t6552;
  double t6553;
  double t6554;
  double t6290;
  double t6535;
  double t6536;
  double t6567;
  double t6568;
  double t6570;
  double t6571;
  double t6572;
  double t6585;
  double t6586;
  double t6587;
  double t6589;
  double t6590;
  double t6591;
  double t6289;
  double t6291;
  double t6561;
  double t6602;
  double t6603;
  double t6604;
  double t6606;
  double t6607;
  double t6614;
  double t6615;
  double t6616;
  double t6575;
  double t6579;
  double t6626;
  double t6627;
  double t6628;
  double t6594;
  double t6598;
  t2817 = Sin(var1[3]);
  t6096 = Cos(var1[3]);
  t3788 = Cos(var1[5]);
  t4934 = Sin(var1[4]);
  t6102 = Sin(var1[5]);
  t3001 = Sin(var1[9]);
  t6055 = -1.*t3788*t2817*t4934;
  t6126 = t6096*t6102;
  t6127 = t6055 + t6126;
  t6142 = Cos(var1[9]);
  t6160 = -1.*t6096*t3788;
  t6171 = -1.*t2817*t4934*t6102;
  t6172 = t6160 + t6171;
  t2239 = Cos(var1[4]);
  t6189 = Sin(var1[10]);
  t5 = Cos(var1[10]);
  t6194 = t6142*t6127;
  t6208 = -1.*t3001*t6172;
  t6209 = t6194 + t6208;
  t6212 = Cos(var1[11]);
  t6215 = -1.*t6212;
  t6216 = 1. + t6215;
  t6218 = Sin(var1[11]);
  t6221 = -1.*t2239*t6189*t2817;
  t6222 = t5*t6209;
  t6223 = t6221 + t6222;
  t6231 = -1.*t5*t2239*t2817;
  t6232 = -1.*t6189*t6209;
  t6235 = t6231 + t6232;
  t734 = -1.*t5;
  t909 = 1. + t734;
  t1864 = 0.183*t909;
  t1870 = 0. + t1864;
  t3008 = -0.047*t3001;
  t3710 = 0. + t3008;
  t6152 = -1.*t6142;
  t6155 = 1. + t6152;
  t6156 = 0.047*t6155;
  t6157 = 0. + t6156;
  t6258 = t6096*t3788*t4934;
  t6259 = t2817*t6102;
  t6260 = t6258 + t6259;
  t6262 = -1.*t3788*t2817;
  t6263 = t6096*t4934*t6102;
  t6264 = t6262 + t6263;
  t6192 = 0.183*t6189;
  t6193 = 0. + t6192;
  t6217 = -0.2*t6216;
  t6219 = 0.183*t6218;
  t6220 = 0. + t6217 + t6219;
  t6272 = t6142*t6260;
  t6274 = -1.*t3001*t6264;
  t6276 = t6272 + t6274;
  t6227 = 0.183*t6216;
  t6229 = 0.2*t6218;
  t6230 = 0. + t6227 + t6229;
  t6281 = t6096*t2239*t6189;
  t6282 = t5*t6276;
  t6283 = t6281 + t6282;
  t6285 = t5*t6096*t2239;
  t6286 = -1.*t6189*t6276;
  t6287 = t6285 + t6286;
  t6312 = t6142*t6096*t2239*t3788;
  t6313 = -1.*t6096*t2239*t3001*t6102;
  t6314 = t6312 + t6313;
  t6316 = -1.*t6096*t6189*t4934;
  t6317 = t5*t6314;
  t6318 = t6316 + t6317;
  t6320 = -1.*t5*t6096*t4934;
  t6321 = -1.*t6189*t6314;
  t6322 = t6320 + t6321;
  t6347 = t6142*t2239*t3788*t2817;
  t6349 = -1.*t2239*t3001*t2817*t6102;
  t6350 = t6347 + t6349;
  t6354 = -1.*t6189*t2817*t4934;
  t6355 = t5*t6350;
  t6356 = t6354 + t6355;
  t6358 = -1.*t5*t2817*t4934;
  t6359 = -1.*t6189*t6350;
  t6360 = t6358 + t6359;
  t6382 = -1.*t6142*t3788*t4934;
  t6383 = t3001*t4934*t6102;
  t6384 = t6382 + t6383;
  t6386 = -1.*t2239*t6189;
  t6387 = t5*t6384;
  t6388 = t6386 + t6387;
  t6390 = -1.*t5*t2239;
  t6391 = -1.*t6189*t6384;
  t6392 = t6390 + t6391;
  t6420 = t3788*t2817;
  t6421 = -1.*t6096*t4934*t6102;
  t6422 = t6420 + t6421;
  t6424 = -1.*t3001*t6260;
  t6425 = t6142*t6422;
  t6426 = t6424 + t6425;
  t6447 = t3788*t2817*t4934;
  t6448 = -1.*t6096*t6102;
  t6449 = t6447 + t6448;
  t6183 = t6142*t6172;
  t6452 = -1.*t3001*t6449;
  t6453 = t6452 + t6183;
  t6478 = -1.*t2239*t3788*t3001;
  t6479 = -1.*t6142*t2239*t6102;
  t6481 = t6478 + t6479;
  t6507 = -1.*t6142*t6264;
  t6508 = t6424 + t6507;
  t6524 = t6096*t3788;
  t6525 = t2817*t4934*t6102;
  t6526 = t6524 + t6525;
  t6528 = -1.*t6142*t6526;
  t6529 = t6452 + t6528;
  t6459 = t6142*t6449;
  t6482 = t6193*t6481;
  t6483 = t5*t6220*t6481;
  t6485 = -1.*t6189*t6230*t6481;
  t6486 = t6142*t2239*t3788;
  t6487 = -1.*t2239*t3001*t6102;
  t6488 = t6486 + t6487;
  t6489 = 0.13205*t6488;
  t6490 = -1.*t6212*t6189*t6481;
  t6492 = -1.*t5*t6218*t6481;
  t6493 = t6490 + t6492;
  t6497 = 0.183*t6493;
  t6498 = t5*t6212*t6481;
  t6500 = -1.*t6189*t6218*t6481;
  t6501 = t6498 + t6500;
  t6502 = -0.4*t6501;
  t6552 = -1.*t6096*t2239*t6189;
  t6553 = -1.*t5*t6276;
  t6554 = t6552 + t6553;
  t6290 = t6212*t6287;
  t6535 = -1.*t3001*t6526;
  t6536 = t6459 + t6535;
  t6567 = -1.*t5*t6536;
  t6568 = t6221 + t6567;
  t6570 = t5*t2239*t2817;
  t6571 = -1.*t6189*t6536;
  t6572 = t6570 + t6571;
  t6585 = t6189*t4934;
  t6586 = -1.*t5*t6488;
  t6587 = t6585 + t6586;
  t6589 = -1.*t5*t4934;
  t6590 = -1.*t6189*t6488;
  t6591 = t6589 + t6590;
  t6289 = -1.*t6218*t6283;
  t6291 = t6289 + t6290;
  t6561 = -1.*t6218*t6287;
  t6602 = 0.183*t6212;
  t6603 = -0.2*t6218;
  t6604 = t6602 + t6603;
  t6606 = 0.2*t6212;
  t6607 = t6606 + t6219;
  t6614 = t2239*t6189*t2817;
  t6615 = t5*t6536;
  t6616 = t6614 + t6615;
  t6575 = t6212*t6572;
  t6579 = -1.*t6218*t6572;
  t6626 = -1.*t6189*t4934;
  t6627 = t5*t6488;
  t6628 = t6626 + t6627;
  t6594 = t6212*t6591;
  t6598 = -1.*t6218*t6591;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=-1.*t1870*t2239*t2817 + t3710*t6127 + t6157*t6172 + 0.13205*(t3001*t6127 + t6183) + t6193*t6209 + t6220*t6223 + t6230*t6235 + 0.183*(-1.*t6218*t6223 + t6212*t6235) - 0.4*(t6212*t6223 + t6218*t6235);
  p_output1[10]=t1870*t2239*t6096 + t3710*t6260 + t6157*t6264 + 0.13205*(t3001*t6260 + t6142*t6264) + t6193*t6276 + t6220*t6283 + t6230*t6287 - 0.4*(t6212*t6283 + t6218*t6287) + 0.183*t6291;
  p_output1[11]=0;
  p_output1[12]=t2239*t3710*t3788*t6096 - 1.*t1870*t4934*t6096 + 0.13205*(t2239*t3001*t3788*t6096 + t2239*t6096*t6102*t6142) + t2239*t6096*t6102*t6157 + t6193*t6314 + t6220*t6318 + t6230*t6322 + 0.183*(-1.*t6218*t6318 + t6212*t6322) - 0.4*(t6212*t6318 + t6218*t6322);
  p_output1[13]=t2239*t2817*t3710*t3788 - 1.*t1870*t2817*t4934 + 0.13205*(t2239*t2817*t3001*t3788 + t2239*t2817*t6102*t6142) + t2239*t2817*t6102*t6157 + t6193*t6350 + t6220*t6356 + t6230*t6360 + 0.183*(-1.*t6218*t6356 + t6212*t6360) - 0.4*(t6212*t6356 + t6218*t6360);
  p_output1[14]=-1.*t1870*t2239 - 1.*t3710*t3788*t4934 + 0.13205*(-1.*t3001*t3788*t4934 - 1.*t4934*t6102*t6142) - 1.*t4934*t6102*t6157 + t6193*t6384 + t6220*t6388 + t6230*t6392 + 0.183*(-1.*t6218*t6388 + t6212*t6392) - 0.4*(t6212*t6388 + t6218*t6392);
  p_output1[15]=t6157*t6260 + t3710*t6422 + 0.13205*(t6272 + t3001*t6422) + t6193*t6426 + t5*t6220*t6426 - 1.*t6189*t6230*t6426 + 0.183*(-1.*t6189*t6212*t6426 - 1.*t5*t6218*t6426) - 0.4*(t5*t6212*t6426 - 1.*t6189*t6218*t6426);
  p_output1[16]=t3710*t6172 + t6157*t6449 + t6193*t6453 + t5*t6220*t6453 - 1.*t6189*t6230*t6453 + 0.183*(-1.*t6189*t6212*t6453 - 1.*t5*t6218*t6453) - 0.4*(t5*t6212*t6453 - 1.*t6189*t6218*t6453) + 0.13205*(t3001*t6172 + t6459);
  p_output1[17]=-1.*t2239*t3710*t6102 + t2239*t3788*t6157 + t6482 + t6483 + t6485 + t6489 + t6497 + t6502;
  p_output1[18]=0;
  p_output1[19]=0;
  p_output1[20]=0;
  p_output1[21]=0;
  p_output1[22]=0;
  p_output1[23]=0;
  p_output1[24]=0;
  p_output1[25]=0;
  p_output1[26]=0;
  p_output1[27]=-0.047*t6142*t6260 + 0.047*t3001*t6264 + 0.13205*t6276 + t6193*t6508 + t5*t6220*t6508 - 1.*t6189*t6230*t6508 + 0.183*(-1.*t6189*t6212*t6508 - 1.*t5*t6218*t6508) - 0.4*(t5*t6212*t6508 - 1.*t6189*t6218*t6508);
  p_output1[28]=-0.047*t6142*t6449 + 0.047*t3001*t6526 + t6193*t6529 + t5*t6220*t6529 - 1.*t6189*t6230*t6529 + 0.183*(-1.*t6189*t6212*t6529 - 1.*t5*t6218*t6529) - 0.4*(t5*t6212*t6529 - 1.*t6189*t6218*t6529) + 0.13205*t6536;
  p_output1[29]=0.047*t2239*t3001*t6102 - 0.047*t2239*t3788*t6142 + t6482 + t6483 + t6485 + t6489 + t6497 + t6502;
  p_output1[30]=0.183*t2239*t6096*t6189 + 0.183*t5*t6276 + t6220*t6287 + t6230*t6554 - 0.4*(t6290 + t6218*t6554) + 0.183*(t6212*t6554 + t6561);
  p_output1[31]=0.183*t2239*t2817*t6189 + 0.183*t5*t6536 + t6230*t6568 + t6220*t6572 - 0.4*(t6218*t6568 + t6575) + 0.183*(t6212*t6568 + t6579);
  p_output1[32]=-0.183*t4934*t6189 + 0.183*t5*t6488 + t6230*t6587 + t6220*t6591 - 0.4*(t6218*t6587 + t6594) + 0.183*(t6212*t6587 + t6598);
  p_output1[33]=-0.4*t6291 + 0.183*(-1.*t6212*t6283 + t6561) + t6283*t6604 + t6287*t6607;
  p_output1[34]=t6572*t6607 + t6604*t6616 + 0.183*(t6579 - 1.*t6212*t6616) - 0.4*(t6575 - 1.*t6218*t6616);
  p_output1[35]=t6591*t6607 + t6604*t6628 + 0.183*(t6598 - 1.*t6212*t6628) - 0.4*(t6594 - 1.*t6218*t6628);
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



void Jp_FLfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
