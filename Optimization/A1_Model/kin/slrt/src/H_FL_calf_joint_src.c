/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:34 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FL_calf_joint_src.h"

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
  double t2250;
  double t2302;
  double t2306;
  double t2304;
  double t2307;
  double t2277;
  double t2256;
  double t2258;
  double t2294;
  double t2305;
  double t2308;
  double t2316;
  double t2321;
  double t2323;
  double t2328;
  double t2336;
  double t2340;
  double t2344;
  double t2345;
  double t2228;
  double t2371;
  double t2384;
  double t2392;
  double t2402;
  double t2407;
  double t2409;
  double t2413;
  double t2414;
  double t2415;
  double t2416;
  double t2437;
  double t2445;
  double t2449;
  double t2260;
  double t2347;
  double t2362;
  double t2372;
  double t2373;
  double t2374;
  double t2377;
  double t2418;
  double t2422;
  double t2427;
  double t2429;
  double t2432;
  double t2436;
  double t2457;
  double t2458;
  double t2462;
  double t2463;
  double t2467;
  double t2471;
  double t2472;
  double t2473;
  double t2546;
  double t2547;
  double t2366;
  double t2375;
  double t2376;
  double t2503;
  double t2508;
  double t2512;
  double t2521;
  double t2522;
  double t2523;
  double t2524;
  double t2530;
  double t2531;
  double t2537;
  double t2538;
  double t2539;
  double t2540;
  double t2474;
  double t2476;
  double t2477;
  double t2543;
  double t2544;
  double t2548;
  double t2551;
  double t2552;
  double t2554;
  double t2557;
  double t2559;
  double t2423;
  double t2434;
  double t2435;
  double t2513;
  double t2514;
  double t2516;
  double t2489;
  double t2497;
  double t2501;
  double t2459;
  double t2469;
  double t2470;
  double t2518;
  double t2519;
  double t2520;
  t2250 = Cos(var1[3]);
  t2302 = Cos(var1[5]);
  t2306 = Sin(var1[3]);
  t2304 = Sin(var1[4]);
  t2307 = Sin(var1[5]);
  t2277 = Cos(var1[10]);
  t2256 = Cos(var1[4]);
  t2258 = Sin(var1[10]);
  t2294 = Cos(var1[9]);
  t2305 = t2250*t2302*t2304;
  t2308 = t2306*t2307;
  t2316 = t2305 + t2308;
  t2321 = t2294*t2316;
  t2323 = Sin(var1[9]);
  t2328 = -1.*t2302*t2306;
  t2336 = t2250*t2304*t2307;
  t2340 = t2328 + t2336;
  t2344 = -1.*t2323*t2340;
  t2345 = t2321 + t2344;
  t2228 = Sin(var1[11]);
  t2371 = Cos(var1[11]);
  t2384 = t2302*t2306*t2304;
  t2392 = -1.*t2250*t2307;
  t2402 = t2384 + t2392;
  t2407 = t2294*t2402;
  t2409 = t2250*t2302;
  t2413 = t2306*t2304*t2307;
  t2414 = t2409 + t2413;
  t2415 = -1.*t2323*t2414;
  t2416 = t2407 + t2415;
  t2437 = t2294*t2256*t2302;
  t2445 = -1.*t2256*t2323*t2307;
  t2449 = t2437 + t2445;
  t2260 = t2250*t2256*t2258;
  t2347 = t2277*t2345;
  t2362 = t2260 + t2347;
  t2372 = t2277*t2250*t2256;
  t2373 = -1.*t2258*t2345;
  t2374 = t2372 + t2373;
  t2377 = t2256*t2258*t2306;
  t2418 = t2277*t2416;
  t2422 = t2377 + t2418;
  t2427 = t2277*t2256*t2306;
  t2429 = -1.*t2258*t2416;
  t2432 = t2427 + t2429;
  t2436 = -1.*t2258*t2304;
  t2457 = t2277*t2449;
  t2458 = t2436 + t2457;
  t2462 = -1.*t2277*t2304;
  t2463 = -1.*t2258*t2449;
  t2467 = t2462 + t2463;
  t2471 = t2323*t2316;
  t2472 = t2294*t2340;
  t2473 = t2471 + t2472;
  t2546 = -1.*t2371;
  t2547 = 1. + t2546;
  t2366 = -1.*t2228*t2362;
  t2375 = t2371*t2374;
  t2376 = t2366 + t2375;
  t2503 = t2371*t2362;
  t2508 = t2228*t2374;
  t2512 = t2503 + t2508;
  t2521 = -1.*t2277;
  t2522 = 1. + t2521;
  t2523 = 0.183*t2522;
  t2524 = 0. + t2523;
  t2530 = -0.047*t2323;
  t2531 = 0. + t2530;
  t2537 = -1.*t2294;
  t2538 = 1. + t2537;
  t2539 = 0.047*t2538;
  t2540 = 0. + t2539;
  t2474 = t2323*t2402;
  t2476 = t2294*t2414;
  t2477 = t2474 + t2476;
  t2543 = 0.183*t2258;
  t2544 = 0. + t2543;
  t2548 = -0.2*t2547;
  t2551 = 0.183*t2228;
  t2552 = 0. + t2548 + t2551;
  t2554 = 0.183*t2547;
  t2557 = 0.2*t2228;
  t2559 = 0. + t2554 + t2557;
  t2423 = -1.*t2228*t2422;
  t2434 = t2371*t2432;
  t2435 = t2423 + t2434;
  t2513 = t2371*t2422;
  t2514 = t2228*t2432;
  t2516 = t2513 + t2514;
  t2489 = t2256*t2302*t2323;
  t2497 = t2294*t2256*t2307;
  t2501 = t2489 + t2497;
  t2459 = -1.*t2228*t2458;
  t2469 = t2371*t2467;
  t2470 = t2459 + t2469;
  t2518 = t2371*t2458;
  t2519 = t2228*t2467;
  t2520 = t2518 + t2519;
  p_output1[0]=t2376;
  p_output1[1]=t2435;
  p_output1[2]=t2470;
  p_output1[3]=0.;
  p_output1[4]=t2473;
  p_output1[5]=t2477;
  p_output1[6]=t2501;
  p_output1[7]=0.;
  p_output1[8]=t2512;
  p_output1[9]=t2516;
  p_output1[10]=t2520;
  p_output1[11]=0.;
  p_output1[12]=0. + 0.183*t2376 + 0.13205*t2473 - 0.2*t2512 + t2250*t2256*t2524 + t2316*t2531 + t2340*t2540 + t2345*t2544 + t2362*t2552 + t2374*t2559 + var1[0];
  p_output1[13]=0. + 0.183*t2435 + 0.13205*t2477 - 0.2*t2516 + t2256*t2306*t2524 + t2402*t2531 + t2414*t2540 + t2416*t2544 + t2422*t2552 + t2432*t2559 + var1[1];
  p_output1[14]=0. + 0.183*t2470 + 0.13205*t2501 - 0.2*t2520 - 1.*t2304*t2524 + t2256*t2302*t2531 + t2256*t2307*t2540 + t2449*t2544 + t2458*t2552 + t2467*t2559 + var1[2];
  p_output1[15]=1.;
}



void H_FL_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
