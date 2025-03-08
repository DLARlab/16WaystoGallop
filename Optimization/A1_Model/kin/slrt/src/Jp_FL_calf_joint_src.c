/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:33 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FL_calf_joint_src.h"

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
  double t2131;
  double t2160;
  double t2156;
  double t2158;
  double t2163;
  double t2148;
  double t2159;
  double t2168;
  double t2170;
  double t2176;
  double t2196;
  double t2197;
  double t2198;
  double t2120;
  double t2204;
  double t2075;
  double t2207;
  double t2209;
  double t2211;
  double t2216;
  double t2218;
  double t2220;
  double t2225;
  double t2230;
  double t2232;
  double t2233;
  double t2241;
  double t2242;
  double t2243;
  double t2087;
  double t2100;
  double t2118;
  double t2119;
  double t2149;
  double t2153;
  double t2180;
  double t2184;
  double t2185;
  double t2193;
  double t2258;
  double t2260;
  double t2264;
  double t2266;
  double t2267;
  double t2268;
  double t2205;
  double t2206;
  double t2224;
  double t2228;
  double t2229;
  double t2274;
  double t2275;
  double t2276;
  double t2238;
  double t2239;
  double t2240;
  double t2278;
  double t2279;
  double t2280;
  double t2284;
  double t2286;
  double t2288;
  double t2310;
  double t2312;
  double t2314;
  double t2320;
  double t2321;
  double t2322;
  double t2324;
  double t2325;
  double t2326;
  double t2347;
  double t2348;
  double t2349;
  double t2351;
  double t2352;
  double t2353;
  double t2357;
  double t2360;
  double t2361;
  double t2390;
  double t2392;
  double t2394;
  double t2397;
  double t2398;
  double t2401;
  double t2403;
  double t2404;
  double t2405;
  double t2418;
  double t2421;
  double t2422;
  double t2426;
  double t2427;
  double t2428;
  double t2446;
  double t2447;
  double t2448;
  double t2201;
  double t2452;
  double t2453;
  double t2481;
  double t2482;
  double t2484;
  double t2504;
  double t2506;
  double t2523;
  double t2524;
  double t2525;
  double t2527;
  double t2528;
  double t2458;
  double t2485;
  double t2486;
  double t2488;
  double t2489;
  double t2490;
  double t2491;
  double t2492;
  double t2493;
  double t2494;
  double t2495;
  double t2496;
  double t2497;
  double t2498;
  double t2499;
  double t2500;
  double t2549;
  double t2550;
  double t2551;
  double t2294;
  double t2532;
  double t2533;
  double t2564;
  double t2565;
  double t2567;
  double t2568;
  double t2569;
  double t2582;
  double t2583;
  double t2584;
  double t2586;
  double t2587;
  double t2588;
  double t2293;
  double t2295;
  double t2558;
  double t2599;
  double t2600;
  double t2601;
  double t2603;
  double t2604;
  double t2611;
  double t2612;
  double t2613;
  double t2572;
  double t2576;
  double t2623;
  double t2624;
  double t2625;
  double t2591;
  double t2595;
  t2131 = Sin(var1[3]);
  t2160 = Cos(var1[3]);
  t2156 = Cos(var1[5]);
  t2158 = Sin(var1[4]);
  t2163 = Sin(var1[5]);
  t2148 = Sin(var1[9]);
  t2159 = -1.*t2156*t2131*t2158;
  t2168 = t2160*t2163;
  t2170 = t2159 + t2168;
  t2176 = Cos(var1[9]);
  t2196 = -1.*t2160*t2156;
  t2197 = -1.*t2131*t2158*t2163;
  t2198 = t2196 + t2197;
  t2120 = Cos(var1[4]);
  t2204 = Sin(var1[10]);
  t2075 = Cos(var1[10]);
  t2207 = t2176*t2170;
  t2209 = -1.*t2148*t2198;
  t2211 = t2207 + t2209;
  t2216 = Cos(var1[11]);
  t2218 = -1.*t2216;
  t2220 = 1. + t2218;
  t2225 = Sin(var1[11]);
  t2230 = -1.*t2120*t2204*t2131;
  t2232 = t2075*t2211;
  t2233 = t2230 + t2232;
  t2241 = -1.*t2075*t2120*t2131;
  t2242 = -1.*t2204*t2211;
  t2243 = t2241 + t2242;
  t2087 = -1.*t2075;
  t2100 = 1. + t2087;
  t2118 = 0.183*t2100;
  t2119 = 0. + t2118;
  t2149 = -0.047*t2148;
  t2153 = 0. + t2149;
  t2180 = -1.*t2176;
  t2184 = 1. + t2180;
  t2185 = 0.047*t2184;
  t2193 = 0. + t2185;
  t2258 = t2160*t2156*t2158;
  t2260 = t2131*t2163;
  t2264 = t2258 + t2260;
  t2266 = -1.*t2156*t2131;
  t2267 = t2160*t2158*t2163;
  t2268 = t2266 + t2267;
  t2205 = 0.183*t2204;
  t2206 = 0. + t2205;
  t2224 = -0.2*t2220;
  t2228 = 0.183*t2225;
  t2229 = 0. + t2224 + t2228;
  t2274 = t2176*t2264;
  t2275 = -1.*t2148*t2268;
  t2276 = t2274 + t2275;
  t2238 = 0.183*t2220;
  t2239 = 0.2*t2225;
  t2240 = 0. + t2238 + t2239;
  t2278 = t2160*t2120*t2204;
  t2279 = t2075*t2276;
  t2280 = t2278 + t2279;
  t2284 = t2075*t2160*t2120;
  t2286 = -1.*t2204*t2276;
  t2288 = t2284 + t2286;
  t2310 = t2176*t2160*t2120*t2156;
  t2312 = -1.*t2160*t2120*t2148*t2163;
  t2314 = t2310 + t2312;
  t2320 = -1.*t2160*t2204*t2158;
  t2321 = t2075*t2314;
  t2322 = t2320 + t2321;
  t2324 = -1.*t2075*t2160*t2158;
  t2325 = -1.*t2204*t2314;
  t2326 = t2324 + t2325;
  t2347 = t2176*t2120*t2156*t2131;
  t2348 = -1.*t2120*t2148*t2131*t2163;
  t2349 = t2347 + t2348;
  t2351 = -1.*t2204*t2131*t2158;
  t2352 = t2075*t2349;
  t2353 = t2351 + t2352;
  t2357 = -1.*t2075*t2131*t2158;
  t2360 = -1.*t2204*t2349;
  t2361 = t2357 + t2360;
  t2390 = -1.*t2176*t2156*t2158;
  t2392 = t2148*t2158*t2163;
  t2394 = t2390 + t2392;
  t2397 = -1.*t2120*t2204;
  t2398 = t2075*t2394;
  t2401 = t2397 + t2398;
  t2403 = -1.*t2075*t2120;
  t2404 = -1.*t2204*t2394;
  t2405 = t2403 + t2404;
  t2418 = t2156*t2131;
  t2421 = -1.*t2160*t2158*t2163;
  t2422 = t2418 + t2421;
  t2426 = -1.*t2148*t2264;
  t2427 = t2176*t2422;
  t2428 = t2426 + t2427;
  t2446 = t2156*t2131*t2158;
  t2447 = -1.*t2160*t2163;
  t2448 = t2446 + t2447;
  t2201 = t2176*t2198;
  t2452 = -1.*t2148*t2448;
  t2453 = t2452 + t2201;
  t2481 = -1.*t2120*t2156*t2148;
  t2482 = -1.*t2176*t2120*t2163;
  t2484 = t2481 + t2482;
  t2504 = -1.*t2176*t2268;
  t2506 = t2426 + t2504;
  t2523 = t2160*t2156;
  t2524 = t2131*t2158*t2163;
  t2525 = t2523 + t2524;
  t2527 = -1.*t2176*t2525;
  t2528 = t2452 + t2527;
  t2458 = t2176*t2448;
  t2485 = t2206*t2484;
  t2486 = t2075*t2229*t2484;
  t2488 = -1.*t2204*t2240*t2484;
  t2489 = t2176*t2120*t2156;
  t2490 = -1.*t2120*t2148*t2163;
  t2491 = t2489 + t2490;
  t2492 = 0.13205*t2491;
  t2493 = -1.*t2216*t2204*t2484;
  t2494 = -1.*t2075*t2225*t2484;
  t2495 = t2493 + t2494;
  t2496 = 0.183*t2495;
  t2497 = t2075*t2216*t2484;
  t2498 = -1.*t2204*t2225*t2484;
  t2499 = t2497 + t2498;
  t2500 = -0.2*t2499;
  t2549 = -1.*t2160*t2120*t2204;
  t2550 = -1.*t2075*t2276;
  t2551 = t2549 + t2550;
  t2294 = t2216*t2288;
  t2532 = -1.*t2148*t2525;
  t2533 = t2458 + t2532;
  t2564 = -1.*t2075*t2533;
  t2565 = t2230 + t2564;
  t2567 = t2075*t2120*t2131;
  t2568 = -1.*t2204*t2533;
  t2569 = t2567 + t2568;
  t2582 = t2204*t2158;
  t2583 = -1.*t2075*t2491;
  t2584 = t2582 + t2583;
  t2586 = -1.*t2075*t2158;
  t2587 = -1.*t2204*t2491;
  t2588 = t2586 + t2587;
  t2293 = -1.*t2225*t2280;
  t2295 = t2293 + t2294;
  t2558 = -1.*t2225*t2288;
  t2599 = 0.183*t2216;
  t2600 = -0.2*t2225;
  t2601 = t2599 + t2600;
  t2603 = 0.2*t2216;
  t2604 = t2603 + t2228;
  t2611 = t2120*t2204*t2131;
  t2612 = t2075*t2533;
  t2613 = t2611 + t2612;
  t2572 = t2216*t2569;
  t2576 = -1.*t2225*t2569;
  t2623 = -1.*t2204*t2158;
  t2624 = t2075*t2491;
  t2625 = t2623 + t2624;
  t2591 = t2216*t2588;
  t2595 = -1.*t2225*t2588;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=-1.*t2119*t2120*t2131 + t2153*t2170 + t2193*t2198 + 0.13205*(t2148*t2170 + t2201) + t2206*t2211 + t2229*t2233 + t2240*t2243 + 0.183*(-1.*t2225*t2233 + t2216*t2243) - 0.2*(t2216*t2233 + t2225*t2243);
  p_output1[10]=t2119*t2120*t2160 + t2153*t2264 + t2193*t2268 + 0.13205*(t2148*t2264 + t2176*t2268) + t2206*t2276 + t2229*t2280 + t2240*t2288 - 0.2*(t2216*t2280 + t2225*t2288) + 0.183*t2295;
  p_output1[11]=0;
  p_output1[12]=t2120*t2153*t2156*t2160 - 1.*t2119*t2158*t2160 + 0.13205*(t2120*t2148*t2156*t2160 + t2120*t2160*t2163*t2176) + t2120*t2160*t2163*t2193 + t2206*t2314 + t2229*t2322 + t2240*t2326 + 0.183*(-1.*t2225*t2322 + t2216*t2326) - 0.2*(t2216*t2322 + t2225*t2326);
  p_output1[13]=t2120*t2131*t2153*t2156 - 1.*t2119*t2131*t2158 + 0.13205*(t2120*t2131*t2148*t2156 + t2120*t2131*t2163*t2176) + t2120*t2131*t2163*t2193 + t2206*t2349 + t2229*t2353 + t2240*t2361 + 0.183*(-1.*t2225*t2353 + t2216*t2361) - 0.2*(t2216*t2353 + t2225*t2361);
  p_output1[14]=-1.*t2119*t2120 - 1.*t2153*t2156*t2158 + 0.13205*(-1.*t2148*t2156*t2158 - 1.*t2158*t2163*t2176) - 1.*t2158*t2163*t2193 + t2206*t2394 + t2229*t2401 + t2240*t2405 + 0.183*(-1.*t2225*t2401 + t2216*t2405) - 0.2*(t2216*t2401 + t2225*t2405);
  p_output1[15]=t2193*t2264 + t2153*t2422 + 0.13205*(t2274 + t2148*t2422) + t2206*t2428 + t2075*t2229*t2428 - 1.*t2204*t2240*t2428 + 0.183*(-1.*t2204*t2216*t2428 - 1.*t2075*t2225*t2428) - 0.2*(t2075*t2216*t2428 - 1.*t2204*t2225*t2428);
  p_output1[16]=t2153*t2198 + t2193*t2448 + t2206*t2453 + t2075*t2229*t2453 - 1.*t2204*t2240*t2453 + 0.183*(-1.*t2204*t2216*t2453 - 1.*t2075*t2225*t2453) - 0.2*(t2075*t2216*t2453 - 1.*t2204*t2225*t2453) + 0.13205*(t2148*t2198 + t2458);
  p_output1[17]=-1.*t2120*t2153*t2163 + t2120*t2156*t2193 + t2485 + t2486 + t2488 + t2492 + t2496 + t2500;
  p_output1[18]=0;
  p_output1[19]=0;
  p_output1[20]=0;
  p_output1[21]=0;
  p_output1[22]=0;
  p_output1[23]=0;
  p_output1[24]=0;
  p_output1[25]=0;
  p_output1[26]=0;
  p_output1[27]=-0.047*t2176*t2264 + 0.047*t2148*t2268 + 0.13205*t2276 + t2206*t2506 + t2075*t2229*t2506 - 1.*t2204*t2240*t2506 + 0.183*(-1.*t2204*t2216*t2506 - 1.*t2075*t2225*t2506) - 0.2*(t2075*t2216*t2506 - 1.*t2204*t2225*t2506);
  p_output1[28]=-0.047*t2176*t2448 + 0.047*t2148*t2525 + t2206*t2528 + t2075*t2229*t2528 - 1.*t2204*t2240*t2528 + 0.183*(-1.*t2204*t2216*t2528 - 1.*t2075*t2225*t2528) - 0.2*(t2075*t2216*t2528 - 1.*t2204*t2225*t2528) + 0.13205*t2533;
  p_output1[29]=0.047*t2120*t2148*t2163 - 0.047*t2120*t2156*t2176 + t2485 + t2486 + t2488 + t2492 + t2496 + t2500;
  p_output1[30]=0.183*t2120*t2160*t2204 + 0.183*t2075*t2276 + t2229*t2288 + t2240*t2551 - 0.2*(t2294 + t2225*t2551) + 0.183*(t2216*t2551 + t2558);
  p_output1[31]=0.183*t2120*t2131*t2204 + 0.183*t2075*t2533 + t2240*t2565 + t2229*t2569 - 0.2*(t2225*t2565 + t2572) + 0.183*(t2216*t2565 + t2576);
  p_output1[32]=-0.183*t2158*t2204 + 0.183*t2075*t2491 + t2240*t2584 + t2229*t2588 - 0.2*(t2225*t2584 + t2591) + 0.183*(t2216*t2584 + t2595);
  p_output1[33]=-0.2*t2295 + 0.183*(-1.*t2216*t2280 + t2558) + t2280*t2601 + t2288*t2604;
  p_output1[34]=t2569*t2604 + t2601*t2613 + 0.183*(t2576 - 1.*t2216*t2613) - 0.2*(t2572 - 1.*t2225*t2613);
  p_output1[35]=t2588*t2604 + t2601*t2625 + 0.183*(t2595 - 1.*t2216*t2625) - 0.2*(t2591 - 1.*t2225*t2625);
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



void Jp_FL_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
