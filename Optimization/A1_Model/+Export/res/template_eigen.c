/*
 * Automatically Generated from Mathematica.
 * <* DateString[] <> " " <> DateString["TimeZoneName"]  *>
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "<*TemplateSlot["name"]*>.h"

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
<* StringJoin@@Table[
"static void "<>TemplateSlot["argouts"][[i]]<>"(Eigen::Matrix<double,"<>ToString[TemplateSlot["argoutDims"][[i,1]]]<>","<>ToString[TemplateSlot["argoutDims"][[i,2]]]<>"> &p_"<>TemplateSlot["argouts"][[i]]<>", "<>StringImplode[Table["const Eigen::Matrix<double,"<>ToString[TemplateSlot["arginDims"][[i,1]]]<>","<>ToString[TemplateSlot["arginDims"][[i,2]]]<>"> &" <> ToString[TemplateSlot["argins"][[i]]], {i, Length[TemplateSlot["argins"]]}], ", "]<>")\n"<>
"{\n"<>
StringJoin[Riffle[StringJoin["  double ",ToString[#]]&/@TemplateSlot["lvars"][[i]],";\n"],";\n"]<>
StringJoin@@{"  ", Riffle[TemplateSlot["statements"][[i]], ";\n  "], ";\n"}<>
StringJoin@@{"\n  ", StringReplace[Riffle[TemplateSlot["final"][[i]], ";\n  "], {"["->"(", "]"->")"}], ";\n"}<>
"}\n\n"
,
{i,Length[TemplateSlot["argouts"]]}]
*>
       
<*StringImplode[Table["Eigen::Matrix<double,"<>ToString[TemplateSlot["argoutDims"][[i,1]]]<>","<>ToString[TemplateSlot["argoutDims"][[i,2]]]<>">", {i, Length[TemplateSlot["argouts"]]} ]]*> <*TemplateSlot["name"]*>(<*StringImplode[Table["const Eigen::Matrix<double,"<>ToString[TemplateSlot["arginDims"][[i,1]]]<>","<>ToString[TemplateSlot["arginDims"][[i,2]]]<>"> &" <> ToString[TemplateSlot["argins"][[i]]], {i, Length[TemplateSlot["argins"]]}], ", "]*>)
//void <*TemplateSlot["name"]*>(<*StringImplode[Table["Eigen::Matrix<double,"<>ToString[TemplateSlot["argoutDims"][[i,1]]]<>","<>ToString[TemplateSlot["argoutDims"][[i,2]]]<>"> &p_" <> TemplateSlot["argouts"][[i]], {i, Length[TemplateSlot["argouts"]]}], ", "]*>, <*StringImplode[Table["const Eigen::Matrix<double,"<>ToString[TemplateSlot["arginDims"][[i,1]]]<>","<>ToString[TemplateSlot["arginDims"][[i,2]]]<>"> &" <> ToString[TemplateSlot["argins"][[i]]], {i, Length[TemplateSlot["argins"]]}], ", "]*>)
{
  /* Call Subroutines */
  <*StringImplode[Table["Eigen::Matrix<double,"<>ToString[TemplateSlot["argoutDims"][[i,1]]]<>","<>ToString[TemplateSlot["argoutDims"][[i,2]]]<>">", {i, Length[TemplateSlot["argouts"]]} ]]*>  p_output1;
  
<*StringJoin@@Table[
"  "<>TemplateSlot["argouts"][[i]]<>"(p_"<>TemplateSlot["argouts"][[i]]<>", "<> StringImplode[Table[ToString[arg], {arg, TemplateSlot["argins"]}], ", "]<>");\n"
  , {i, Length[TemplateSlot["argouts"]]}
]*>
  return p_output1;
}



