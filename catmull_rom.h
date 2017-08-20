// http://ideone.com/NoEbVM
// https://stackoverflow.com/questions/9489736/catmull-rom-curve-with-no-cusps-and-no-self-intersections/23980479#23980479

#ifndef CATMULL_ROM_H
#define CATMULL_ROM_H

#include <iostream>
#include <cmath>
#include "vec_math.h"

using namespace std;
using namespace Vec_Math;

struct CubicPoly
{
	float c0, c1, c2, c3;
	
	float eval(float t)
	{
		float t2 = t * t;
		float t3 = t2 * t;
		return c0 + c1 * t + c2 * t2 + c3 * t3;
	}
};

/*
 * Compute coefficients for a cubic polynomial
 *   p(s) = c0 + c1 * s + c2 * s^2 + c3 * s^3
 * such that
 *   p(0) = x0, p(1) = x1
 *  and
 *   p'(0) = t0, p'(1) = t1.
 */
void InitCubicPoly(float x0, float x1, float t0, float t1, CubicPoly &p)
{
    p.c0 = x0;
    p.c1 = t0;
    p.c2 = -3.0f * x0 + 3.0f * x1 - 2.0f * t0 - t1;
    p.c3 = 2.0f * x0 - 2.0f * x1 + t0 + t1;
}

// compute coefficients for a nonuniform Catmull-Rom spline
void InitNonuniformCatmullRom(float x0, float x1, float x2, float x3, float dt0, float dt1, float dt2, CubicPoly &p)
{
    // compute tangents when parameterized in [t1,t2]
    float t1 = (x1 - x0) / dt0 - (x2 - x0) / (dt0 + dt1) + (x2 - x1) / dt1;
    float t2 = (x2 - x1) / dt1 - (x3 - x1) / (dt1 + dt2) + (x3 - x2) / dt2;

    // rescale tangents for parametrization in [0,1]
    t1 *= dt1;
    t2 *= dt1;

    InitCubicPoly(x1, x2, t1, t2, p);
}

void InitCentripetalCR(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
                       const float dt0, const float dt1, const float dt2,
                       CubicPoly &px, CubicPoly &py, CubicPoly &pz)
{
	InitNonuniformCatmullRom(p0.x, p1.x, p2.x, p3.x, dt0, dt1, dt2, px);
	InitNonuniformCatmullRom(p0.y, p1.y, p2.y, p3.y, dt0, dt1, dt2, py);
    InitNonuniformCatmullRom(p0.z, p1.z, p2.z, p3.z, dt0, dt1, dt2, pz);
}

#endif
