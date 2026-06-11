/*
atan and atan2 approximations

Constants below lifted from the Cephes Mathematical Library:
https://www.netlib.org/cephes/cmath.tgz

Copyright 2019 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include "const.hh"

namespace DSP {

template <typename TYPE>
TYPE atan_kernel(TYPE x)
{
	static const TYPE
		P0 = -8.750608600031904122785E-1,
		P1 = -1.615753718733365076637E1,
		P2 = -7.500855792314704667340E1,
		P3 = -1.228866684490136173410E2,
		P4 = -6.485021904942025371773E1,
		Q0 = 2.485846490142306297962E1,
		Q1 = 1.650270098316988542046E2,
		Q2 = 4.328810604912902668951E2,
		Q3 = 4.853903996359136964868E2,
		Q4 = 1.945506571482613964425E2;
	TYPE xx = x * x;
	return x + x * xx *
		(xx*(xx*(xx*(xx*P0+P1)+P2)+P3)+P4) /
		(xx*(xx*(xx*(xx*(xx+Q0)+Q1)+Q2)+Q3)+Q4);
}

template <typename TYPE>
TYPE atan_approx(TYPE x)
{
	static const TYPE
		MOREBITS = 6.123233995736765886130E-17;
	if (x <= TYPE(0.66))
		return atan_kernel(x);
	if (x > TYPE(2.41421356237309504880))
		return Const<TYPE>::HalfPi() + MOREBITS -
			atan_kernel(TYPE(1) / x);
	return Const<TYPE>::FourthPi() + TYPE(0.5) * MOREBITS +
		atan_kernel((x - TYPE(1)) / (x + TYPE(1)));
}

template <typename TYPE>
TYPE atan(TYPE x)
{
	return x >= TYPE(0) ? atan_approx(x) : -atan_approx(-x);
}

template <typename TYPE>
TYPE atan2(TYPE y, TYPE x)
{
	if (y == TYPE(0))
		return x >= TYPE(0) ?
			copysign(TYPE(0), y) :
			copysign(Const<TYPE>::Pi(), y);
	if (x == TYPE(0))
		return copysign(Const<TYPE>::HalfPi(), y);
	TYPE phase = atan(y / x);
	if (x < TYPE(0))
		return phase > TYPE(0) ?
			phase - Const<TYPE>::Pi() :
			phase + Const<TYPE>::Pi();
	return phase;
}

}

