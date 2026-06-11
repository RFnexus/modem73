/*
Least mean squares filter

Copyright 2020 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include "bip_buffer.hh"

namespace DSP {

template <typename TYPE, int SIZE>
class NLMS
{
	BipBuffer<TYPE, SIZE> history;
	TYPE filter[SIZE];
public:
	NLMS()
	{
		for (int i = 0; i < SIZE; ++i)
			filter[i] = 0;
	}
	inline operator const TYPE * () const
	{
		return filter;
	}
	TYPE operator () (TYPE input, TYPE desired, TYPE mu = 1)
	{
		const TYPE *hist = history(input);
		TYPE estimate = 0;
		for (int i = 0; i < SIZE; ++i)
			estimate += filter[i] * hist[i];
		TYPE error = desired - estimate;
		TYPE power = 0;
		for (int i = 0; i < SIZE; ++i)
			power += hist[i] * hist[i];
		if (power == 0)
			return error;
		TYPE rate = mu / power;
		for (int i = 0; i < SIZE; ++i)
			filter[i] += rate * error * hist[i];
		return error;
	}
};

template <typename CMPLX, int SIZE>
class NCLMS
{
	typedef typename CMPLX::value_type VALUE;
	BipBuffer<CMPLX, SIZE> history;
	CMPLX filter[SIZE];
public:
	NCLMS()
	{
		for (int i = 0; i < SIZE; ++i)
			filter[i] = 0;
	}
	inline operator const CMPLX * () const
	{
		return filter;
	}
	CMPLX operator () (CMPLX input, CMPLX desired, VALUE mu = 1)
	{
		const CMPLX *hist = history(conj(input));
		CMPLX estimate = 0;
		for (int i = 0; i < SIZE; ++i)
			estimate += conj(filter[i]) * hist[i];
		CMPLX error = conj(conj(desired) - estimate);
		VALUE power = 0;
		for (int i = 0; i < SIZE; ++i)
			power += norm(hist[i]);
		if (power == 0)
			return error;
		VALUE rate = mu / power;
		for (int i = 0; i < SIZE; ++i)
			filter[i] += rate * error * hist[i];
		return error;
	}
};

}

