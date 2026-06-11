/*
Digital biquad filter

Copyright 2019 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include "const.hh"
#include "decibel.hh"
#include "unit_circle.hh"

namespace DSP {

template <typename TYPE, typename VALUE>
class Biquad
{
	TYPE x1, x2, y1, y2;
	VALUE b0a0, b1a0, b2a0, a1a0, a2a0;
public:
	constexpr Biquad() : x1(0), x2(0), y1(0), y2(0),
		b0a0(1), b1a0(0), b2a0(0), a1a0(0), a2a0(0)
	{
	}
	void lowpass(int n, int N, VALUE Q = Const<VALUE>::InvSqrtTwo())
	{
		VALUE alpha = UnitCircle<VALUE>::sin(n, N) / (VALUE(2) * Q),
			cn = UnitCircle<VALUE>::cos(n, N),
			b0 = (VALUE(1) - cn) / VALUE(2),
			b1 = VALUE(1) - cn,
			b2 = (VALUE(1) - cn) / VALUE(2),
			a0 = VALUE(1) + alpha,
			a1 = -VALUE(2) * cn,
			a2 = VALUE(1) - alpha;
		b0a0 = b0 / a0;
		b1a0 = b1 / a0;
		b2a0 = b2 / a0;
		a1a0 = a1 / a0;
		a2a0 = a2 / a0;
	}
	void highpass(int n, int N, VALUE Q = Const<VALUE>::InvSqrtTwo())
	{
		VALUE alpha = UnitCircle<VALUE>::sin(n, N) / (VALUE(2) * Q),
			cn = UnitCircle<VALUE>::cos(n, N),
			b0 = (VALUE(1) + cn) / VALUE(2),
			b1 = -(VALUE(1) + cn),
			b2 = (VALUE(1) + cn) / VALUE(2),
			a0 = VALUE(1) + alpha,
			a1 = -VALUE(2) * cn,
			a2 = VALUE(1) - alpha;
		b0a0 = b0 / a0;
		b1a0 = b1 / a0;
		b2a0 = b2 / a0;
		a1a0 = a1 / a0;
		a2a0 = a2 / a0;
	}
	void notch(int n, int N, VALUE QdB)
	{
		VALUE alpha = UnitCircle<VALUE>::sin(n, N) / (VALUE(2) * idecibel(QdB)),
			cn = UnitCircle<VALUE>::cos(n, N),
			b0 = VALUE(1),
			b1 = -VALUE(2) * cn,
			b2 = VALUE(1),
			a0 = VALUE(1) + alpha,
			a1 = -VALUE(2) * cn,
			a2 = VALUE(1) - alpha;
		b0a0 = b0 / a0;
		b1a0 = b1 / a0;
		b2a0 = b2 / a0;
		a1a0 = a1 / a0;
		a2a0 = a2 / a0;
	}
	void bandpass(int n, int N, VALUE QdB)
	{
		VALUE alpha = UnitCircle<VALUE>::sin(n, N) / (VALUE(2) * idecibel(QdB)),
			cn = UnitCircle<VALUE>::cos(n, N),
			b0 = alpha,
			b1 = VALUE(0),
			b2 = -alpha,
			a0 = VALUE(1) + alpha,
			a1 = -VALUE(2) * cn,
			a2 = VALUE(1) - alpha;
		b0a0 = b0 / a0;
		b1a0 = b1 / a0;
		b2a0 = b2 / a0;
		a1a0 = a1 / a0;
		a2a0 = a2 / a0;
	}
	void allpass(int n, int N, VALUE QdB)
	{
		VALUE alpha = UnitCircle<VALUE>::sin(n, N) / (VALUE(2) * idecibel(QdB)),
			cn = UnitCircle<VALUE>::cos(n, N),
			b0 = VALUE(1) - alpha,
			b1 = -VALUE(2) * cn,
			b2 = VALUE(1) + alpha,
			a0 = VALUE(1) + alpha,
			a1 = -VALUE(2) * cn,
			a2 = VALUE(1) - alpha;
		b0a0 = b0 / a0;
		b1a0 = b1 / a0;
		b2a0 = b2 / a0;
		a1a0 = a1 / a0;
		a2a0 = a2 / a0;
	}
	TYPE operator()(TYPE x0)
	{
		TYPE y0 = b0a0*x0 + b1a0*x1 + b2a0*x2
			- a1a0*y1 - a2a0*y2;
		x2 = x1; x1 = x0;
		y2 = y1; y1 = y0;
		return y0;
	}
};

template <typename TYPE, typename VALUE, int NUM = 1>
class BiquadCascade
{
	static const int ORDER = 2 * NUM;
	Biquad<TYPE, VALUE> cascade[NUM];
public:
	void lowpass(int n, int N)
	{
		for (int i = 0; i < NUM; ++i)
			cascade[i].lowpass(n, N, VALUE(1) / (VALUE(2) *
				UnitCircle<VALUE>::cos(2*i+1, 4*ORDER)));
	}
	void highpass(int n, int N)
	{
		for (int i = 0; i < NUM; ++i)
			cascade[i].highpass(n, N, VALUE(1) / (VALUE(2) *
				UnitCircle<VALUE>::cos(2*i+1, 4*ORDER)));
	}
	TYPE operator()(TYPE input)
	{
		for (int i = 0; i < NUM; ++i)
			input = cascade[i](input);
		return input;
	}
};

}

