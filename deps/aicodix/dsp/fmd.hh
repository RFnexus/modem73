/*
FM Demodulation

Copyright 2019 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace DSP {

template <typename TYPE>
class FMD1
{
	typedef TYPE complex_type;
	typedef typename complex_type::value_type value_type;
	value_type prev;
	value_type scale;
	value_type wrap_around(value_type angle)
	{
		if (angle < -DSP::Const<value_type>::Pi())
			return angle + DSP::Const<value_type>::TwoPi();
		if (angle > DSP::Const<value_type>::Pi())
			return angle - DSP::Const<value_type>::TwoPi();
		return angle;
	}
public:
	constexpr FMD1() : prev(0), scale(0)
	{
	}
	void bandwidth(value_type bw)
	{
		scale = value_type(1) / (bw * DSP::Const<value_type>::Pi());
	}
	value_type operator()(complex_type input)
	{
		value_type phase = arg(input);
		value_type delta = phase - prev;
		delta = wrap_around(delta);
		prev = phase;
		return scale * delta;
	}
};

template <typename TYPE>
class FMD2
{
	typedef TYPE complex_type;
	typedef typename complex_type::value_type value_type;
	complex_type prev;
	value_type scale;
public:
	constexpr FMD2() : prev(1, 0), scale(0)
	{
	}
	void bandwidth(value_type bw)
	{
		scale = value_type(1) / (bw * DSP::Const<value_type>::Pi());
	}
	value_type operator()(complex_type input)
	{
		value_type phase = arg(input / prev);
		prev = input;
		return scale * phase;
	}
};

template <typename TYPE>
class FMD3
{
	typedef TYPE complex_type;
	typedef typename complex_type::value_type value_type;
	complex_type prev;
	value_type scale;
public:
	constexpr FMD3() : prev(1, 0), scale(0)
	{
	}
	void bandwidth(value_type bw)
	{
		scale = value_type(1) / (bw * DSP::Const<value_type>::Pi());
	}
	value_type operator()(complex_type input)
	{
		value_type phase = arg(input * conj(prev));
		prev = input;
		return scale * phase;
	}
};

template <typename TYPE>
class FMD4
{
	typedef TYPE complex_type;
	typedef typename complex_type::value_type value_type;
	complex_type prev;
	value_type scale;
public:
	constexpr FMD4() : prev(1, 0), scale(0)
	{
	}
	void bandwidth(value_type bw)
	{
		scale = value_type(1) / (bw * DSP::Const<value_type>::Pi());
	}
	value_type operator()(complex_type input)
	{
		value_type phase =
			prev.real() * input.imag() -
			prev.imag() * input.real();
		phase /= norm(input);
		prev = input;
		return scale * phase;
	}
};

template <typename TYPE>
class FMD5
{
	typedef TYPE complex_type;
	typedef typename complex_type::value_type value_type;
	complex_type prev1, prev2;
	value_type scale;
public:
	constexpr FMD5() : prev1(1, 0), prev2(1, 0), scale(0)
	{
	}
	void bandwidth(value_type bw)
	{
		scale = value_type(1) / (bw * DSP::Const<value_type>::TwoPi());
	}
	value_type operator()(complex_type input)
	{
		complex_type diff = input - prev2;
		value_type phase =
			prev1.real() * diff.imag() -
			prev1.imag() * diff.real();
		phase /= norm(prev1);
		prev2 = prev1; prev1 = input;
		return scale * phase;
	}
};

}

