/*
Normalizers for periodic signals

Copyright 2019 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include "blockdc.hh"
#include "ema.hh"

namespace DSP {

template <typename TYPE>
class NormalizeSine
{
	DSP::BlockDC<TYPE, TYPE> hpf;
	DSP::EMA<TYPE, TYPE> lpf;
public:
	constexpr NormalizeSine() : lpf(0.5)
	{
	}
	void samples(int s)
	{
		hpf.samples(s);
		lpf.samples(s);
	}
	TYPE operator()(TYPE input)
	{
		TYPE tmp = hpf(input);
		TYPE amp = sqrt(TYPE(2) * lpf(tmp * tmp));
		return tmp / amp;
	}
};

template <typename COMPLEX>
class NormalizeIQ
{
	NormalizeSine<typename COMPLEX::value_type> ni, nq;
public:
	void samples(int s)
	{
		ni.samples(s);
		nq.samples(s);
	}
	COMPLEX operator()(COMPLEX iq)
	{
		return COMPLEX(ni(iq.real()), nq(iq.imag()));
	}
};

}

