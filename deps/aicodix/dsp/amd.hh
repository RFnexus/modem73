/*
AM Demodulation

Copyright 2019 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace DSP {

template <typename TYPE>
class AMD1
{
	typedef TYPE complex_type;
	typedef typename complex_type::value_type value_type;
	value_type avg, att, dec, lo, hi, idx;
public:
	AMD1() : avg(0), att(0), dec(0), lo(0.001), hi(1), idx(1)
	{
	}
	void index(value_type modidx)
	{
		idx = modidx;
	}
	void agc(value_type attack, value_type decay, value_type low = value_type(0.001), value_type high = value_type(1))
	{
		att = attack;
		dec = decay;
		lo = low;
		hi = high;
	}
	value_type operator()(complex_type input)
	{
		value_type amp = abs(input);
		avg = lerp(avg, amp, amp < avg ? dec : att);
		avg = min(max(avg, lo), hi);
		amp /= avg;
		return (amp * (value_type(1)+idx) - value_type(1)) / idx;
	}
};

}

