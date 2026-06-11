/*
Some calculus functions

Copyright 2019 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace DSP {

template <typename TYPE>
class Integrator
{
	TYPE sum;
public:
	constexpr Integrator(TYPE sum = TYPE(0)) : sum(sum)
	{
	}
	TYPE operator()(TYPE input)
	{
		return sum += input;
	}
};

template <typename TYPE>
class Differentiator
{
	TYPE prev;
public:
	constexpr Differentiator(TYPE prev = TYPE(0)) : prev(prev)
	{
	}
	TYPE operator()(TYPE input)
	{
		TYPE diff = input - prev;
		prev = input;
		return diff;
	}
};

}

