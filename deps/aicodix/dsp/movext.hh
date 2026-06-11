/*
Moving Extrema

Copyright 2020 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include "delay.hh"
#include "stack.hh"

namespace DSP {

template <typename TYPE, typename EQUAL, typename COMP, int NUM>
class MovExt
{
	Delay<TYPE, NUM> window;
	Stack<TYPE, NUM> dispenser, refill;
	EQUAL equal;
	COMP comp;
public:
	MovExt(TYPE init) : window(init)
	{
		dispenser.push(init);
	}
	TYPE operator () (TYPE input)
	{
		if (window(input) == dispenser.top())
			dispenser.pop();

		while (!refill.empty() && comp(input, refill.top()))
			refill.pop();
		refill.push(input);

		if (dispenser.empty()) {
			while (!refill.empty()) {
				dispenser.push(refill.top());
				refill.pop();
			}
			return dispenser.top();
		}
		return comp(dispenser.top(), refill.first()) ? dispenser.top() : refill.first();
	}
};

template <typename TYPE, int NUM>
class MovMin
{
	MovExt<TYPE, std::equal_to<TYPE>, std::less<TYPE>, NUM> movmin;
public:
	MovMin() : movmin(std::numeric_limits<TYPE>::max())
	{
	}
	TYPE operator () (TYPE input)
	{
		return movmin(input);
	}
};

template <typename TYPE, int NUM>
class MovMax
{
	MovExt<TYPE, std::equal_to<TYPE>, std::greater<TYPE>, NUM> movmax;
public:
	MovMax() : movmax(std::numeric_limits<TYPE>::min())
	{
	}
	TYPE operator () (TYPE input)
	{
		return movmax(input);
	}
};

}

