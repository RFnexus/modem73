/*
Construct reliability sequences for polar codes

Copyright 2023 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include "sort.hh"

namespace CODE {

template <int MAX_M>
class BhattacharyyaSequence
{
	void compute(double pe, int i, int h)
	{
		if (h) {
			compute(pe * (2-pe), i, h/2);
			compute(pe * pe, i+h, h/2);
		} else {
			prob[i] = pe;
		}
	}
	double prob[1<<MAX_M];
	MergeSort<int, 1<<MAX_M> sort;
public:
	void operator()(int *sequence, int level, double erasure_probability = std::exp(-1.))
	{
		assert(level <= MAX_M);
		int length = 1 << level;
		compute(erasure_probability, 0, length / 2);
		for (int i = 0; i < length; ++i)
			sequence[i] = i;
		sort(sequence, length, [this](int a, int b){ return prob[a] > prob[b]; });
	}
};

template <int MAX_M>
class ReedMullerSequence
{
	MergeSort<int, 1<<MAX_M> sort;
public:
	void operator()(int *sequence, int level)
	{
		assert(level <= MAX_M);
		int length = 1 << level;
		for (int i = 0; i < length; ++i)
			sequence[i] = i;
		sort(sequence, length, [](int a, int b){
			int x = __builtin_popcount(a);
			int y = __builtin_popcount(b);
			if (x != y)
				return x < y;
			return a < b;
		});
	}
};

}

