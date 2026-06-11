/*
Reversible permutations

Copyright 2023 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include "xorshift.hh"

namespace CODE {

template <int SIZE>
struct FisherYatesShuffle
{
	template <typename TYPE>
	void operator()(TYPE *array)
	{
		CODE::Xorshift32 prng;
		for (int i = 0; i < SIZE-1; ++i)
			std::swap(array[i], array[i + prng() % (SIZE - i)]);
	}
};

template <int SIZE>
class ReverseFisherYatesShuffle
{
	int seq[SIZE-1];
public:
	ReverseFisherYatesShuffle()
	{
		CODE::Xorshift32 prng;
		for (int i = 0; i < SIZE-1; ++i)
			seq[i] = i + prng() % (SIZE - i);
	}
	template <typename TYPE>
	void operator()(TYPE *array)
	{
		for (int i = SIZE-2; i >= 0; --i)
			std::swap(array[i], array[seq[i]]);
	}
};

template <int SIZE, typename TYPE>
static void BitReversalPermute(TYPE *array)
{
	static_assert(SIZE > 0 && (SIZE & (SIZE - 1)) == 0, "SIZE not power of two");
	for (int i = 0, j = 0; i < SIZE - 1; ++i) {
		if (i < j)
			std::swap(array[i], array[j]);
		int k = SIZE >> 1;
		while (j & k) {
			j ^= k;
			k >>= 1;
		}
		j ^= k;
	}
}

}

