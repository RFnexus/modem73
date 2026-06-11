/*
Bit freezers for polar codes

Copyright 2020 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include <algorithm>

namespace CODE {

class PolarFreezer
{
	static bool get_bit(const uint32_t *bits, int idx)
	{
		return (bits[idx/32] >> (idx%32)) & 1;
	}
	static void set_bit(uint32_t *bits, int idx, bool val)
	{
		bits[idx/32] &= ~(1 << (idx%32));
		bits[idx/32] |= (uint32_t)val << (idx%32);
	}
	static void freeze(uint32_t *bits, double pe, double th, int i, int h)
	{
		if (h) {
			freeze(bits, pe * (2-pe), th, i, h/2);
			freeze(bits, pe * pe, th, i+h, h/2);
		} else {
			set_bit(bits, i, pe > th);
		}
	}
public:
	int operator()(uint32_t *frozen_bits, int level, double erasure_probability = 0.5, double freezing_threshold = 0.5)
	{
		int length = 1 << level;
		freeze(frozen_bits, erasure_probability, freezing_threshold, 0, length / 2);
		int K = length;
		for (int i = 0; i < length; ++i)
			K -= (frozen_bits[i/32] >> (i%32)) & 1;
		return K;
	}
};

template <int MAX_M>
class PolarCodeConst0
{
	static void inform_bit(uint32_t *bits, int idx)
	{
		bits[idx/32] &= ~(1 << (idx%32));
	}
	static void frozen_bit(uint32_t *bits, int idx)
	{
		bits[idx/32] |= 1 << (idx%32);
	}
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
	int index[1<<MAX_M];
public:
	void operator()(uint32_t *frozen_bits, int level, int K, double erasure_probability = std::exp(-1.))
	{
		assert(level <= MAX_M);
		int length = 1 << level;
		compute(erasure_probability, 0, length / 2);
		for (int i = 0; i < length; ++i)
			index[i] = i;
		std::nth_element(index, index+K, index+length, [this](int a, int b){ return prob[a] < prob[b]; });
		for (int i = 0; i < K; ++i)
			inform_bit(frozen_bits, index[i]);
		for (int i = K; i < length; ++i)
			frozen_bit(frozen_bits, index[i]);
	}
};

}

