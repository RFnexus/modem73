/*
Encoder for augmented Hadamard codes

Copyright 2020 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace CODE {

template <int K>
class HadamardEncoder
{
	static const int W = 1 << K;
	static const int N = 1 << (K - 1);
	int8_t mod[W];

	static bool parity(unsigned x)
	{
		x ^= x >> 16;
		x ^= x >> 8;
		x ^= x >> 4;
		x ^= x >> 2;
		x ^= x >> 1;
		return x & 1;
	}
public:
	HadamardEncoder()
	{
		for (int i = 0; i < W; ++i)
			mod[i] = 1 - 2 * parity(i);
	}
	void operator()(int8_t *code, int msg)
	{
		for (int i = 0; i < N; ++i)
			code[i] = mod[msg&(i|N)];
	}
};

}

