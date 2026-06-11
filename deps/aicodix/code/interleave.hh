/*
Interleavers

Copyright 2023 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace CODE {

template <int SIZE, int ORDER, typename TYPE>
static void Interleave(TYPE *out, const TYPE *in)
{
	static_assert(SIZE % ORDER == 0, "ORDER does not divide SIZE");
	int LENGTH = SIZE / ORDER;
	for (int i = 0; i < LENGTH; ++i)
		for (int j = 0; j < ORDER; ++j)
			out[ORDER*i+j] = in[i+j*LENGTH];
}

template <int SIZE, int ORDER, typename TYPE>
static void Deinterleave(TYPE *out, const TYPE *in)
{
	static_assert(SIZE % ORDER == 0, "ORDER does not divide SIZE");
	int LENGTH = SIZE / ORDER;
	for (int i = 0; i < LENGTH; ++i)
		for (int j = 0; j < ORDER; ++j)
			out[i+j*LENGTH] = in[ORDER*i+j];
}

}

