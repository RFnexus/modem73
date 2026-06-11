/*
SIMD element wise horizontal rotation

Copyright 2019 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include "simd.hh"

namespace CODE {

template <typename TYPE, int WIDTH>
class Rotate
{
	static const int SIZE = TYPE::SIZE;
	static_assert(WIDTH <= SIZE, "width must be smaller or equal to SIMD size");
public:
	TYPE operator()(TYPE a, int s)
	{
		if (s < 0)
			s += WIDTH;
		int t = WIDTH - s;
		TYPE ret;
		for (int n = 0; n < s; ++n)
			ret.v[n] = a.v[n+t];
		for (int n = 0; n < t; ++n)
			ret.v[n+s] = a.v[n];
		return ret;
	}
};

#ifdef __ARM_NEON
template <int WIDTH>
class Rotate<SIMD<int8_t, 16>, WIDTH>
{
	static const int SIZE = 16;
	static_assert(WIDTH <= SIZE, "width must be smaller or equal to SIMD size");
	typedef SIMD<int8_t, SIZE> TYPE;
	TYPE rot[WIDTH];
public:
	Rotate()
	{
		for (int i = 0; i < WIDTH; ++i) {
			rot[i] = vdup<TYPE>(0x80);
			for (int j = 0; j < WIDTH; ++j)
				rot[i].v[j] = (j - i + WIDTH) % WIDTH;
		}
	}
	TYPE operator()(TYPE a, int s)
	{
		if (s < 0)
			s += WIDTH;
		TYPE ret;
#ifdef __aarch64__
		ret.m = vqtbl1q_s8(a.m, vunsigned(rot[s]).m);
#else
		int8x8x2_t b { vget_low_s8(a.m), vget_high_s8(a.m) };
		int8x8_t c = vtbl2_s8(b, vget_low_s8(rot[s].m));
		int8x8_t d = vtbl2_s8(b, vget_high_s8(rot[s].m));
		ret.m = vcombine_s8(c, d);
#endif
		return ret;
	}
};
#endif

#ifdef __AVX2__
template <int WIDTH>
class Rotate<SIMD<int8_t, 32>, WIDTH>
{
	static const int SIZE = 32;
	static_assert(WIDTH <= SIZE, "width must be smaller or equal to SIMD size");
	typedef SIMD<int8_t, SIZE> TYPE;
	TYPE rot0[WIDTH], rot1[WIDTH];
public:
	Rotate()
	{
		for (int i = 0; i < WIDTH; ++i) {
			rot0[i] = vdup<TYPE>(0x80);
			rot1[i] = vdup<TYPE>(0x80);
			for (int j = 0; j < WIDTH; ++j) {
				int pos = (j - i + WIDTH) % WIDTH;
				if (j < 16) {
					if (pos < 16) {
						rot0[i].v[j] = pos;
					} else {
						rot1[i].v[j+16] = pos-16;
					}
				} else {
					if (pos < 16) {
						rot1[i].v[j-16] = pos;
					} else {
						rot0[i].v[j] = pos-16;
					}
				}
			}
		}
	}
	TYPE operator()(TYPE a, int s)
	{
		if (s < 0)
			s += WIDTH;
		__m256i b = _mm256_shuffle_epi8(a.m, rot0[s].m);
		__m256i c = _mm256_shuffle_epi8(a.m, rot1[s].m);
		__m256i d = _mm256_permute2x128_si256(c, c, 1);
		TYPE ret;
		ret.m = _mm256_or_si256(b, d);
		return ret;
	}
};
#else
#ifdef __SSE4_1__
template <int WIDTH>
class Rotate<SIMD<int8_t, 16>, WIDTH>
{
	static const int SIZE = 16;
	static_assert(WIDTH <= SIZE, "width must be smaller or equal to SIMD size");
	typedef SIMD<int8_t, SIZE> TYPE;
	TYPE rot[WIDTH];
public:
	Rotate()
	{
		for (int i = 0; i < WIDTH; ++i) {
			rot[i] = vdup<TYPE>(0x80);
			for (int j = 0; j < WIDTH; ++j)
				rot[i].v[j] = (j - i + WIDTH) % WIDTH;
		}
	}
	TYPE operator()(TYPE a, int s)
	{
		if (s < 0)
			s += WIDTH;
		TYPE ret;
		ret.m = _mm_shuffle_epi8(a.m, rot[s].m);
		return ret;
	}
};
#endif
#endif

}

