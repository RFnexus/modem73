/*
Intel AVX2 acceleration times two

Copyright 2024 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include <immintrin.h>

template <>
union SIMD<float, 16>
{
	static const int SIZE = 16;
	typedef float value_type;
	typedef uint32_t uint_type;
	__m256 m[2];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<double, 8>
{
	static const int SIZE = 8;
	typedef double value_type;
	typedef uint64_t uint_type;
	__m256d m[2];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<int8_t, 64>
{
	static const int SIZE = 64;
	typedef int8_t value_type;
	typedef uint8_t uint_type;
	__m256i m[2];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<int16_t, 32>
{
	static const int SIZE = 32;
	typedef int16_t value_type;
	typedef uint16_t uint_type;
	__m256i m[2];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<int32_t, 16>
{
	static const int SIZE = 16;
	typedef int32_t value_type;
	typedef uint32_t uint_type;
	__m256i m[2];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<int64_t, 8>
{
	static const int SIZE = 8;
	typedef int64_t value_type;
	typedef uint64_t uint_type;
	__m256i m[2];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<uint8_t, 64>
{
	static const int SIZE = 64;
	typedef uint8_t value_type;
	typedef uint8_t uint_type;
	__m256i m[2];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<uint16_t, 32>
{
	static const int SIZE = 32;
	typedef uint16_t value_type;
	typedef uint16_t uint_type;
	__m256i m[2];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<uint32_t, 16>
{
	static const int SIZE = 16;
	typedef uint32_t value_type;
	typedef uint32_t uint_type;
	__m256i m[2];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<uint64_t, 8>
{
	static const int SIZE = 8;
	typedef uint64_t value_type;
	typedef uint64_t uint_type;
	__m256i m[2];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
inline SIMD<float, 16> vreinterpret(SIMD<uint32_t, 16> a)
{
	SIMD<float, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vreinterpret(SIMD<float, 16> a)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)a.m[i];
	return tmp;
}

template <>
inline SIMD<double, 8> vreinterpret(SIMD<uint64_t, 8> a)
{
	SIMD<double, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256d)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vreinterpret(SIMD<double, 8> a)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint8_t, 64> vreinterpret(SIMD<int8_t, 64> a)
{
	SIMD<uint8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)a.m[i];
	return tmp;
}

template <>
inline SIMD<int8_t, 64> vreinterpret(SIMD<uint8_t, 64> a)
{
	SIMD<int8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint16_t, 32> vreinterpret(SIMD<int16_t, 32> a)
{
	SIMD<uint16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)a.m[i];
	return tmp;
}

template <>
inline SIMD<int16_t, 32> vreinterpret(SIMD<uint16_t, 32> a)
{
	SIMD<int16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vreinterpret(SIMD<int32_t, 16> a)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)a.m[i];
	return tmp;
}

template <>
inline SIMD<int32_t, 16> vreinterpret(SIMD<uint32_t, 16> a)
{
	SIMD<int32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vreinterpret(SIMD<int64_t, 8> a)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)a.m[i];
	return tmp;
}

template <>
inline SIMD<int64_t, 8> vreinterpret(SIMD<uint64_t, 8> a)
{
	SIMD<int64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)a.m[i];
	return tmp;
}

template <>
inline SIMD<float, 16> vdup<SIMD<float, 16>>(float a)
{
	SIMD<float, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_set1_ps(a);
	return tmp;
}

template <>
inline SIMD<double, 8> vdup<SIMD<double, 8>>(double a)
{
	SIMD<double, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_set1_pd(a);
	return tmp;
}

template <>
inline SIMD<int8_t, 64> vdup<SIMD<int8_t, 64>>(int8_t a)
{
	SIMD<int8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_set1_epi8(a);
	return tmp;
}

template <>
inline SIMD<int16_t, 32> vdup<SIMD<int16_t, 32>>(int16_t a)
{
	SIMD<int16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_set1_epi16(a);
	return tmp;
}

template <>
inline SIMD<int32_t, 16> vdup<SIMD<int32_t, 16>>(int32_t a)
{
	SIMD<int32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_set1_epi32(a);
	return tmp;
}

template <>
inline SIMD<int64_t, 8> vdup<SIMD<int64_t, 8>>(int64_t a)
{
	SIMD<int64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_set1_epi64x(a);
	return tmp;
}

template <>
inline SIMD<float, 16> vzero()
{
	SIMD<float, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_setzero_ps();
	return tmp;
}

template <>
inline SIMD<double, 8> vzero()
{
	SIMD<double, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_setzero_pd();
	return tmp;
}

template <>
inline SIMD<int8_t, 64> vzero()
{
	SIMD<int8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_setzero_si256();
	return tmp;
}

template <>
inline SIMD<int16_t, 32> vzero()
{
	SIMD<int16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_setzero_si256();
	return tmp;
}

template <>
inline SIMD<int32_t, 16> vzero()
{
	SIMD<int32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_setzero_si256();
	return tmp;
}

template <>
inline SIMD<int64_t, 8> vzero()
{
	SIMD<int64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_setzero_si256();
	return tmp;
}

template <>
inline SIMD<float, 16> vadd(SIMD<float, 16> a, SIMD<float, 16> b)
{
	SIMD<float, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_add_ps(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<double, 8> vadd(SIMD<double, 8> a, SIMD<double, 8> b)
{
	SIMD<double, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_add_pd(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 64> vadd(SIMD<int8_t, 64> a, SIMD<int8_t, 64> b)
{
	SIMD<int8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_add_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 32> vadd(SIMD<int16_t, 32> a, SIMD<int16_t, 32> b)
{
	SIMD<int16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_add_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 16> vadd(SIMD<int32_t, 16> a, SIMD<int32_t, 16> b)
{
	SIMD<int32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_add_epi32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int64_t, 8> vadd(SIMD<int64_t, 8> a, SIMD<int64_t, 8> b)
{
	SIMD<int64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_add_epi64(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 64> vqadd(SIMD<int8_t, 64> a, SIMD<int8_t, 64> b)
{
	SIMD<int8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_adds_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 32> vqadd(SIMD<int16_t, 32> a, SIMD<int16_t, 32> b)
{
	SIMD<int16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_adds_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 16> vsub(SIMD<float, 16> a, SIMD<float, 16> b)
{
	SIMD<float, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_sub_ps(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<double, 8> vsub(SIMD<double, 8> a, SIMD<double, 8> b)
{
	SIMD<double, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_sub_pd(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 64> vsub(SIMD<int8_t, 64> a, SIMD<int8_t, 64> b)
{
	SIMD<int8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_sub_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 32> vsub(SIMD<int16_t, 32> a, SIMD<int16_t, 32> b)
{
	SIMD<int16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_sub_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 16> vsub(SIMD<int32_t, 16> a, SIMD<int32_t, 16> b)
{
	SIMD<int32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_sub_epi32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int64_t, 8> vsub(SIMD<int64_t, 8> a, SIMD<int64_t, 8> b)
{
	SIMD<int64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_sub_epi64(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 64> vqsub(SIMD<int8_t, 64> a, SIMD<int8_t, 64> b)
{
	SIMD<int8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_subs_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 32> vqsub(SIMD<int16_t, 32> a, SIMD<int16_t, 32> b)
{
	SIMD<int16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_subs_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 64> vqsub(SIMD<uint8_t, 64> a, SIMD<uint8_t, 64> b)
{
	SIMD<uint8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_subs_epu8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 32> vqsub(SIMD<uint16_t, 32> a, SIMD<uint16_t, 32> b)
{
	SIMD<uint16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_subs_epu16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 16> vmul(SIMD<float, 16> a, SIMD<float, 16> b)
{
	SIMD<float, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_mul_ps(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<double, 8> vmul(SIMD<double, 8> a, SIMD<double, 8> b)
{
	SIMD<double, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_mul_pd(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 16> vabs(SIMD<float, 16> a)
{
	SIMD<float, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_andnot_ps(_mm256_set1_ps(-0.f), a.m[i]);
	return tmp;
}

template <>
inline SIMD<double, 8> vabs(SIMD<double, 8> a)
{
	SIMD<double, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_andnot_pd(_mm256_set1_pd(-0.), a.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 64> vqabs(SIMD<int8_t, 64> a)
{
	SIMD<int8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_abs_epi8(_mm256_max_epi8(a.m[i], _mm256_set1_epi8(-INT8_MAX)));
	return tmp;
}

template <>
inline SIMD<int16_t, 32> vqabs(SIMD<int16_t, 32> a)
{
	SIMD<int16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_abs_epi16(_mm256_max_epi16(a.m[i], _mm256_set1_epi16(-INT16_MAX)));
	return tmp;
}

template <>
inline SIMD<int32_t, 16> vqabs(SIMD<int32_t, 16> a)
{
	SIMD<int32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_abs_epi32(_mm256_max_epi32(a.m[i], _mm256_set1_epi32(-INT32_MAX)));
	return tmp;
}

template <>
inline SIMD<float, 16> vsignum(SIMD<float, 16> a)
{
	SIMD<float, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_andnot_ps(
			_mm256_cmp_ps(a.m[i], _mm256_setzero_ps(), _CMP_EQ_OQ),
			_mm256_or_ps(_mm256_set1_ps(1.f), _mm256_and_ps(_mm256_set1_ps(-0.f), a.m[i])));
	return tmp;
}

template <>
inline SIMD<double, 8> vsignum(SIMD<double, 8> a)
{
	SIMD<double, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_andnot_pd(
			_mm256_cmp_pd(a.m[i], _mm256_setzero_pd(), _CMP_EQ_OQ),
			_mm256_or_pd(_mm256_set1_pd(1.), _mm256_and_pd(_mm256_set1_pd(-0.), a.m[i])));
	return tmp;
}

template <>
inline SIMD<int8_t, 64> vsignum(SIMD<int8_t, 64> a)
{
	SIMD<int8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_sign_epi8(_mm256_set1_epi8(1), a.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 32> vsignum(SIMD<int16_t, 32> a)
{
	SIMD<int16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_sign_epi16(_mm256_set1_epi16(1), a.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 16> vsignum(SIMD<int32_t, 16> a)
{
	SIMD<int32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_sign_epi32(_mm256_set1_epi32(1), a.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 16> vsign(SIMD<float, 16> a, SIMD<float, 16> b)
{
	SIMD<float, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_andnot_ps(
			_mm256_cmp_ps(b.m[i], _mm256_setzero_ps(), _CMP_EQ_OQ),
			_mm256_xor_ps(a.m[i], _mm256_and_ps(_mm256_set1_ps(-0.f), b.m[i])));
	return tmp;
}

template <>
inline SIMD<double, 8> vsign(SIMD<double, 8> a, SIMD<double, 8> b)
{
	SIMD<double, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_andnot_pd(
			_mm256_cmp_pd(b.m[i], _mm256_setzero_pd(), _CMP_EQ_OQ),
			_mm256_xor_pd(a.m[i], _mm256_and_pd(_mm256_set1_pd(-0.), b.m[i])));
	return tmp;
}

template <>
inline SIMD<int8_t, 64> vsign(SIMD<int8_t, 64> a, SIMD<int8_t, 64> b)
{
	SIMD<int8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_sign_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 32> vsign(SIMD<int16_t, 32> a, SIMD<int16_t, 32> b)
{
	SIMD<int16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_sign_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 16> vsign(SIMD<int32_t, 16> a, SIMD<int32_t, 16> b)
{
	SIMD<int32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_sign_epi32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 16> vcopysign(SIMD<float, 16> a, SIMD<float, 16> b)
{
	SIMD<float, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_ps(
			_mm256_andnot_ps(_mm256_set1_ps(-0.f), a.m[i]),
			_mm256_and_ps(_mm256_set1_ps(-0.f), b.m[i]));
	return tmp;
}

template <>
inline SIMD<double, 8> vcopysign(SIMD<double, 8> a, SIMD<double, 8> b)
{
	SIMD<double, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_pd(
			_mm256_andnot_pd(_mm256_set1_pd(-0.), a.m[i]),
			_mm256_and_pd(_mm256_set1_pd(-0.), b.m[i]));
	return tmp;
}

template <>
inline SIMD<uint8_t, 64> vorr(SIMD<uint8_t, 64> a, SIMD<uint8_t, 64> b)
{
	SIMD<uint8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_si256(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 32> vorr(SIMD<uint16_t, 32> a, SIMD<uint16_t, 32> b)
{
	SIMD<uint16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_si256(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vorr(SIMD<uint32_t, 16> a, SIMD<uint32_t, 16> b)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_si256(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vorr(SIMD<uint64_t, 8> a, SIMD<uint64_t, 8> b)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_si256(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 64> vand(SIMD<uint8_t, 64> a, SIMD<uint8_t, 64> b)
{
	SIMD<uint8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_and_si256(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 32> vand(SIMD<uint16_t, 32> a, SIMD<uint16_t, 32> b)
{
	SIMD<uint16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_and_si256(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vand(SIMD<uint32_t, 16> a, SIMD<uint32_t, 16> b)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_and_si256(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vand(SIMD<uint64_t, 8> a, SIMD<uint64_t, 8> b)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_and_si256(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 64> veor(SIMD<uint8_t, 64> a, SIMD<uint8_t, 64> b)
{
	SIMD<uint8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_xor_si256(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 32> veor(SIMD<uint16_t, 32> a, SIMD<uint16_t, 32> b)
{
	SIMD<uint16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_xor_si256(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> veor(SIMD<uint32_t, 16> a, SIMD<uint32_t, 16> b)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_xor_si256(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> veor(SIMD<uint64_t, 8> a, SIMD<uint64_t, 8> b)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_xor_si256(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 64> vbic(SIMD<uint8_t, 64> a, SIMD<uint8_t, 64> b)
{
	SIMD<uint8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_andnot_si256(b.m[i], a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 32> vbic(SIMD<uint16_t, 32> a, SIMD<uint16_t, 32> b)
{
	SIMD<uint16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_andnot_si256(b.m[i], a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vbic(SIMD<uint32_t, 16> a, SIMD<uint32_t, 16> b)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_andnot_si256(b.m[i], a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vbic(SIMD<uint64_t, 8> a, SIMD<uint64_t, 8> b)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_andnot_si256(b.m[i], a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 64> vbsl(SIMD<uint8_t, 64> a, SIMD<uint8_t, 64> b, SIMD<uint8_t, 64> c)
{
	SIMD<uint8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_si256(_mm256_and_si256(a.m[i], b.m[i]), _mm256_andnot_si256(a.m[i], c.m[i]));
	return tmp;
}

template <>
inline SIMD<uint16_t, 32> vbsl(SIMD<uint16_t, 32> a, SIMD<uint16_t, 32> b, SIMD<uint16_t, 32> c)
{
	SIMD<uint16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_si256(_mm256_and_si256(a.m[i], b.m[i]), _mm256_andnot_si256(a.m[i], c.m[i]));
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vbsl(SIMD<uint32_t, 16> a, SIMD<uint32_t, 16> b, SIMD<uint32_t, 16> c)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_si256(_mm256_and_si256(a.m[i], b.m[i]), _mm256_andnot_si256(a.m[i], c.m[i]));
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vbsl(SIMD<uint64_t, 8> a, SIMD<uint64_t, 8> b, SIMD<uint64_t, 8> c)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_si256(_mm256_and_si256(a.m[i], b.m[i]), _mm256_andnot_si256(a.m[i], c.m[i]));
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vceqz(SIMD<float, 16> a)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)_mm256_cmp_ps(a.m[i], _mm256_setzero_ps(), _CMP_EQ_OQ);
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vceqz(SIMD<double, 8> a)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)_mm256_cmp_pd(a.m[i], _mm256_setzero_pd(), _CMP_EQ_OQ);
	return tmp;
}

template <>
inline SIMD<uint8_t, 64> vceqz(SIMD<int8_t, 64> a)
{
	SIMD<uint8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpeq_epi8(a.m[i], _mm256_setzero_si256());
	return tmp;
}

template <>
inline SIMD<uint16_t, 32> vceqz(SIMD<int16_t, 32> a)
{
	SIMD<uint16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpeq_epi16(a.m[i], _mm256_setzero_si256());
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vceqz(SIMD<int32_t, 16> a)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpeq_epi32(a.m[i], _mm256_setzero_si256());
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vceqz(SIMD<int64_t, 8> a)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpeq_epi64(a.m[i], _mm256_setzero_si256());
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vceq(SIMD<float, 16> a, SIMD<float, 16> b)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)_mm256_cmp_ps(a.m[i], b.m[i], _CMP_EQ_OQ);
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vceq(SIMD<double, 8> a, SIMD<double, 8> b)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)_mm256_cmp_pd(a.m[i], b.m[i], _CMP_EQ_OQ);
	return tmp;
}

template <>
inline SIMD<uint8_t, 64> vceq(SIMD<int8_t, 64> a, SIMD<int8_t, 64> b)
{
	SIMD<uint8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpeq_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 32> vceq(SIMD<int16_t, 32> a, SIMD<int16_t, 32> b)
{
	SIMD<uint16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpeq_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vceq(SIMD<int32_t, 16> a, SIMD<int32_t, 16> b)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpeq_epi32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vceq(SIMD<int64_t, 8> a, SIMD<int64_t, 8> b)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpeq_epi64(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vcgtz(SIMD<float, 16> a)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)_mm256_cmp_ps(a.m[i], _mm256_setzero_ps(), _CMP_GT_OQ);
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vcgtz(SIMD<double, 8> a)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)_mm256_cmp_pd(a.m[i], _mm256_setzero_pd(), _CMP_GT_OQ);
	return tmp;
}

template <>
inline SIMD<uint8_t, 64> vcgtz(SIMD<int8_t, 64> a)
{
	SIMD<uint8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpgt_epi8(a.m[i], _mm256_setzero_si256());
	return tmp;
}

template <>
inline SIMD<uint16_t, 32> vcgtz(SIMD<int16_t, 32> a)
{
	SIMD<uint16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpgt_epi16(a.m[i], _mm256_setzero_si256());
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vcgtz(SIMD<int32_t, 16> a)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpgt_epi32(a.m[i], _mm256_setzero_si256());
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vcgtz(SIMD<int64_t, 8> a)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpgt_epi64(a.m[i], _mm256_setzero_si256());
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vcltz(SIMD<float, 16> a)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)_mm256_cmp_ps(a.m[i], _mm256_setzero_ps(), _CMP_LT_OQ);
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vcltz(SIMD<double, 8> a)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)_mm256_cmp_pd(a.m[i], _mm256_setzero_pd(), _CMP_LT_OQ);
	return tmp;
}

template <>
inline SIMD<uint8_t, 64> vcltz(SIMD<int8_t, 64> a)
{
	SIMD<uint8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpgt_epi8(_mm256_setzero_si256(), a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 32> vcltz(SIMD<int16_t, 32> a)
{
	SIMD<uint16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpgt_epi16(_mm256_setzero_si256(), a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vcltz(SIMD<int32_t, 16> a)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpgt_epi32(_mm256_setzero_si256(), a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vcltz(SIMD<int64_t, 8> a)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_cmpgt_epi64(_mm256_setzero_si256(), a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vclez(SIMD<float, 16> a)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)_mm256_cmp_ps(a.m[i], _mm256_setzero_ps(), _CMP_LE_OQ);
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vclez(SIMD<double, 8> a)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = (__m256i)_mm256_cmp_pd(a.m[i], _mm256_setzero_pd(), _CMP_LE_OQ);
	return tmp;
}

template <>
inline SIMD<uint8_t, 64> vclez(SIMD<int8_t, 64> a)
{
	SIMD<uint8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_si256(
			_mm256_cmpeq_epi8(a.m[i], _mm256_setzero_si256()),
			_mm256_cmpgt_epi8(_mm256_setzero_si256(), a.m[i]));
	return tmp;
}

template <>
inline SIMD<uint16_t, 32> vclez(SIMD<int16_t, 32> a)
{
	SIMD<uint16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_si256(
			_mm256_cmpeq_epi16(a.m[i], _mm256_setzero_si256()),
			_mm256_cmpgt_epi16(_mm256_setzero_si256(), a.m[i]));
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vclez(SIMD<int32_t, 16> a)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_si256(
			_mm256_cmpeq_epi32(a.m[i], _mm256_setzero_si256()),
			_mm256_cmpgt_epi32(_mm256_setzero_si256(), a.m[i]));
	return tmp;
}

template <>
inline SIMD<uint64_t, 8> vclez(SIMD<int64_t, 8> a)
{
	SIMD<uint64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_si256(
			_mm256_cmpeq_epi64(a.m[i], _mm256_setzero_si256()),
			_mm256_cmpgt_epi64(_mm256_setzero_si256(), a.m[i]));
	return tmp;
}

template <>
inline SIMD<float, 16> vmin(SIMD<float, 16> a, SIMD<float, 16> b)
{
	SIMD<float, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_min_ps(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<double, 8> vmin(SIMD<double, 8> a, SIMD<double, 8> b)
{
	SIMD<double, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_min_pd(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 64> vmin(SIMD<int8_t, 64> a, SIMD<int8_t, 64> b)
{
	SIMD<int8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_min_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 32> vmin(SIMD<int16_t, 32> a, SIMD<int16_t, 32> b)
{
	SIMD<int16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_min_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 16> vmin(SIMD<int32_t, 16> a, SIMD<int32_t, 16> b)
{
	SIMD<int32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_min_epi32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 16> vmax(SIMD<float, 16> a, SIMD<float, 16> b)
{
	SIMD<float, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_max_ps(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<double, 8> vmax(SIMD<double, 8> a, SIMD<double, 8> b)
{
	SIMD<double, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_max_pd(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 64> vmax(SIMD<int8_t, 64> a, SIMD<int8_t, 64> b)
{
	SIMD<int8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_max_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 32> vmax(SIMD<int16_t, 32> a, SIMD<int16_t, 32> b)
{
	SIMD<int16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_max_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 16> vmax(SIMD<int32_t, 16> a, SIMD<int32_t, 16> b)
{
	SIMD<int32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_max_epi32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 16> vclamp(SIMD<float, 16> x, float a, float b)
{
	SIMD<float, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_min_ps(_mm256_max_ps(x.m[i], _mm256_set1_ps(a)), _mm256_set1_ps(b));
	return tmp;
}

template <>
inline SIMD<double, 8> vclamp(SIMD<double, 8> x, double a, double b)
{
	SIMD<double, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_min_pd(_mm256_max_pd(x.m[i], _mm256_set1_pd(a)), _mm256_set1_pd(b));
	return tmp;
}

template <>
inline SIMD<int8_t, 64> vclamp(SIMD<int8_t, 64> x, int8_t a, int8_t b)
{
	SIMD<int8_t, 64> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_min_epi8(_mm256_max_epi8(x.m[i], _mm256_set1_epi8(a)), _mm256_set1_epi8(b));
	return tmp;
}

template <>
inline SIMD<int16_t, 32> vclamp(SIMD<int16_t, 32> x, int16_t a, int16_t b)
{
	SIMD<int16_t, 32> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_min_epi16(_mm256_max_epi16(x.m[i], _mm256_set1_epi16(a)), _mm256_set1_epi16(b));
	return tmp;
}

template <>
inline SIMD<int32_t, 16> vclamp(SIMD<int32_t, 16> x, int32_t a, int32_t b)
{
	SIMD<int32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_min_epi32(_mm256_max_epi32(x.m[i], _mm256_set1_epi32(a)), _mm256_set1_epi32(b));
	return tmp;
}

template <>
inline SIMD<int64_t, 8> vclamp(SIMD<int64_t, 8> x, int64_t a, int64_t b)
{
	SIMD<int64_t, 8> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_min_epi64(_mm256_max_epi64(x.m[i], _mm256_set1_epi64x(a)), _mm256_set1_epi64x(b));
	return tmp;
}

template <>
inline SIMD<uint8_t, 64> vshuf(SIMD<uint8_t, 64> a, SIMD<uint8_t, 64> b)
{
	SIMD<uint8_t, 64> tmp;
	for (int i = 0; i < 2; ++i) {
		__m256i c = _mm256_or_si256(b.m[i], _mm256_cmpgt_epi8(b.m[i], _mm256_set1_epi8(15)));
		__m256i d = _mm256_or_si256(_mm256_sub_epi8(b.m[i], _mm256_set1_epi8(16)), _mm256_cmpgt_epi8(b.m[i], _mm256_set1_epi8(31)));
		__m256i e = _mm256_or_si256(_mm256_sub_epi8(b.m[i], _mm256_set1_epi8(32)), _mm256_cmpgt_epi8(b.m[i], _mm256_set1_epi8(47)));
		__m256i f = _mm256_sub_epi8(b.m[i], _mm256_set1_epi8(48));
		__m256i g = _mm256_or_si256(
			_mm256_shuffle_epi8(_mm256_permute2x128_si256(a.m[0], a.m[0], 0), c),
			_mm256_shuffle_epi8(_mm256_permute2x128_si256(a.m[0], a.m[0], 17), d));
		__m256i h = _mm256_or_si256(
			_mm256_shuffle_epi8(_mm256_permute2x128_si256(a.m[1], a.m[1], 0), e),
			_mm256_shuffle_epi8(_mm256_permute2x128_si256(a.m[1], a.m[1], 17), f));
		tmp.m[i] = _mm256_or_si256(g, h);
	}
	return tmp;
}

template <>
inline SIMD<int8_t, 64> vshuf(SIMD<int8_t, 64> a, SIMD<uint8_t, 64> b)
{
	SIMD<int8_t, 64> tmp;
	for (int i = 0; i < 2; ++i) {
		__m256i c = _mm256_or_si256(b.m[i], _mm256_cmpgt_epi8(b.m[i], _mm256_set1_epi8(15)));
		__m256i d = _mm256_or_si256(_mm256_sub_epi8(b.m[i], _mm256_set1_epi8(16)), _mm256_cmpgt_epi8(b.m[i], _mm256_set1_epi8(31)));
		__m256i e = _mm256_or_si256(_mm256_sub_epi8(b.m[i], _mm256_set1_epi8(32)), _mm256_cmpgt_epi8(b.m[i], _mm256_set1_epi8(47)));
		__m256i f = _mm256_sub_epi8(b.m[i], _mm256_set1_epi8(48));
		__m256i g = _mm256_or_si256(
			_mm256_shuffle_epi8(_mm256_permute2x128_si256(a.m[0], a.m[0], 0), c),
			_mm256_shuffle_epi8(_mm256_permute2x128_si256(a.m[0], a.m[0], 17), d));
		__m256i h = _mm256_or_si256(
			_mm256_shuffle_epi8(_mm256_permute2x128_si256(a.m[1], a.m[1], 0), e),
			_mm256_shuffle_epi8(_mm256_permute2x128_si256(a.m[1], a.m[1], 17), f));
		tmp.m[i] = _mm256_or_si256(g, h);
	}
	return tmp;
}

template <>
inline SIMD<uint32_t, 16> vshuf(SIMD<uint32_t, 16> a, SIMD<uint32_t, 16> b)
{
	SIMD<uint32_t, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_si256(
			_mm256_and_si256(_mm256_permutevar8x32_epi32(a.m[0], b.m[i]), _mm256_cmpgt_epi32(_mm256_set1_epi32(8), b.m[i])),
			_mm256_and_si256(_mm256_permutevar8x32_epi32(a.m[1], _mm256_sub_epi32(b.m[i], _mm256_set1_epi32(8))), _mm256_cmpgt_epi32(b.m[i], _mm256_set1_epi32(7))));
	return tmp;
}

template <>
inline SIMD<float, 16> vshuf(SIMD<float, 16> a, SIMD<uint32_t, 16> b)
{
	SIMD<float, 16> tmp;
	for (int i = 0; i < 2; ++i)
		tmp.m[i] = _mm256_or_ps(
			_mm256_and_ps(_mm256_permutevar8x32_ps(a.m[0], b.m[i]), (__m256)_mm256_cmpgt_epi32(_mm256_set1_epi32(8), b.m[i])),
			_mm256_and_ps(_mm256_permutevar8x32_ps(a.m[1], _mm256_sub_epi32(b.m[i], _mm256_set1_epi32(8))), (__m256)_mm256_cmpgt_epi32(b.m[i], _mm256_set1_epi32(7))));
	return tmp;
}

