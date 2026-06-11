/*
Intel SSE4.1 acceleration times three

Copyright 2024 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include <smmintrin.h>

template <>
union SIMD<float, 12>
{
	static const int SIZE = 12;
	typedef float value_type;
	typedef uint32_t uint_type;
	__m128 m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<double, 6>
{
	static const int SIZE = 6;
	typedef double value_type;
	typedef uint64_t uint_type;
	__m128d m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<int8_t, 48>
{
	static const int SIZE = 48;
	typedef int8_t value_type;
	typedef uint8_t uint_type;
	__m128i m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<int16_t, 24>
{
	static const int SIZE = 24;
	typedef int16_t value_type;
	typedef uint16_t uint_type;
	__m128i m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<int32_t, 12>
{
	static const int SIZE = 12;
	typedef int32_t value_type;
	typedef uint32_t uint_type;
	__m128i m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<int64_t, 6>
{
	static const int SIZE = 6;
	typedef int64_t value_type;
	typedef uint64_t uint_type;
	__m128i m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<uint8_t, 48>
{
	static const int SIZE = 48;
	typedef uint8_t value_type;
	typedef uint8_t uint_type;
	__m128i m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<uint16_t, 24>
{
	static const int SIZE = 24;
	typedef uint16_t value_type;
	typedef uint16_t uint_type;
	__m128i m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<uint32_t, 12>
{
	static const int SIZE = 12;
	typedef uint32_t value_type;
	typedef uint32_t uint_type;
	__m128i m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<uint64_t, 6>
{
	static const int SIZE = 6;
	typedef uint64_t value_type;
	typedef uint64_t uint_type;
	__m128i m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
inline SIMD<float, 12> vreinterpret(SIMD<uint32_t, 12> a)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vreinterpret(SIMD<float, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)a.m[i];
	return tmp;
}

template <>
inline SIMD<double, 6> vreinterpret(SIMD<uint64_t, 6> a)
{
	SIMD<double, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128d)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vreinterpret(SIMD<double, 6> a)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vreinterpret(SIMD<int8_t, 48> a)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)a.m[i];
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vreinterpret(SIMD<uint8_t, 48> a)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vreinterpret(SIMD<int16_t, 24> a)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)a.m[i];
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vreinterpret(SIMD<uint16_t, 24> a)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vreinterpret(SIMD<int32_t, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)a.m[i];
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vreinterpret(SIMD<uint32_t, 12> a)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vreinterpret(SIMD<int64_t, 6> a)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)a.m[i];
	return tmp;
}

template <>
inline SIMD<int64_t, 6> vreinterpret(SIMD<uint64_t, 6> a)
{
	SIMD<int64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)a.m[i];
	return tmp;
}

template <>
inline SIMD<float, 12> vdup<SIMD<float, 12>>(float a)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_set1_ps(a);
	return tmp;
}

template <>
inline SIMD<double, 6> vdup<SIMD<double, 6>>(double a)
{
	SIMD<double, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_set1_pd(a);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vdup<SIMD<int8_t, 48>>(int8_t a)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_set1_epi8(a);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vdup<SIMD<int16_t, 24>>(int16_t a)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_set1_epi16(a);
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vdup<SIMD<int32_t, 12>>(int32_t a)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_set1_epi32(a);
	return tmp;
}

template <>
inline SIMD<int64_t, 6> vdup<SIMD<int64_t, 6>>(int64_t a)
{
	SIMD<int64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_set1_epi64x(a);
	return tmp;
}

template <>
inline SIMD<float, 12> vzero()
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_setzero_ps();
	return tmp;
}

template <>
inline SIMD<double, 6> vzero()
{
	SIMD<double, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_setzero_pd();
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vzero()
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_setzero_si128();
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vzero()
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_setzero_si128();
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vzero()
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_setzero_si128();
	return tmp;
}

template <>
inline SIMD<int64_t, 6> vzero()
{
	SIMD<int64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_setzero_si128();
	return tmp;
}

template <>
inline SIMD<float, 12> vadd(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_add_ps(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<double, 6> vadd(SIMD<double, 6> a, SIMD<double, 6> b)
{
	SIMD<double, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_add_pd(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vadd(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_add_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vadd(SIMD<int16_t, 24> a, SIMD<int16_t, 24> b)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_add_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vadd(SIMD<int32_t, 12> a, SIMD<int32_t, 12> b)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_add_epi32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int64_t, 6> vadd(SIMD<int64_t, 6> a, SIMD<int64_t, 6> b)
{
	SIMD<int64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_add_epi64(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vqadd(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_adds_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vqadd(SIMD<int16_t, 24> a, SIMD<int16_t, 24> b)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_adds_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 12> vsub(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_sub_ps(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<double, 6> vsub(SIMD<double, 6> a, SIMD<double, 6> b)
{
	SIMD<double, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_sub_pd(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vsub(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_sub_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vsub(SIMD<int16_t, 24> a, SIMD<int16_t, 24> b)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_sub_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vsub(SIMD<int32_t, 12> a, SIMD<int32_t, 12> b)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_sub_epi32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int64_t, 6> vsub(SIMD<int64_t, 6> a, SIMD<int64_t, 6> b)
{
	SIMD<int64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_sub_epi64(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vqsub(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_subs_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vqsub(SIMD<int16_t, 24> a, SIMD<int16_t, 24> b)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_subs_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vqsub(SIMD<uint8_t, 48> a, SIMD<uint8_t, 48> b)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_subs_epu8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vqsub(SIMD<uint16_t, 24> a, SIMD<uint16_t, 24> b)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_subs_epu16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 12> vmul(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_mul_ps(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<double, 6> vmul(SIMD<double, 6> a, SIMD<double, 6> b)
{
	SIMD<double, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_mul_pd(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 12> vabs(SIMD<float, 12> a)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_andnot_ps(_mm_set1_ps(-0.f), a.m[i]);
	return tmp;
}

template <>
inline SIMD<double, 6> vabs(SIMD<double, 6> a)
{
	SIMD<double, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_andnot_pd(_mm_set1_pd(-0.), a.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vqabs(SIMD<int8_t, 48> a)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_abs_epi8(_mm_max_epi8(a.m[i], _mm_set1_epi8(-INT8_MAX)));
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vqabs(SIMD<int16_t, 24> a)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_abs_epi16(_mm_max_epi16(a.m[i], _mm_set1_epi16(-INT16_MAX)));
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vqabs(SIMD<int32_t, 12> a)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_abs_epi32(_mm_max_epi32(a.m[i], _mm_set1_epi32(-INT32_MAX)));
	return tmp;
}

template <>
inline SIMD<float, 12> vsignum(SIMD<float, 12> a)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_andnot_ps(
			_mm_cmpeq_ps(a.m[i], _mm_setzero_ps()),
			_mm_or_ps(_mm_set1_ps(1.f), _mm_and_ps(_mm_set1_ps(-0.f), a.m[i])));
	return tmp;
}

template <>
inline SIMD<double, 6> vsignum(SIMD<double, 6> a)
{
	SIMD<double, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_andnot_pd(
			_mm_cmpeq_pd(a.m[i], _mm_setzero_pd()),
			_mm_or_pd(_mm_set1_pd(1.), _mm_and_pd(_mm_set1_pd(-0.), a.m[i])));
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vsignum(SIMD<int8_t, 48> a)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_sign_epi8(_mm_set1_epi8(1), a.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vsignum(SIMD<int16_t, 24> a)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_sign_epi16(_mm_set1_epi16(1), a.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vsignum(SIMD<int32_t, 12> a)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_sign_epi32(_mm_set1_epi32(1), a.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 12> vsign(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_andnot_ps(
			_mm_cmpeq_ps(b.m[i], _mm_setzero_ps()),
			_mm_xor_ps(a.m[i], _mm_and_ps(_mm_set1_ps(-0.f), b.m[i])));
	return tmp;
}

template <>
inline SIMD<double, 6> vsign(SIMD<double, 6> a, SIMD<double, 6> b)
{
	SIMD<double, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_andnot_pd(
			_mm_cmpeq_pd(b.m[i], _mm_setzero_pd()),
			_mm_xor_pd(a.m[i], _mm_and_pd(_mm_set1_pd(-0.), b.m[i])));
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vsign(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_sign_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vsign(SIMD<int16_t, 24> a, SIMD<int16_t, 24> b)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_sign_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vsign(SIMD<int32_t, 12> a, SIMD<int32_t, 12> b)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_sign_epi32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 12> vcopysign(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_ps(
			_mm_andnot_ps(_mm_set1_ps(-0.f), a.m[i]),
			_mm_and_ps(_mm_set1_ps(-0.f), b.m[i]));
	return tmp;
}

template <>
inline SIMD<double, 6> vcopysign(SIMD<double, 6> a, SIMD<double, 6> b)
{
	SIMD<double, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_pd(
			_mm_andnot_pd(_mm_set1_pd(-0.), a.m[i]),
			_mm_and_pd(_mm_set1_pd(-0.), b.m[i]));
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vorr(SIMD<uint8_t, 48> a, SIMD<uint8_t, 48> b)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_si128(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vorr(SIMD<uint16_t, 24> a, SIMD<uint16_t, 24> b)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_si128(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vorr(SIMD<uint32_t, 12> a, SIMD<uint32_t, 12> b)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_si128(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vorr(SIMD<uint64_t, 6> a, SIMD<uint64_t, 6> b)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_si128(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vand(SIMD<uint8_t, 48> a, SIMD<uint8_t, 48> b)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_and_si128(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vand(SIMD<uint16_t, 24> a, SIMD<uint16_t, 24> b)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_and_si128(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vand(SIMD<uint32_t, 12> a, SIMD<uint32_t, 12> b)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_and_si128(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vand(SIMD<uint64_t, 6> a, SIMD<uint64_t, 6> b)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_and_si128(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> veor(SIMD<uint8_t, 48> a, SIMD<uint8_t, 48> b)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_xor_si128(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> veor(SIMD<uint16_t, 24> a, SIMD<uint16_t, 24> b)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_xor_si128(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> veor(SIMD<uint32_t, 12> a, SIMD<uint32_t, 12> b)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_xor_si128(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> veor(SIMD<uint64_t, 6> a, SIMD<uint64_t, 6> b)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_xor_si128(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vbic(SIMD<uint8_t, 48> a, SIMD<uint8_t, 48> b)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_andnot_si128(b.m[i], a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vbic(SIMD<uint16_t, 24> a, SIMD<uint16_t, 24> b)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_andnot_si128(b.m[i], a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vbic(SIMD<uint32_t, 12> a, SIMD<uint32_t, 12> b)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_andnot_si128(b.m[i], a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vbic(SIMD<uint64_t, 6> a, SIMD<uint64_t, 6> b)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_andnot_si128(b.m[i], a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vbsl(SIMD<uint8_t, 48> a, SIMD<uint8_t, 48> b, SIMD<uint8_t, 48> c)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_si128(_mm_and_si128(a.m[i], b.m[i]), _mm_andnot_si128(a.m[i], c.m[i]));
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vbsl(SIMD<uint16_t, 24> a, SIMD<uint16_t, 24> b, SIMD<uint16_t, 24> c)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_si128(_mm_and_si128(a.m[i], b.m[i]), _mm_andnot_si128(a.m[i], c.m[i]));
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vbsl(SIMD<uint32_t, 12> a, SIMD<uint32_t, 12> b, SIMD<uint32_t, 12> c)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_si128(_mm_and_si128(a.m[i], b.m[i]), _mm_andnot_si128(a.m[i], c.m[i]));
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vbsl(SIMD<uint64_t, 6> a, SIMD<uint64_t, 6> b, SIMD<uint64_t, 6> c)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_si128(_mm_and_si128(a.m[i], b.m[i]), _mm_andnot_si128(a.m[i], c.m[i]));
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vceqz(SIMD<float, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)_mm_cmpeq_ps(a.m[i], _mm_setzero_ps());
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vceqz(SIMD<double, 6> a)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)_mm_cmpeq_pd(a.m[i], _mm_setzero_pd());
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vceqz(SIMD<int8_t, 48> a)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpeq_epi8(a.m[i], _mm_setzero_si128());
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vceqz(SIMD<int16_t, 24> a)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpeq_epi16(a.m[i], _mm_setzero_si128());
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vceqz(SIMD<int32_t, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpeq_epi32(a.m[i], _mm_setzero_si128());
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vceqz(SIMD<int64_t, 6> a)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpeq_epi64(a.m[i], _mm_setzero_si128());
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vceq(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)_mm_cmpeq_ps(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vceq(SIMD<double, 6> a, SIMD<double, 6> b)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)_mm_cmpeq_pd(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vceq(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpeq_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vceq(SIMD<int16_t, 24> a, SIMD<int16_t, 24> b)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpeq_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vceq(SIMD<int32_t, 12> a, SIMD<int32_t, 12> b)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpeq_epi32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vceq(SIMD<int64_t, 6> a, SIMD<int64_t, 6> b)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpeq_epi64(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vcgtz(SIMD<float, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)_mm_cmpgt_ps(a.m[i], _mm_setzero_ps());
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vcgtz(SIMD<double, 6> a)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)_mm_cmpgt_pd(a.m[i], _mm_setzero_pd());
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vcgtz(SIMD<int8_t, 48> a)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpgt_epi8(a.m[i], _mm_setzero_si128());
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vcgtz(SIMD<int16_t, 24> a)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpgt_epi16(a.m[i], _mm_setzero_si128());
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vcgtz(SIMD<int32_t, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpgt_epi32(a.m[i], _mm_setzero_si128());
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vcgtz(SIMD<int64_t, 6> a)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpgt_epi64(a.m[i], _mm_setzero_si128());
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vcltz(SIMD<float, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)_mm_cmplt_ps(a.m[i], _mm_setzero_ps());
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vcltz(SIMD<double, 6> a)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)_mm_cmplt_pd(a.m[i], _mm_setzero_pd());
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vcltz(SIMD<int8_t, 48> a)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpgt_epi8(_mm_setzero_si128(), a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vcltz(SIMD<int16_t, 24> a)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpgt_epi16(_mm_setzero_si128(), a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vcltz(SIMD<int32_t, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpgt_epi32(_mm_setzero_si128(), a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vcltz(SIMD<int64_t, 6> a)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_cmpgt_epi64(_mm_setzero_si128(), a.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vclez(SIMD<float, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)_mm_cmple_ps(a.m[i], _mm_setzero_ps());
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vclez(SIMD<double, 6> a)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (__m128i)_mm_cmple_pd(a.m[i], _mm_setzero_pd());
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vclez(SIMD<int8_t, 48> a)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_si128(
			_mm_cmpeq_epi8(a.m[i], _mm_setzero_si128()),
			_mm_cmpgt_epi8(_mm_setzero_si128(), a.m[i]));
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vclez(SIMD<int16_t, 24> a)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_si128(
			_mm_cmpeq_epi16(a.m[i], _mm_setzero_si128()),
			_mm_cmpgt_epi16(_mm_setzero_si128(), a.m[i]));
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vclez(SIMD<int32_t, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_si128(
			_mm_cmpeq_epi32(a.m[i], _mm_setzero_si128()),
			_mm_cmpgt_epi32(_mm_setzero_si128(), a.m[i]));
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vclez(SIMD<int64_t, 6> a)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_si128(
			_mm_cmpeq_epi64(a.m[i], _mm_setzero_si128()),
			_mm_cmpgt_epi64(_mm_setzero_si128(), a.m[i]));
	return tmp;
}

template <>
inline SIMD<float, 12> vmin(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_min_ps(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<double, 6> vmin(SIMD<double, 6> a, SIMD<double, 6> b)
{
	SIMD<double, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_min_pd(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vmin(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_min_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vmin(SIMD<int16_t, 24> a, SIMD<int16_t, 24> b)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_min_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vmin(SIMD<int32_t, 12> a, SIMD<int32_t, 12> b)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_min_epi32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 12> vmax(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_max_ps(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<double, 6> vmax(SIMD<double, 6> a, SIMD<double, 6> b)
{
	SIMD<double, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_max_pd(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vmax(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_max_epi8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vmax(SIMD<int16_t, 24> a, SIMD<int16_t, 24> b)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_max_epi16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vmax(SIMD<int32_t, 12> a, SIMD<int32_t, 12> b)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_max_epi32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 12> vclamp(SIMD<float, 12> x, float a, float b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_min_ps(_mm_max_ps(x.m[i], _mm_set1_ps(a)), _mm_set1_ps(b));
	return tmp;
}

template <>
inline SIMD<double, 6> vclamp(SIMD<double, 6> x, double a, double b)
{
	SIMD<double, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_min_pd(_mm_max_pd(x.m[i], _mm_set1_pd(a)), _mm_set1_pd(b));
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vclamp(SIMD<int8_t, 48> x, int8_t a, int8_t b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_min_epi8(_mm_max_epi8(x.m[i], _mm_set1_epi8(a)), _mm_set1_epi8(b));
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vclamp(SIMD<int16_t, 24> x, int16_t a, int16_t b)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_min_epi16(_mm_max_epi16(x.m[i], _mm_set1_epi16(a)), _mm_set1_epi16(b));
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vclamp(SIMD<int32_t, 12> x, int32_t a, int32_t b)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_min_epi32(_mm_max_epi32(x.m[i], _mm_set1_epi32(a)), _mm_set1_epi32(b));
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vshuf(SIMD<uint8_t, 48> a, SIMD<uint8_t, 48> b)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_si128(_mm_or_si128(
			_mm_shuffle_epi8(a.m[0], _mm_or_si128(b.m[i], _mm_cmpgt_epi8(b.m[i], _mm_set1_epi8(15)))),
			_mm_shuffle_epi8(a.m[1], _mm_or_si128(_mm_sub_epi8(b.m[i], _mm_set1_epi8(16)), _mm_cmpgt_epi8(b.m[i], _mm_set1_epi8(31))))),
			_mm_shuffle_epi8(a.m[2], _mm_sub_epi8(b.m[i], _mm_set1_epi8(32))));
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vshuf(SIMD<int8_t, 48> a, SIMD<uint8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = _mm_or_si128(_mm_or_si128(
			_mm_shuffle_epi8(a.m[0], _mm_or_si128(b.m[i], _mm_cmpgt_epi8(b.m[i], _mm_set1_epi8(15)))),
			_mm_shuffle_epi8(a.m[1], _mm_or_si128(_mm_sub_epi8(b.m[i], _mm_set1_epi8(16)), _mm_cmpgt_epi8(b.m[i], _mm_set1_epi8(31))))),
			_mm_shuffle_epi8(a.m[2], _mm_sub_epi8(b.m[i], _mm_set1_epi8(32))));
	return tmp;
}

