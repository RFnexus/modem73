/*
ARM NEON acceleration times three

Copyright 2024 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include <arm_neon.h>

template <>
union SIMD<float, 12>
{
	static const int SIZE = 12;
	typedef float value_type;
	typedef uint32_t uint_type;
	float32x4_t m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<int8_t, 48>
{
	static const int SIZE = 48;
	typedef int8_t value_type;
	typedef uint8_t uint_type;
	int8x16_t m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<int16_t, 24>
{
	static const int SIZE = 24;
	typedef int16_t value_type;
	typedef uint16_t uint_type;
	int16x8_t m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<int32_t, 12>
{
	static const int SIZE = 12;
	typedef int32_t value_type;
	typedef uint32_t uint_type;
	int32x4_t m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<int64_t, 6>
{
	static const int SIZE = 6;
	typedef int64_t value_type;
	typedef uint64_t uint_type;
	int64x2_t m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<uint8_t, 48>
{
	static const int SIZE = 48;
	typedef uint8_t value_type;
	typedef uint8_t uint_type;
	uint8x16_t m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<uint16_t, 24>
{
	static const int SIZE = 24;
	typedef uint16_t value_type;
	typedef uint16_t uint_type;
	uint16x8_t m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<uint32_t, 12>
{
	static const int SIZE = 12;
	typedef uint32_t value_type;
	typedef uint32_t uint_type;
	uint32x4_t m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
union SIMD<uint64_t, 6>
{
	static const int SIZE = 6;
	typedef uint64_t value_type;
	typedef uint64_t uint_type;
	uint64x2_t m[3];
	value_type v[SIZE];
	uint_type u[SIZE];
};

template <>
inline SIMD<float, 12> vreinterpret(SIMD<uint32_t, 12> a)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (float32x4_t)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vreinterpret(SIMD<float, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (uint32x4_t)a.m[i];
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vreinterpret(SIMD<uint8_t, 48> a)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (int8x16_t)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vreinterpret(SIMD<int8_t, 48> a)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (uint8x16_t)a.m[i];
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vreinterpret(SIMD<uint16_t, 24> a)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (int16x8_t)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vreinterpret(SIMD<int16_t, 24> a)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (uint16x8_t)a.m[i];
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vreinterpret(SIMD<uint32_t, 12> a)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (int32x4_t)a.m[i];
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vreinterpret(SIMD<int32_t, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (uint32x4_t)a.m[i];
	return tmp;
}

template <>
inline SIMD<float, 12> vdup(float a)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vdupq_n_f32(a);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vdup(int8_t a)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vdupq_n_s8(a);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vdup(int16_t a)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vdupq_n_s16(a);
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vdup(int32_t a)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vdupq_n_s32(a);
	return tmp;
}

template <>
inline SIMD<int64_t, 6> vdup(int64_t a)
{
	SIMD<int64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vdupq_n_s64(a);
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vdup(uint8_t a)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vdupq_n_u8(a);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vdup(uint16_t a)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vdupq_n_u16(a);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vdup(uint32_t a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vdupq_n_u32(a);
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vdup(uint64_t a)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vdupq_n_u64(a);
	return tmp;
}

template <>
inline SIMD<float, 12> vzero()
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (float32x4_t)veorq_u32((uint32x4_t)tmp.m[i], (uint32x4_t)tmp.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vzero()
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = veorq_s8(tmp.m[i], tmp.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vzero()
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = veorq_s16(tmp.m[i], tmp.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vzero()
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = veorq_s32(tmp.m[i], tmp.m[i]);
	return tmp;
}

template <>
inline SIMD<int64_t, 6> vzero()
{
	SIMD<int64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = veorq_s64(tmp.m[i], tmp.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vzero()
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = veorq_u8(tmp.m[i], tmp.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vzero()
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = veorq_u16(tmp.m[i], tmp.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vzero()
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = veorq_u32(tmp.m[i], tmp.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vzero()
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = veorq_u64(tmp.m[i], tmp.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 12> vadd(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vaddq_f32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vadd(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vaddq_s8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vadd(SIMD<int16_t, 24> a, SIMD<int16_t, 24> b)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vaddq_s16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vadd(SIMD<int32_t, 12> a, SIMD<int32_t, 12> b)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vaddq_s32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int64_t, 6> vadd(SIMD<int64_t, 6> a, SIMD<int64_t, 6> b)
{
	SIMD<int64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vaddq_s64(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vqadd(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vqaddq_s8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vqadd(SIMD<int16_t, 24> a, SIMD<int16_t, 24> b)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vqaddq_s16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 12> vsub(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vsubq_f32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vsub(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vsubq_s8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vsub(SIMD<int16_t, 24> a, SIMD<int16_t, 24> b)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vsubq_s16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vsub(SIMD<int32_t, 12> a, SIMD<int32_t, 12> b)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vsubq_s32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int64_t, 6> vsub(SIMD<int64_t, 6> a, SIMD<int64_t, 6> b)
{
	SIMD<int64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vsubq_s64(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vqsub(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vqsubq_s8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vqsub(SIMD<int16_t, 24> a, SIMD<int16_t, 24> b)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vqsubq_s16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vqsub(SIMD<uint8_t, 48> a, SIMD<uint8_t, 48> b)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vqsubq_u8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vqsub(SIMD<uint16_t, 24> a, SIMD<uint16_t, 24> b)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vqsubq_u16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 12> vmul(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vmulq_f32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vmul(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vmulq_s8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 12> vabs(SIMD<float, 12> a)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vabsq_f32(a.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vqabs(SIMD<int8_t, 48> a)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vqabsq_s8(a.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vqabs(SIMD<int16_t, 24> a)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vqabsq_s16(a.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 12> vsignum(SIMD<float, 12> a)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (float32x4_t)vbicq_u32(
			veorq_u32((uint32x4_t)vdupq_n_f32(1.f), vandq_u32((uint32x4_t)vdupq_n_f32(-0.f), (uint32x4_t)a.m[i])),
			vceqq_f32(a.m[i], vdupq_n_f32(0.f)));
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vsignum(SIMD<int8_t, 48> a)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (int8x16_t)vorrq_u8(vcgtq_s8(vdupq_n_s8(0), a.m[i]),
			vandq_u8(vcgtq_s8(a.m[i], vdupq_n_s8(0)), (uint8x16_t)vdupq_n_s8(1)));
	return tmp;
}

template <>
inline SIMD<float, 12> vsign(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (float32x4_t)vbicq_u32(
			veorq_u32((uint32x4_t)a.m[i], vandq_u32((uint32x4_t)vdupq_n_f32(-0.f), (uint32x4_t)b.m[i])),
			vceqq_f32(b.m[i], vdupq_n_f32(0.f)));
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vsign(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (int8x16_t)vorrq_u8(
			vandq_u8(vcgtq_s8(vdupq_n_s8(0), b.m[i]), (uint8x16_t)vnegq_s8(a.m[i])),
			vandq_u8(vcgtq_s8(b.m[i], vdupq_n_s8(0)), (uint8x16_t)a.m[i]));
	return tmp;
}

template <>
inline SIMD<float, 12> vcopysign(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = (float32x4_t)vorrq_u32(
			vbicq_u32((uint32x4_t)a.m[i], (uint32x4_t)vdupq_n_f32(-0.f)),
			vandq_u32((uint32x4_t)b.m[i], (uint32x4_t)vdupq_n_f32(-0.f)));
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vorr(SIMD<uint8_t, 48> a, SIMD<uint8_t, 48> b)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vorrq_u8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vorr(SIMD<uint16_t, 24> a, SIMD<uint16_t, 24> b)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vorrq_u16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vorr(SIMD<uint32_t, 12> a, SIMD<uint32_t, 12> b)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vorrq_u32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vorr(SIMD<uint64_t, 6> a, SIMD<uint64_t, 6> b)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vorrq_u64(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vand(SIMD<uint8_t, 48> a, SIMD<uint8_t, 48> b)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vandq_u8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vand(SIMD<uint16_t, 24> a, SIMD<uint16_t, 24> b)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vandq_u16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vand(SIMD<uint32_t, 12> a, SIMD<uint32_t, 12> b)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vandq_u32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vand(SIMD<uint64_t, 6> a, SIMD<uint64_t, 6> b)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vandq_u64(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> veor(SIMD<uint8_t, 48> a, SIMD<uint8_t, 48> b)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = veorq_u8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> veor(SIMD<uint16_t, 24> a, SIMD<uint16_t, 24> b)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = veorq_u16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> veor(SIMD<uint32_t, 12> a, SIMD<uint32_t, 12> b)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = veorq_u32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> veor(SIMD<uint64_t, 6> a, SIMD<uint64_t, 6> b)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = veorq_u64(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vbic(SIMD<uint8_t, 48> a, SIMD<uint8_t, 48> b)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vbicq_u8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vbic(SIMD<uint16_t, 24> a, SIMD<uint16_t, 24> b)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vbicq_u16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vbic(SIMD<uint32_t, 12> a, SIMD<uint32_t, 12> b)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vbicq_u32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vbic(SIMD<uint64_t, 6> a, SIMD<uint64_t, 6> b)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vbicq_u64(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vbsl(SIMD<uint8_t, 48> a, SIMD<uint8_t, 48> b, SIMD<uint8_t, 48> c)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vbslq_u8(a.m[i], b.m[i], c.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vbsl(SIMD<uint16_t, 24> a, SIMD<uint16_t, 24> b, SIMD<uint16_t, 24> c)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vbslq_u16(a.m[i], b.m[i], c.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vbsl(SIMD<uint32_t, 12> a, SIMD<uint32_t, 12> b, SIMD<uint32_t, 12> c)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vbslq_u32(a.m[i], b.m[i], c.m[i]);
	return tmp;
}

template <>
inline SIMD<uint64_t, 6> vbsl(SIMD<uint64_t, 6> a, SIMD<uint64_t, 6> b, SIMD<uint64_t, 6> c)
{
	SIMD<uint64_t, 6> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vbslq_u64(a.m[i], b.m[i], c.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vceqz(SIMD<float, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vceqq_f32(a.m[i], vdupq_n_f32(0.f));
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vceqz(SIMD<int8_t, 48> a)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vceqq_s8(a.m[i], vdupq_n_s8(0));
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vceqz(SIMD<int16_t, 24> a)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vceqq_s16(a.m[i], vdupq_n_s16(0));
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vceqz(SIMD<int32_t, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vceqq_s32(a.m[i], vdupq_n_s32(0));
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vceq(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vceqq_f32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vceq(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vceqq_s8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vceq(SIMD<int16_t, 24> a, SIMD<int16_t, 24> b)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vceqq_s16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vceq(SIMD<int32_t, 12> a, SIMD<int32_t, 12> b)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vceqq_s32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vcgtz(SIMD<float, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vcgtq_f32(a.m[i], vdupq_n_f32(0.f));
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vcgtz(SIMD<int8_t, 48> a)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vcgtq_s8(a.m[i], vdupq_n_s8(0));
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vcgtz(SIMD<int16_t, 24> a)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vcgtq_s16(a.m[i], vdupq_n_s16(0));
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vcgtz(SIMD<int32_t, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vcgtq_s32(a.m[i], vdupq_n_s32(0));
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vcltz(SIMD<float, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vcltq_f32(a.m[i], vdupq_n_f32(0.f));
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vcltz(SIMD<int8_t, 48> a)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vcltq_s8(a.m[i], vdupq_n_s8(0));
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vcltz(SIMD<int16_t, 24> a)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vcltq_s16(a.m[i], vdupq_n_s16(0));
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vcltz(SIMD<int32_t, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vcltq_s32(a.m[i], vdupq_n_s32(0));
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vclez(SIMD<float, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vcleq_f32(a.m[i], vdupq_n_f32(0.f));
	return tmp;
}

template <>
inline SIMD<uint8_t, 48> vclez(SIMD<int8_t, 48> a)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vcleq_s8(a.m[i], vdupq_n_s8(0));
	return tmp;
}

template <>
inline SIMD<uint16_t, 24> vclez(SIMD<int16_t, 24> a)
{
	SIMD<uint16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vcleq_s16(a.m[i], vdupq_n_s16(0));
	return tmp;
}

template <>
inline SIMD<uint32_t, 12> vclez(SIMD<int32_t, 12> a)
{
	SIMD<uint32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vcleq_s32(a.m[i], vdupq_n_s32(0));
	return tmp;
}

template <>
inline SIMD<float, 12> vmin(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vminq_f32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vmin(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vminq_s8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vmin(SIMD<int16_t, 24> a, SIMD<int16_t, 24> b)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vminq_s16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vmin(SIMD<int32_t, 12> a, SIMD<int32_t, 12> b)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vminq_s32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 12> vmax(SIMD<float, 12> a, SIMD<float, 12> b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vmaxq_f32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vmax(SIMD<int8_t, 48> a, SIMD<int8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vmaxq_s8(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vmax(SIMD<int16_t, 24> a, SIMD<int16_t, 24> b)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vmaxq_s16(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vmax(SIMD<int32_t, 12> a, SIMD<int32_t, 12> b)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vmaxq_s32(a.m[i], b.m[i]);
	return tmp;
}

template <>
inline SIMD<float, 12> vclamp(SIMD<float, 12> x, float a, float b)
{
	SIMD<float, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vminq_f32(vmaxq_f32(x.m[i], vdupq_n_f32(a)), vdupq_n_f32(b));
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vclamp(SIMD<int8_t, 48> x, int8_t a, int8_t b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vminq_s8(vmaxq_s8(x.m[i], vdupq_n_s8(a)), vdupq_n_s8(b));
	return tmp;
}

template <>
inline SIMD<int16_t, 24> vclamp(SIMD<int16_t, 24> x, int16_t a, int16_t b)
{
	SIMD<int16_t, 24> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vminq_s16(vmaxq_s16(x.m[i], vdupq_n_s16(a)), vdupq_n_s16(b));
	return tmp;
}

template <>
inline SIMD<int32_t, 12> vclamp(SIMD<int32_t, 12> x, int32_t a, int32_t b)
{
	SIMD<int32_t, 12> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vminq_s32(vmaxq_s32(x.m[i], vdupq_n_s32(a)), vdupq_n_s32(b));
	return tmp;
}

#ifdef __aarch64__
template <>
inline SIMD<uint8_t, 48> vshuf(SIMD<uint8_t, 48> a, SIMD<uint8_t, 48> b)
{
	SIMD<uint8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vorrq_u8(vorrq_u8(vqtbl1q_u8(a.m[0], b.m[i]),
			vqtbl1q_u8(a.m[1], vsubq_u8(b.m[i], vdupq_n_u8(16)))),
			vqtbl1q_u8(a.m[2], vsubq_u8(b.m[i], vdupq_n_u8(32))));
	return tmp;
}

template <>
inline SIMD<int8_t, 48> vshuf(SIMD<int8_t, 48> a, SIMD<uint8_t, 48> b)
{
	SIMD<int8_t, 48> tmp;
	for (int i = 0; i < 3; ++i)
		tmp.m[i] = vorrq_s8(vorrq_s8(vqtbl1q_s8(a.m[0], b.m[i]),
			vqtbl1q_s8(a.m[1], vsubq_u8(b.m[i], vdupq_n_u8(16)))),
			vqtbl1q_s8(a.m[2], vsubq_u8(b.m[i], vdupq_n_u8(32))));
	return tmp;
}
#endif

