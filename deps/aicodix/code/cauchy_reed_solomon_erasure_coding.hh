/*
Cauchy Reed Solomon Erasure Coding

Copyright 2023 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif
#ifdef __AVX2__
#include <immintrin.h>
#else
#ifdef __SSE4_1__
#include <smmintrin.h>
#endif
#endif

namespace CODE {

template <typename GF>
struct CauchyReedSolomonErasureCoding
{
	typedef typename GF::value_type value_type;
	typedef typename GF::ValueType ValueType;
	typedef typename GF::IndexType IndexType;
	IndexType row_num, row_den;
	// $a_{ij} = \frac{1}{x_i + y_j}$
	__attribute__((flatten))
	IndexType cauchy_matrix(int i, int j)
	{
		ValueType row(i), col(j);
		return rcp(index(row + col));
	}
	// $b_{ij} = \frac{\prod_{k=1}^{n}{(x_j + y_k)(x_k + y_i)}}{(x_j + y_i)\prod_{k \ne j}^{n}{(x_j - x_k)}\prod_{k \ne i}^{n}{(y_i - y_k)}}$
	__attribute__((flatten))
	IndexType inverse_cauchy_matrix(const ValueType *rows, int i, int j, int n)
	{
#if 0
		ValueType col_i(i);
		IndexType prod_xy(0), prod_x(0), prod_y(0);
		for (int k = 0; k < n; k++) {
			ValueType col_k(k);
			prod_xy *= index(rows[j] + col_k) * index(rows[k] + col_i);
			if (k != j)
				prod_x *= index(rows[j] - rows[k]);
			if (k != i)
				prod_y *= index(col_i - col_k);
		}
		return prod_xy / (index(rows[j] + col_i) * prod_x * prod_y);
#else
		ValueType col_i(i);
		if (j == 0) {
			IndexType num(0), den(0);
			for (int k = 0; k < n; k++) {
				ValueType col_k(k);
				num *= index(rows[k] + col_i);
				if (k != i)
					den *= index(col_i - col_k);
			}
			row_num = num;
			row_den = den;
		}
		IndexType num(row_num), den(row_den);
		for (int k = 0; k < n; k++) {
			ValueType col_k(k);
			num *= index(rows[j] + col_k);
			if (k != j)
				den *= index(rows[j] - rows[k]);
		}
		return num / (index(rows[j] + col_i) * den);
#endif
	}
#if defined(__ARM_NEON) && !defined(__aarch64__)
	static inline uint8x16_t vqtbl1q_u8(uint8x16_t lut, uint8x16_t idx)
	{
		uint8x8x2_t l82 = {{ vget_low_u8(lut), vget_high_u8(lut) }};
		uint8x8_t lo = vtbl2_u8(l82, vget_low_u8(idx));
		uint8x8_t hi = vtbl2_u8(l82, vget_high_u8(idx));
		return vcombine_u8(lo, hi);
	}
#endif
#if defined(__ARM_NEON) || defined(__AVX2__) || defined(__SSE4_1__)
	__attribute__((flatten))
	static inline void mac_simd(uint8_t *c, const uint8_t *a, IndexType b, int size, bool init)
	{
		alignas(16) uint8_t bln[16], bhn[16];
		for (int i = 0; i < 16; ++i) {
			bln[i] = (b * ValueType(i)).v;
			bhn[i] = (b * ValueType(i << 4)).v;
		}
#ifdef __ARM_NEON
		uint8x16_t l16 = vld1q_u8(bln);
		uint8x16_t h16 = vld1q_u8(bhn);
		for (int i = 0; i < size; i += 16, a += 16, c += 16) {
			uint8x16_t a16 = vld1q_u8(reinterpret_cast<const uint8_t *>(__builtin_assume_aligned(a, 16)));
			uint8x16_t aln = vandq_u8(a16, vdupq_n_u8(15));
			uint8x16_t ahn = vshrq_n_u8(a16, 4);
			uint8x16_t cln = vqtbl1q_u8(l16, aln);
			uint8x16_t chn = vqtbl1q_u8(h16, ahn);
			uint8x16_t c16 = veorq_u8(cln, chn);
			if (!init)
				c16 = veorq_u8(c16, vld1q_u8(reinterpret_cast<const uint8_t *>(__builtin_assume_aligned(c, 16))));
			vst1q_u8(reinterpret_cast<uint8_t *>(__builtin_assume_aligned(c, 16)), c16);
		}
#else
#ifdef __AVX2__
		__m128i l16 = _mm_load_si128(reinterpret_cast<const __m128i *>(bln));
		__m128i h16 = _mm_load_si128(reinterpret_cast<const __m128i *>(bhn));
		__m256i l162 = _mm256_broadcastsi128_si256(l16);
		__m256i h162 = _mm256_broadcastsi128_si256(h16);
		for (int i = 0; i < size; i += 32, a += 32, c += 32) {
			__m256i a32 = _mm256_load_si256(reinterpret_cast<const __m256i *>(__builtin_assume_aligned(a, 32)));
			__m256i aln = _mm256_and_si256(a32, _mm256_set1_epi8(15));
			__m256i cln = _mm256_shuffle_epi8(l162, aln);
			__m256i ahn = _mm256_and_si256(_mm256_srli_epi16(a32, 4), _mm256_set1_epi8(15));
			__m256i chn = _mm256_shuffle_epi8(h162, ahn);
			__m256i c32 = _mm256_xor_si256(cln, chn);
			if (!init)
				c32 = _mm256_xor_si256(c32, _mm256_load_si256(reinterpret_cast<const __m256i *>(__builtin_assume_aligned(c, 32))));
			_mm256_store_si256(reinterpret_cast<__m256i *>(__builtin_assume_aligned(c, 32)), c32);
		}
#else
		__m128i l16 = _mm_load_si128(reinterpret_cast<const __m128i *>(bln));
		__m128i h16 = _mm_load_si128(reinterpret_cast<const __m128i *>(bhn));
		for (int i = 0; i < size; i += 16, a += 16, c += 16) {
			__m128i a16 = _mm_load_si128(reinterpret_cast<const __m128i *>(__builtin_assume_aligned(a, 16)));
			__m128i aln = _mm_and_si128(a16, _mm_set1_epi8(15));
			__m128i cln = _mm_shuffle_epi8(l16, aln);
			__m128i ahn = _mm_and_si128(_mm_srli_epi16(a16, 4), _mm_set1_epi8(15));
			__m128i chn = _mm_shuffle_epi8(h16, ahn);
			__m128i c16 = _mm_xor_si128(cln, chn);
			if (!init)
				c16 = _mm_xor_si128(c16, _mm_load_si128(reinterpret_cast<const __m128i *>(__builtin_assume_aligned(c, 16))));
			_mm_store_si128(reinterpret_cast<__m128i *>(__builtin_assume_aligned(c, 16)), c16);
		}
#endif
#endif
	}
	__attribute__((flatten))
	static inline void mac_simd(uint16_t *c, const uint16_t *a, IndexType b, int size, bool init)
	{
		alignas(16) uint8_t blll[16], bllh[16], blhl[16], blhh[16], bhll[16], bhlh[16], bhhl[16], bhhh[16];
		for (int i = 0; i < 16; ++i) {
			uint16_t bll = (b * ValueType(i)).v;
			uint16_t blh = (b * ValueType(i << 4)).v;
			uint16_t bhl = (b * ValueType(i << 8)).v;
			uint16_t bhh = (b * ValueType(i << 12)).v;
			blll[i] = bll; bhll[i] = bll >> 8;
			bllh[i] = blh; bhlh[i] = blh >> 8;
			blhl[i] = bhl; bhhl[i] = bhl >> 8;
			blhh[i] = bhh; bhhh[i] = bhh >> 8;
		}
#ifdef __ARM_NEON
		uint8x16_t lll16 = vld1q_u8(blll);
		uint8x16_t llh16 = vld1q_u8(bllh);
		uint8x16_t lhl16 = vld1q_u8(blhl);
		uint8x16_t lhh16 = vld1q_u8(blhh);
		uint8x16_t hll16 = vld1q_u8(bhll);
		uint8x16_t hlh16 = vld1q_u8(bhlh);
		uint8x16_t hhl16 = vld1q_u8(bhhl);
		uint8x16_t hhh16 = vld1q_u8(bhhh);
		for (int i = 0; i < size; i += 16, a += 16, c += 16) {
			uint16x8_t al8 = vld1q_u16(reinterpret_cast<const uint16_t *>(__builtin_assume_aligned(a, 16)));
			uint16x8_t ah8 = vld1q_u16(reinterpret_cast<const uint16_t *>(__builtin_assume_aligned(a+8, 16)));
			uint8x16_t al16 = (uint8x16_t)vorrq_u16(vshlq_n_u16(ah8, 8), vandq_u16(al8, vdupq_n_u16(255)));
			uint8x16_t ah16 = (uint8x16_t)vorrq_u16(vandq_u16(ah8, vdupq_n_u16(0xff00)), vshrq_n_u16(al8, 8));
			uint8x16_t alln = vandq_u8(al16, vdupq_n_u8(15));
			uint8x16_t clll = vqtbl1q_u8(lll16, alln);
			uint8x16_t chll = vqtbl1q_u8(hll16, alln);
			uint8x16_t alhn = vshrq_n_u8(al16, 4);
			uint8x16_t cllh = vqtbl1q_u8(llh16, alhn);
			uint8x16_t chlh = vqtbl1q_u8(hlh16, alhn);
			uint8x16_t ahln = vandq_u8(ah16, vdupq_n_u8(15));
			uint8x16_t clhl = vqtbl1q_u8(lhl16, ahln);
			uint8x16_t chhl = vqtbl1q_u8(hhl16, ahln);
			uint8x16_t ahhn = vshrq_n_u8(ah16, 4);
			uint8x16_t clhh = vqtbl1q_u8(lhh16, ahhn);
			uint8x16_t chhh = vqtbl1q_u8(hhh16, ahhn);
			uint16x8_t cl16 = (uint16x8_t)veorq_u8(veorq_u8(clll, cllh), veorq_u8(clhl, clhh));
			uint16x8_t ch16 = (uint16x8_t)veorq_u8(veorq_u8(chll, chlh), veorq_u8(chhl, chhh));
			uint16x8_t cl8 = vorrq_u16(vshlq_n_u16(ch16, 8), vandq_u16(cl16, vdupq_n_u16(255)));
			uint16x8_t ch8 = vorrq_u16(vandq_u16(ch16, vdupq_n_u16(0xff00)), vshrq_n_u16(cl16, 8));
			if (!init) {
				cl8 = veorq_u16(cl8, vld1q_u16(reinterpret_cast<const uint16_t *>(__builtin_assume_aligned(c, 16))));
				ch8 = veorq_u16(ch8, vld1q_u16(reinterpret_cast<const uint16_t *>(__builtin_assume_aligned(c+8, 16))));
			}
			vst1q_u16(reinterpret_cast<uint16_t *>(__builtin_assume_aligned(c, 16)), cl8);
			vst1q_u16(reinterpret_cast<uint16_t *>(__builtin_assume_aligned(c+8, 16)), ch8);
		}
#else
#ifdef __AVX2__
		__m128i lll16 = _mm_load_si128(reinterpret_cast<const __m128i *>(blll));
		__m128i llh16 = _mm_load_si128(reinterpret_cast<const __m128i *>(bllh));
		__m128i lhl16 = _mm_load_si128(reinterpret_cast<const __m128i *>(blhl));
		__m128i lhh16 = _mm_load_si128(reinterpret_cast<const __m128i *>(blhh));
		__m128i hll16 = _mm_load_si128(reinterpret_cast<const __m128i *>(bhll));
		__m128i hlh16 = _mm_load_si128(reinterpret_cast<const __m128i *>(bhlh));
		__m128i hhl16 = _mm_load_si128(reinterpret_cast<const __m128i *>(bhhl));
		__m128i hhh16 = _mm_load_si128(reinterpret_cast<const __m128i *>(bhhh));
		__m256i lll162 = _mm256_broadcastsi128_si256(lll16);
		__m256i llh162 = _mm256_broadcastsi128_si256(llh16);
		__m256i lhl162 = _mm256_broadcastsi128_si256(lhl16);
		__m256i lhh162 = _mm256_broadcastsi128_si256(lhh16);
		__m256i hll162 = _mm256_broadcastsi128_si256(hll16);
		__m256i hlh162 = _mm256_broadcastsi128_si256(hlh16);
		__m256i hhl162 = _mm256_broadcastsi128_si256(hhl16);
		__m256i hhh162 = _mm256_broadcastsi128_si256(hhh16);
		for (int i = 0; i < size; i += 32, a += 32, c += 32) {
			__m256i al16 = _mm256_load_si256(reinterpret_cast<const __m256i *>(__builtin_assume_aligned(a, 32)));
			__m256i ah16 = _mm256_load_si256(reinterpret_cast<const __m256i *>(__builtin_assume_aligned(a+16, 32)));
			__m256i al32 = _mm256_or_si256(_mm256_slli_epi16(ah16, 8), _mm256_and_si256(al16, _mm256_set1_epi16(255)));
			__m256i ah32 = _mm256_or_si256(_mm256_and_si256(ah16, _mm256_set1_epi16(0xff00)), _mm256_srli_epi16(al16, 8));
			__m256i alln = _mm256_and_si256(al32, _mm256_set1_epi8(15));
			__m256i clll = _mm256_shuffle_epi8(lll162, alln);
			__m256i chll = _mm256_shuffle_epi8(hll162, alln);
			__m256i alhn = _mm256_and_si256(_mm256_srli_epi16(al32, 4), _mm256_set1_epi8(15));
			__m256i cllh = _mm256_shuffle_epi8(llh162, alhn);
			__m256i chlh = _mm256_shuffle_epi8(hlh162, alhn);
			__m256i ahln = _mm256_and_si256(ah32, _mm256_set1_epi8(15));
			__m256i clhl = _mm256_shuffle_epi8(lhl162, ahln);
			__m256i chhl = _mm256_shuffle_epi8(hhl162, ahln);
			__m256i ahhn = _mm256_and_si256(_mm256_srli_epi16(ah32, 4), _mm256_set1_epi8(15));
			__m256i clhh = _mm256_shuffle_epi8(lhh162, ahhn);
			__m256i chhh = _mm256_shuffle_epi8(hhh162, ahhn);
			__m256i cl32 = _mm256_xor_si256(_mm256_xor_si256(clll, cllh), _mm256_xor_si256(clhl, clhh));
			__m256i ch32 = _mm256_xor_si256(_mm256_xor_si256(chll, chlh), _mm256_xor_si256(chhl, chhh));
			__m256i cl16 = _mm256_or_si256(_mm256_slli_epi16(ch32, 8), _mm256_and_si256(cl32, _mm256_set1_epi16(255)));
			__m256i ch16 = _mm256_or_si256(_mm256_and_si256(ch32, _mm256_set1_epi16(0xff00)), _mm256_srli_epi16(cl32, 8));
			if (!init) {
				cl16 = _mm256_xor_si256(cl16, _mm256_load_si256(reinterpret_cast<const __m256i *>(__builtin_assume_aligned(c, 32))));
				ch16 = _mm256_xor_si256(ch16, _mm256_load_si256(reinterpret_cast<const __m256i *>(__builtin_assume_aligned(c+16, 32))));
			}
			_mm256_store_si256(reinterpret_cast<__m256i *>(__builtin_assume_aligned(c, 32)), cl16);
			_mm256_store_si256(reinterpret_cast<__m256i *>(__builtin_assume_aligned(c+16, 32)), ch16);
		}
#else
		__m128i lll16 = _mm_load_si128(reinterpret_cast<const __m128i *>(blll));
		__m128i llh16 = _mm_load_si128(reinterpret_cast<const __m128i *>(bllh));
		__m128i lhl16 = _mm_load_si128(reinterpret_cast<const __m128i *>(blhl));
		__m128i lhh16 = _mm_load_si128(reinterpret_cast<const __m128i *>(blhh));
		__m128i hll16 = _mm_load_si128(reinterpret_cast<const __m128i *>(bhll));
		__m128i hlh16 = _mm_load_si128(reinterpret_cast<const __m128i *>(bhlh));
		__m128i hhl16 = _mm_load_si128(reinterpret_cast<const __m128i *>(bhhl));
		__m128i hhh16 = _mm_load_si128(reinterpret_cast<const __m128i *>(bhhh));
		for (int i = 0; i < size; i += 16, a += 16, c += 16) {
			__m128i al8 = _mm_load_si128(reinterpret_cast<const __m128i *>(__builtin_assume_aligned(a, 16)));
			__m128i ah8 = _mm_load_si128(reinterpret_cast<const __m128i *>(__builtin_assume_aligned(a+8, 16)));
			__m128i al16 = _mm_or_si128(_mm_slli_epi16(ah8, 8), _mm_and_si128(al8, _mm_set1_epi16(255)));
			__m128i ah16 = _mm_or_si128(_mm_and_si128(ah8, _mm_set1_epi16(0xff00)), _mm_srli_epi16(al8, 8));
			__m128i alln = _mm_and_si128(al16, _mm_set1_epi8(15));
			__m128i clll = _mm_shuffle_epi8(lll16, alln);
			__m128i chll = _mm_shuffle_epi8(hll16, alln);
			__m128i alhn = _mm_and_si128(_mm_srli_epi16(al16, 4), _mm_set1_epi8(15));
			__m128i cllh = _mm_shuffle_epi8(llh16, alhn);
			__m128i chlh = _mm_shuffle_epi8(hlh16, alhn);
			__m128i ahln = _mm_and_si128(ah16, _mm_set1_epi8(15));
			__m128i clhl = _mm_shuffle_epi8(lhl16, ahln);
			__m128i chhl = _mm_shuffle_epi8(hhl16, ahln);
			__m128i ahhn = _mm_and_si128(_mm_srli_epi16(ah16, 4), _mm_set1_epi8(15));
			__m128i clhh = _mm_shuffle_epi8(lhh16, ahhn);
			__m128i chhh = _mm_shuffle_epi8(hhh16, ahhn);
			__m128i cl16 = _mm_xor_si128(_mm_xor_si128(clll, cllh), _mm_xor_si128(clhl, clhh));
			__m128i ch16 = _mm_xor_si128(_mm_xor_si128(chll, chlh), _mm_xor_si128(chhl, chhh));
			__m128i cl8 = _mm_or_si128(_mm_slli_epi16(ch16, 8), _mm_and_si128(cl16, _mm_set1_epi16(255)));
			__m128i ch8 = _mm_or_si128(_mm_and_si128(ch16, _mm_set1_epi16(0xff00)), _mm_srli_epi16(cl16, 8));
			if (!init) {
				cl8 = _mm_xor_si128(cl8, _mm_load_si128(reinterpret_cast<const __m128i *>(__builtin_assume_aligned(c, 16))));
				ch8 = _mm_xor_si128(ch8, _mm_load_si128(reinterpret_cast<const __m128i *>(__builtin_assume_aligned(c+8, 16))));
			}
			_mm_store_si128(reinterpret_cast<__m128i *>(__builtin_assume_aligned(c, 16)), cl8);
			_mm_store_si128(reinterpret_cast<__m128i *>(__builtin_assume_aligned(c+8, 16)), ch8);
		}
#endif
#endif
	}
#endif
	__attribute__((flatten))
	static inline void multiply_accumulate(ValueType *c, const ValueType *a, IndexType b, int len, bool init)
	{
#if defined(__ARM_NEON) || defined(__AVX2__) || defined(__SSE4_1__)
#ifdef __AVX2__
		int SIMD = 32;
#else
		int SIMD = 16;
#endif
		if (((GF::M == 8 && sizeof(value_type) == 1) || (GF::M == 16 && sizeof(value_type) == 2)) && reinterpret_cast<uintptr_t>(c) % SIMD == 0 && reinterpret_cast<uintptr_t>(a) % SIMD == 0 && len % SIMD == 0) {
			mac_simd(reinterpret_cast<value_type *>(c), reinterpret_cast<const value_type *>(a), b, len, init);
		} else
#endif
		if (init) {
			for (int i = 0; i < len; i++)
				c[i] = b * a[i];
		} else {
			for (int i = 0; i < len; i++)
				c[i] = fma(b, a[i], c[i]);
		}
	}
	void encode(const ValueType *data, ValueType *block, int block_id, int block_len, int block_cnt)
	{
		assert(block_id >= block_cnt && block_id <= ValueType::N);
		for (int k = 0; k < block_cnt; k++) {
			IndexType a_ik = cauchy_matrix(block_id, k);
			multiply_accumulate(block, data + block_len * k, a_ik, block_len, !k);
		}
	}
	void decode(ValueType *data, const ValueType *blocks, const ValueType *block_ids, int block_idx, int block_len, int block_cnt)
	{
		for (int k = 0; k < block_cnt; k++) {
			IndexType b_ik = inverse_cauchy_matrix(block_ids, block_idx, k, block_cnt);
			multiply_accumulate(data, blocks + block_len * k, b_ik, block_len, !k);
		}
	}
	void encode(const value_type *data, value_type *block, int block_id, int block_len, int block_cnt)
	{
		encode(reinterpret_cast<const ValueType *>(data), reinterpret_cast<ValueType *>(block), block_id, block_len, block_cnt);
	}
	void decode(value_type *data, const value_type *blocks, const value_type *block_ids, int block_idx, int block_len, int block_cnt)
	{
		decode(reinterpret_cast<ValueType *>(data), reinterpret_cast<const ValueType *>(blocks), reinterpret_cast<const ValueType *>(block_ids), block_idx, block_len, block_cnt);
	}
	void encode(const void *data, void *block, int block_identifier, int block_bytes, int block_count)
	{
		assert(block_bytes % sizeof(value_type) == 0);
		encode(reinterpret_cast<const value_type *>(data), reinterpret_cast<value_type *>(block), block_identifier, block_bytes / sizeof(value_type), block_count);
	}
	void decode(void *data, const void *blocks, const value_type *block_identifiers, int block_index, int block_bytes, int block_count)
	{
		assert(block_bytes % sizeof(value_type) == 0);
		decode(reinterpret_cast<value_type *>(data), reinterpret_cast<const value_type *>(blocks), block_identifiers, block_index, block_bytes / sizeof(value_type), block_count);
	}
};

}

