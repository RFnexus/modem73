/*
Parity aided successive cancellation list decoding of polar codes

Copyright 2023 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include "sort.hh"
#include "polar_helper.hh"

namespace CODE {

template <typename TYPE>
class PolarParityEncoder
{
	typedef PolarHelper<TYPE> PH;
	static bool get(const uint32_t *bits, int idx)
	{
		return (bits[idx/32] >> (idx%32)) & 1;
	}
public:
	void operator()(TYPE *codeword, const TYPE *message, const uint32_t *frozen, int level, int stride, int first)
	{
		int length = 1 << level;
		int count = first;
		TYPE parity = PH::one();
		for (int i = 0; i < length; i += 2) {
			TYPE msg0, msg1;
			if (get(frozen, i)) {
				msg0 = PH::one();
			} else if (count) {
				msg0 = *message++;
				parity = PH::qmul(parity, msg0);
				--count;
			} else {
				msg0 = parity;
				parity = PH::one();
				count = stride;
			}
			if (get(frozen, i + 1)) {
				msg1 = PH::one();
			} else if (count) {
				msg1 = *message++;
				parity = PH::qmul(parity, msg1);
				--count;
			} else {
				msg1 = parity;
				parity = PH::one();
				count = stride;
			}
			codeword[i] = PH::qmul(msg0, msg1);
			codeword[i+1] = msg1;
		}
		for (int h = 2; h < length; h *= 2)
			for (int i = 0; i < length; i += 2 * h)
				for (int j = i; j < i + h; ++j)
					codeword[j] = PH::qmul(codeword[j], codeword[j+h]);
	}
};

template <typename TYPE, int M>
struct PolarParityNode
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static const int N = 1 << M;
	static MAP rate0(PATH *metric, TYPE *hard, TYPE *soft)
	{
		for (int i = 0; i < N; ++i)
			hard[i] = PH::one();
		for (int i = 0; i < N; ++i)
			for (int k = 0; k < TYPE::SIZE; ++k)
				if (soft[i+N].v[k] < 0)
					metric[k] -= soft[i+N].v[k];
		MAP map;
		for (int k = 0; k < TYPE::SIZE; ++k)
			map.v[k] = k;
		return map;
	}
};

template <typename TYPE>
struct PolarParityNode<TYPE, 0>
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static MAP rate0(PATH *metric, TYPE *hard, TYPE *soft)
	{
		*hard = PH::one();
		for (int k = 0; k < TYPE::SIZE; ++k)
			if (soft[1].v[k] < 0)
				metric[k] -= soft[1].v[k];
		MAP map;
		for (int k = 0; k < TYPE::SIZE; ++k)
			map.v[k] = k;
		return map;
	}
	static MAP rate1(PATH *metric, TYPE *message, MAP *maps, int *index, TYPE *hard, TYPE *soft, TYPE *parity, int *count, int stride)
	{
		TYPE sft = soft[1];
		PATH fork[2*TYPE::SIZE];
		for (int k = 0; k < TYPE::SIZE; ++k)
			fork[2*k] = fork[2*k+1] = metric[k];
		for (int k = 0; k < TYPE::SIZE; ++k)
			if (sft.v[k] < 0)
				fork[2*k] -= sft.v[k];
			else
				fork[2*k+1] += sft.v[k];
		if (*count == 0) {
			TYPE chk = *parity;
			*parity = PH::one();
			for (int k = 0; k < TYPE::SIZE; ++k)
				if (chk.v[k] < 0)
					fork[2*k] = 1000000;
				else
					fork[2*k+1] = 1000000;
		}
		int perm[2*TYPE::SIZE];
		CODE::insertion_sort(perm, fork, 2*TYPE::SIZE);
		for (int k = 0; k < TYPE::SIZE; ++k)
			metric[k] = fork[k];
		MAP map;
		for (int k = 0; k < TYPE::SIZE; ++k)
			map.v[k] = perm[k] >> 1;
		TYPE hrd;
		for (int k = 0; k < TYPE::SIZE; ++k)
			hrd.v[k] = 1 - 2 * (perm[k] & 1);
		if (*count) {
			message[*index] = hrd;
			*parity = PH::qmul(vshuf(*parity, map), hrd);
			maps[*index] = map;
			++*index;
			--*count;
		} else {
			message[*index-1] = vshuf(message[*index-1], map);
			maps[*index-1] = vshuf(maps[*index-1], map);
			*count = stride;
		}
		*hard = hrd;
		return map;
	}
};

template <typename TYPE, int M>
struct PolarParityTree
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static const int N = 1 << M;
	static MAP decode(PATH *metric, TYPE *message, MAP *maps, int *index, TYPE *hard, TYPE *soft, const uint32_t *frozen, TYPE *parity, int *count, int stride)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		MAP lmap = PolarParityTree<TYPE, M-1>::decode(metric, message, maps, index, hard, soft, frozen, parity, count, stride);
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], vshuf(soft[i+N], lmap), vshuf(soft[i+N/2+N], lmap));
		MAP rmap = PolarParityTree<TYPE, M-1>::decode(metric, message, maps, index, hard+N/2, soft, frozen+N/2/32, parity, count, stride);
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(vshuf(hard[i], rmap), hard[i+N/2]);
		return vshuf(lmap, rmap);
	}
};

template <typename TYPE>
struct PolarParityTree<TYPE, 6>
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static const int M = 6;
	static const int N = 1 << M;
	static MAP decode(PATH *metric, TYPE *message, MAP *maps, int *index, TYPE *hard, TYPE *soft, const uint32_t *frozen, TYPE *parity, int *count, int stride)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		MAP lmap, rmap;
		if (frozen[0] == 0xffffffff)
			lmap = PolarParityNode<TYPE, M-1>::rate0(metric, hard, soft);
		else
			lmap = PolarParityTree<TYPE, M-1>::decode(metric, message, maps, index, hard, soft, frozen[0], parity, count, stride);
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], vshuf(soft[i+N], lmap), vshuf(soft[i+N/2+N], lmap));
		if (frozen[1] == 0xffffffff)
			rmap = PolarParityNode<TYPE, M-1>::rate0(metric, hard+N/2, soft);
		else
			rmap = PolarParityTree<TYPE, M-1>::decode(metric, message, maps, index, hard+N/2, soft, frozen[1], parity, count, stride);
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(vshuf(hard[i], rmap), hard[i+N/2]);
		return vshuf(lmap, rmap);
	}
};

template <typename TYPE>
struct PolarParityTree<TYPE, 5>
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static const int M = 5;
	static const int N = 1 << M;
	static MAP decode(PATH *metric, TYPE *message, MAP *maps, int *index, TYPE *hard, TYPE *soft, uint32_t frozen, TYPE *parity, int *count, int stride)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		MAP lmap, rmap;
		if ((frozen & ((1<<(1<<(M-1)))-1)) == ((1<<(1<<(M-1)))-1))
			lmap = PolarParityNode<TYPE, M-1>::rate0(metric, hard, soft);
		else
			lmap = PolarParityTree<TYPE, M-1>::decode(metric, message, maps, index, hard, soft, frozen & ((1<<(1<<(M-1)))-1), parity, count, stride);
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], vshuf(soft[i+N], lmap), vshuf(soft[i+N/2+N], lmap));
		if (frozen >> (N/2) == ((1<<(1<<(M-1)))-1))
			rmap = PolarParityNode<TYPE, M-1>::rate0(metric, hard+N/2, soft);
		else
			rmap = PolarParityTree<TYPE, M-1>::decode(metric, message, maps, index, hard+N/2, soft, frozen >> (N/2), parity, count, stride);
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(vshuf(hard[i], rmap), hard[i+N/2]);
		return vshuf(lmap, rmap);
	}
};

template <typename TYPE>
struct PolarParityTree<TYPE, 4>
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static const int M = 4;
	static const int N = 1 << M;
	static MAP decode(PATH *metric, TYPE *message, MAP *maps, int *index, TYPE *hard, TYPE *soft, uint32_t frozen, TYPE *parity, int *count, int stride)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		MAP lmap, rmap;
		if ((frozen & ((1<<(1<<(M-1)))-1)) == ((1<<(1<<(M-1)))-1))
			lmap = PolarParityNode<TYPE, M-1>::rate0(metric, hard, soft);
		else
			lmap = PolarParityTree<TYPE, M-1>::decode(metric, message, maps, index, hard, soft, frozen & ((1<<(1<<(M-1)))-1), parity, count, stride);
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], vshuf(soft[i+N], lmap), vshuf(soft[i+N/2+N], lmap));
		if (frozen >> (N/2) == ((1<<(1<<(M-1)))-1))
			rmap = PolarParityNode<TYPE, M-1>::rate0(metric, hard+N/2, soft);
		else
			rmap = PolarParityTree<TYPE, M-1>::decode(metric, message, maps, index, hard+N/2, soft, frozen >> (N/2), parity, count, stride);
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(vshuf(hard[i], rmap), hard[i+N/2]);
		return vshuf(lmap, rmap);
	}
};

template <typename TYPE>
struct PolarParityTree<TYPE, 3>
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static const int M = 3;
	static const int N = 1 << M;
	static MAP decode(PATH *metric, TYPE *message, MAP *maps, int *index, TYPE *hard, TYPE *soft, uint32_t frozen, TYPE *parity, int *count, int stride)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		MAP lmap, rmap;
		if ((frozen & ((1<<(1<<(M-1)))-1)) == ((1<<(1<<(M-1)))-1))
			lmap = PolarParityNode<TYPE, M-1>::rate0(metric, hard, soft);
		else
			lmap = PolarParityTree<TYPE, M-1>::decode(metric, message, maps, index, hard, soft, frozen & ((1<<(1<<(M-1)))-1), parity, count, stride);
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], vshuf(soft[i+N], lmap), vshuf(soft[i+N/2+N], lmap));
		if (frozen >> (N/2) == ((1<<(1<<(M-1)))-1))
			rmap = PolarParityNode<TYPE, M-1>::rate0(metric, hard+N/2, soft);
		else
			rmap = PolarParityTree<TYPE, M-1>::decode(metric, message, maps, index, hard+N/2, soft, frozen >> (N/2), parity, count, stride);
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(vshuf(hard[i], rmap), hard[i+N/2]);
		return vshuf(lmap, rmap);
	}
};

template <typename TYPE>
struct PolarParityTree<TYPE, 2>
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static const int M = 2;
	static const int N = 1 << M;
	static MAP decode(PATH *metric, TYPE *message, MAP *maps, int *index, TYPE *hard, TYPE *soft, uint32_t frozen, TYPE *parity, int *count, int stride)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		MAP lmap, rmap;
		if ((frozen & ((1<<(1<<(M-1)))-1)) == ((1<<(1<<(M-1)))-1))
			lmap = PolarParityNode<TYPE, M-1>::rate0(metric, hard, soft);
		else
			lmap = PolarParityTree<TYPE, M-1>::decode(metric, message, maps, index, hard, soft, frozen & ((1<<(1<<(M-1)))-1), parity, count, stride);
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], vshuf(soft[i+N], lmap), vshuf(soft[i+N/2+N], lmap));
		if (frozen >> (N/2) == ((1<<(1<<(M-1)))-1))
			rmap = PolarParityNode<TYPE, M-1>::rate0(metric, hard+N/2, soft);
		else
			rmap = PolarParityTree<TYPE, M-1>::decode(metric, message, maps, index, hard+N/2, soft, frozen >> (N/2), parity, count, stride);
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(vshuf(hard[i], rmap), hard[i+N/2]);
		return vshuf(lmap, rmap);
	}
};

template <typename TYPE>
struct PolarParityTree<TYPE, 1>
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static MAP decode(PATH *metric, TYPE *message, MAP *maps, int *index, TYPE *hard, TYPE *soft, uint32_t frozen, TYPE *parity, int *count, int stride)
	{
		soft[1] = PH::prod(soft[2], soft[3]);
		MAP lmap, rmap;
		if (frozen & 1)
			lmap = PolarParityNode<TYPE, 0>::rate0(metric, hard, soft);
		else
			lmap = PolarParityNode<TYPE, 0>::rate1(metric, message, maps, index, hard, soft, parity, count, stride);
		soft[1] = PH::madd(hard[0], vshuf(soft[2], lmap), vshuf(soft[3], lmap));
		if (frozen >> 1)
			rmap = PolarParityNode<TYPE, 0>::rate0(metric, hard+1, soft);
		else
			rmap = PolarParityNode<TYPE, 0>::rate1(metric, message, maps, index, hard+1, soft, parity, count, stride);
		hard[0] = PH::qmul(vshuf(hard[0], rmap), hard[1]);
		return vshuf(lmap, rmap);
	}
};

template <typename TYPE, int MAX_M>
class PolarParityDecoder
{
	static_assert(MAX_M >= 5 && MAX_M <= 16);
	typedef PolarHelper<TYPE> PH;
	typedef typename TYPE::value_type VALUE;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static const int MAX_N = 1 << MAX_M;
	TYPE soft[2*MAX_N];
	TYPE hard[MAX_N];
	MAP maps[MAX_N];
public:
	void operator()(int *rank, TYPE *message, const VALUE *codeword, const uint32_t *frozen, int level, int stride, int first)
	{
		assert(level <= MAX_M);
		int index = 0;
		PATH metric[TYPE::SIZE];
		metric[0] = 0;
		for (int k = 1; k < TYPE::SIZE; ++k)
			metric[k] = 1000000;
		int length = 1 << level;
		for (int i = 0; i < length; ++i)
			soft[length+i] = vdup<TYPE>(codeword[i]);
		TYPE parity = PH::one();
		int count = first;

		switch (level) {
		case 5: PolarParityTree<TYPE, 5>::decode(metric, message, maps, &index, hard, soft, *frozen, &parity, &count, stride); break;
		case 6: PolarParityTree<TYPE, 6>::decode(metric, message, maps, &index, hard, soft, frozen, &parity, &count, stride); break;
		case 7: PolarParityTree<TYPE, 7>::decode(metric, message, maps, &index, hard, soft, frozen, &parity, &count, stride); break;
		case 8: PolarParityTree<TYPE, 8>::decode(metric, message, maps, &index, hard, soft, frozen, &parity, &count, stride); break;
		case 9: PolarParityTree<TYPE, 9>::decode(metric, message, maps, &index, hard, soft, frozen, &parity, &count, stride); break;
		case 10: PolarParityTree<TYPE, 10>::decode(metric, message, maps, &index, hard, soft, frozen, &parity, &count, stride); break;
		case 11: PolarParityTree<TYPE, 11>::decode(metric, message, maps, &index, hard, soft, frozen, &parity, &count, stride); break;
		case 12: PolarParityTree<TYPE, 12>::decode(metric, message, maps, &index, hard, soft, frozen, &parity, &count, stride); break;
		case 13: PolarParityTree<TYPE, 13>::decode(metric, message, maps, &index, hard, soft, frozen, &parity, &count, stride); break;
		case 14: PolarParityTree<TYPE, 14>::decode(metric, message, maps, &index, hard, soft, frozen, &parity, &count, stride); break;
		case 15: PolarParityTree<TYPE, 15>::decode(metric, message, maps, &index, hard, soft, frozen, &parity, &count, stride); break;
		case 16: PolarParityTree<TYPE, 16>::decode(metric, message, maps, &index, hard, soft, frozen, &parity, &count, stride); break;
		default: assert(false);
		}

		for (int i = 0, r = 0; rank != nullptr && i < TYPE::SIZE; ++i) {
			if (i > 0 && metric[i-1] != metric[i])
				++r;
			rank[i] = r;
		}
		MAP acc = maps[index-1];
		for (int i = index-2; i >= 0; --i) {
			message[i] = vshuf(message[i], acc);
			acc = vshuf(maps[i], acc);
		}
	}
};

}

