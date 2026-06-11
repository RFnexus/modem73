/*
List decoding of Polarization-Adjusted Convolutional codes

Copyright 2025 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include "sort.hh"
#include "polar_helper.hh"

namespace CODE {

template <typename TYPE>
struct PACListLeaf
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static bool conv(int *state, bool input)
	{
		// 1011011
		bool b1 = (*state >> 1) & 1;
		bool b3 = (*state >> 3) & 1;
		bool b4 = (*state >> 4) & 1;
		bool b6 = (*state >> 6) & 1;
		bool output = input ^ b1 ^ b3 ^ b4 ^ b6;
		*state = ((*state & 126) << 1) | (input ? 2 : 0) | (output ? 1 : 0);
		return output;
	}
	static MAP rate0(PATH *metric, int *state, TYPE *hard, TYPE *soft)
	{
		TYPE sft = soft[1];
		for (int k = 0; k < TYPE::SIZE; ++k)
			if (conv(state+k, 0) != (sft.v[k] < 0))
				metric[k] += std::abs(sft.v[k]);
		TYPE hrd;
		for (int k = 0; k < TYPE::SIZE; ++k)
			hrd.v[k] = 1 - 2 * (state[k] & 1);
		*hard = hrd;
		MAP map;
		for (int k = 0; k < TYPE::SIZE; ++k)
			map.v[k] = k;
		return map;
	}
	static MAP rate1(PATH *metric, TYPE *message, MAP *maps, int *count, int *state, TYPE *hard, TYPE *soft)
	{
		TYPE sft = soft[1];
		PATH mfork[2*TYPE::SIZE];
		for (int k = 0; k < TYPE::SIZE; ++k)
			mfork[2*k] = mfork[2*k+1] = metric[k];
		int sfork[2*TYPE::SIZE];
		for (int k = 0; k < TYPE::SIZE; ++k)
			sfork[2*k] = sfork[2*k+1] = state[k];
		for (int k = 0; k < 2*TYPE::SIZE; ++k)
			if (conv(sfork+k, k&1) != (sft.v[k>>1] < 0))
				mfork[k] += std::abs(sft.v[k>>1]);
		int perm[2*TYPE::SIZE];
		CODE::insertion_sort(perm, mfork, 2*TYPE::SIZE);
		for (int k = 0; k < TYPE::SIZE; ++k)
			metric[k] = mfork[k];
		MAP map;
		for (int k = 0; k < TYPE::SIZE; ++k)
			map.v[k] = perm[k] >> 1;
		TYPE msg;
		for (int k = 0; k < TYPE::SIZE; ++k)
			msg.v[k] = 1 - 2 * (perm[k] & 1);
		for (int k = 0; k < TYPE::SIZE; ++k)
			state[k] = sfork[perm[k]];
		TYPE hrd;
		for (int k = 0; k < TYPE::SIZE; ++k)
			hrd.v[k] = 1 - 2 * (state[k] & 1);
		message[*count] = msg;
		maps[*count] = map;
		++*count;
		*hard = hrd;
		return map;
	}
};

template <typename TYPE, int M>
struct PACListTree
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static const int N = 1 << M;
	static MAP decode(PATH *metric, TYPE *message, MAP *maps, int *count, int *state, TYPE *hard, TYPE *soft, const uint32_t *frozen)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		MAP lmap = PACListTree<TYPE, M-1>::decode(metric, message, maps, count, state, hard, soft, frozen);
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], vshuf(soft[i+N], lmap), vshuf(soft[i+N/2+N], lmap));
		MAP rmap = PACListTree<TYPE, M-1>::decode(metric, message, maps, count, state, hard+N/2, soft, frozen+N/2/32);
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(vshuf(hard[i], rmap), hard[i+N/2]);
		return vshuf(lmap, rmap);
	}
};

template <typename TYPE>
struct PACListTree<TYPE, 6>
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static const int M = 6;
	static const int N = 1 << M;
	static MAP decode(PATH *metric, TYPE *message, MAP *maps, int *count, int *state, TYPE *hard, TYPE *soft, const uint32_t *frozen)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		MAP lmap = PACListTree<TYPE, M-1>::decode(metric, message, maps, count, state, hard, soft, frozen[0]);
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], vshuf(soft[i+N], lmap), vshuf(soft[i+N/2+N], lmap));
		MAP rmap = PACListTree<TYPE, M-1>::decode(metric, message, maps, count, state, hard+N/2, soft, frozen[1]);
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(vshuf(hard[i], rmap), hard[i+N/2]);
		return vshuf(lmap, rmap);
	}
};

template <typename TYPE>
struct PACListTree<TYPE, 5>
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static const int M = 5;
	static const int N = 1 << M;
	static MAP decode(PATH *metric, TYPE *message, MAP *maps, int *count, int *state, TYPE *hard, TYPE *soft, uint32_t frozen)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		MAP lmap = PACListTree<TYPE, M-1>::decode(metric, message, maps, count, state, hard, soft, frozen & ((1<<(1<<(M-1)))-1));
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], vshuf(soft[i+N], lmap), vshuf(soft[i+N/2+N], lmap));
		MAP rmap = PACListTree<TYPE, M-1>::decode(metric, message, maps, count, state, hard+N/2, soft, frozen >> (N/2));
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(vshuf(hard[i], rmap), hard[i+N/2]);
		return vshuf(lmap, rmap);
	}
};

template <typename TYPE>
struct PACListTree<TYPE, 4>
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static const int M = 4;
	static const int N = 1 << M;
	static MAP decode(PATH *metric, TYPE *message, MAP *maps, int *count, int *state, TYPE *hard, TYPE *soft, uint32_t frozen)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		MAP lmap = PACListTree<TYPE, M-1>::decode(metric, message, maps, count, state, hard, soft, frozen & ((1<<(1<<(M-1)))-1));
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], vshuf(soft[i+N], lmap), vshuf(soft[i+N/2+N], lmap));
		MAP rmap = PACListTree<TYPE, M-1>::decode(metric, message, maps, count, state, hard+N/2, soft, frozen >> (N/2));
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(vshuf(hard[i], rmap), hard[i+N/2]);
		return vshuf(lmap, rmap);
	}
};

template <typename TYPE>
struct PACListTree<TYPE, 3>
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static const int M = 3;
	static const int N = 1 << M;
	static MAP decode(PATH *metric, TYPE *message, MAP *maps, int *count, int *state, TYPE *hard, TYPE *soft, uint32_t frozen)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		MAP lmap = PACListTree<TYPE, M-1>::decode(metric, message, maps, count, state, hard, soft, frozen & ((1<<(1<<(M-1)))-1));
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], vshuf(soft[i+N], lmap), vshuf(soft[i+N/2+N], lmap));
		MAP rmap = PACListTree<TYPE, M-1>::decode(metric, message, maps, count, state, hard+N/2, soft, frozen >> (N/2));
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(vshuf(hard[i], rmap), hard[i+N/2]);
		return vshuf(lmap, rmap);
	}
};

template <typename TYPE>
struct PACListTree<TYPE, 2>
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static const int M = 2;
	static const int N = 1 << M;
	static MAP decode(PATH *metric, TYPE *message, MAP *maps, int *count, int *state, TYPE *hard, TYPE *soft, uint32_t frozen)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		MAP lmap = PACListTree<TYPE, M-1>::decode(metric, message, maps, count, state, hard, soft, frozen & ((1<<(1<<(M-1)))-1));
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], vshuf(soft[i+N], lmap), vshuf(soft[i+N/2+N], lmap));
		MAP rmap = PACListTree<TYPE, M-1>::decode(metric, message, maps, count, state, hard+N/2, soft, frozen >> (N/2));
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(vshuf(hard[i], rmap), hard[i+N/2]);
		return vshuf(lmap, rmap);
	}
};

template <typename TYPE>
struct PACListTree<TYPE, 1>
{
	typedef PolarHelper<TYPE> PH;
	typedef typename PH::PATH PATH;
	typedef typename PH::MAP MAP;
	static MAP decode(PATH *metric, TYPE *message, MAP *maps, int *count, int *state, TYPE *hard, TYPE *soft, uint32_t frozen)
	{
		soft[1] = PH::prod(soft[2], soft[3]);
		MAP lmap, rmap;
		if (frozen & 1)
			lmap = PACListLeaf<TYPE>::rate0(metric, state, hard, soft);
		else
			lmap = PACListLeaf<TYPE>::rate1(metric, message, maps, count, state, hard, soft);
		soft[1] = PH::madd(hard[0], vshuf(soft[2], lmap), vshuf(soft[3], lmap));
		if (frozen >> 1)
			rmap = PACListLeaf<TYPE>::rate0(metric, state, hard+1, soft);
		else
			rmap = PACListLeaf<TYPE>::rate1(metric, message, maps, count, state, hard+1, soft);
		hard[0] = PH::qmul(vshuf(hard[0], rmap), hard[1]);
		return vshuf(lmap, rmap);
	}
};

template <typename TYPE, int MAX_M>
class PACListDecoder
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
	void operator()(int *rank, TYPE *message, const VALUE *codeword, const uint32_t *frozen, int level)
	{
		assert(level <= MAX_M);
		PATH metric[TYPE::SIZE];
		int count = 0;
		metric[0] = 0;
		for (int k = 1; k < TYPE::SIZE; ++k)
			metric[k] = 1000000;
		int length = 1 << level;
		for (int i = 0; i < length; ++i)
			soft[length+i] = vdup<TYPE>(codeword[i]);
		int state[TYPE::SIZE];
		for (int i = 0; i < TYPE::SIZE; ++i)
			state[i] = 0;

		switch (level) {
		case 5: PACListTree<TYPE, 5>::decode(metric, message, maps, &count, state, hard, soft, *frozen); break;
		case 6: PACListTree<TYPE, 6>::decode(metric, message, maps, &count, state, hard, soft, frozen); break;
		case 7: PACListTree<TYPE, 7>::decode(metric, message, maps, &count, state, hard, soft, frozen); break;
		case 8: PACListTree<TYPE, 8>::decode(metric, message, maps, &count, state, hard, soft, frozen); break;
		case 9: PACListTree<TYPE, 9>::decode(metric, message, maps, &count, state, hard, soft, frozen); break;
		case 10: PACListTree<TYPE, 10>::decode(metric, message, maps, &count, state, hard, soft, frozen); break;
		case 11: PACListTree<TYPE, 11>::decode(metric, message, maps, &count, state, hard, soft, frozen); break;
		case 12: PACListTree<TYPE, 12>::decode(metric, message, maps, &count, state, hard, soft, frozen); break;
		case 13: PACListTree<TYPE, 13>::decode(metric, message, maps, &count, state, hard, soft, frozen); break;
		case 14: PACListTree<TYPE, 14>::decode(metric, message, maps, &count, state, hard, soft, frozen); break;
		case 15: PACListTree<TYPE, 15>::decode(metric, message, maps, &count, state, hard, soft, frozen); break;
		case 16: PACListTree<TYPE, 16>::decode(metric, message, maps, &count, state, hard, soft, frozen); break;
		default: assert(false);
		}

		for (int i = 0, r = 0; rank != nullptr && i < TYPE::SIZE; ++i) {
			if (i > 0 && metric[i-1] != metric[i])
				++r;
			rank[i] = r;
		}
		MAP acc = maps[count-1];
		for (int i = count-2; i >= 0; --i) {
			message[i] = vshuf(message[i], acc);
			acc = vshuf(maps[i], acc);
		}
	}
};

}

