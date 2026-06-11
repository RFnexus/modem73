/*
Successive cancellation decoding of polar codes

Copyright 2020 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include <algorithm>
#include "polar_helper.hh"

namespace CODE {

template <typename TYPE, int M>
struct PolarNode
{
	typedef PolarHelper<TYPE> PH;
	static const int N = 1 << M;
	static void trans(TYPE *out, const TYPE *inp)
	{
		for (int i = 0; i < N; i += 2) {
			out[i] = PH::qmul(inp[i], inp[i+1]);
			out[i+1] = inp[i+1];
		}
		for (int h = 2; h < N; h *= 2)
			for (int i = 0; i < N; i += 2 * h)
				for (int j = i; j < i + h; ++j)
					out[j] = PH::qmul(out[j], out[j+h]);
	}
	static void rate0(TYPE *hard)
	{
		for (int i = 0; i < N; ++i)
			hard[i] = PH::one();
	}
	static void rate1(TYPE **message, TYPE *hard, TYPE *soft)
	{
		for (int i = 0; i < N; ++i)
			hard[i] = PH::signum(soft[i+N]);
		trans(*message, hard);
		*message += N;
	}
};

template <typename TYPE>
struct PolarNode<TYPE, 0>
{
	typedef PolarHelper<TYPE> PH;
	static void rate0(TYPE *hard)
	{
		*hard = PH::one();
	}
	static void rate1(TYPE **message, TYPE *hard, TYPE *soft)
	{
		*hard = PH::signum(soft[1]);
		*(*message)++ = *hard;
	}
};

template <typename TYPE, int M>
struct PolarTree
{
	typedef PolarHelper<TYPE> PH;
	static const int N = 1 << M;
	static void decode(TYPE **message, TYPE *hard, TYPE *soft, const uint32_t *frozen)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		PolarTree<TYPE, M-1>::decode(message, hard, soft, frozen);
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], soft[i+N], soft[i+N/2+N]);
		PolarTree<TYPE, M-1>::decode(message, hard+N/2, soft, frozen+N/2/32);
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(hard[i], hard[i+N/2]);
	}
};

template <typename TYPE>
struct PolarTree<TYPE, 6>
{
	typedef PolarHelper<TYPE> PH;
	static const int M = 6;
	static const int N = 1 << M;
	static void decode(TYPE **message, TYPE *hard, TYPE *soft, const uint32_t *frozen)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		if (frozen[0] == 0xffffffff)
			PolarNode<TYPE, M-1>::rate0(hard);
		else if (frozen[0] == 0)
			PolarNode<TYPE, M-1>::rate1(message, hard, soft);
		else
			PolarTree<TYPE, M-1>::decode(message, hard, soft, frozen[0]);
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], soft[i+N], soft[i+N/2+N]);
		if (frozen[1] == 0xffffffff)
			PolarNode<TYPE, M-1>::rate0(hard+N/2);
		else if (frozen[1] == 0)
			PolarNode<TYPE, M-1>::rate1(message, hard+N/2, soft);
		else
			PolarTree<TYPE, M-1>::decode(message, hard+N/2, soft, frozen[1]);
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(hard[i], hard[i+N/2]);
	}
};

template <typename TYPE>
struct PolarTree<TYPE, 5>
{
	typedef PolarHelper<TYPE> PH;
	static const int M = 5;
	static const int N = 1 << M;
	static void decode(TYPE **message, TYPE *hard, TYPE *soft, uint32_t frozen)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		if ((frozen & ((1<<(1<<(M-1)))-1)) == ((1<<(1<<(M-1)))-1))
			PolarNode<TYPE, M-1>::rate0(hard);
		else if ((frozen & ((1<<(1<<(M-1)))-1)) == 0)
			PolarNode<TYPE, M-1>::rate1(message, hard, soft);
		else
			PolarTree<TYPE, M-1>::decode(message, hard, soft, frozen & ((1<<(1<<(M-1)))-1));
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], soft[i+N], soft[i+N/2+N]);
		if (frozen >> (N/2) == ((1<<(1<<(M-1)))-1))
			PolarNode<TYPE, M-1>::rate0(hard+N/2);
		else if (frozen >> (N/2) == 0)
			PolarNode<TYPE, M-1>::rate1(message, hard+N/2, soft);
		else
			PolarTree<TYPE, M-1>::decode(message, hard+N/2, soft, frozen >> (N/2));
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(hard[i], hard[i+N/2]);
	}
};

template <typename TYPE>
struct PolarTree<TYPE, 4>
{
	typedef PolarHelper<TYPE> PH;
	static const int M = 4;
	static const int N = 1 << M;
	static void decode(TYPE **message, TYPE *hard, TYPE *soft, uint32_t frozen)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		if ((frozen & ((1<<(1<<(M-1)))-1)) == ((1<<(1<<(M-1)))-1))
			PolarNode<TYPE, M-1>::rate0(hard);
		else if ((frozen & ((1<<(1<<(M-1)))-1)) == 0)
			PolarNode<TYPE, M-1>::rate1(message, hard, soft);
		else
			PolarTree<TYPE, M-1>::decode(message, hard, soft, frozen & ((1<<(1<<(M-1)))-1));
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], soft[i+N], soft[i+N/2+N]);
		if (frozen >> (N/2) == ((1<<(1<<(M-1)))-1))
			PolarNode<TYPE, M-1>::rate0(hard+N/2);
		else if (frozen >> (N/2) == 0)
			PolarNode<TYPE, M-1>::rate1(message, hard+N/2, soft);
		else
			PolarTree<TYPE, M-1>::decode(message, hard+N/2, soft, frozen >> (N/2));
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(hard[i], hard[i+N/2]);
	}
};

template <typename TYPE>
struct PolarTree<TYPE, 3>
{
	typedef PolarHelper<TYPE> PH;
	static const int M = 3;
	static const int N = 1 << M;
	static void decode(TYPE **message, TYPE *hard, TYPE *soft, uint32_t frozen)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		if ((frozen & ((1<<(1<<(M-1)))-1)) == ((1<<(1<<(M-1)))-1))
			PolarNode<TYPE, M-1>::rate0(hard);
		else if ((frozen & ((1<<(1<<(M-1)))-1)) == 0)
			PolarNode<TYPE, M-1>::rate1(message, hard, soft);
		else
			PolarTree<TYPE, M-1>::decode(message, hard, soft, frozen & ((1<<(1<<(M-1)))-1));
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], soft[i+N], soft[i+N/2+N]);
		if (frozen >> (N/2) == ((1<<(1<<(M-1)))-1))
			PolarNode<TYPE, M-1>::rate0(hard+N/2);
		else if (frozen >> (N/2) == 0)
			PolarNode<TYPE, M-1>::rate1(message, hard+N/2, soft);
		else
			PolarTree<TYPE, M-1>::decode(message, hard+N/2, soft, frozen >> (N/2));
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(hard[i], hard[i+N/2]);
	}
};

template <typename TYPE>
struct PolarTree<TYPE, 2>
{
	typedef PolarHelper<TYPE> PH;
	static const int M = 2;
	static const int N = 1 << M;
	static void decode(TYPE **message, TYPE *hard, TYPE *soft, uint32_t frozen)
	{
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::prod(soft[i+N], soft[i+N/2+N]);
		if ((frozen & ((1<<(1<<(M-1)))-1)) == ((1<<(1<<(M-1)))-1))
			PolarNode<TYPE, M-1>::rate0(hard);
		else if ((frozen & ((1<<(1<<(M-1)))-1)) == 0)
			PolarNode<TYPE, M-1>::rate1(message, hard, soft);
		else
			PolarTree<TYPE, M-1>::decode(message, hard, soft, frozen & ((1<<(1<<(M-1)))-1));
		for (int i = 0; i < N/2; ++i)
			soft[i+N/2] = PH::madd(hard[i], soft[i+N], soft[i+N/2+N]);
		if (frozen >> (N/2) == ((1<<(1<<(M-1)))-1))
			PolarNode<TYPE, M-1>::rate0(hard+N/2);
		else if (frozen >> (N/2) == 0)
			PolarNode<TYPE, M-1>::rate1(message, hard+N/2, soft);
		else
			PolarTree<TYPE, M-1>::decode(message, hard+N/2, soft, frozen >> (N/2));
		for (int i = 0; i < N/2; ++i)
			hard[i] = PH::qmul(hard[i], hard[i+N/2]);
	}
};

template <typename TYPE>
struct PolarTree<TYPE, 1>
{
	typedef PolarHelper<TYPE> PH;
	static void decode(TYPE **message, TYPE *hard, TYPE *soft, uint32_t frozen)
	{
		soft[1] = PH::prod(soft[2], soft[3]);
		if (frozen & 1)
			PolarNode<TYPE, 0>::rate0(hard);
		else
			PolarNode<TYPE, 0>::rate1(message, hard, soft);
		soft[1] = PH::madd(hard[0], soft[2], soft[3]);
		if (frozen >> 1)
			PolarNode<TYPE, 0>::rate0(hard+1);
		else
			PolarNode<TYPE, 0>::rate1(message, hard+1, soft);
		hard[0] = PH::qmul(hard[0], hard[1]);
	}
};

template <typename TYPE, int MAX_M>
class PolarDecoder
{
	static_assert(MAX_M >= 5 && MAX_M <= 20);
	typedef PolarHelper<TYPE> PH;
	static const int MAX_N = 1 << MAX_M;
	TYPE soft[2*MAX_N];
	TYPE hard[MAX_N];
public:
	void operator()(TYPE *message, const TYPE *codeword, const uint32_t *frozen, int level)
	{
		assert(level <= MAX_M);
		int length = 1 << level;
		for (int i = 0; i < length; ++i)
			soft[length+i] = codeword[i];

		switch (level) {
		case 5: PolarTree<TYPE, 5>::decode(&message, hard, soft, *frozen); break;
		case 6: PolarTree<TYPE, 6>::decode(&message, hard, soft, frozen); break;
		case 7: PolarTree<TYPE, 7>::decode(&message, hard, soft, frozen); break;
		case 8: PolarTree<TYPE, 8>::decode(&message, hard, soft, frozen); break;
		case 9: PolarTree<TYPE, 9>::decode(&message, hard, soft, frozen); break;
		case 10: PolarTree<TYPE, 10>::decode(&message, hard, soft, frozen); break;
		case 11: PolarTree<TYPE, 11>::decode(&message, hard, soft, frozen); break;
		case 12: PolarTree<TYPE, 12>::decode(&message, hard, soft, frozen); break;
		case 13: PolarTree<TYPE, 13>::decode(&message, hard, soft, frozen); break;
		case 14: PolarTree<TYPE, 14>::decode(&message, hard, soft, frozen); break;
		case 15: PolarTree<TYPE, 15>::decode(&message, hard, soft, frozen); break;
		case 16: PolarTree<TYPE, 16>::decode(&message, hard, soft, frozen); break;
		case 17: PolarTree<TYPE, 17>::decode(&message, hard, soft, frozen); break;
		case 18: PolarTree<TYPE, 18>::decode(&message, hard, soft, frozen); break;
		case 19: PolarTree<TYPE, 19>::decode(&message, hard, soft, frozen); break;
		case 20: PolarTree<TYPE, 20>::decode(&message, hard, soft, frozen); break;
		default: assert(false);
		}
	}
};

}

