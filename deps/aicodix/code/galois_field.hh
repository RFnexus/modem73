/*
Galois field arithmetic

Copyright 2018 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include <cassert>

namespace CODE {
namespace GF {

template <int M, int POLY, typename TYPE>
struct Index;

template <int M, int POLY, typename TYPE>
struct Value
{
	static const int Q = 1 << M, N = Q - 1;
	static_assert(M <= 8 * sizeof(TYPE), "TYPE not wide enough");
	static_assert(Q == (POLY & ~N), "POLY not of degree Q");
	TYPE v;
	Value() {}
	explicit Value(TYPE v) : v(v)
	{
		assert(v <= N);
	}
	explicit operator bool () const { return v; }
	explicit operator int () const { return v; }
	Value<M, POLY, TYPE> operator *= (Index<M, POLY, TYPE> a)
	{
		assert(a.i < a.modulus());
		return *this = *this * a;
	}
	Value<M, POLY, TYPE> operator *= (Value<M, POLY, TYPE> a)
	{
		assert(a.v <= a.N);
		return *this = *this * a;
	}
	Value<M, POLY, TYPE> operator += (Value<M, POLY, TYPE> a)
	{
		assert(a.v <= a.N);
		return *this = *this + a;
	}
	static const Value<M, POLY, TYPE> zero()
	{
		return Value<M, POLY, TYPE>(0);
	}
};

template <int M, int POLY, typename TYPE>
struct Index
{
	static const int Q = 1 << M, N = Q - 1;
	static_assert(M <= 8 * sizeof(TYPE), "TYPE not wide enough");
	static_assert(Q == (POLY & ~N), "POLY not of degree Q");
	TYPE i;
	Index() {}
	explicit Index(TYPE i) : i(i)
	{
		assert(i < modulus());
	}
	explicit operator int () const { return i; }
	Index<M, POLY, TYPE> operator *= (Index<M, POLY, TYPE> a)
	{
		assert(a.i < a.modulus());
		assert(i < modulus());
		return *this = *this * a;
	}
	Index<M, POLY, TYPE> operator /= (Index<M, POLY, TYPE> a)
	{
		assert(a.i < a.modulus());
		assert(i < modulus());
		return *this = *this / a;
	}
	static const TYPE modulus()
	{
		return N;
	}
};

template <int M, int POLY, typename TYPE>
struct Tables
{
	static const int Q = 1 << M, N = Q - 1;
	static_assert(M <= 8 * sizeof(TYPE), "TYPE not wide enough");
	static_assert(Q == (POLY & ~N), "POLY not of degree Q");
	static TYPE *LOG, *EXP;
	TYPE log_[Q], exp_[Q];
	static TYPE next(TYPE a)
	{
		return a & (TYPE)(Q >> 1) ? (a << 1) ^ (TYPE)POLY : a << 1;
	}
	static TYPE log(TYPE a)
	{
		assert(LOG != nullptr);
		assert(a <= N);
		return LOG[a];
	}
	static TYPE exp(TYPE a)
	{
		assert(EXP != nullptr);
		assert(a <= N);
		return EXP[a];
	}
	Tables()
	{
		assert(LOG == nullptr);
		LOG = log_;
		assert(EXP == nullptr);
		EXP = exp_;
		log_[exp_[N] = 0] = N;
		TYPE a = 1;
		for (int i = 0; i < N; ++i, a = next(a)) {
			log_[exp_[i] = a] = i;
			assert(!i || a != 1);
		}
		assert(1 == a);
	}
	~Tables()
	{
		assert(LOG != nullptr);
		LOG = nullptr;
		assert(EXP != nullptr);
		EXP = nullptr;
	}
};

template <int M, int POLY, typename TYPE>
TYPE *Tables<M, POLY, TYPE>::LOG;

template <int M, int POLY, typename TYPE>
TYPE *Tables<M, POLY, TYPE>::EXP;

template <int M, int POLY, typename TYPE>
Index<M, POLY, TYPE> index(Value<M, POLY, TYPE> a)
{
	assert(a.v <= a.N);
	assert(a.v);
	return Index<M, POLY, TYPE>(Tables<M, POLY, TYPE>::log(a.v));
}

template <int M, int POLY, typename TYPE>
Value<M, POLY, TYPE> value(Index<M, POLY, TYPE> a) {
	assert(a.i < a.modulus());
	return Value<M, POLY, TYPE>(Tables<M, POLY, TYPE>::exp(a.i));
}

template <int M, int POLY, typename TYPE>
bool operator == (Value<M, POLY, TYPE> a, Value<M, POLY, TYPE> b)
{
	assert(a.v <= a.N);
	assert(b.v <= b.N);
	return a.v == b.v;
}

template <int M, int POLY, typename TYPE>
bool operator != (Value<M, POLY, TYPE> a, Value<M, POLY, TYPE> b)
{
	assert(a.v <= a.N);
	assert(b.v <= b.N);
	return a.v != b.v;
}

template <int M, int POLY, typename TYPE>
Value<M, POLY, TYPE> operator + (Value<M, POLY, TYPE> a, Value<M, POLY, TYPE> b)
{
	assert(a.v <= a.N);
	assert(b.v <= b.N);
	return Value<M, POLY, TYPE>(a.v ^ b.v);
}

template <int M, int POLY, typename TYPE>
Value<M, POLY, TYPE> operator - (Value<M, POLY, TYPE> a)
{
	return a;
}

template <int M, int POLY, typename TYPE>
Value<M, POLY, TYPE> operator - (Value<M, POLY, TYPE> a, Value<M, POLY, TYPE> b)
{
	return a + b;
}

template <int M, int POLY, typename TYPE>
Index<M, POLY, TYPE> operator * (Index<M, POLY, TYPE> a, Index<M, POLY, TYPE> b)
{
	assert(a.i < a.modulus());
	assert(b.i < b.modulus());
	TYPE tmp = a.i + b.i;
	return Index<M, POLY, TYPE>(a.modulus() - a.i <= b.i ? tmp - a.modulus() : tmp);
}

template <int M, int POLY, typename TYPE>
Value<M, POLY, TYPE> operator * (Value<M, POLY, TYPE> a, Value<M, POLY, TYPE> b)
{
	assert(a.v <= a.N);
	assert(b.v <= b.N);
	return (!a.v || !b.v) ? a.zero() : value(index(a) * index(b));
}

template <int M, int POLY, typename TYPE>
Index<M, POLY, TYPE> rcp(Index<M, POLY, TYPE> a)
{
	assert(a.i < a.modulus());
	return Index<M, POLY, TYPE>(!a.i ? 0 : a.modulus() - a.i);
}

template <int M, int POLY, typename TYPE>
Value<M, POLY, TYPE> rcp(Value<M, POLY, TYPE> a)
{
	assert(a.v <= a.N);
	assert(a.v);
	return value(rcp(index(a)));
}

template <int M, int POLY, typename TYPE>
Index<M, POLY, TYPE> operator / (Index<M, POLY, TYPE> a, Index<M, POLY, TYPE> b)
{
	assert(a.i < a.modulus());
	assert(b.i < b.modulus());
	TYPE tmp = a.i - b.i;
	return Index<M, POLY, TYPE>(a.i < b.i ? tmp + a.modulus() : tmp);
}

template <int M, int POLY, typename TYPE>
Value<M, POLY, TYPE> operator / (Value<M, POLY, TYPE> a, Value<M, POLY, TYPE> b)
{
	assert(a.v <= a.N);
	assert(b.v <= b.N);
	assert(b.v);
	return !a.v ? a.zero() : value(index(a) / index(b));
}

template <int M, int POLY, typename TYPE>
Value<M, POLY, TYPE> operator / (Index<M, POLY, TYPE> a, Value<M, POLY, TYPE> b)
{
	assert(a.i < a.modulus());
	assert(b.v <= b.N);
	assert(b.v);
	return value(a / index(b));
}

template <int M, int POLY, typename TYPE>
Value<M, POLY, TYPE> operator / (Value<M, POLY, TYPE> a, Index<M, POLY, TYPE> b)
{
	assert(a.v <= a.N);
	assert(b.i < b.modulus());
	return !a.v ? a.zero() : value(index(a) / b);
}

template <int M, int POLY, typename TYPE>
Value<M, POLY, TYPE> operator * (Index<M, POLY, TYPE> a, Value<M, POLY, TYPE> b)
{
	assert(a.i < a.modulus());
	assert(b.v <= b.N);
	return !b.v ? b.zero() : value(a * index(b));
}

template <int M, int POLY, typename TYPE>
Value<M, POLY, TYPE> operator * (Value<M, POLY, TYPE> a, Index<M, POLY, TYPE> b)
{
	assert(a.v <= a.N);
	assert(b.i < b.modulus());
	return !a.v ? a.zero() : value(index(a) * b);
}

template <int M, int POLY, typename TYPE>
Value<M, POLY, TYPE> fma(Index<M, POLY, TYPE> a, Index<M, POLY, TYPE> b, Value<M, POLY, TYPE> c)
{
	assert(a.i < a.modulus());
	assert(b.i < b.modulus());
	assert(c.v <= c.N);
	return value(a * b) + c;
}

template <int M, int POLY, typename TYPE>
Value<M, POLY, TYPE> fma(Index<M, POLY, TYPE> a, Value<M, POLY, TYPE> b, Value<M, POLY, TYPE> c)
{
	assert(a.i < a.modulus());
	assert(b.v <= b.N);
	assert(c.v <= c.N);
	return !b.v ? c : (value(a * index(b)) + c);
}

template <int M, int POLY, typename TYPE>
Value<M, POLY, TYPE> fma(Value<M, POLY, TYPE> a, Index<M, POLY, TYPE> b, Value<M, POLY, TYPE> c)
{
	assert(a.v <= a.N);
	assert(b.i < b.modulus());
	assert(c.v <= c.N);
	return !a.v ? c : (value(index(a) * b) + c);
}

template <int M, int POLY, typename TYPE>
Value<M, POLY, TYPE> fma(Value<M, POLY, TYPE> a, Value<M, POLY, TYPE> b, Value<M, POLY, TYPE> c)
{
	assert(a.v <= a.N);
	assert(b.v <= b.N);
	assert(c.v <= c.N);
	return (!a.v || !b.v) ? c : (value(index(a) * index(b)) + c);
}

}

template <int WIDTH, int POLY, typename TYPE>
class GaloisField
{
public:
	static const int M = WIDTH, Q = 1 << M, N = Q - 1;
	static_assert(M <= 8 * sizeof(TYPE), "TYPE not wide enough");
	static_assert(Q == (POLY & ~N), "POLY not of degree Q");
	typedef TYPE value_type;
	typedef GF::Value<M, POLY, TYPE> ValueType;
	typedef GF::Index<M, POLY, TYPE> IndexType;
private:
	GF::Tables<M, POLY, TYPE> Tables;
};

template <int M, int64_t POLY, typename TYPE>
struct GaloisFieldReference
{
	static const int64_t Q = int64_t(1) << M;
	static const TYPE N = Q - 1;
	static_assert(M <= 8 * sizeof(TYPE), "TYPE not wide enough");
	static_assert(Q == (POLY & ~(Q - 1)), "POLY not of degree Q");
	static const TYPE P = TYPE(POLY);
	TYPE v;
	GaloisFieldReference() = default;
	explicit GaloisFieldReference(TYPE v) : v(v)
	{
		assert(v <= N);
	}
	GaloisFieldReference<M, POLY, TYPE> operator *= (GaloisFieldReference<M, POLY, TYPE> a)
	{
		return *this = *this * a;
	}
	GaloisFieldReference<M, POLY, TYPE> operator += (GaloisFieldReference<M, POLY, TYPE> a)
	{
		return *this = *this + a;
	}
};

template <int M, int64_t POLY, typename TYPE>
bool operator == (GaloisFieldReference<M, POLY, TYPE> a, GaloisFieldReference<M, POLY, TYPE> b)
{
	return a.v == b.v;
}

template <int M, int64_t POLY, typename TYPE>
bool operator != (GaloisFieldReference<M, POLY, TYPE> a, GaloisFieldReference<M, POLY, TYPE> b)
{
	return a.v != b.v;
}

template <int M, int64_t POLY, typename TYPE>
GaloisFieldReference<M, POLY, TYPE> operator + (GaloisFieldReference<M, POLY, TYPE> a, GaloisFieldReference<M, POLY, TYPE> b)
{
	return GaloisFieldReference<M, POLY, TYPE>(a.v ^ b.v);
}

template <int M, int64_t POLY, typename TYPE>
GaloisFieldReference<M, POLY, TYPE> operator - (GaloisFieldReference<M, POLY, TYPE> a, GaloisFieldReference<M, POLY, TYPE> b)
{
	return a + b;
}

template <int M, int64_t POLY, typename TYPE>
GaloisFieldReference<M, POLY, TYPE> operator - (GaloisFieldReference<M, POLY, TYPE> a)
{
	return a;
}

template <int M, int64_t POLY, typename TYPE>
GaloisFieldReference<M, POLY, TYPE> operator * (GaloisFieldReference<M, POLY, TYPE> a, GaloisFieldReference<M, POLY, TYPE> b)
{
	GaloisFieldReference<M, POLY, TYPE> p(0);
#if 0
	if (a.v < b.v)
		std::swap(a, b);
	while (a.v && b.v) {
#else
	for (int i = 0; i < M; ++i) {
#endif
		if (b.v & 1)
			p.v ^= a.v;
		if (a.v & (TYPE(1) << (M - 1)))
			a.v = (a.v << 1) ^ a.P;
		else
			a.v <<= 1;
		b.v >>= 1;
	}
	return p;
}

template <int M, int64_t POLY, typename TYPE>
GaloisFieldReference<M, POLY, TYPE> rcp(GaloisFieldReference<M, POLY, TYPE> a)
{
	assert(a.v);
#if 1
	if (a.v == 1)
		return a;
	TYPE newr = a.P, r = a.v;
	TYPE newt = 0, t = 1;
	auto degree = [](TYPE a) {
#if 1
		return 31 - __builtin_clz(a);
#else
		int d = 0;
		while (a >>= 1)
			++d;
		return d;
#endif
	};
	int k = degree(r);
	int j = M - k;
	newr ^= r << j;
	newt ^= t << j;
	while (newr != 1) {
		int l = degree(newr);
		int j = l - k;
		if (j < 0) {
			j = -j;
			k = l;
			std::swap(newr, r);
			std::swap(newt, t);
		}
		newr ^= r << j;
		newt ^= t << j;
	}
	return GaloisFieldReference<M, POLY, TYPE>(newt);
#else
	GaloisFieldReference<M, POLY, TYPE> t(a *= a);
	for (int i = 0; i < M - 2; ++i)
		t *= a *= a;
	return t;
#endif
}

template <int M, int64_t POLY, typename TYPE>
GaloisFieldReference<M, POLY, TYPE> operator / (GaloisFieldReference<M, POLY, TYPE> a, GaloisFieldReference<M, POLY, TYPE> b)
{
	assert(b.v);
	return a * rcp(b);
}

}
