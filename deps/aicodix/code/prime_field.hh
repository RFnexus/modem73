/*
Prime field arithmetic

Copyright 2024 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace CODE {

template <typename TYPE, TYPE PRIME>
struct PrimeField
{
	static constexpr TYPE P = PRIME;
	TYPE v;
	PrimeField() = default;
	explicit PrimeField(TYPE v) : v(v)
	{
	}
	PrimeField<TYPE, PRIME> operator *= (PrimeField<TYPE, PRIME> a)
	{
		return *this = *this * a;
	}
	PrimeField<TYPE, PRIME> operator /= (PrimeField<TYPE, PRIME> a)
	{
		return *this = *this / a;
	}
	PrimeField<TYPE, PRIME> operator += (PrimeField<TYPE, PRIME> a)
	{
		return *this = *this + a;
	}
	PrimeField<TYPE, PRIME> operator -= (PrimeField<TYPE, PRIME> a)
	{
		return *this = *this - a;
	}
	TYPE operator () ()
	{
		return v;
	}
};

template <typename TYPE, TYPE PRIME>
PrimeField<TYPE, PRIME> reduce(PrimeField<TYPE, PRIME> a)
{
	return PrimeField<TYPE, PRIME>(a.v % a.P);
}

template <typename TYPE, TYPE PRIME>
bool operator == (PrimeField<TYPE, PRIME> a, PrimeField<TYPE, PRIME> b)
{
	return a.v == b.v;
}

template <typename TYPE, TYPE PRIME>
bool operator != (PrimeField<TYPE, PRIME> a, PrimeField<TYPE, PRIME> b)
{
	return a.v != b.v;
}

template <typename TYPE, TYPE PRIME>
PrimeField<TYPE, PRIME> add(PrimeField<TYPE, PRIME> a, PrimeField<TYPE, PRIME> b)
{
	return PrimeField<TYPE, PRIME>(a.v + b.v);
}

template <typename TYPE, TYPE PRIME>
PrimeField<TYPE, PRIME> operator + (PrimeField<TYPE, PRIME> a, PrimeField<TYPE, PRIME> b)
{
	return reduce(add(a, b));
}

template <typename TYPE, TYPE PRIME>
PrimeField<TYPE, PRIME> sub(PrimeField<TYPE, PRIME> a, PrimeField<TYPE, PRIME> b)
{
	return PrimeField<TYPE, PRIME>(a.v - b.v + (a.v < b.v ? a.P : 0));
}

template <typename TYPE, TYPE PRIME>
PrimeField<TYPE, PRIME> operator - (PrimeField<TYPE, PRIME> a, PrimeField<TYPE, PRIME> b)
{
	return reduce(sub(a, b));
}

template <typename TYPE, TYPE PRIME>
PrimeField<TYPE, PRIME> neg(PrimeField<TYPE, PRIME> a)
{
	return PrimeField<TYPE, PRIME>(a.P - a.v);
}

template <typename TYPE, TYPE PRIME>
PrimeField<TYPE, PRIME> operator - (PrimeField<TYPE, PRIME> a)
{
	return reduce(neg(a));
}

template <typename TYPE, TYPE PRIME>
PrimeField<TYPE, PRIME> mul(PrimeField<TYPE, PRIME> a, PrimeField<TYPE, PRIME> b)
{
	return PrimeField<TYPE, PRIME>(a.v * b.v);
}

template <typename TYPE, TYPE PRIME>
PrimeField<TYPE, PRIME> operator * (PrimeField<TYPE, PRIME> a, PrimeField<TYPE, PRIME> b)
{
	return reduce(mul(a, b));
}

template <typename TYPE, TYPE PRIME>
PrimeField<TYPE, PRIME> pow(PrimeField<TYPE, PRIME> a, TYPE m)
{
	PrimeField<TYPE, PRIME> t(1);
	for (;m; m >>= 1, a *= a)
		if (m & 1)
			t *= a;
	return t;
}

template <typename TYPE, TYPE PRIME>
PrimeField<TYPE, PRIME> rcp(PrimeField<TYPE, PRIME> a)
{
	return pow(a, a.P - 2);
}

template <typename TYPE, TYPE PRIME>
PrimeField<TYPE, PRIME> div(PrimeField<TYPE, PRIME> a, PrimeField<TYPE, PRIME> b)
{
	return mul(a, rcp(b));
}

template <typename TYPE, TYPE PRIME>
PrimeField<TYPE, PRIME> operator / (PrimeField<TYPE, PRIME> a, PrimeField<TYPE, PRIME> b)
{
	return reduce(div(a, b));
}

}
