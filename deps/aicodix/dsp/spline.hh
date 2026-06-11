/*
Some spline algorithms

Copyright 2018 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace DSP {

template <int KNOTS, typename OTYPE, typename ITYPE>
class UniformNaturalCubicSpline
{
	OTYPE A[KNOTS-1], B[KNOTS-1], C[KNOTS-1], D[KNOTS-1];
	ITYPE x0, dx;
public:
	UniformNaturalCubicSpline() = default;
	UniformNaturalCubicSpline(const OTYPE *Y, ITYPE x0 = 0, ITYPE dx = 1, int STRIDE = 1) : x0(x0), dx(dx)
	{
		ITYPE U[KNOTS-1];
		U[0] = ITYPE(0);
		OTYPE Z[KNOTS-1];
		Z[0] = ITYPE(0);
		for (int i = 1; i < KNOTS - 1; ++i) {
			ITYPE l = ITYPE(4) - U[i-1];
			U[i] = ITYPE(1) / l;
			Z[i] = (ITYPE(3) * (Y[(i+1)*STRIDE] - ITYPE(2) * Y[i*STRIDE] + Y[(i-1)*STRIDE]) - Z[i-1]) / l;
		}
		OTYPE c(ITYPE(0));
		for (int i = KNOTS - 2; i >= 0; --i) {
			A[i] = Y[i * STRIDE];
			C[i] = Z[i] - U[i] * c;
			B[i] = Y[(i+1)*STRIDE] - Y[i*STRIDE] - (c + ITYPE(2) * C[i]) / ITYPE(3);
			D[i] = (c - C[i]) / ITYPE(3);
			c = C[i];
		}
	}
	OTYPE operator () (ITYPE x)
	{
		ITYPE tx = (x - x0) / dx;
		int k = tx;
		ITYPE t = tx - ITYPE(k);
		if (k < 0) {
			t = tx;
			k = 0;
		}
		if (k >= KNOTS - 1) {
			t = tx - ITYPE(KNOTS-2);
			k = KNOTS-2;
		}
		return A[k] + t * (B[k] + t * (C[k] + t * D[k]));
	}
};

template <typename OTYPE, typename ITYPE>
struct CubicHermiteSpline
{
	static constexpr ITYPE h00(ITYPE t)
	{
		return (ITYPE(1) + ITYPE(2) * t) * (ITYPE(1) - t) * (ITYPE(1) - t);
	}
	static constexpr ITYPE h10(ITYPE t)
	{
		return t * (ITYPE(1) - t) * (ITYPE(1) - t);
	}
	static constexpr ITYPE h01(ITYPE t)
	{
		return t * t * (ITYPE(3) - ITYPE(2) * t);
	}
	static constexpr ITYPE h11(ITYPE t)
	{
		return t * t * (t - ITYPE(1));
	}
	static constexpr OTYPE left(const ITYPE *X, const OTYPE *Y)
	{
		return (Y[0] - Y[-1]) / (X[0] - X[-1]);
	}
	static constexpr OTYPE right(const ITYPE *X, const OTYPE *Y)
	{
		return (Y[1] - Y[0]) / (X[1] - X[0]);
	}
	static constexpr OTYPE central(const ITYPE *X, const OTYPE *Y)
	{
		return ITYPE(0.5) * (left(X, Y) + right(X, Y));
	}
	static constexpr OTYPE eval(const ITYPE *X, const OTYPE *Y, ITYPE t, int k, int n)
	{
		return k < 1 ?
			h00(t) * Y[0] + h10(t) * (X[1]-X[0]) * right(X, Y) + h01(t) * Y[1] + h11(t) * (X[1]-X[0]) * central(X+1, Y+1)
		: k < n-2 ?
			h00(t) * Y[k] + h10(t) * (X[k+1]-X[k]) * central(X+k, Y+k) + h01(t) * Y[k+1] + h11(t) * (X[k+1]-X[k]) * central(X+k+1, Y+k+1)
		:
			h00(t) * Y[n-2] + h10(t) * (X[n-1]-X[n-2]) * central(X+n-2, Y+n-2) + h01(t) * Y[n-1] + h11(t) * (X[n-1]-X[n-2]) * left(X+n-1, Y+n-1);
	}
	static OTYPE eval(const ITYPE *X, const OTYPE *Y, ITYPE x, int n)
	{
		int k = std::lower_bound(X, X+n, x) - X;
		return k < 1 ?
			eval(X, Y, (x - X[0]) / (X[1] - X[0]), 0, n)
		: k < n ?
			eval(X, Y, (x - X[k-1]) / (X[k] - X[k-1]), k-1, n)
		:
			eval(X, Y, (x - X[n-2]) / (X[n-1] - X[n-2]), n-1, n);
	}
	static constexpr OTYPE left(const OTYPE *Y)
	{
		return Y[0] - Y[-1];
	}
	static constexpr OTYPE right(const OTYPE *Y)
	{
		return Y[1] - Y[0];
	}
	static constexpr OTYPE central(const OTYPE *Y)
	{
		return ITYPE(0.5) * (Y[1] - Y[-1]);
	}
	static constexpr OTYPE eval(const OTYPE *Y, ITYPE t, int k, int n)
	{
		return k < 1 ?
			h00(t) * Y[0] + h10(t) * right(Y) + h01(t) * Y[1] + h11(t) * central(Y+1)
		: k < n-2 ?
			h00(t) * Y[k] + h10(t) * central(Y+k) + h01(t) * Y[k+1] + h11(t) * central(Y+k+1)
		:
			h00(t) * Y[n-2] + h10(t) * central(Y+n-2) + h01(t) * Y[n-1] + h11(t) * left(Y+n-1);
	}
	static OTYPE eval(const OTYPE *Y, ITYPE x, int n, ITYPE x0 = 0, ITYPE dx = 1)
	{
		ITYPE tx = (x - x0) / dx;
		int k = tx;
		ITYPE t = tx - ITYPE(k);
		return k < 0 ?
			eval(Y, tx, 0, n)
		: k < n-1 ?
			eval(Y, t, k, n)
		:
			eval(Y, tx-ITYPE(n-2), n-2, n);
	}
};

}

