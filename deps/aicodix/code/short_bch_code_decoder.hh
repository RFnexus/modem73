/*
Soft decoder for short BCH codes

Copyright 2020 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace CODE {

template <int N, int K, int G, int T>
class ShortBCHCodeDecoder
{
	static const int P = N - K;
	static const int W = 1 << K;
	static const int R = 1 << P;
	int err[R];
	short par[W];
	static_assert(N < 8 * sizeof(err[0]), "codeword type not wide enough");
	static_assert(P < 8 * sizeof(par[0]), "parity type not wide enough");
	static_assert(T > 0 && T <= 3, "unsupported radius T");
	static int modgen(int inp)
	{
		for (int i = K-1; i >= 0; --i) {
			int tmp = inp >> (i+P);
			inp ^= (tmp & 1) * (G << i);
		}
		return inp;
	}
	static int metric(const int8_t *soft, int hard)
	{
		if (hard < 0)
			return -1;
		int lol = 8 * sizeof(hard) - 1;
		int sum = 0;
		for (int i = 0; i < N; ++i)
			sum += ((hard << (lol-i)) >> lol) * soft[i];
		return sum;
	}
public:
	ShortBCHCodeDecoder()
	{
		for (int i = 0; i < W; ++i)
			par[i] = modgen(i << P);
		for (int i = 0; i < R; ++i)
			err[i] = 0;
		for (int a = 1<<(N-1); T >= 1 && a; a >>= 1) {
			err[modgen(a)] = a;
			for (int b = a >> 1; T >= 2 && b; b >>= 1) {
				err[modgen(a|b)] = a|b;
				for (int c = b >> 1; T >= 3 && c; c >>= 1) {
					err[modgen(a|b|c)] = a|b|c;
				}
			}
		}
	}
	int operator()(int inp)
	{
		int syn = (par[inp>>P] ^ inp) & (R-1);
		int pat = err[syn];
		return pat || !syn ? inp ^ pat : -1;
	}
	int operator()(const int8_t *code)
	{
		int cw = 0;
		for (int i = 0; i < N; ++i)
			cw |= (code[i] < 0) << i;
		int word = (*this)(cw);
		// hard decision
		if (0)
			return word;
		// metric of hard decision
		int best = metric(code, word);
		int next = -2;
		auto update = [code, &word, &best, &next](int hard) {
			if (hard == word)
				return;
			int met = metric(code, hard);
			if (met > best) {
				next = best;
				best = met;
				word = hard;
			} else if (met > next) {
				next = met;
			}
		};
		// flip each bit and see ..
		if (0) {
			for (int j = 0; j < N; ++j) {
				int tmp = 1 << j;
				int dec = (*this)(cw ^ tmp);
				update(dec);
			}
		}
		// Chase algorithm
		if (K > 4) {
			const int L = T;
			int worst[L] = { 0 };
			for (int k = 0; k < N; ++k) {
				int j = 0;
				while (j < L && std::abs(code[worst[j]]) < std::abs(code[k]))
					++j;
				for (int i = L-1; i > j; --i)
					worst[i] = worst[i-1];
				if (j < L)
					worst[j] = k;
			}
			for (int j = 1; j < (1 << L); ++j) {
				int tmp = 0;
				for (int i = 0; i < L; ++i)
					tmp |= ((j>>i)&1) << worst[i];
				int dec = (*this)(cw ^ tmp);
				update(dec);
			}
		}
		// maximum likelihood
		if (K <= 4) {
			for (int msg = 0; msg < W; ++msg) {
				int enc = (msg << P) | par[msg];
				update(enc);
			}
		}
		if (best == next)
			return -1;
		return word;
	}
};

}

