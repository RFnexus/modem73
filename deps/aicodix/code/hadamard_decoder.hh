/*
Soft decoder for augmented Hadamard codes

Copyright 2020 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace CODE {

template <int K>
class HadamardDecoder
{
	static const int N = 1 << (K - 1);
public:
	int operator()(const int8_t *code)
	{
		int sum[N];
		for (int i = 0; i < N-1; i += 2) {
			sum[i] = code[i] + code[i+1];
			sum[i+1] = code[i] - code[i+1];
		}
		for (int h = 2; h < N; h *= 2) {
			for (int i = 0; i < N; i += 2 * h) {
				for (int j = i; j < i + h; ++j) {
					int x = sum[j] + sum[j+h];
					int y = sum[j] - sum[j+h];
					sum[j] = x;
					sum[j+h] = y;
				}
			}
		}
		int word = 0, best = 0, next = 0;
		for (int i = 0; i < N; ++i) {
			int mag = std::abs(sum[i]);
			int msg = i + N * (sum[i] < 0);
			if (mag > best) {
				next = best;
				best = mag;
				word = msg;
			} else if (mag > next) {
				next = mag;
			}
		}
		if (best == next)
			return -1;
		return word;
	}
};

template <>
struct HadamardDecoder<2>
{
	int operator()(const int8_t *code)
	{
		int x = code[0] + code[1];
		int y = code[0] - code[1];
		int mx = std::abs(x);
		int my = std::abs(y);
		if (mx == my)
			return -1;
		if (mx > my)
			return 2 * (x < 0);
		return 1 + 2 * (y < 0);
	}
};

template <>
class HadamardDecoder<3>
{
	static const int K = 3;
	static const int N = 1 << (K - 1);
public:
	int operator()(const int8_t *code)
	{
		int tmp[4] = {
			code[0] + code[1],
			code[0] - code[1],
			code[2] + code[3],
			code[2] - code[3],
		};
		int sum[4] = {
			tmp[0] + tmp[2],
			tmp[1] + tmp[3],
			tmp[0] - tmp[2],
			tmp[1] - tmp[3],
		};
		int word = 0, best = 0, next = 0;
		for (int i = 0; i < N; ++i) {
			int mag = std::abs(sum[i]);
			int msg = i + N * (sum[i] < 0);
			if (mag > best) {
				next = best;
				best = mag;
				word = msg;
			} else if (mag > next) {
				next = mag;
			}
		}
		if (best == next)
			return -1;
		return word;
	}
};

template <>
class HadamardDecoder<4>
{
	static const int K = 4;
	static const int N = 1 << (K - 1);
public:
	int operator()(const int8_t *c)
	{
		int d[8] = {
			c[0] + c[1],
			c[0] - c[1],
			c[2] + c[3],
			c[2] - c[3],
			c[4] + c[5],
			c[4] - c[5],
			c[6] + c[7],
			c[6] - c[7],
		};
		int e[8] = {
			d[0] + d[2],
			d[1] + d[3],
			d[0] - d[2],
			d[1] - d[3],
			d[4] + d[6],
			d[5] + d[7],
			d[4] - d[6],
			d[5] - d[7],
		};
		int sum[8] = {
			e[0] + e[4],
			e[1] + e[5],
			e[2] + e[6],
			e[3] + e[7],
			e[0] - e[4],
			e[1] - e[5],
			e[2] - e[6],
			e[3] - e[7],
		};
		int word = 0, best = 0, next = 0;
		for (int i = 0; i < N; ++i) {
			int mag = std::abs(sum[i]);
			int msg = i + N * (sum[i] < 0);
			if (mag > best) {
				next = best;
				best = mag;
				word = msg;
			} else if (mag > next) {
				next = mag;
			}
		}
		if (best == next)
			return -1;
		return word;
	}
};

}

