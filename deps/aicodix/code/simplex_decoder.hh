/*
Soft decoder for Simplex codes

Copyright 2020 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace CODE {

template <int K>
class SimplexDecoder
{
	static const int W = 1 << K;
public:
	int operator()(const int8_t *code)
	{
		int sum[W];
		sum[0] = code[0];
		sum[1] = -code[0];
		for (int i = 1; i < W-2; i += 2) {
			sum[i+1] = code[i] + code[i+1];
			sum[i+2] = code[i] - code[i+1];
		}
		for (int h = 2; h < W; h *= 2) {
			for (int i = 0; i < W; i += 2 * h) {
				for (int j = i; j < i + h; ++j) {
					int x = sum[j] + sum[j+h];
					int y = sum[j] - sum[j+h];
					sum[j] = x;
					sum[j+h] = y;
				}
			}
		}
		int word = 0, best = 0, next = 0;
		for (int msg = 0; msg < W; ++msg) {
			int mag = sum[msg];
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
class SimplexDecoder<2>
{
	static const int K = 2;
	static const int W = 1 << K;
public:
	int operator()(const int8_t *code)
	{
		int tmp[4] = {
			code[0], -code[0],
			code[1] + code[2],
			code[1] - code[2],
		};
		int sum[W] = {
			tmp[0] + tmp[2],
			tmp[1] + tmp[3],
			tmp[0] - tmp[2],
			tmp[1] - tmp[3],
		};
		int word = 0, best = 0, next = 0;
		for (int msg = 0; msg < W; ++msg) {
			int mag = sum[msg];
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
class SimplexDecoder<3>
{
	static const int K = 3;
	static const int W = 1 << K;
public:
	int operator()(const int8_t *c)
	{
		int d[8] = {
			c[0], -c[0],
			c[1] + c[2],
			c[1] - c[2],
			c[3] + c[4],
			c[3] - c[4],
			c[5] + c[6],
			c[5] - c[6],
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
		int sum[W] = {
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
		for (int msg = 0; msg < W; ++msg) {
			int mag = sum[msg];
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
class SimplexDecoder<4>
{
	static const int K = 4;
	static const int W = 1 << K;
public:
	int operator()(const int8_t *c)
	{
		int d[16] = {
			c[0], -c[0],
			c[1] + c[2],
			c[1] - c[2],
			c[3] + c[4],
			c[3] - c[4],
			c[5] + c[6],
			c[5] - c[6],
			c[7] + c[8],
			c[7] - c[8],
			c[9] + c[10],
			c[9] - c[10],
			c[11] + c[12],
			c[11] - c[12],
			c[13] + c[14],
			c[13] - c[14],
		};
		int e[16] = {
			d[0] + d[2],
			d[1] + d[3],
			d[0] - d[2],
			d[1] - d[3],
			d[4] + d[6],
			d[5] + d[7],
			d[4] - d[6],
			d[5] - d[7],
			d[8] + d[10],
			d[9] + d[11],
			d[8] - d[10],
			d[9] - d[11],
			d[12] + d[14],
			d[13] + d[15],
			d[12] - d[14],
			d[13] - d[15],
		};
		int f[16] = {
			e[0] + e[4],
			e[1] + e[5],
			e[2] + e[6],
			e[3] + e[7],
			e[0] - e[4],
			e[1] - e[5],
			e[2] - e[6],
			e[3] - e[7],
			e[8] + e[12],
			e[9] + e[13],
			e[10] + e[14],
			e[11] + e[15],
			e[8] - e[12],
			e[9] - e[13],
			e[10] - e[14],
			e[11] - e[15],
		};
		int sum[W] = {
			f[0] + f[8],
			f[1] + f[9],
			f[2] + f[10],
			f[3] + f[11],
			f[4] + f[12],
			f[5] + f[13],
			f[6] + f[14],
			f[7] + f[15],
			f[0] - f[8],
			f[1] - f[9],
			f[2] - f[10],
			f[3] - f[11],
			f[4] - f[12],
			f[5] - f[13],
			f[6] - f[14],
			f[7] - f[15],
		};
		int word = 0, best = 0, next = 0;
		for (int msg = 0; msg < W; ++msg) {
			int mag = sum[msg];
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

