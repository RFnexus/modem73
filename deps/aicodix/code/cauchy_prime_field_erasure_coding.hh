/*
Cauchy Prime Field Erasure Coding

Copyright 2024 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace CODE {

template <typename PF, typename IO, int MAX_LEN>
struct CauchyPrimeFieldErasureCoding
{
	static_assert(MAX_LEN < int(PF::P-1), "Block length must be smaller than largest field value");
	PF temp[MAX_LEN];
	typedef unsigned used_word;
	static constexpr int used_width = 8 * sizeof(used_word);
	static constexpr int used_length = (PF::P + used_width - 1) / used_width;
	used_word used_values[used_length];
	PF row_num, row_den;
	// $a_{ij} = \frac{1}{x_i + y_j}$
	PF cauchy_matrix(int i, int j)
	{
		PF row(i), col(j);
		return rcp(row + col);
	}
	// $b_{ij} = \frac{\prod_{k=1}^{n}{(x_j + y_k)(x_k + y_i)}}{(x_j + y_i)\prod_{k \ne j}^{n}{(x_j - x_k)}\prod_{k \ne i}^{n}{(y_i - y_k)}}$
	PF inverse_cauchy_matrix(const IO *rows, int i, int j, int n)
	{
#if 0
		PF row_j(rows[j]), col_i(i);
		PF prod_xy(1), prod_x(1), prod_y(1);
		for (int k = 0; k < n; k++) {
			PF row_k(rows[k]), col_k(k);
			prod_xy *= (row_j + col_k) * (row_k + col_i);
			if (k != j)
				prod_x *= (row_j - row_k);
			if (k != i)
				prod_y *= (col_i - col_k);
		}
		return prod_xy / ((row_j + col_i) * prod_x * prod_y);
#else
		PF row_j(rows[j]), col_i(i);
		if (j == 0) {
			PF num(1), den(1);
			for (int k = 0, r = 2; k < n; k++, --r) {
				PF row_k(rows[k]), col_k(k);
				num = mul(num, add(row_k, col_i));
				if (k != i)
					den = mul(den, sub(col_i, col_k));
				if (!r) {
					r = 3;
					num = reduce(num);
					den = reduce(den);
				}
			}
			row_num = reduce(num);
			row_den = reduce(den);
		}
		PF num(row_num), den(row_den);
		for (int k = 0, r = 2; k < n; k++, --r) {
			PF row_k(rows[k]), col_k(k);
			num = mul(num, add(row_j, col_k));
			if (k != j)
				den = mul(den, sub(row_j, row_k));
			if (!r) {
				r = 3;
				num = reduce(num);
				den = reduce(den);
			}
		}
		num = reduce(num);
		den = reduce(den);
		return num / (add(row_j, col_i) * den);
#endif
	}
	void mac(const IO *a, PF b, int len, bool first, bool last)
	{
		if (first && last) {
			for (int i = 0; i < len; i++)
				temp[i] = b * PF(a[i]);
		} else if (first) {
			for (int i = 0; i < len; i++)
				temp[i] = mul(b, PF(a[i]));
		} else if (last) {
			for (int i = 0; i < len; i++)
				temp[i] = reduce(add(temp[i], mul(b, PF(a[i]))));
		} else {
			for (int i = 0; i < len; i++)
				temp[i] = add(temp[i], mul(b, PF(a[i])));
		}
	}
	void mac_sub(IO *c, const IO *a, PF b, IO s, int len, bool first, bool last)
	{
		int v = PF::P-1;
		if (first && last) {
			for (int i = 0; i < len; i++)
				c[i] = (b * PF(a[i] == s ? v : a[i]))();
		} else if (first) {
			for (int i = 0; i < len; i++)
				temp[i] = mul(b, PF(a[i] == s ? v : a[i]));
		} else if (last) {
			for (int i = 0; i < len; i++)
				c[i] = reduce(add(temp[i], mul(b, PF(a[i] == s ? v : a[i]))))();
		} else {
			for (int i = 0; i < len; i++)
				temp[i] = add(temp[i], mul(b, PF(a[i] == s ? v : a[i])));
		}
	}
	int find_unused(int block_len)
	{
		for (int i = 0; i < used_length; ++i)
			used_values[i] = 0;
		for (int i = 0; i < block_len; ++i)
			used_values[temp[i]()/used_width] |= 1 << temp[i]()%used_width;
		int s = 0;
		while (used_values[s/used_width] & 1 << s%used_width)
			++s;
		return s;
	}
	int encode(const IO *data, IO *block, int block_id, int block_len, int block_cnt)
	{
		assert(block_id >= block_cnt && block_id < int(PF::P) / 2);
		assert(block_len <= MAX_LEN);
		for (int k = 0; k < block_cnt; k++) {
			PF a_ik = cauchy_matrix(block_id, k);
			mac(data + block_len * k, a_ik, block_len, !k, k == block_cnt - 1);
		}
		int sub = find_unused(block_len);
		for (int i = 0; i < block_len; ++i)
			block[i] = temp[i]() == PF::P-1 ? sub : temp[i]();
		return sub;
	}
	void decode(IO *data, const IO *blocks, const IO *block_subs, const IO *block_ids, int block_idx, int block_len, int block_cnt)
	{
		assert(block_len <= MAX_LEN);
		for (int k = 0; k < block_cnt; k++) {
			PF b_ik = inverse_cauchy_matrix(block_ids, block_idx, k, block_cnt);
			mac_sub(data, blocks + block_len * k, b_ik, block_subs[k], block_len, !k, k == block_cnt - 1);
		}
	}
};

}

