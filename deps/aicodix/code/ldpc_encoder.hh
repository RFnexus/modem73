/*
LDPC SISO encoder

Copyright 2018 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace CODE {

template <typename TABLE>
class LDPCEncoder
{
	static const int M = TABLE::M;
	static const int N = TABLE::N;
	static const int K = TABLE::K;
	static const int R = N-K;
	static const int q = R/M;
	static const int CNC = TABLE::LINKS_MAX_CN - 2;

	uint16_t pos[R * CNC];
	uint8_t cnc[q];
public:
	LDPCEncoder()
	{
		for (int i = 0; i < q; ++i)
			cnc[i] = 0;
		int bit_pos = 0;
		const int *row_ptr = TABLE::POS;
		for (int g = 0; TABLE::LEN[g]; ++g) {
			int bit_deg = TABLE::DEG[g];
			for (int r = 0; r < TABLE::LEN[g]; ++r) {
				for (int d = 0; d < bit_deg; ++d) {
					int n = row_ptr[d] % q;
					int m = row_ptr[d] / q;
					pos[CNC*n+cnc[n]++] = bit_pos + (M - m) % M;
				}
				row_ptr += bit_deg;
				bit_pos += M;
			}
		}
		for (int i = 0; i < q; ++i) {
			int cnt = cnc[i];
			int offset[CNC], shift[CNC];
			for (int c = 0; c < cnt; ++c) {
				shift[c] = pos[CNC*i+c] % M;
				offset[c] = pos[CNC*i+c] - shift[c];
			}
			for (int j = 1; j < M; ++j) {
				for (int c = 0; c < cnt; ++c) {
					shift[c] = (shift[c] + 1) % M;
					pos[CNC*(q*j+i)+c] = offset[c] + shift[c];
				}
			}
		}
	}
	void operator()(const int8_t *data, int8_t *parity)
	{
		int8_t tmp = 1;
		for (int j = 0; j < M; ++j) {
			for (int i = 0; i < q; ++i) {
				int cnt = cnc[i];
				for (int c = 0; c < cnt; ++c)
					tmp *= data[pos[CNC*(q*j+i)+c]];
				parity[q*j+i] = tmp;
			}
		}
	}
};

}

