/*
LDPC SISO layered decoder

Copyright 2018 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include <algorithm>
#include "simd.hh"
#include "rotate.hh"

namespace CODE {

template <typename TABLE, int BETA>
class LDPCDecoder
{
#ifdef __AVX2__
	static const int SIMD_SIZE = 32;
	// M = 360 = 30 * 12
	static const int WORD_SIZE = 30;
#else
	static const int SIMD_SIZE = 16;
	// M = 360 = 15 * 24
	static const int WORD_SIZE = 15;
#endif
	static_assert(TABLE::M % WORD_SIZE == 0, "M must be multiple of word size");
	static_assert(WORD_SIZE <= SIMD_SIZE, "SIMD size must be bigger or equal word size");
	static const int M = TABLE::M;
	static const int N = TABLE::N;
	static const int K = TABLE::K;
	static const int R = N-K;
	static const int q = R/M;
	static const int D = WORD_SIZE;
	static const int W = M/D;
	static const int PTY = R/D;
	static const int MSG = K/D;
	static const int CNC = TABLE::LINKS_MAX_CN - 2;
	static const int BNL = (TABLE::LINKS_TOTAL + D-1) / D;
	static const int LOC = (TABLE::LINKS_TOTAL - (2*R-1) + D-1) / D;

	typedef SIMD<int8_t, SIMD_SIZE> TYPE;
	typedef struct { uint16_t off; uint16_t shi; } Loc;
	typedef uint32_t wd_t;
	static_assert(sizeof(wd_t) * 8 >= CNC, "write disable mask needs at least as many bits as max check node links");
	Rotate<TYPE, D> rotate;

	TYPE bnl[BNL];
	TYPE msg[MSG];
	TYPE pty[PTY];
	Loc loc[LOC];
	wd_t wd[PTY];
	uint8_t cnc[q];

	static TYPE eor(TYPE a, TYPE b)
	{
		return vreinterpret<TYPE>(veor(vmask(a), vmask(b)));
	}
	static TYPE orr(TYPE a, TYPE b)
	{
		return vreinterpret<TYPE>(vorr(vmask(a), vmask(b)));
	}
	static TYPE other(TYPE a, TYPE b, TYPE c)
	{
		return vreinterpret<TYPE>(vbsl(vceq(a, b), vmask(c), vmask(b)));
	}
	static TYPE mine(TYPE a, TYPE b)
	{
		return orr(eor(a, b), vdup<TYPE>(127));
	}
	static TYPE selfcorr(TYPE a, TYPE b)
	{
		return vreinterpret<TYPE>(vand(vmask(b), vorr(vceqz(a), veor(vcgtz(a), vcltz(b)))));
	}

	bool bad()
	{
		Loc *lo = loc;
		for (int i = 0; i < q; ++i) {
			int cnt = cnc[i];
			int deg = cnt + 2;
			auto res = vmask(vzero<TYPE>());
			for (int j = 0; j < W; ++j) {
				TYPE cnv = vdup<TYPE>(1);
				for (int k = 0; k < deg; ++k) {
					TYPE tmp;
					if (k < cnt) {
						tmp = rotate(msg[lo[k].off], -lo[k].shi);
					} else if (k == cnt) {
						tmp = pty[W*i+j];
					} else {
						if (i) {
							tmp = pty[W*(i-1)+j];
						} else if (j) {
							tmp = pty[W*(q-1)+j-1];
						} else {
							tmp = rotate(pty[PTY-1], 1);
							tmp.v[0] = 127;
						}
					}
					cnv = vsign(cnv, tmp);
				}
				res = vorr(res, vclez(cnv));
				lo += cnt;
			}
			for (int n = 0; n < D; ++n)
				if (res.v[n])
					return true;
		}
		return false;
	}
	void update()
	{
		TYPE *bl = bnl;
		Loc *lo = loc;
		for (int i = 0; i < q; ++i) {
			int cnt = cnc[i];
			int deg = cnt + 2;
			for (int j = 0; j < W; ++j) {
				TYPE mags[CNC+2], inps[CNC+2];
				TYPE min0 = vdup<TYPE>(127);
				TYPE min1 = vdup<TYPE>(127);
				TYPE signs = vdup<TYPE>(127);

				for (int k = 0; k < deg; ++k) {
					TYPE tmp;
					if (k < cnt) {
						tmp = rotate(msg[lo[k].off], -lo[k].shi);
					} else if (k == cnt) {
						tmp = pty[W*i+j];
					} else {
						if (i) {
							tmp = pty[W*(i-1)+j];
						} else if (j) {
							tmp = pty[W*(q-1)+j-1];
						} else {
							tmp = rotate(pty[PTY-1], 1);
							tmp.v[0] = 127;
						}
					}

					TYPE inp = vqsub(tmp, bl[k]);

					TYPE mag = vqabs(inp);

					if (BETA) {
						auto beta = vunsigned(vdup<TYPE>(BETA));
						mag = vsigned(vqsub(vunsigned(mag), beta));
					}

					min1 = vmin(min1, vmax(min0, mag));
					min0 = vmin(min0, mag);

					signs = eor(signs, inp);

					inps[k] = inp;
					mags[k] = mag;
				}
				for (int k = 0; k < deg; ++k) {
					TYPE mag = mags[k];
					TYPE inp = inps[k];

					TYPE out = vsign(other(mag, min0, min1), mine(signs, inp));

					out = vclamp(out, -31, 31);

					out = selfcorr(bl[k], out);

					TYPE tmp = vqadd(inp, out);

					if (k < cnt) {
						if (!((wd[W*i+j]>>k)&1)) {
							bl[k] = out;
							msg[lo[k].off] = rotate(tmp, lo[k].shi);
						}
					} else if (k == cnt) {
						bl[k] = out;
						pty[W*i+j] = tmp;
					} else {
						bl[k] = out;
						if (i) {
							pty[W*(i-1)+j] = tmp;
						} else if (j) {
							pty[W*(q-1)+j-1] = tmp;
						} else {
							tmp.v[0] = pty[PTY-1].v[D-1];
							pty[PTY-1] = rotate(tmp, -1);
						}
					}
				}
				if (wd[W*i+j]) {
					for (int first = 0, c = 1; c < cnt; ++c) {
						if (lo[first].off != lo[c].off || c == cnt-1) {
							int last = c - 1;
							if (c == cnt-1)
								++last;
							if (last != first) {
								int count = last - first + 1;
								wd_t mask = ((1 << count) - 1) << first;
								wd_t cur = wd[W*i+j];
								wd_t tmp = cur & mask;
								wd_t ror = (tmp >> 1) | (tmp << (count-1));
								wd[W*i+j] = (cur & ~mask) | (ror & mask);
							}
							first = c;
						}
					}
				}
				lo += cnt;
				bl += deg;
			}
		}
		//assert(bl <= bnl + BNL);
		//std::cerr << BNL - (bl - bnl) << std::endl;
	}
public:
	LDPCDecoder()
	{
		uint16_t pos[q * CNC];
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
		Loc *lo = loc;
		for (int i = 0; i < q; ++i) {
			int cnt = cnc[i];
			int offset[CNC], shift[CNC];
			for (int c = 0; c < cnt; ++c) {
				shift[c] = pos[CNC*i+c] % M;
				offset[c] = pos[CNC*i+c] - shift[c];
			}
			for (int j = 0; j < W; ++j) {
				for (int c = 0; c < cnt; ++c) {
					lo[c].off = offset[c] / D + shift[c] % W;
					lo[c].shi = shift[c] / W;
					shift[c] = (shift[c] + 1) % M;
				}
				std::sort(lo, lo + cnt, [](const Loc &a, const Loc &b){ return a.off < b.off; });
				wd[W*i+j] = 0;
				for (int c = 0; c < cnt-1; ++c)
					if (lo[c].off == lo[c+1].off)
						wd[W*i+j] |= 1 << c;
				lo += cnt;
			}
		}
		//assert(lo <= loc + LOC);
		//std::cerr << LOC - (lo - loc) << std::endl;
	}
	int operator()(int8_t *message, int8_t *parity, int trials = 25)
	{
		for (int i = 0; i < BNL; ++i)
			bnl[i] = vzero<TYPE>();
		for (int i = 0; i < K/M; ++i)
			for (int j = 0; j < W; ++j)
				for (int n = 0; n < D; ++n)
					msg[W*i+j].v[n] = message[M*i+W*n+j];
		for (int i = 0; i < q; ++i)
			for (int j = 0; j < W; ++j)
				for (int n = 0; n < D; ++n)
					pty[W*i+j].v[n] = parity[q*(W*n+j)+i];
		while (bad() && --trials >= 0)
			update();
		for (int i = 0; i < K/M; ++i)
			for (int j = 0; j < W; ++j)
				for (int n = 0; n < D; ++n)
					message[M*i+W*n+j] = msg[W*i+j].v[n];
		for (int i = 0; i < q; ++i)
			for (int j = 0; j < W; ++j)
				for (int n = 0; n < D; ++n)
					parity[q*(W*n+j)+i] = pty[W*i+j].v[n];
		return trials;
	}
};

}

