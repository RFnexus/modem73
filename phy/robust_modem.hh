// RFDM: Robust Data Modes for fading HF channels.
// QPSK carriers at 75 hz spacing, 20 ms symbols (13.3 ms + 6.7 ms cyclic prefix)
//   wide (2400 hz, 32 carriers)          narrow (600 hz, 8 carriers)
//   rdm-1200  (8192,4128) r=1/2  ~1150   rdmn-300  (8192,4128)  ~297 bps
//   rdm-800   (16384,4128) 3/4-punctured, r~1/3  ~780
//   rdm-600   (16384,4128) r=1/4  ~585   rdmn-150  (16384,4128) ~149 bps
//   rdm-300   r=1/4 chase x2      ~296
//
#pragma once

#include <vector>
#include <cstring>
#include <cmath>
#include <functional>
#include <algorithm>
#include "common.hh"
#include "polar_tables_short.hh"

#ifndef IMPULSE_BLANKER
#define IMPULSE_BLANKER 1
#endif
#include "hilbert.hh"
#include "blockdc.hh"
#include "polar_encoder.hh"
#include "polar_list_decoder.hh"


enum class RobustMode {
    RDM_1200 = 0,
    RDM_600  = 1,
    RDM_300  = 2,
    RDMN_300 = 3,
    RDMN_150 = 4,
    RDM_1200S = 5,
    RDM_600S  = 6,
    RDM_300S  = 7,
    RDMN_300S = 8,
    RDMN_150S = 9,
    // appended after the 0-4/5-9 family blocks to keep those numbers stable
    RDM_800   = 10,
    RDM_800S  = 11
};

static const char* ROBUST_MODE_NAMES[] =
    {"RDM-1200", "RDM-600", "RDM-300", "RDMN-300", "RDMN-150",
     "RDM-1200S", "RDM-600S", "RDM-300S", "RDMN-300S", "RDMN-150S",
     "RDM-800", "RDM-800S"};


constexpr int ROBUST_MODE_COUNT = 12;

struct RobustParams {
    static constexpr int SAMPLE_RATE = 48000;
    static constexpr int NFFT = 640;
    static constexpr int CP = 320;
    static constexpr int SYM = NFFT + CP;
    static constexpr int SPACING = SAMPLE_RATE / NFFT;
    static constexpr int NS = 4;
    static constexpr int NC_MAX = 32;
    static constexpr int DATA_BYTES = 512;
    static constexpr int DATA_BITS = 8 * DATA_BYTES;
    static constexpr int CRC_BITS = 32;
    static constexpr int NROWS_MAX = 1369;

    static bool is_narrow(RobustMode m) {
        int i = (int)m;
        return i < 10 && (i % 5 == 3 || i % 5 == 4);
    }
    static bool is_short(RobustMode m) {
        int i = (int)m;
        return (i >= 5 && i < 10) || i == 11;
    }
    static RobustMode with_framing(RobustMode m, bool short_frame) {
        if ((int)m >= 10)
            return short_frame ? RobustMode::RDM_800S : RobustMode::RDM_800;
        return (RobustMode)((int)m % 5 + (short_frame ? 5 : 0));
    }
    static int nc(RobustMode m) { return is_narrow(m) ? 8 : 32; }
    static int data_bytes(RobustMode m) { return is_short(m) ? 172 : DATA_BYTES; }
    static int data_bits(RobustMode m) { return 8 * data_bytes(m); }
    static int mesg_bits(RobustMode m) { return data_bits(m) + CRC_BITS; }
    static int code_order(RobustMode m) {
        static const int t[] = {13, 14, 14, 13, 14, 12, 13, 13, 12, 13, 14, 13};
        return t[(int)m];
    }
    static int code_bits(RobustMode m) { return 1 << code_order(m); }
    static int copies(RobustMode m) {
        return m == RobustMode::RDM_300 || m == RobustMode::RDM_300S ? 2 : 1;
    }
    // RDM-800 transmits only 3/4 of the shuffled codeword; the receiver
    // treats the rest as erasures, so the rate-1/4 mother code acts as a
    // rate-1/3 code
    static int sent_bits(RobustMode m) {
        if ((int)m >= 10)
            return 3 * code_bits(m) / 4;
        return code_bits(m) * copies(m);
    }
    // the punctured quarter is the HEAD of the shuffled codeword, so
    // RDM-800 bits are misaligned relative to an RDM-600 prefix: a strong
    // RDM-600 frame must not decode early (mislabeled, DCD dropped with
    // the sender still keyed) at the shorter RDM-800 checkpoint
    static int sent_offset(RobustMode m) {
        return (int)m >= 10 ? code_bits(m) / 4 : 0;
    }
    static const uint32_t* frozen(RobustMode m) {
        if (is_short(m))
            return code_order(m) == 12 ? frozen_4096_1408 : frozen_8192_1408;
        return code_order(m) == 13 ? frozen_8192_4128 : frozen_16384_4128;
    }
    static int nrows(RobustMode m) {
        static const int t[] = {173, 345, 685, 685, 1369,
                                89, 177, 349, 345, 689,
                                257, 129};
        return t[(int)m];
    }
    static bool is_pilot_row(int i) { return i % NS == 0; }
    static int frame_symbols(RobustMode m) { return 3 + nrows(m) + 2; }
    static int base_bin(int center_freq, RobustMode m) {
        return center_freq / SPACING - nc(m) / 2;
    }
    static float frame_duration(RobustMode m) {
        return frame_symbols(m) * (float)SYM / SAMPLE_RATE;
    }
    static int bitrate(RobustMode m) {
        return (int)(data_bits(m) / frame_duration(m));
    }
};

namespace robust_detail {

typedef float value;
typedef DSP::Complex<float> cmplx;

inline int nrz(bool bit) { return 1 - 2 * bit; }

template <typename T>
inline void shuffle_enc(T* dest, const T* src, int order) {
    if (order == 12) {
        CODE::XorShiftMask<int, 12, 1, 1, 4, 1> seq;
        dest[0] = src[0];
        for (int i = 1; i < 4096; ++i) dest[i] = src[seq()];
    } else if (order == 13) {
        CODE::XorShiftMask<int, 13, 1, 1, 9, 1> seq;
        dest[0] = src[0];
        for (int i = 1; i < 8192; ++i) dest[i] = src[seq()];
    } else {
        CODE::XorShiftMask<int, 14, 1, 5, 10, 1> seq;
        dest[0] = src[0];
        for (int i = 1; i < 16384; ++i) dest[i] = src[seq()];
    }
}

template <typename T>
inline void shuffle_dec(T* dest, const T* src, int order) {
    if (order == 12) {
        CODE::XorShiftMask<int, 12, 1, 1, 4, 1> seq;
        dest[0] = src[0];
        for (int i = 1; i < 4096; ++i) dest[seq()] = src[i];
    } else if (order == 13) {
        CODE::XorShiftMask<int, 13, 1, 1, 9, 1> seq;
        dest[0] = src[0];
        for (int i = 1; i < 8192; ++i) dest[seq()] = src[i];
    } else {
        CODE::XorShiftMask<int, 14, 1, 5, 10, 1> seq;
        dest[0] = src[0];
        for (int i = 1; i < 16384; ++i) dest[seq()] = src[i];
    }
}

}

class RobustEncoder {
    typedef robust_detail::value value;
    typedef robust_detail::cmplx cmplx;
public:
    std::vector<float> encode(const uint8_t* data, size_t len,
                              int center_freq, RobustMode mode) {
        using namespace robust_detail;
        if (rot_nc_ != RobustParams::nc(mode)) {
            rot_nc_ = RobustParams::nc(mode);
            for (int k = 0; k < rot_nc_; ++k)
                rot_[k] = DSP::polar<value>(1,
                    (value)M_PI * k * k / rot_nc_);
        }
        const int order = RobustParams::code_order(mode);
        const int cbits = RobustParams::code_bits(mode);
        const int nrows = RobustParams::nrows(mode);
        const int nc = RobustParams::nc(mode);
        const int base = RobustParams::base_bin(center_freq, mode);

        const int dbytes = RobustParams::data_bytes(mode);
        const int dbits = RobustParams::data_bits(mode);
        uint8_t buf[RobustParams::DATA_BYTES];
        std::memset(buf, 0, sizeof(buf));
        std::memcpy(buf, data, std::min(len, (size_t)dbytes));
        CODE::Xorshift32 scrambler;
        for (int i = 0; i < dbytes; ++i)
            buf[i] ^= scrambler();
        for (int i = 0; i < dbits; ++i)
            mesg_[i] = nrz(CODE::get_le_bit(buf, i));
        crc_.reset();
        for (int i = 0; i < dbytes; ++i)
            crc_(buf[i]);
        for (int i = 0; i < RobustParams::CRC_BITS; ++i)
            mesg_[dbits + i] = nrz((crc_() >> i) & 1);
        polar_encoder_(code_, mesg_, RobustParams::frozen(mode), order);
        shuffle_enc(perm_, code_, order);

        int8_t pre[RobustParams::NC_MAX], post[RobustParams::NC_MAX];
        CODE::MLS pre_seq(0x331, 214), post_seq(0x331, 97);
        for (int k = 0; k < nc; ++k) {
            pre[k] = nrz(pre_seq());
            post[k] = nrz(post_seq());
        }
        CODE::MLS pilot_seq(0x163, RobustParams::is_narrow(mode) ? 89 : 1);
        CODE::MLS filler_seq(0x43);

        std::vector<float> out;
        out.reserve((size_t)RobustParams::frame_symbols(mode) * RobustParams::SYM);

        {
            cmplx tones[RobustParams::NC_MAX];
            CODE::MLS noise_seq(0x331);
            for (int k = 0; k < nc; ++k)
                tones[k] = cmplx(nrz(noise_seq()), 0);
            emit_symbol(out, tones, base, nc, true);
        }
        {
            cmplx tones[RobustParams::NC_MAX];
            for (int k = 0; k < nc; ++k)
                tones[k] = cmplx(pre[k], 0);
            emit_symbol(out, tones, base, nc, true);
            emit_symbol(out, tones, base, nc, true);
        }
        int kbit = 0;
        const int total_bits = RobustParams::sent_bits(mode);
        const int poff = RobustParams::sent_offset(mode);
        for (int row = 0; row < nrows; ++row) {
            cmplx tones[RobustParams::NC_MAX];
            if (RobustParams::is_pilot_row(row)) {
                for (int k = 0; k < nc; ++k)
                    tones[k] = cmplx(nrz(pilot_seq()), 0);
            } else {
                for (int k = 0; k < nc; ++k) {
                    int8_t b[2];
                    if (kbit + 1 < total_bits) {
                        b[0] = perm_[(poff + kbit) % cbits];
                        b[1] = perm_[(poff + kbit + 1) % cbits];
                    } else {
                        b[0] = nrz(filler_seq());
                        b[1] = nrz(filler_seq());
                    }
                    kbit += 2;
                    tones[k] = PhaseShiftKeying<4, cmplx, int8_t>::map(b);
                }
            }
            emit_symbol(out, tones, base, nc);
        }
        {
            cmplx tones[RobustParams::NC_MAX];
            for (int k = 0; k < nc; ++k)
                tones[k] = cmplx(post[k], 0);
            emit_symbol(out, tones, base, nc, true);
            emit_symbol(out, tones, base, nc, true);
        }
        tx_filter(out, center_freq, nc);
        return out;
    }

    int get_payload_size(RobustMode m) { return RobustParams::data_bytes(m); }

private:
    // clipping raised the average power codec2-style; this contains the
    // spectral regrowth so the drive stays inside the occupied bandwidth
    void tx_filter(std::vector<float>& out, int center_freq, int nc) {
        const int L = 257;
        uint64_t key = ((uint64_t)center_freq << 8) | (uint32_t)nc;
        if (key != tx_bpf_key_) {
            int half = nc * RobustParams::SPACING / 2 + 400;
            value f1 = std::max(100, center_freq - half);
            value f2 = std::min(RobustParams::SAMPLE_RATE / 2 - 100,
                                center_freq + half);
            for (int i = 0; i < L; ++i) {
                int k = i - L / 2;
                value lp2 = k == 0 ? 2 * f2 / RobustParams::SAMPLE_RATE
                    : std::sin(2 * (value)M_PI * f2 * k / RobustParams::SAMPLE_RATE)
                        / ((value)M_PI * k);
                value lp1 = k == 0 ? 2 * f1 / RobustParams::SAMPLE_RATE
                    : std::sin(2 * (value)M_PI * f1 * k / RobustParams::SAMPLE_RATE)
                        / ((value)M_PI * k);
                value w = value(0.54) - value(0.46)
                    * std::cos(2 * (value)M_PI * i / (L - 1));
                tx_bpf_[i] = (lp2 - lp1) * w;
            }
            tx_bpf_key_ = key;
        }
        std::vector<float> x;
        x.swap(out);
        out.assign(x.size() + L - 1, 0.0f);
        for (size_t i = 0; i < x.size(); ++i) {
            float v = x[i];
            if (v == 0.0f)
                continue;
            for (int j = 0; j < L; ++j)
                out[i + j] += v * tx_bpf_[j];
        }
        for (auto& v : out)
            v = std::max(-0.95f, std::min(0.95f, v));
    }

    DSP::FastFourierTransform<RobustParams::NFFT, robust_detail::cmplx, 1> bwd_;
    robust_detail::cmplx rot_[RobustParams::NC_MAX];
    int rot_nc_ = 0;
    value tx_bpf_[257];
    uint64_t tx_bpf_key_ = (uint64_t)-1;
    CODE::PolarEncoder<int8_t> polar_encoder_;
    CODE::CRC<uint32_t> crc_{0x8F6E37A0};
    int8_t mesg_[RobustParams::DATA_BITS + RobustParams::CRC_BITS];
    int8_t code_[1 << 14];



    int8_t perm_[1 << 14];

    void emit_symbol(std::vector<float>& out, const robust_detail::cmplx* tones,
                     int base, int nc, bool rotate = false) {
        typedef robust_detail::cmplx cmplx;
        cmplx fdom[RobustParams::NFFT], tdom[RobustParams::NFFT];
        for (int i = 0; i < RobustParams::NFFT; ++i)
            fdom[i] = cmplx(0, 0);
        for (int k = 0; k < nc; ++k)
            fdom[base + k] = rotate ? tones[k] * rot_[k] : tones[k];
        bwd_(tdom, fdom);
        const float scale = 0.62f / std::sqrt((float)nc);
        auto clip = [](float v) {
            float a = v / 0.95f, a2 = a * a;
            return 0.95f * a / std::pow(1.0f + a2 * a2 * a2, 1.0f / 6.0f);
        };
        for (int i = RobustParams::NFFT - RobustParams::CP; i < RobustParams::NFFT; ++i)
            out.push_back(clip(scale * tdom[i].real()));
        for (int i = 0; i < RobustParams::NFFT; ++i)
            out.push_back(clip(scale * tdom[i].real()));
    }
};

class RobustDecoder {
    typedef robust_detail::value value;
    typedef robust_detail::cmplx cmplx;
    typedef int16_t code_type;
    typedef SIMD<code_type, 32> mesg_type;
    typedef SIMD<code_type, 64> mesg64_type;
public:
    using FrameCallback = std::function<void(const uint8_t*, size_t)>;

    RobustDecoder(int center_freq = 1500, bool narrow = false)
        : narrow_(narrow) {
        blockdc_.samples(129);
        if (narrow_) {
            modes_[0] = RobustMode::RDMN_300S;
            modes_[1] = RobustMode::RDMN_300;
            modes_[2] = RobustMode::RDMN_150S;
            modes_[3] = RobustMode::RDMN_150;
            nmodes_ = 4;
        } else {
            // ascending nrows so each mode's checkpoint is reached in order
            modes_[0] = RobustMode::RDM_1200S;  // 89 rows
            modes_[1] = RobustMode::RDM_800S;   // 129
            modes_[2] = RobustMode::RDM_1200;   // 173
            modes_[3] = RobustMode::RDM_600S;   // 177
            modes_[4] = RobustMode::RDM_800;    // 257
            modes_[5] = RobustMode::RDM_600;    // 345
            modes_[6] = RobustMode::RDM_300S;   // 349
            modes_[7] = RobustMode::RDM_300;    // 685
            nmodes_ = 8;
        }
        nc_ = RobustParams::nc(modes_[0]);
        for (int k = 0; k < nc_; ++k)
            rot_[k] = DSP::polar<value>(1, (value)M_PI * k * k / nc_);
        nrows_top_ = RobustParams::nrows(modes_[nmodes_ - 1]);
        keep_ = (size_t)(nrows_top_ + 10) * RobustParams::SYM + 2 * D;
        configure(center_freq);
    }

    void configure(int center_freq) {
        base_ = RobustParams::base_bin(center_freq, modes_[0]);
        int half = nc_ * RobustParams::SPACING / 2 + 200;
        value f1 = std::max(100, center_freq - half);
        value f2 = std::min(RobustParams::SAMPLE_RATE / 2 - 100, center_freq + half);
        for (int i = 0; i < bpf_len; ++i) {
            int k = i - bpf_len / 2;
            value lp2 = k == 0 ? 2 * f2 / RobustParams::SAMPLE_RATE
                : std::sin(2 * (value)M_PI * f2 * k / RobustParams::SAMPLE_RATE)
                    / ((value)M_PI * k);
            value lp1 = k == 0 ? 2 * f1 / RobustParams::SAMPLE_RATE
                : std::sin(2 * (value)M_PI * f1 * k / RobustParams::SAMPLE_RATE)
                    / ((value)M_PI * k);
            value w = value(0.54) - value(0.46)
                * std::cos(2 * (value)M_PI * i / (bpf_len - 1));
            bpf_taps_[i] = (lp2 - lp1) * w;
        }
        std::memset(bpf_hist_, 0, sizeof(bpf_hist_));
        bpf_pos_ = 0;
        reset();
    }

    void reset() {
        buf_.clear();
        state_ = State::SEARCH;
        P_ = cmplx(0, 0);
        Ra_ = Rb_ = 0;
        rows_done_ = 0;
        tried_mask_ = 0;
        confirmed_ = false;
        pilot_alive_total_ = -1;
    }

    void process(const float* samples, size_t count, FrameCallback callback) {
        for (size_t s = 0; s < count; ++s) {
            value smp = samples[s];
#if IMPULSE_BLANKER
            {
                value mag = std::abs(smp);
                blank_fast_ += (mag - blank_fast_) * value(1.0 / 64);
                if (blank_env_ < value(1e-9))
                    blank_env_ = mag;
                if (blank_env_ < blank_fast_ * value(1.0 / 6)) {
                    if (++blank_low_ >= 256) {
                        blank_env_ = blank_fast_;
                        blank_low_ = 0;
                    }
                } else {
                    blank_low_ = 0;
                }
                if (mag > 8 * blank_env_)
                    smp = 0;
                else if (mag > 6 * blank_env_)
                    smp *= 6 * blank_env_ / mag;
                blank_env_ += (std::min(mag, 3 * blank_env_) - blank_env_)
                            * value(1.0 / 4096);
            }
#endif
            cmplx x = hilbert_(blockdc_(bandpass(smp)));
            buf_.push_back(x);
            ++total_in_;
            int64_t n = (int64_t)buf_.size() - 1;

            switch (state_) {
            case State::SEARCH: {
                if (n < 2 * D)
                    break;
                if (n == 2 * D || (total_in_ & 0x3fff) == 0) {
                    refresh_sums(n);
                } else {
                    P_ = P_ + buf_[n] * conj(buf_[n - D])
                           - buf_[n - D] * conj(buf_[n - 2 * D]);
                    Rb_ += norm(buf_[n]) - norm(buf_[n - D]);
                    Ra_ += norm(buf_[n - D]) - norm(buf_[n - 2 * D]);
                }
                value m = norm(P_) / (Ra_ * Rb_ + value(1e-9));
                if (m > value(0.15)) {
                    state_ = State::PEAK;
                    peak_metric_ = m;
                    peak_pos_ = n;
                    peak_deadline_ = n + 2 * D;
                }
                break;
            }
            case State::PEAK: {
                if ((total_in_ & 0x3fff) == 0) {
                    refresh_sums(n);
                } else {
                    P_ = P_ + buf_[n] * conj(buf_[n - D])
                           - buf_[n - D] * conj(buf_[n - 2 * D]);
                    Rb_ += norm(buf_[n]) - norm(buf_[n - D]);
                    Ra_ += norm(buf_[n - D]) - norm(buf_[n - 2 * D]);
                }
                value m = norm(P_) / (Ra_ * Rb_ + value(1e-9));
                if (m > peak_metric_) {
                    peak_metric_ = m;
                    peak_pos_ = n;
                }
                if (n >= peak_deadline_) {
                    int kind = lock();
                    if (kind == 1) {
                        state_ = State::COLLECT;
                        locked_q_ = lock_q_;
                        cand_deadline_ = -1;
                        pilot_alive_total_ = total_in_;
                    } else {
                        if (kind == 2) {
                            if (rescue_backward(callback))
                                ++stats_rescues;
                            else
                                consume_anchor();
                        }
                        state_ = State::SEARCH;
                    }
                }
                break;
            }
            case State::COLLECT: {
                if ((total_in_ & 0x3fff) == 0) {
                    refresh_sums(n);
                } else {
                    P_ = P_ + buf_[n] * conj(buf_[n - D])
                           - buf_[n - D] * conj(buf_[n - 2 * D]);
                    Rb_ += norm(buf_[n]) - norm(buf_[n - D]);
                    Ra_ += norm(buf_[n - D]) - norm(buf_[n - 2 * D]);
                }
                {
                    value m = norm(P_) / (Ra_ * Rb_ + value(1e-9));
                    if (cand_deadline_ < 0) {
                        if (m > value(0.15) && n > frame_pos_ + 2 * D) {
                            cand_metric_ = m;
                            cand_pos_ = n;
                            cand_deadline_ = n + 2 * D;
                        }
                    } else if (m > cand_metric_) {
                        cand_metric_ = m;
                        cand_pos_ = n;
                    }
                }
                if (cand_deadline_ >= 0 && n >= cand_deadline_) {
                    cand_deadline_ = -1;
                    int64_t s_fp = frame_pos_, s_au = anchor_u_, s_pp = peak_pos_;
                    value s_om = omega_;
                    int s_bu = base_use_, s_rd = rows_done_;
                    unsigned s_tm = tried_mask_;
                    bool s_cf = confirmed_;
                    peak_pos_ = cand_pos_;
                    bool stale = pilot_alive_total_ >= 0 &&
                                 total_in_ - pilot_alive_total_ >= PREEMPT_STALE;
                    if (lock() == 1 && (lock_q_ > locked_q_ + value(0.08) || stale)) {
                        std::cerr << "RDM" << (narrow_ ? "n" : "")
                                  << ": collect preempted (q=" << lock_q_
                                  << " over " << locked_q_
                                  << (stale ? ", stale" : "") << ")" << std::endl;
                        locked_q_ = lock_q_;
                        pilot_alive_total_ = total_in_;
                        ++stats_false_locks;
                        break;
                    }
                    frame_pos_ = s_fp; anchor_u_ = s_au; peak_pos_ = s_pp;
                    omega_ = s_om; base_use_ = s_bu;
                    rows_done_ = s_rd; tried_mask_ = s_tm;
                    confirmed_ = s_cf;
                }
                while (rows_done_ < nrows_top_) {
                    int64_t start = row_start(rows_done_);
                    if (start + RobustParams::NFFT > (int64_t)buf_.size())
                        break;
                    take_row(rows_done_, start);
                    ++rows_done_;
                    if ((rows_done_ - 1) % RobustParams::NS == 0) {
                        int pr = (rows_done_ - 1) / RobustParams::NS;
                        int wrows = nc_ <= 8 ? 48 : 12;
                        int w = pr + 1 < wrows ? pr + 1 : wrows;
                        if (pr >= 2 && (pr + 1) % 2 == 0 &&
                            pilot_window_live(pr, w, 0.10f))
                            pilot_alive_total_ = total_in_;
                    }
                    if (rows_done_ == 2 * RobustParams::NS + 1 &&
                        pilot_sanity(3, nc_ <= 8 ? 0.45f : 0.28f)) {
                        ++stats_sync_count;
                        confirmed_ = true;
                    } else if ((rows_done_ == 1 && !pilot_sanity(1, nc_ <= 8 ? 0.30f : 0.18f)) ||
                        (rows_done_ == 2 * RobustParams::NS + 1 &&
                         !pilot_sanity(3, nc_ <= 8 ? 0.45f : 0.28f))) {
                        std::cerr << "RDM" << (narrow_ ? "n" : "")
                                  << ": collect abort at row " << rows_done_
                                  << std::endl;
                        state_ = State::SEARCH;
                        rows_done_ = 0;
                        confirmed_ = false;
                        ++stats_false_locks;
                        break;
                    }
                    bool done = false;
                    for (int mi = 0; mi < nmodes_; ++mi) {
                        if (rows_done_ != RobustParams::nrows(modes_[mi]) ||
                            ((tried_mask_ >> mi) & 1))
                            continue;
                        tried_mask_ |= 1u << mi;
                        bool last = mi == nmodes_ - 1;
                        if (try_decode(modes_[mi], callback)) {
                            finish_frame(modes_[mi]);
                            done = true;
                        } else if (last) {
                            ++stats_crc_errors;
                            std::cerr << "RDM" << (narrow_ ? "n" : "")
                                      << ": ladder exhausted at "
                                      << rows_done_ << " rows" << std::endl;
                            finish_frame(modes_[mi]);
                            done = true;
                        }
                        break;
                    }
                    if (done)
                        break;
                }
                break;
            }
            }
        }
        if (state_ == State::SEARCH && buf_.size() > keep_ + 262144) {
            size_t cut = buf_.size() - keep_;
            buf_.erase(buf_.begin(), buf_.begin() + cut);
            refresh_sums((int64_t)buf_.size() - 1);
        }
    }

    float get_last_snr() const { return last_snr_; }
    float get_last_ber() const { return last_ber_; }
    float get_ber_ema() const { return ber_ema_; }
    RobustMode get_last_mode() const { return last_mode_; }

    bool in_frame() const { return state_ != State::SEARCH; }

    bool carrier_active() const {
        return state_ != State::SEARCH && confirmed_ &&
               (pilot_alive_total_ < 0 ||
                total_in_ - pilot_alive_total_ < DCD_GRACE);
    }

    int stats_sync_count = 0;
    int stats_preamble_errors = 0;
    int stats_false_locks = 0;
    int stats_crc_errors = 0;
    int stats_rescues = 0;
    int stats_retry_success = 0;

    void reset_stats() {
        stats_sync_count = 0;
        stats_preamble_errors = 0;
        stats_false_locks = 0;
        stats_crc_errors = 0;
        stats_rescues = 0;
        stats_retry_success = 0;
        last_snr_ = 0;
        last_ber_ = -1;
        ber_ema_ = -1;
    }

private:
    static constexpr int D = RobustParams::SYM;

    enum class State { SEARCH, PEAK, COLLECT };
    State state_ = State::SEARCH;

    DSP::BlockDC<value, value> blockdc_;
    DSP::Hilbert<cmplx, 129> hilbert_;
    DSP::FastFourierTransform<RobustParams::NFFT, cmplx, -1> fwd_;
    CODE::PolarListDecoder<mesg_type, 14> polar_decoder_;
    CODE::PolarListDecoder<mesg64_type, 14> polar_decoder64_;
    CODE::PolarEncoder<int8_t> ber_encoder_;
    CODE::CRC<uint32_t> crc_{0x8F6E37A0};

    static const int bpf_len = 257;
    value bpf_taps_[bpf_len];
    value bpf_hist_[bpf_len];
    int bpf_pos_ = 0;
    value blank_env_ = 0;
    value blank_fast_ = 0;
    int blank_low_ = 0;

    value bandpass(value x) {
        bpf_hist_[bpf_pos_] = x;
        value acc = 0;
        int idx = bpf_pos_;
        for (int i = 0; i < bpf_len; ++i) {
            acc += bpf_taps_[i] * bpf_hist_[idx];
            if (--idx < 0)
                idx = bpf_len - 1;
        }
        if (++bpf_pos_ >= bpf_len)
            bpf_pos_ = 0;
        return acc;
    }



    bool narrow_ = false;
    int nc_ = 32;
    RobustMode modes_[8] = {RobustMode::RDM_1200, RobustMode::RDM_600,
                            RobustMode::RDM_300};
    int nmodes_ = 3;
    int nrows_top_ = 685;
    size_t keep_ = 0;


    std::vector<cmplx> buf_;
    int64_t total_in_ = 0;
    int64_t last_decode_total_ = 0;

    int base_ = 4;
    int base_use_ = 4;

    cmplx P_{0, 0};
    value Ra_ = 0, Rb_ = 0, peak_metric_ = 0;
    int64_t peak_pos_ = 0, peak_deadline_ = 0;

    value omega_ = 0;
    int64_t frame_pos_ = 0;
    int64_t anchor_u_ = 0;
    int rows_done_ = 0;
    unsigned tried_mask_ = 0;
    value lock_q_ = 0;
    value locked_q_ = 0;
    int64_t spent_anchor_ = -(int64_t)1 << 60;
    value cand_metric_ = 0;
    int64_t cand_pos_ = 0;
    int64_t cand_deadline_ = -1;
    // set once the row-9 pilot sanity passes: the collect is a real frame,
    // not a lock that will be culled; gates the DCD so noise-floor false
    // locks never assert carrier_active
    bool confirmed_ = false;
    int64_t pilot_alive_total_ = -1;
    static constexpr int64_t PREEMPT_STALE = 288000;
    static constexpr int64_t DCD_GRACE = 48000;

    cmplx rows_[RobustParams::NROWS_MAX][RobustParams::NC_MAX];
    cmplx rot_[RobustParams::NC_MAX];
    int8_t pre_[RobustParams::NC_MAX];
    int8_t post_[RobustParams::NC_MAX];
    bool seq_init_ = false;

    mesg_type mesg_[1 << 14];
    code_type code_[1 << 14];
    code_type perm_[1 << 15];
    int8_t ber_mesg_[1 << 14];
    int8_t ber_code_[1 << 14];

    value last_snr_ = 0, last_ber_ = -1, ber_ema_ = -1;
    RobustMode last_mode_ = RobustMode::RDM_1200;

    void refresh_sums(int64_t n) {
        P_ = cmplx(0, 0);
        Ra_ = Rb_ = 0;
        if (n < 2 * D)
            return;
        for (int64_t m = n - D + 1; m <= n; ++m) {
            P_ = P_ + buf_[m] * conj(buf_[m - D]);
            Rb_ += norm(buf_[m]);
            Ra_ += norm(buf_[m - D]);
        }
    }

    void window_fft(int64_t start, cmplx* bins) {
        cmplx tdom[RobustParams::NFFT], fdom[RobustParams::NFFT];
        for (int i = 0; i < RobustParams::NFFT; ++i) {
            value ph = -omega_ * (value)(start + i - frame_pos_);
            tdom[i] = buf_[start + i] * DSP::polar<value>(1, ph);
        }
        fwd_(fdom, tdom);
        for (int k = 0; k < nc_ + 4; ++k)
            bins[k] = fdom[base_use_ - 2 + k];
    }

    cmplx cp_corr(int64_t p2u, int margin, int len) {
        cmplx acc(0, 0);
        for (int s = 0; s < 2; ++s) {
            int64_t u = p2u - (int64_t)s * D;
            for (int j = margin; j < margin + len; ++j) {
                int64_t a = u - RobustParams::CP + j;
                int64_t b = u + RobustParams::NFFT - RobustParams::CP + j;
                if (a >= 0 && b < (int64_t)buf_.size())
                    acc = acc + buf_[b] * conj(buf_[a]);
            }
        }
        return acc;
    }

    value probe2(const cmplx* bins, const cmplx* bins_b, int boff,
                 const int8_t* known, value* tau) {
        cmplx s(0, 0);
        value pwr = 0;
        for (int pass = 0; pass < 2; ++pass) {
            const cmplx* b = pass ? bins_b : bins;
            if (!b)
                continue;
            cmplx d[RobustParams::NC_MAX];
            for (int k = 0; k < nc_; ++k)
                d[k] = (value)known[k] * conj(rot_[k]) * b[2 + boff + k];
            for (int k = 0; k + 1 < nc_; ++k) {
                s = s + d[k + 1] * conj(d[k]);
                pwr += norm(d[k]);
            }
            pwr += norm(d[nc_ - 1]);
        }
        if (tau)
            *tau = -arg(s) * RobustParams::NFFT / (2 * (value)M_PI);
        return abs(s) / (pwr + value(1e-9));
    }

    int lock() {
        using namespace robust_detail;
        if (!seq_init_) {
            CODE::MLS pre_seq(0x331, 214), post_seq(0x331, 97);
            for (int k = 0; k < nc_; ++k) {
                pre_[k] = nrz(pre_seq());
                post_[k] = nrz(post_seq());
            }
            seq_init_ = true;
        }
        frame_pos_ = peak_pos_ + 1;
        int64_t p2u = peak_pos_ - RobustParams::NFFT + 1;
        if (p2u - D - RobustParams::CP < 0)
            return 0;

        omega_ = arg(cp_corr(p2u, 96, 128)) / (value)RobustParams::NFFT;

        cmplx bins[RobustParams::NC_MAX + 4], bins1[RobustParams::NC_MAX + 4];
        base_use_ = base_;
        window_fft(p2u, bins);
        window_fft(p2u - D, bins1);
        int best_off = 0, best_kind = 1;
        value best_q = 0, best_tau = 0;
        for (int boff = -2; boff <= 2; ++boff) {
            if (base_ + boff < 2 ||
                base_ + boff + nc_ + 2 > RobustParams::NFFT / 2)
                continue;
            value tau = 0;
            value q = probe2(bins, bins1, boff, pre_, &tau);
            if (q > best_q) {
                best_q = q; best_off = boff; best_tau = tau; best_kind = 1;
            }
            q = probe2(bins, bins1, boff, post_, &tau);
            if (q > best_q) {
                best_q = q; best_off = boff; best_tau = tau; best_kind = 2;
            }
        }
        value qgate = nc_ <= 8 ? value(0.62) : value(0.4);
        lock_q_ = best_q;
        if (best_q < qgate) {
            ++stats_false_locks;
            return 0;
        }
        const value bin_step = 2 * (value)M_PI * RobustParams::SPACING
                             / RobustParams::SAMPLE_RATE;
        omega_ += best_off * bin_step;
        base_use_ = base_;

        int shift = (int)std::lround(best_tau) - 96;
        p2u += shift;
        if (p2u - D - RobustParams::CP < 0)
            return 0;
        omega_ = arg(cp_corr(p2u, 112, 192)) / (value)RobustParams::NFFT
               + best_off * bin_step;

        if (best_kind == 2) {
            int64_t a_abs = total_in_ - (int64_t)buf_.size() + p2u;
            if (std::llabs(a_abs - spent_anchor_) < RobustParams::SYM / 2)
                return 0;
        }
        anchor_u_ = p2u;
        std::cerr << "RDM" << (narrow_ ? "n" : "") << ": Sync ("
                  << (best_kind == 1 ? "lead" : "TRAIL") << " q=" << best_q
                  << " boff=" << best_off << " tau=" << best_tau
                  << " cfo=" << omega_ * RobustParams::SAMPLE_RATE
                      / (2 * (value)M_PI) << " Hz) t="
                  << (double)total_in_ / RobustParams::SAMPLE_RATE << "s"
                  << std::endl;
        if (best_kind == 1) {
            frame_pos_ = p2u + D;
            rows_done_ = 0;
            tried_mask_ = 0;
            confirmed_ = false;
        }
        return best_kind;
    }

    int64_t row_start(int row) const {
        return frame_pos_ + (int64_t)row * RobustParams::SYM;
    }

    bool pilot_sanity(int npr, float gate) {
        using namespace robust_detail;
        CODE::MLS ps(0x163, narrow_ ? 89 : 1);
        cmplx s(0, 0);
        value pwr = 0;
        value binp[RobustParams::NC_MAX] = {};
        for (int r = 0; r < npr; ++r) {
            cmplx d[RobustParams::NC_MAX];
            for (int k = 0; k < nc_; ++k) {
                d[k] = (value)nrz(ps()) * rows_[r * RobustParams::NS][k];
                binp[k] += norm(d[k]);
            }
            for (int k = 0; k + 1 < nc_; ++k) {
                s = s + d[k + 1] * conj(d[k]);
                pwr += norm(d[k]);
            }
            pwr += norm(d[nc_ - 1]);
        }
        value peak = 0;
        for (int k = 0; k < nc_; ++k)
            peak = std::max(peak, binp[k]);
        int lit = 0;
        for (int k = 0; k < nc_; ++k)
            lit += binp[k] > value(0.02) * peak;
        if (nc_ > 8 && lit < nc_ / 2)
            return false;
        value q = abs(s) / (pwr + value(1e-9));
        return q >= (value)gate;
    }

    void take_row(int row, int64_t start) {
        cmplx bins[RobustParams::NC_MAX + 4];
        window_fft(start, bins);
        for (int k = 0; k < nc_; ++k)
            rows_[row][k] = bins[2 + k];
    }

    bool pilot_window_live(int last_pr, int wrows, float gate) {
        using namespace robust_detail;
        CODE::MLS ps(0x163, narrow_ ? 89 : 1);
        int first = last_pr - wrows + 1;
        for (int i = 0; i < first * nc_; ++i)
            ps();
        cmplx s(0, 0);
        value pwr = 0;
        for (int p = first; p <= last_pr; ++p) {
            cmplx d[RobustParams::NC_MAX];
            for (int k = 0; k < nc_; ++k)
                d[k] = (value)nrz(ps()) * rows_[p * RobustParams::NS][k];
            for (int k = 0; k + 1 < nc_; ++k) {
                s = s + d[k + 1] * conj(d[k]);
                pwr += norm(d[k]);
            }
            pwr += norm(d[nc_ - 1]);
        }
        return abs(s) / (pwr + value(1e-9)) >= (value)gate;
    }

    void consume_anchor() {
        spent_anchor_ = total_in_ - (int64_t)buf_.size() + anchor_u_;
    }

    void finish_frame(RobustMode mode) {
        int64_t end = row_start(RobustParams::nrows(mode));
        if (end > 0 && (size_t)end <= buf_.size())
            buf_.erase(buf_.begin(), buf_.begin() + (size_t)end);
        else
            buf_.clear();
        state_ = State::SEARCH;
        rows_done_ = 0;
        confirmed_ = false;
        refresh_sums((int64_t)buf_.size() - 1);
    }

    bool rescue_backward(FrameCallback callback) {
        for (int mi = 0; mi < nmodes_; ++mi) {
            RobustMode m = modes_[mi];
            int n = RobustParams::nrows(m);
            int64_t row0 = anchor_u_ - D - (int64_t)n * RobustParams::SYM;
            // late join: head rows never captured decode as erasures, inside
            // the margin the rate-1/4 mother code leaves (RDM-800 already
            // spends half of that margin on puncturing)
            int missing = row0 < 0
                ? (int)((-row0 + RobustParams::SYM - 1) / RobustParams::SYM)
                : 0;
            if (missing > (RobustParams::sent_offset(m) ? n / 8 : n / 4))
                continue;
            int64_t head_abs = total_in_ - (int64_t)buf_.size()
                             + std::max<int64_t>(row0, 0);
            if (head_abs < last_decode_total_)
                continue;
            frame_pos_ = row0;
            if (mi == 0)
                ++stats_sync_count;
            for (int i = 0; i < n; ++i) {
                int64_t start = row_start(i);
                if (start < 0) {
                    for (int k = 0; k < nc_; ++k)
                        rows_[i][k] = cmplx(0, 0);
                } else {
                    take_row(i, start);
                }
            }
            if (try_decode(m, callback)) {
                std::cerr << "RDM" << (narrow_ ? "n" : "")
                          << ": backward rescue " << ROBUST_MODE_NAMES[(int)m]
                          << (missing ? " (late join)" : "")
                          << std::endl;
                int64_t end = anchor_u_ + RobustParams::NFFT;
                if (end > 0 && (size_t)end <= buf_.size())
                    buf_.erase(buf_.begin(), buf_.begin() + (size_t)end);
                else
                    buf_.clear();
                rows_done_ = 0;
                refresh_sums((int64_t)buf_.size() - 1);
                return true;
            }
        }
        return false;
    }

    bool try_decode(RobustMode mode, FrameCallback callback) {
        using namespace robust_detail;
        const int order = RobustParams::code_order(mode);
        const int cbits = RobustParams::code_bits(mode);
        const int nrows = RobustParams::nrows(mode);
        const int copies = RobustParams::copies(mode);
        const int total_bits = RobustParams::sent_bits(mode);

        CODE::MLS pilot_seq(0x163, narrow_ ? 89 : 1);
        static cmplx chanP[RobustParams::NROWS_MAX / RobustParams::NS + 2]
                          [RobustParams::NC_MAX];
        static value pagree[RobustParams::NROWS_MAX / RobustParams::NS + 2];
        int pilot_row_of[RobustParams::NROWS_MAX];
        int npil = 0;
        for (int i = 0; i < nrows; ++i) {
            if (RobustParams::is_pilot_row(i)) {
                cmplx raw[RobustParams::NC_MAX];
                for (int k = 0; k < nc_; ++k)
                    raw[k] = (value)nrz(pilot_seq()) * rows_[i][k];
                {
                    value pw[RobustParams::NC_MAX], srt[RobustParams::NC_MAX];
                    for (int k = 0; k < nc_; ++k)
                        srt[k] = pw[k] = norm(raw[k]);
                    std::nth_element(srt, srt + nc_ / 2, srt + nc_);
                    value med = srt[nc_ / 2];
                    for (int k = 0; k < nc_; ++k)
                        if (pw[k] > 8 * med + value(1e-12))
                            raw[k] = cmplx(0, 0);
                }
                cmplx sl(0, 0);
                for (int k = 0; k + 1 < nc_; ++k)
                    sl = sl + raw[k + 1] * conj(raw[k]);
                value slope = arg(sl);
                cmplx flat[RobustParams::NC_MAX];
                for (int k = 0; k < nc_; ++k)
                    flat[k] = raw[k] * DSP::polar<value>(1, -slope * k);
                for (int k = 0; k < nc_; ++k) {
                    cmplx acc = value(0.6) * flat[k];
                    value w = value(0.6);
                    if (k > 0) { acc = acc + value(0.2) * flat[k - 1]; w += value(0.2); }
                    if (k + 1 < nc_) { acc = acc + value(0.2) * flat[k + 1]; w += value(0.2); }
                    chanP[npil][k] = (value(1) / w) * acc * DSP::polar<value>(1, slope * k);
                }
                pilot_row_of[i] = npil;
                ++npil;
            } else {
                pilot_row_of[i] = npil - 1;
            }
        }
        for (int p = 0; p + 1 < npil; ++p) {
            cmplx dot(0, 0);
            value a2 = 0, b2 = 0;
            for (int k = 0; k < nc_; ++k) {
                dot = dot + chanP[p + 1][k] * conj(chanP[p][k]);
                a2 += norm(chanP[p][k]);
                b2 += norm(chanP[p + 1][k]);
            }
            value agree = abs(dot) / (std::sqrt(a2 * b2) + value(1e-12));
            pagree[p] = agree > value(0.7) ? value(1) : agree * agree * 2;
        }
        pagree[npil - 1] = 1;

        static value cgate[RobustParams::NROWS_MAX / RobustParams::NS + 2]
                          [RobustParams::NC_MAX];
        for (int p = 0; p + 1 < npil; ++p) {
            value r[RobustParams::NC_MAX], srt[RobustParams::NC_MAX];
            for (int k = 0; k < nc_; ++k) {
                value num = norm(chanP[p][k] + chanP[p + 1][k]);
                value den = 2 * (norm(chanP[p][k]) + norm(chanP[p + 1][k]))
                          + value(1e-12);
                r[k] = num / den;
                srt[k] = r[k];
            }
            std::nth_element(srt, srt + nc_ / 2, srt + nc_);
            value med = std::max(srt[nc_ / 2], value(1e-3));
            for (int k = 0; k < nc_; ++k) {
                value w = r[k] / med;
                cgate[p][k] = w >= value(0.7) ? value(1)
                            : std::max(w * w * 2, value(0.05));
            }
        }
        for (int k = 0; k < nc_; ++k)
            cgate[npil - 1][k] = 1;

        int kbit = 0;
        value snr_acc = 0, row_pwr = 0;
        int snr_rows = 0;
        static int drow_start[RobustParams::NROWS_MAX];
        static value drow_prec[RobustParams::NROWS_MAX];
        int ndrows = 0;
        for (int i = 0; i < nrows && kbit < total_bits; ++i) {
            if (RobustParams::is_pilot_row(i))
                continue;
            int pa = pilot_row_of[i];
            int pb = pa + 1 < npil ? pa + 1 : pa;
            int row_a = pa * RobustParams::NS;
            value t = pb > pa
                ? (value)(i - row_a) / (value)RobustParams::NS : value(0);
            cmplx chan[RobustParams::NC_MAX], dem[RobustParams::NC_MAX];
            value cp_mean = 0;
            // Catmull-Rom over four pilot rows: through a fading null the
            // complex gain curves and its phase slews, which linear
            // interpolation between the bracketing pilots gets wrong
            int p0 = pa > 0 ? pa - 1 : pa;
            int p3 = pb + 1 < npil ? pb + 1 : pb;
            value t2 = t * t, t3 = t2 * t;
            value w0 = value(0.5) * (2 * t2 - t3 - t);
            value w1 = value(0.5) * (3 * t3 - 5 * t2 + 2);
            value w2 = value(0.5) * (4 * t2 - 3 * t3 + t);
            value w3 = value(0.5) * (t3 - t2);
            for (int k = 0; k < nc_; ++k) {
                chan[k] = w0 * chanP[p0][k] + w1 * chanP[pa][k]
                        + w2 * chanP[pb][k] + w3 * chanP[p3][k];
                cp_mean += norm(chan[k]);
            }
            cp_mean /= nc_;
            value sp = 0, np = 0;
            for (int k = 0; k < nc_; ++k) {
                if (norm(chan[k]) > 0) {
                    cmplx d = rows_[i][k] / chan[k];
                    dem[k] = norm(d) < 9 ? d : cmplx(0, 0);
                } else {
                    dem[k] = cmplx(0, 0);
                }
                code_type hb[2];
                PhaseShiftKeying<4, cmplx, code_type>::hard(hb, dem[k]);
                cmplx hard = PhaseShiftKeying<4, cmplx, code_type>::map(hb);
                sp += norm(hard);
                np += norm(dem[k] - hard);
            }
            value precision = std::min(sp / (np + value(1e-9)), value(1023));
            precision *= pagree[pa];
            snr_acc += precision;
            row_pwr += cp_mean;
            ++snr_rows;
            drow_start[ndrows] = kbit;
            drow_prec[ndrows] = precision;
            ++ndrows;
            for (int k = 0; k < nc_ && kbit + 1 < total_bits + 2; ++k) {
                value prec = precision;
                if (cp_mean > 0)
                    prec = std::min(precision * norm(chan[k]) / cp_mean, value(1023));
                prec *= cgate[pa][k];
                code_type b[2];
                PhaseShiftKeying<4, cmplx, code_type>::soft(b, dem[k], prec);
                if (kbit < total_bits) perm_[kbit] = b[0];
                if (kbit + 1 < total_bits) perm_[kbit + 1] = b[1];
                kbit += 2;
            }
        }
        if (kbit < total_bits)
            return false;
        if (row_pwr < value(1e-12))
            return false;

        static code_type perm_raw[1 << 15];
        std::memcpy(perm_raw, perm_, sizeof(code_type) * total_bits);

        const int crc_bits = RobustParams::mesg_bits(mode);
        const int poff = RobustParams::sent_offset(mode);
        auto combine_shuffle = [&]() {
            if (copies == 2) {
                for (int i = 0; i < cbits; ++i) {
                    int32_t v = (int32_t)perm_[i] + (int32_t)perm_[i + cbits];
                    perm_[i] = (code_type)(v > 32767 ? 32767
                                         : v < -32767 ? -32767 : v);
                }
            }
            if (poff) {
                static code_type full[1 << 14];
                for (int i = 0; i < poff; ++i)
                    full[i] = 0;        // punctured head decodes as erasures
                std::memcpy(full + poff, perm_,
                            sizeof(code_type) * total_bits);
                shuffle_dec(code_, full, order);
            } else {
                shuffle_dec(code_, perm_, order);
            }
        };
        auto scan = [&](auto& mesg, auto& dec) -> bool {
            dec(nullptr, mesg, code_, RobustParams::frozen(mode), order);
            for (int lane = 0;
                 lane < std::remove_reference_t<decltype(mesg[0])>::SIZE;
                 ++lane) {
                crc_.reset();
                for (int i = 0; i < crc_bits; ++i)
                    crc_(mesg[i].v[lane] < 0);
                if (crc_() != 0)
                    continue;
                bool any = false;
                for (int i = 0; i < crc_bits && !any; ++i)
                    any = mesg[i].v[lane] < 0;
                if (!any)
                    continue;
                for (int i = 0; i < crc_bits; ++i)
                    ber_mesg_[i] = mesg[i].v[lane] < 0 ? -1 : 1;
                return true;
            }
            return false;
        };

        combine_shuffle();
        bool decoded = scan(mesg_, polar_decoder_);

        if (!decoded && ndrows >= 8) {
            static int order_idx[RobustParams::NROWS_MAX];
            for (int i = 0; i < ndrows; ++i)
                order_idx[i] = i;
            std::sort(order_idx, order_idx + ndrows, [&](int a, int b) {
                return drow_prec[a] < drow_prec[b];
            });
            static mesg64_type mesg64[1 << 14];
            struct Attempt { int erase_frac; bool wide; };
            static const Attempt attempts[] = {
                {8, false}, {4, false}, {0, true}, {4, true}};
            for (const auto& at : attempts) {
                int nerase = at.erase_frac ? std::max(1, ndrows / at.erase_frac) : 0;
                std::memcpy(perm_, perm_raw, sizeof(code_type) * total_bits);
                for (int e = 0; e < nerase; ++e) {
                    int r = order_idx[e];
                    int lim = std::min(drow_start[r] + 2 * nc_, total_bits);
                    for (int b = drow_start[r]; b < lim; ++b)
                        perm_[b] = 0;
                }
                combine_shuffle();
                decoded = at.wide ? scan(mesg64, polar_decoder64_)
                                  : scan(mesg_, polar_decoder_);
                if (decoded) {
                    ++stats_retry_success;
                    break;
                }
            }
        }
        if (!decoded)
            return false;

        uint8_t out[RobustParams::DATA_BYTES];
        for (int i = 0; i < RobustParams::data_bits(mode); ++i)
            CODE::set_le_bit(out, i, ber_mesg_[i] < 0);

        ber_encoder_(ber_code_, ber_mesg_, RobustParams::frozen(mode), order);
        int errs = 0, counted = 0;
        for (int i = 0; i < cbits; ++i) {
            if (code_[i] == 0)          // punctured or erased, never received
                continue;
            ++counted;
            if ((code_[i] < 0) != (ber_code_[i] < 0))
                ++errs;
        }
        last_ber_ = counted ? (value)errs / counted : 0;
        ber_ema_ = ber_ema_ < 0 ? last_ber_
                                : value(0.3) * last_ber_ + value(0.7) * ber_ema_;
        last_snr_ = snr_rows > 0
                  ? 10 * std::log10(std::max(snr_acc / snr_rows, value(0.1)))
                  : 0;
        last_mode_ = mode;
        last_decode_total_ = total_in_;

        CODE::Xorshift32 scrambler;
        for (int i = 0; i < RobustParams::data_bytes(mode); ++i)
            out[i] ^= scrambler();

        std::cerr << "RDM" << (narrow_ ? "n" : "") << ": Decoded "
                  << ROBUST_MODE_NAMES[(int)mode] << " SNR=" << last_snr_
                  << " dB BER=" << 100 * last_ber_ << "%" << std::endl;
        callback(out, RobustParams::data_bytes(mode));
        return true;
    }
};
