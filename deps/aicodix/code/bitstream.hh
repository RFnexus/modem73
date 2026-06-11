/*
Bitwise stream container

Copyright 2018 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace CODE {

template <int SIZE>
class Bitstream
{
public:
	static const int BITS = SIZE;
	static const int BYTES = (SIZE + 7) / 8;
private:
	uint8_t buf_[BYTES];
	int pos_ = 0;
public:
	void seek(int pos)
	{
		pos_ = pos;
	}
	int tell()
	{
		return pos_;
	}
	uint8_t *data()
	{
		return buf_;
	}
	void reset()
	{
		pos_ = 0;
		for (int i = 0; i < BYTES; ++i)
			buf_[i] = 0;
	}
	void putbit(bool b)
	{
		int bit = pos_ % 8;
		int byte = pos_ / 8;
		++pos_;
		uint8_t mask = 1 << bit;
		uint8_t tmp = ~mask & buf_[byte];
		buf_[byte] = tmp | b << bit;
	}
	bool getbit()
	{
		int bit = pos_ % 8;
		int byte = pos_ / 8;
		++pos_;
		return (buf_[byte] >> bit) & 1;
	}
	void putbyte(const uint8_t b)
	{
		int bit = pos_ % 8;
		int byte = pos_ / 8;
		pos_ += 8;
		if (bit) {
			uint8_t mask = (1 << bit) - 1;
			uint8_t lsb = mask & buf_[byte];
			buf_[byte++] = lsb | b << bit;
			uint8_t msb = ~mask & buf_[byte];
			buf_[byte] = msb | b >> (8 - bit);
		} else {
			buf_[byte] = b;
		}
	}
	uint8_t getbyte()
	{
		int bit = pos_ % 8;
		int byte = pos_ / 8;
		pos_ += 8;
		if (bit) {
			uint8_t lsb = buf_[byte++] >> bit;
			uint8_t msb = buf_[byte] << (8 - bit);
			return msb | lsb;
		}
		return buf_[byte];
	}
	void writebytes(const uint8_t *buf, int len)
	{
		int bit = pos_ % 8;
		int byte = pos_ / 8;
		pos_ += 8 * len;
		if (bit) {
			uint8_t mask = (1 << bit) - 1;
			for (int i = 0; i < len; ++i) {
				uint8_t lsb = mask & buf_[byte];
				buf_[byte++] = lsb | buf[i] << bit;
				uint8_t msb = ~mask & buf_[byte];
				buf_[byte] = msb | buf[i] >> (8 - bit);
			}
		} else {
			for (int i = 0; i < len; ++i)
				buf_[byte++] = buf[i];
		}
	}
	void readbytes(uint8_t *buf, int len)
	{
		int bit = pos_ % 8;
		int byte = pos_ / 8;
		pos_ += 8 * len;
		if (bit) {
			for (int i = 0; i < len; ++i) {
				uint8_t lsb = buf_[byte++] >> bit;
				uint8_t msb = buf_[byte] << (8 - bit);
				buf[i] = msb | lsb;
			}
		} else {
			for (int i = 0; i < len; ++i)
				buf[i] = buf_[byte++];
		}
	}
	template <typename TYPE>
	void writebits(TYPE b, int num)
	{
		for (int sum = 0; num;) {
			int bit = pos_ % 8;
			int byte = pos_ / 8;
			int copy = std::min(8 - bit, num);
			uint8_t mask = (1 << copy) - 1;
			uint8_t src = (mask & (b >> sum)) << bit;
			uint8_t dst = ~(mask << bit) & buf_[byte];
			buf_[byte++] = dst | src;
			pos_ += copy;
			num -= copy;
			sum += copy;
		}
	}
	template <typename TYPE>
	void readbits(TYPE *b, int num)
	{
		TYPE a = 0;
		for (int sum = 0; num;) {
			int bit = pos_ % 8;
			int byte = pos_ / 8;
			int copy = std::min(8 - bit, num);
			uint8_t mask = (1 << copy) - 1;
			TYPE tmp = mask & (buf_[byte++] >> bit);
			a |= tmp << sum;
			pos_ += copy;
			num -= copy;
			sum += copy;
		}
		*b = a;
	}
};

}

