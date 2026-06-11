/*
Read and write Netpbm files

Copyright 2020 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include <fstream>
#include "pel.hh"

namespace DSP {

template <typename TYPE>
class ReadPNM : public ReadPEL<TYPE>
{
	std::ifstream is;
	int width_, height_, maxval_;
	bool mono_;

	TYPE linear(int val)
	{
		int K0 = std::nearbyint(0.04045 * maxval_);
		TYPE phi(12.92 * maxval_), a(0.055 * maxval_), gamma(2.4);
		return val <= K0 ? val / phi : std::pow((val + a) / (maxval_ + a), gamma);
	}
	static bool space(int c)
	{
		return c == ' ' || c == '\n' || c == '\t';
	}
	static bool newline(int c)
	{
		return c == '\n';
	}
	static bool digit(int c)
	{
		return c >= '0' && c <= '9';
	}
	static int value(int c)
	{
		return c - '0';
	}
public:
	ReadPNM(const char *name) : is(name, std::ios::binary)
	{
		if (is.get() != 'P')
			return;
		int c = is.get();
		if (!digit(c))
			return;
		int number = value(c);
		if (number == 5)
			mono_ = true;
		else if (number == 6)
			mono_ = false;
		else
			return;
		c = is.get();
		if (!space(c))
			return;
		int num[3];
		for (int i = 0; i < 3; ++i) {
			if (newline(c))
				for (c = is.get(); c == '#'; c = is.get())
					while (is.good() && !newline(is.get()));
			while (space(c))
				c = is.get();
			for (num[i] = 0; digit(c); c = is.get())
				num[i] = 10 * num[i] + value(c);
		}
		if (!space(c))
			return;
		width_ = num[0];
		height_ = num[1];
		maxval_ = num[2];
	}
	void read(TYPE *buf, int num, int stride = -1)
	{
		if (mono_) {
			if (stride < 0)
				stride = 1;
			for (int i = 0; i < num; ++i) {
				int G = is.get();
				buf[stride*i] = linear(G);
			}
		} else {
			if (stride < 0)
				stride = 3;
			for (int i = 0; i < num; ++i) {
				int R = is.get();
				int G = is.get();
				int B = is.get();
				buf[stride*i+0] = linear(R);
				buf[stride*i+1] = linear(G);
				buf[stride*i+2] = linear(B);
			}
		}
	}
	bool good()
	{
		return is.good();
	}
	bool mono()
	{
		return mono_;
	}
	int width()
	{
		return width_;
	}
	int height()
	{
		return height_;
	}
};

template <typename TYPE>
class WritePNM : public WritePEL<TYPE>
{
	std::ofstream os;
	int width_, height_, maxval_;
	bool mono_;

	int srgb(TYPE val)
	{
		TYPE K0(0.04045), a(0.055), phi(12.92), gamma(2.4);
		val = std::min(std::max(val, TYPE(0)), TYPE(1));
		val = val <= K0 / phi ? val * phi : (1 + a) * std::pow(val, 1 / gamma) - a;
		return std::nearbyint(maxval_ * val);
	}
public:
	WritePNM(const char *name, int width, int height, bool mono = false, int maxval = 255) :
		os(name, std::ios::binary | std::ios::trunc),
		width_(width), height_(height), maxval_(maxval), mono_(mono)
	{
		int number = mono ? 5 : 6;
		os << 'P' << number << ' ' << width << ' ' << height << ' ' << maxval << '\n';
	}
	void write(const TYPE *buf, int num, int stride = -1)
	{
		if (mono_) {
			if (stride < 0)
				stride = 1;
			for (int i = 0; i < num; ++i) {
				int G = srgb(buf[stride*i]);
				os.put(G);
			}
		} else {
			if (stride < 0)
				stride = 3;
			for (int i = 0; i < num; ++i) {
				int R = srgb(buf[stride*i+0]);
				int G = srgb(buf[stride*i+1]);
				int B = srgb(buf[stride*i+2]);
				os.put(R);
				os.put(G);
				os.put(B);
			}
		}
	}
	bool good()
	{
		return os.good();
	}
	bool mono()
	{
		return mono_;
	}
	int width()
	{
		return width_;
	}
	int height()
	{
		return height_;
	}
};

}

