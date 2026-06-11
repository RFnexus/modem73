/*
Read and write WAV files

Copyright 2018 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include <fstream>
#include "pcm.hh"

namespace DSP {

template <typename TYPE>
class ReadWAV : public ReadPCM<TYPE>
{
	std::ifstream is;
	int bits_, bytes, rate_, channels_, frames_;
	int offset, factor;
	int readLE(int b)
	{
		int v = 0;
		for (int i = 0; i < b; ++i)
			v |= is.get() << (8 * i);
		if (b == 2 && v > 32767)
			v |= ~32767;
		if (b == 3 && v > 8388607)
			v |= ~8388607;
		return v;
	}
	bool cmp4(const char *a, const char *b)
	{
		for (int i = 0; i < 4; ++i)
			if (a[i] != b[i])
				return true;
		return false;
	}
	static float int2float(int value)
	{
		union {
			int i;
			float f;
		} u;
		u.i = value;
		return u.f;
	}
public:
	ReadWAV(const char *name) : is(name, std::ios::binary)
	{
		char ChunkID[4];
		is.read(ChunkID, 4);
		if (cmp4("RIFF", ChunkID))
			return;
		int ChunkSize = readLE(4);
		char Format[4];
		is.read(Format, 4);
		if (cmp4("WAVE", Format))
			return;
		char Subchunk1ID[4];
		is.read(Subchunk1ID, 4);
		if (cmp4("fmt ", Subchunk1ID))
			return;
		int Subchunk1Size = readLE(4);
		if (Subchunk1Size != 16 && Subchunk1Size != 18)
			return;
		int AudioFormat = readLE(2);
		if (AudioFormat != 1 && AudioFormat != 3)
			return;
		channels_ = readLE(2);
		rate_ = readLE(4);
		int ByteRate = readLE(4);
		int BlockAlign = readLE(2);
		bits_ = readLE(2);
		if (bits_ != 8 && bits_ != 16 && bits_ != 32)
			return;
		if ((bits_ == 8 || bits_ == 16) && AudioFormat != 1)
			return;
		if (bits_ == 32 && AudioFormat != 3)
			return;
		bytes = bits_ / 8;
		if (bytes * channels_ != BlockAlign)
			return;
		if (rate_ * bytes * channels_ != ByteRate)
			return;
		if (Subchunk1Size == 18) {
			int ExtSize = readLE(2);
			if (ExtSize != 0)
				return;
			char SubchunkID[4];
			is.read(SubchunkID, 4);
			if (cmp4("fact", SubchunkID))
				return;
			int SubchunkSize = readLE(4);
			if (SubchunkSize != 4)
				return;
			frames_ = readLE(4);
		}
		char Subchunk2ID[4];
		is.read(Subchunk2ID, 4);
		if (cmp4("data", Subchunk2ID))
			return;
		int Subchunk2Size = readLE(4);
		int overhead = bits_ == 32 ? 58 : 44;
		if (Subchunk2Size <= 0 || ChunkSize <= 0) {
			frames_ = -1;
		} else {
			if (Subchunk2Size + overhead - 8 != ChunkSize)
				return;
			if (Subchunk1Size == 16)
				frames_ = Subchunk2Size / (bytes * channels_);
		}
		switch (bits_) {
			case 8:
				offset = 128;
				factor = 127;
				break;
			case 16:
				offset = 0;
				factor = 32767;
				break;
			case 32:
				offset = 0;
				factor = 1;
				break;
			default:
				return;
		}
	}
	void read(TYPE *buf, int num, int stride = -1)
	{
		if (stride < 0)
			stride = channels_;
		for (int n = 0; n < num; ++n) {
			for (int c = 0; c < channels_; ++c) {
				if (bytes == 4)
					buf[stride * n + c] = int2float(readLE(4));
				else
					buf[stride * n + c] = TYPE(readLE(bytes) - offset) / TYPE(factor);
			}
		}
	}
	bool good()
	{
		return is.good();
	}
	void skip(int num)
	{
		is.seekg(num * channels_ * bytes, std::ios_base::cur);
	}
	int frames()
	{
		return frames_;
	}
	int channels()
	{
		return channels_;
	}
	int rate()
	{
		return rate_;
	}
	int bits()
	{
		return bits_;
	}
};

template <typename TYPE>
class WriteWAV : public WritePCM<TYPE>
{
	std::ofstream os;
	int bytes, channels_, rate_;
	int offset, factor, min, max;
	void writeLE(int v, int b)
	{
		for (int i = 0; i < b; ++i)
			os.put(255 & (v >> (8 * i)));
	}
	static int float2int(float value)
	{
		union {
			float f;
			int i;
		} u;
		u.f = value;
		return u.i;
	}
public:
	WriteWAV(const char *name, int rate, int bits, int channels) :
		os(name, std::ios::binary | std::ios::trunc),
		bytes(bits / 8), channels_(channels), rate_(rate)
	{
		switch (bits) {
			case 8:
				offset = 128;
				factor = 127;
				min = 0;
				max = 255;
				break;
			case 16:
				offset = 0;
				factor = 32767;
				min = -32768;
				max = 32767;
				break;
			default:
				bits = 32;
				bytes = 4;
				offset = 0;
				factor = 1;
				min = -1;
				max = 1;
		}
		os.write("RIFF", 4); // ChunkID
		writeLE(bits == 32 ? 50 : 36, 4); // ChunkSize
		os.write("WAVE", 4); // Format
		os.write("fmt ", 4); // Subchunk1ID
		if (bits == 32) {
			writeLE(18, 4); // Subchunk1Size
			writeLE(3, 2); // AudioFormat
		} else {
			writeLE(16, 4); // Subchunk1Size
			writeLE(1, 2); // AudioFormat
		}
		writeLE(channels_, 2); // NumChannels
		writeLE(rate_, 4); // SampleRate
		writeLE(rate_ * channels_ * bytes, 4); // ByteRate
		writeLE(channels_ * bytes, 2); // BlockAlign
		writeLE(8 * bytes, 2); // BitsPerSample
		if (bits == 32) {
			writeLE(0, 2); // ExtSize
			os.write("fact", 4); // SubchunkID
			writeLE(4, 4); // SubchunkSize
			writeLE(0, 4); // FrameCount
		}
		os.write("data", 4); // Subchunk2ID
		writeLE(-1, 4); // Subchunk2Size
	}
	~WriteWAV()
	{
		int overhead = bytes == 4 ? 58 : 44;
		int position = int(os.tellp());
		int size = position - overhead;
		os.seekp(4);
		writeLE(position - 8, 4); // ChunkSize
		if (bytes == 4) {
			os.seekp(46);
			int frames = size / (bytes * channels_);
			writeLE(frames, 4); // FrameCount
		}
		os.seekp(overhead - 4);
		writeLE(size, 4); // Subchunk2Size
	}
	void write(const TYPE *buf, int num, int stride = -1)
	{
		if (stride < 0)
			stride = channels_;
		for (int n = 0; n < num; ++n) {
			for (int c = 0; c < channels_; ++c) {
				if (bytes == 4) {
					writeLE(float2int(buf[stride * n + c]), 4);
				} else {
					TYPE v = TYPE(offset) + TYPE(factor) * buf[stride * n + c];
					writeLE(std::nearbyint(std::min(std::max(v, TYPE(min)), TYPE(max))), bytes);
				}
			}
		}
	}
	bool good()
	{
		return os.good();
	}
	void silence(int num)
	{
		for (int i = 0; i < num * channels_; ++i)
			writeLE(offset, bytes);
	}
	int channels()
	{
		return channels_;
	}
	int rate()
	{
		return rate_;
	}
};

}

