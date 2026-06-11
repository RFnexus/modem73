/*
Interface for reading and writing pixel data

Copyright 2020 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace DSP {

template <typename TYPE>
struct WritePEL
{
	virtual void write(const TYPE *, int, int = -1) = 0;
	virtual bool good() = 0;
	virtual bool mono() = 0;
	virtual int width() = 0;
	virtual int height() = 0;
	virtual ~WritePEL() = default;
};

template <typename TYPE>
struct ReadPEL
{
	virtual void read(TYPE *, int, int = -1) = 0;
	virtual bool good() = 0;
	virtual bool mono() = 0;
	virtual int width() = 0;
	virtual int height() = 0;
	virtual ~ReadPEL() = default;
};

}

