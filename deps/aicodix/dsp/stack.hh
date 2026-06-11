/*
Fixed-size stack

Copyright 2020 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace DSP {

template <typename TYPE, int SIZE>
class Stack
{
	TYPE buf[SIZE];
	int pos;
public:
	Stack() : pos(-1)
	{
	}
	void clear()
	{
		pos = -1;
	}
	void push(TYPE input)
	{
		assert(pos < SIZE-1);
		buf[++pos] = input;
	}
	void pop()
	{
		assert(pos >= 0);
		--pos;
	}
	TYPE top()
	{
		assert(pos >= 0);
		return buf[pos];
	}
	TYPE first()
	{
		assert(pos >= 0);
		return buf[0];
	}
	bool empty()
	{
		return pos == -1;
	}
	bool full()
	{
		return pos == SIZE-1;
	}
	int size()
	{
		return pos+1;
	}
	int max_size()
	{
		return SIZE;
	}
};

}

