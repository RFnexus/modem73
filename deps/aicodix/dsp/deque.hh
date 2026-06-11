/*
Double-ended queue

Copyright 2020 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace DSP {

template <typename TYPE, int SIZE>
class Deque
{
	TYPE buf[SIZE];
	int head, tail, count;
public:
	Deque() : head(SIZE-1), tail(0), count(0)
	{
	}
	void clear()
	{
		head = SIZE-1;
		tail = 0;
		count = 0;
	}
	void push_back(TYPE input)
	{
		assert(count < SIZE);
		++count;
		if (--tail < 0)
			tail = SIZE-1;
		buf[tail] = input;
	}
	void push_front(TYPE input)
	{
		assert(count < SIZE);
		++count;
		if (++head >= SIZE)
			head = 0;
		buf[head] = input;
	}
	void pop_back()
	{
		assert(count > 0);
		--count;
		if (++tail >= SIZE)
			tail = 0;
	}
	void pop_front()
	{
		assert(count > 0);
		--count;
		if (--head < 0)
			head = SIZE-1;
	}
	TYPE back()
	{
		assert(count > 0);
		return buf[tail];
	}
	TYPE front()
	{
		assert(count > 0);
		return buf[head];
	}
	bool empty()
	{
		return count == 0;
	}
	bool full()
	{
		return count == SIZE;
	}
	int size()
	{
		return count;
	}
	int max_size()
	{
		return SIZE;
	}
};

}

