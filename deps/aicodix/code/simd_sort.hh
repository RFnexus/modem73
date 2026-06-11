/*
Sorting for SIMD types

Copyright 2024 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include "sort.hh"

template <typename TYPE, int WIDTH>
static inline SIMD<typename SIMD<TYPE, WIDTH>::uint_type, WIDTH> vorder(SIMD<TYPE, WIDTH> a)
{
	SIMD<typename SIMD<TYPE, WIDTH>::uint_type, WIDTH> p;
	CODE::insertion_sort(p.v, a.v, WIDTH);
	return p;
}

template <typename TYPE, int WIDTH>
static inline SIMD<TYPE, WIDTH> vsort(SIMD<TYPE, WIDTH> a)
{
	CODE::insertion_sort(a.v, WIDTH);
	return a;
}

