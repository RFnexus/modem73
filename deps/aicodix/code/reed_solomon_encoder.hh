/*
Reed Solomon Encoder

Copyright 2018 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

namespace CODE {

template <int ROOTS, int FCR, typename GF>
class ReedSolomonEncoder
{
public:
	typedef typename GF::value_type value_type;
	typedef typename GF::ValueType ValueType;
	typedef typename GF::IndexType IndexType;
	static const int NR = ROOTS;
	static const int N = GF::N, K = N - NR, NP = NR;
private:
	IndexType generator[NR+1];
public:
	ReedSolomonEncoder()
	{
		// $generator = \prod_{i=0}^{NR}(x-pe^{FCR+i})$
		ValueType tmp[NR+1];
		IndexType root(FCR), pe(1);
		for (int i = 0; i < NR; ++i) {
			tmp[i] = ValueType(1);
			for (int j = i; j > 0; --j)
				tmp[j] = fma(root, tmp[j], tmp[j-1]);
			tmp[0] *= root;
			root *= pe;
		}
		tmp[NR] = ValueType(1);
		if (0) {
			std::cerr << "generator = ";
			for (int i = NR; i > 0; --i) {
				if (!tmp[i])
					continue;
				if (tmp[i] != ValueType(1))
					std::cerr << (int)tmp[i] << "*";
				std::cerr << "x";
				if (i != 1)
					std::cerr << "^" << i;
				std::cerr << " + ";
			}
			std::cerr << (int)tmp[0] << std::endl;
		}
		for (int i = 0; i <= NR; ++i)
			generator[i] = index(tmp[i]);
	}
	void operator()(const ValueType *data, ValueType *parity, int data_len = K)
	{
		assert(0 < data_len && data_len <= K);
		// $code = data * x^{NR} + (data * x^{NR}) \mod{generator}$
		for (int i = 0; i < NR; ++i)
			parity[i] = ValueType(0);
		for (int i = 0; i < data_len; ++i) {
			ValueType feedback = data[i] + parity[0];
			if (feedback) {
				IndexType fb = index(feedback);
				for (int j = 1; j < NR; ++j)
					parity[j-1] = fma(fb, generator[NR-j], parity[j]);
				parity[NP-1] = value(generator[0] * fb);
			} else {
				for (int j = 1; j < NR; ++j)
					parity[j-1] = parity[j];
				parity[NP-1] = ValueType(0);
			}
		}
	}
	void operator()(const value_type *data, value_type *parity, int data_len = K)
	{
		(*this)(reinterpret_cast<const ValueType *>(data), reinterpret_cast<ValueType *>(parity), data_len);
	}
};

}

