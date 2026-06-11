/*
Frequency-domain zero padding

Copyright 2019 Ahmet Inan <inan@aicodix.de>
*/

#pragma once

#include "fft.hh"

namespace DSP {

template <int OUTPUT, int INPUT, typename CMPLX>
class FDZP
{
	typedef typename CMPLX::value_type VALUE;
	FastFourierTransform<INPUT, CMPLX, -1> fwd;
	FastFourierTransform<OUTPUT, CMPLX, 1> bwd;
	CMPLX tmp[OUTPUT];
	static constexpr VALUE SCALE = VALUE(1) / VALUE(INPUT);
	static_assert(INPUT < OUTPUT, "OUTPUT must be larger than INPUT");
public:
	void operator ()(CMPLX *output, const CMPLX *input)
	{
		fwd(tmp, input);
		if (!(INPUT&1)) {
			tmp[INPUT/2] *= VALUE(0.5);
			tmp[OUTPUT-INPUT/2] = tmp[INPUT/2];
		}
		for (int i = (INPUT+1)/2+1; i < INPUT; ++i)
			tmp[OUTPUT-INPUT+i] = tmp[i];
		for (int i = (INPUT+1)/2+1; i < INPUT; ++i)
			tmp[i] = 0;
		bwd(output, tmp);
		for (int i = 0; i < OUTPUT; ++i)
			output[i] *= SCALE;
	}
};

}

