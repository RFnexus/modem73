/*
Exponential moving average

Copyright 2019 Ahmet Inan <inan@aicodix.de>
*/

#include "utils.hh"
#include "unit_circle.hh"

#pragma once

namespace DSP {

template <typename TYPE, typename VALUE>
class EMA
{
	TYPE prev;
	VALUE alpha;
	static VALUE cutoff_alpha(int n, int N)
	{
		VALUE x = UnitCircle<VALUE>::cos(n, N);
		return x-VALUE(1)+sqrt(x*(x-VALUE(4))+VALUE(3));
	}
public:
	constexpr EMA(TYPE prev = TYPE(0), VALUE alpha = VALUE(1)) : prev(prev), alpha(alpha)
	{
	}
	void cutoff(int n, int N)
	{
		alpha = cutoff_alpha(n, N);
	}
	void samples(int s)
	{
		alpha = VALUE(1) / VALUE(s);
	}
	TYPE operator()(TYPE input)
	{
		return prev = lerp(prev, input, alpha);
	}
};

template <typename TYPE, typename VALUE, int ORDER = 1>
class EMACascade
{
	TYPE prev[ORDER];
	VALUE alpha;
	static constexpr VALUE cascade_factor(int n)
	{
		// sqrt(e(1/n*l(2))-1)
		return
			n == 2 ? .6435942529055826247354434374182098089242027424440076511561520093520L :
			n == 3 ? .5098245285339585980866134620415975171004662971729213347866667412748L :
			n == 4 ? .4349794420460822959023617403105260887838801269492020945122292703957L :
			n == 5 ? .3856142567346739149847213965799930737314794827929705449459805476567L :
			n == 6 ? .3499457790992384314823879950215014587750945565870355960474164484675L :
			n == 7 ? .3226290651410878970708971817580301448745259798477682170908870012270L :
			n == 8 ? .3008450309798346315492310951214492771360484456002599192820777432131L :
			n == 9 ? .2829482972069387977799123584440613415063420296903386876512703264517L :
			n == 10 ? .2679056970956257290860485974661891235831324413057223464282157983007L :
			n == 11 ? .2550315459702243937103098780342370981449477556708628034249027758407L :
			n == 12 ? .2438505574307659124524083486415529630644763241448445300317390218311L :
			n == 13 ? .2340215299532217691188557628487431016851279721568792383213019527102L :
			n == 14 ? .2252923404228812762935182605055552181633447787610798690912140601327L :
			n == 15 ? .2174721196397982365782144895603276387766971846333067133969338153904L :
			n == 16 ? .2104133608576552353576644943161359426249259240013118976757629219108L :
			1;
	}
	static VALUE cutoff_alpha(int n, int N)
	{
		VALUE x = UnitCircle<VALUE>::cos(n, N);
		return x-VALUE(1)+sqrt(x*(x-VALUE(4))+VALUE(3));
	}
public:
	EMACascade() : alpha(1)
	{
		for (int i = 0; i < ORDER; ++i)
			prev[i] = TYPE(0);
	}
	void cutoff(int n, int N)
	{
		alpha = cutoff_alpha(nearbyint(n / cascade_factor(ORDER)), N);
	}
	TYPE operator()(TYPE input)
	{
		for (int i = 0; i < ORDER; ++i)
			prev[i] = input = lerp(prev[i], input, alpha);
		return input;
	}
};

}

