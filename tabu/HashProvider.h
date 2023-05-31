#pragma once

#include "robin_hood.h"
#include "tabutypes.h"
#include <tuple>

typedef robin_hood::unordered_map<RequestId, pair<double, ptrdiff_t>> journey_times_map;
class HashProvider
{
private:
	static vector<journey_times_map> journey_times_map_vector;
	journey_times_map& journey_times;
};

