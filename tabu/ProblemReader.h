#pragma once

#include <string>

#include "DARProblem.h"

class ProblemReader
{
public:
	static DARProblem read(const std::string& filename);
};

