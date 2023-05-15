#pragma once

#include <string>

#include "DARProblem.h"

class ProblemReader
{
public:
	static DARProblem readTabu(const filesystem::path& filename);
	static DARProblem readDarp(const filesystem::path& filename);
};

