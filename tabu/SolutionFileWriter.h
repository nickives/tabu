#pragma once

#ifndef TABU_SOLUTIONFILEWRITER_H
#define TABU_SOLUTIONFILEWRITER_H

#include <string>
#include <fstream>
#include <iostream>
#include <filesystem>
#include "tabutypes.h"

class SolutionFileWriter
{
public:
	SolutionFileWriter(const std::string& output_filename)
		: file(std::ofstream(output_filename)) {}
	void addSolution(const SolutionResult& solution_result);
	void addRunningTime(const std::chrono::milliseconds& time_in_ms);
	void close();
	bool is_open();
private:
	std::ofstream file;
};

#endif // TABU_SOLUTIONFILEWRITER_H