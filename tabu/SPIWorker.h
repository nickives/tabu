#pragma once

#ifndef TABU_SPIWORKER_H
#define TABU_SPIWORKER_H

#include "tabutypes.h"
#include "TabuSearch.h"
#include "BlockingMoveQueue.h"
#include "BlockingSolutionQueue.h"

class SPIWorker
{
public:
	SPIWorker(const TabuSearch& ts, BlockingMoveQueue& mq, BlockingSolutionQueue& sq)
		: ts_(ts), move_q_(mq), solution_q_(sq)  {};

	void operator()();

private:
	const TabuSearch& ts_;
	BlockingMoveQueue& move_q_;
	BlockingSolutionQueue& solution_q_;
	bool running_ = true;

private:
	void quit() { running_ = false; }
};

#endif // TABU_SPIWORKER_H
