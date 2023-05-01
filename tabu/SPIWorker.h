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

	[[nodiscard]] Solution
		apply_single_paired_insertion(const Solution& solution, const size_t& from_route_idx, const size_t& request_idx,
			const size_t& to_route_idx, const RelaxationParams& relaxation_params) const;

	void operator()();

private:
	const TabuSearch& ts_;
	BlockingMoveQueue& move_q_;
	BlockingSolutionQueue& solution_q_;

};

#endif // TABU_SPIWORKER_H
