#pragma once

#ifndef TABU_BLOCKINGMOVEQUEUE_H
#define TABU_BLOCKINGMOVEQUEUE_H

#include <queue>
#include <mutex>
#include "BlockingQueue.h"
#include "tabutypes.h"

class BlockingMoveQueue : public BlockingQueue<MoveVariant>
{
public:
	BlockingMoveQueue(const pair<Solution, RelaxationParams>& sp)
	: solution_pair_(sp) {};

	void
		setSolutionPair(const pair<Solution, RelaxationParams>& solution);
	const pair<const Solution&, const RelaxationParams&>
		getSolutionPair();

private:
	mutex solution_mutex_;
	pair<Solution, RelaxationParams> solution_pair_;
};

#endif // TABU_BLOCKINGMOVEQUEUE_H
