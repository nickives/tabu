#include "BlockingMoveQueue.h"

#include "tabutypes.h"


void
BlockingMoveQueue::setSolutionPair(const pair<Solution, RelaxationParams>& solution)
{
	lock_guard<mutex> lock(solution_mutex_);
	solution_pair_ = solution;
}

const pair<const Solution&, const RelaxationParams&>
BlockingMoveQueue::getSolutionPair()
{
	lock_guard<mutex> lock(solution_mutex_);
	return solution_pair_;
}
