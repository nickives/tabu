#pragma once

#ifndef TABU_BLOCKINGSOLUTIONQUEUE_H
#define TABU_BLOCKINGSOLUTIONQUEUE_H

#include <queue>
#include <mutex>
#include "tabutypes.h"
#include "BlockingQueue.h"

class BlockingSolutionQueue : public BlockingQueue<Solution>
{

};

#endif // TABU_BLOCKINGSOLUTIONQUEUE_H