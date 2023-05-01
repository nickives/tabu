#pragma once

#ifndef TABU_BLOCKINGQUEUE_H
#define TABU_BLOCKINGQUEUE_H

#include <queue>
#include <mutex>

template<typename T>
class BlockingQueue
{
public:
	void
		put(const T& t)
	{
		std::unique_lock<std::mutex> ul(mutex_);
		bool was_empty = queue_.empty();
		queue_.push(std::move(t));
		ul.unlock(); // otherwise waiting threads will immediately lock again
		if (was_empty) cv.notify_all();
	}

	T
		get()
	{
		std::unique_lock<std::mutex> ul(mutex_);
		while (queue_.empty()) {
			cv.wait(ul);
		}
		T t = std::move(queue_.front());
		queue_.pop();
		return t;
	}
private:
	std::queue<T, std::deque<T>> queue_;
	std::mutex mutex_;
	std::condition_variable cv;
};

#endif // TABU_BLOCKINGQUEUE_H
