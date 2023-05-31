#include "SPIWorker.h"

#include <tuple>
#include <variant>

#include "tabutypes.h"

template<class... Ts>
struct overloaded : Ts... { using Ts::operator()...; };

void SPIWorker::operator()() {
    while (running_) {
        const auto move_variant = move_q_.get();

        std::visit(overloaded{
            [=](const SPIMove& move) {
                const auto& solution_pair = move_q_.getSolutionPair();
                const auto& previous_solution = get<const Solution&>(solution_pair);
                const auto& relaxation_params = get<const RelaxationParams&>(solution_pair);
                auto start_time = std::chrono::steady_clock::now();
                auto candidate_solution = ts_.apply_single_paired_insertion(previous_solution,
                    move.from_route_idx, move.request_idx, move.to_route_idx, relaxation_params);
                auto end_time = std::chrono::steady_clock::now();
                candidate_solution.move_time = end_time - start_time;
                candidate_solution.attribute_added = move.attribute_added;
                candidate_solution.attribute_removed = move.attribute_removed;
                solution_q_.put(candidate_solution);
            },
            [=](const SwapMove& move) {
                const auto& solution_pair = move_q_.getSolutionPair();
                const auto& previous_solution = get<const Solution&>(solution_pair);
                const auto& relaxation_params = get<const RelaxationParams&>(solution_pair);
                auto candidate_solution = ts_.apply_swap(previous_solution,
                    move.route_1_idx, move.request_1_idx, move.route_2_idx, move.request_2_idx,
                    relaxation_params);
                candidate_solution.attribute_added = move.attribute_1;
                candidate_solution.attribute_removed = move.attribute_2;
                candidate_solution.swap = true;
                solution_q_.put(candidate_solution); },
            [=](const QuitMove& move) {
                quit();
            },
            }, move_variant);
    }
}
