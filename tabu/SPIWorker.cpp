#include "SPIWorker.h"

#include <tuple>

#include "tabutypes.h"

Solution SPIWorker::apply_single_paired_insertion(
    const Solution& solution, const size_t& from_route_idx,
    const size_t& request_idx, const size_t& to_route_idx,
    const RelaxationParams& relaxation_params) const {
    Solution new_solution = solution;
    auto& from_route = new_solution.routes[from_route_idx];
    auto& to_route = new_solution.routes[to_route_idx];

    // put request in destination route
    to_route.requests.push_back(from_route.requests[request_idx]);
    auto& request = to_route.requests[to_route.requests.size() - 1];
    const auto to_request_idx = to_route.requests.size() - 1;

    // remove request from_route
    if (from_route.requests.size() == 1) {
        from_route.requests.clear();
    }
    else {
        from_route.requests.erase(from_route.requests.begin() + request_idx);
    }

    // remove nodes from_route
    from_route.nodes.erase(remove_if(from_route.nodes.begin(), from_route.nodes.end(), [request](const Node* element) {
        return element->request_id == request->id;
        }), from_route.nodes.end());

    // recompute the from route
    const auto from_route_data = ts_.route_evaluation(from_route.nodes);
    from_route.route_excess = get<RouteExcess>(from_route_data);
    from_route.node_attributes = get<vector<NodeAttributes>>(from_route_data);

    const Node* pickup_node = request->pickup_node;

    // identify critical node - e_i != 0 or l_i != T
    const bool critical_pickup = ts_.is_critical(*pickup_node, ts_.darproblem_.planning_horizon);

    // if the pickup node isn't critical, the dropoff is critical
    new_solution.routes[to_route_idx] = critical_pickup
        ? ts_.spi_critical_pickup(to_route, to_request_idx, relaxation_params)
        : ts_.spi_critical_dropoff(to_route, to_request_idx, relaxation_params);

    return new_solution;
}

void SPIWorker::operator()() {
    while (true) {
        const auto& move_variant = move_q_.get();
        if (holds_alternative<QuitMove>(move_variant)) break;
        const auto& move = get<SPIMove>(move_variant);
        const auto& solution_pair = move_q_.getSolutionPair();
        const auto& previous_solution = get<const Solution&>(solution_pair);
        const auto& relaxation_params = get<const RelaxationParams&>(solution_pair);
        auto candidate_solution = apply_single_paired_insertion(previous_solution,
            move.from_route_idx, move.request_idx, move.to_route_idx, relaxation_params);
        candidate_solution.attribute_added = move.attribute_added;
        candidate_solution.attribute_removed = move.attribute_removed;
        solution_q_.put(candidate_solution);
    }
}
