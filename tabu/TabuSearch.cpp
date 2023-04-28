//
// Created by Nicholas Ives on 13/04/2023.
//

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <list>
#include <random>
#include <tuple>
#include <cfenv>

#include "robin_hood.h"
#include "TabuSearch.h"
#include "InvalidSolutionException.h"
#include "SolutionPrinter.h"
#include "tabutypes.h"

double TabuSearch::cost_function_f_s(const SolutionCost& solution_cost, const RelaxationParams& relaxation_params) {
    return solution_cost.duration
        + (relaxation_params.load_alpha * solution_cost.total_load_excess)
        + (relaxation_params.duration_beta * solution_cost.total_duration_excess)
        + (relaxation_params.time_window_gamma * solution_cost.total_time_window_excess)
        + (relaxation_params.ride_time_tau * solution_cost.total_ride_time_excess);
}

double TabuSearch::cost_function_f_s(const RouteExcess& route_excess, const RelaxationParams& relaxation_params) {
    return route_excess.duration
        + (relaxation_params.load_alpha * route_excess.load_excess)
        + (relaxation_params.duration_beta * route_excess.duration_excess)
        + (relaxation_params.time_window_gamma * route_excess.time_window_excess)
        + (relaxation_params.ride_time_tau * route_excess.ride_time_excess_L);
}

double TabuSearch::distance(const NodeId& node_idx1, const NodeId& node_idx2) const {
    return darproblem_.distances[node_idx1][node_idx2];
}

RouteExcess TabuSearch::route_cost(const NodeVector& nodes) const {
    RouteExcess route_cost{};
    const auto& nodes_size = nodes.size();
    if (nodes_size < 2) return route_cost;

    uint16_t current_load = 0;

    robin_hood::unordered_flat_map<RequestId, double> pickup_times;
    for (size_t i = 1; i <= nodes_size - 1; ++i) {
        route_cost.duration += distance(nodes[i - 1]->id, nodes[i]->id);
        const auto& current_node = nodes[i];

        // LOAD EXCESS
        current_load += current_node->load;
        if ((current_load - darproblem_.vehicle_capacity) > route_cost.load_excess) {
            route_cost.load_excess = current_load - darproblem_.vehicle_capacity;
        }

        // DURATION
        if (route_cost.duration < current_node->time_window_start) {
            route_cost.duration = current_node->time_window_start;
        }
        else if (route_cost.duration > current_node->time_window_end) {
            // TIME WINDOW EXCESS
            route_cost.time_window_excess += (route_cost.duration - current_node->time_window_end);
        }

        // RIDE TIME EXCESS
        if (current_node->load > 0)
        {
            pickup_times[current_node->request_id] = route_cost.duration;
        } else if (current_node->load < 0)
        {
            const auto& pickup_time = pickup_times[current_node->request_id];
            const auto ride_time = route_cost.duration - pickup_time;
            route_cost.ride_time_excess_L += ride_time > darproblem_.maximum_ride_time
                ? route_cost.duration - pickup_time
                : 0;
        }
        route_cost.duration += current_node->service_duration;
    }
    route_cost.duration_excess = std::max(route_cost.duration - darproblem_.planning_horizon, 0.0);
    return route_cost;
}

SolutionCost TabuSearch::solution_cost(const Solution& solution, const RelaxationParams& relaxation_params) const {
    SolutionCost sc{};
    for (const auto& route : solution.routes) {
        const auto route_excess = route.route_excess;
        sc.duration += route_excess.duration;
        sc.total_ride_time_excess += route_excess.ride_time_excess_L;
        sc.total_time_window_excess += route_excess.time_window_excess;
        sc.total_load_excess += route_excess.load_excess;
        sc.total_duration_excess += route_excess.duration_excess;
        sc.route_costs.push_back(route_excess);
    }
    sc.total_cost = cost_function_f_s(sc, relaxation_params);
    sc.raw_cost = sc.duration
        + sc.total_duration_excess
        + sc.total_load_excess
        + sc.total_ride_time_excess
        + sc.total_time_window_excess
        + sc.total_duration_excess;
    return sc;
}

bool TabuSearch::is_feasible(const SolutionCost& solution_cost) const {
    return (solution_cost.total_time_window_excess == 0)
        && (solution_cost.total_ride_time_excess == 0)
        && (solution_cost.total_duration_excess == 0)
        && (solution_cost.total_load_excess == 0);
}

SolutionResult TabuSearch::search(int max_iterations) {
    RelaxationParams relaxation_params{};
    
    // Best valid solution
    Solution best_valid_solution = construct_initial_solution(darproblem_.number_of_vehicles);
    SolutionCost best_valid_cost{};
    best_valid_cost.total_cost = std::numeric_limits<double>::max();
    
    Solution current_solution = best_valid_solution;
    SolutionCost current_solution_cost{};

    // Aspiration criterio
    AspirationCriteria aspiration_criteria{};

    // DEBUG STUFF - REMOVE AT SOME POINT
    Solution previous_solution = best_valid_solution;
    SolutionCost previous_solution_cost = current_solution_cost;

    std::cout << "Initial Solution: " << std::endl;
    SolutionPrinter::print_solution(current_solution);
    std::cout << "Total Cost: " << best_valid_cost.total_cost << std::endl;
    //std::cout << "Iterations: " << max_iterations << std::endl;

    int better = 0;
    int worse = 0;
    int same = 0;
    SolutionCost average_costs{};

    // start at 1 so we don't go straight into intra-route optimisations
    for (uint64_t iteration = 1; iteration <= max_iterations; ++iteration) {
        if ((current_solution.attribute_added == current_solution.attribute_removed
            || previous_solution.attribute_removed == current_solution.attribute_added)
            && iteration > 2) {
            std::cout << "LOL" << std::endl;
            current_solution = previous_solution;
            continue;
        }
        //std::cout << "Current Iteration: " << iteration + 1 << '\r';
        const auto td = tabu_duration_theta_;
        // clean tabu list
        //robin_hood::unordered::erase_if(tabu_list_, [iteration, td](const auto& pair)
        //    {
        //    auto const& [key, value] = pair;
        //    return value + td < iteration;
        //    });

        Neighbourhood neighbourhood = generate_neighborhood(current_solution, tabu_list_,
            iteration, relaxation_params, aspiration_criteria, current_solution_cost.total_cost);

        const auto current_solution_tuple = find_best_solution(neighbourhood, relaxation_params,
            { current_solution, current_solution_cost.total_cost });


        // DEBUG STUFF - REMOVE AT SOME POINT
        previous_solution_cost = current_solution_cost;
        previous_solution = current_solution;
        // END DEBUG

        current_solution = get<Solution>(current_solution_tuple);
        current_solution_cost = get<SolutionCost>(current_solution_tuple);

        average_costs.total_duration_excess += current_solution_cost.total_duration_excess;
        average_costs.total_load_excess += current_solution_cost.total_load_excess;
        average_costs.total_ride_time_excess += current_solution_cost.total_ride_time_excess;
        average_costs.total_time_window_excess += current_solution_cost.total_time_window_excess;

        const auto relaxation_factor = 1 + relaxation_params.delta;
        if ((current_solution_cost.total_cost < best_valid_cost.total_cost) && is_feasible(current_solution_cost)) {
            
            best_valid_solution = current_solution;
            best_valid_cost = current_solution_cost;

            const auto solution_pair = intra_route_exchanges(current_solution, relaxation_params);
            const auto& intensified_cost = get<SolutionCost>(solution_pair);
            if ((intensified_cost.total_cost < best_valid_cost.total_cost) && is_feasible(intensified_cost)) {
                ++better;
                best_valid_cost = intensified_cost;
                best_valid_solution = get<Solution>(solution_pair);
            }
            else {
                ++worse;
            }

            // save aspiration for known good solutions
            const auto routes_size = current_solution.routes.size();
            for (size_t route_idx = 0; route_idx < routes_size; ++route_idx) {
                const auto& route = current_solution.routes[route_idx];
                for (const auto& request : route.requests) {
                    TabuKey key{ route_idx, request.id };
                    if (aspiration_criteria[key] < current_solution_cost.total_cost) {
                        continue;
                    }
                    aspiration_criteria[key] = current_solution_cost.total_cost;
                }
            }

            relaxation_params.load_alpha /= relaxation_factor;
            relaxation_params.duration_beta /= relaxation_factor;
            relaxation_params.time_window_gamma /= relaxation_factor;
            relaxation_params.ride_time_tau /= relaxation_factor;
            if (std::fetestexcept(FE_OVERFLOW)) {
                relaxation_params = RelaxationParams{};
                std::feclearexcept(FE_OVERFLOW);
            }
        }
        else {

            // update search params every 10 iterations
            // do this here so the recalculated solution isn't impacted by the update to relaxation
            if (iteration % 10 == 0) {
                const auto solution_pair = intra_route_exchanges(current_solution, relaxation_params);
                const auto& rejigged_cost = get<SolutionCost>(solution_pair).total_cost;
                if ( rejigged_cost < current_solution_cost.total_cost) {
                    ++better;
                    current_solution = get<Solution>(solution_pair);
                    current_solution_cost = get<SolutionCost>(solution_pair);
                }
                else if (rejigged_cost > current_solution_cost.total_cost) {
                    ++worse;
                }
                else {
                    ++same;
                }
                update_search_params(relaxation_params);
            }
            relaxation_params.load_alpha *= relaxation_factor;
            relaxation_params.duration_beta *= relaxation_factor;
            relaxation_params.time_window_gamma *= relaxation_factor;
            relaxation_params.ride_time_tau *= relaxation_factor;
            if (std::fetestexcept(FE_OVERFLOW)) {
                relaxation_params = RelaxationParams{};
                std::feclearexcept(FE_OVERFLOW);
            }
        }

    }

    Solution return_solution = best_valid_solution;
    std::cout << std::endl;
    std::cout << "Better: " << better << std::endl;
    std::cout << "Worse: " << worse << std::endl;

    average_costs.total_duration_excess /= max_iterations;
    const double avg_load_excess = static_cast<double>(average_costs.total_load_excess) / static_cast<double>(max_iterations);
    average_costs.total_ride_time_excess /= max_iterations;
    average_costs.total_time_window_excess /= max_iterations;

    std::cout << "Avg Duration Excess   : " << average_costs.total_duration_excess << std::endl;
    std::cout << "Avg Load Excess       :" << avg_load_excess << std::endl;
    std::cout << "Avg Ride Time Excess  :" << average_costs.total_ride_time_excess << std::endl;
    std::cout << "Avg Time Window Excess:" << average_costs.total_time_window_excess << std::endl;


    return { return_solution, best_valid_cost.total_cost };
}

Solution
TabuSearch::apply_swap(const Solution& solution, const RequestId& request_id1, const RequestId& request_id2,
    const int& route1,
    const int& route2) {
    Solution new_solution = solution;
    //    std::swap(new_solution[route1].requests[request_id1], new_solution[route2]);
    //    std::swap(new_solution[route1].nodes[pos1], new_solution[route2].nodes[pos2]);
    return new_solution;
}

Solution TabuSearch::apply_two_opt(const Solution& solution, const int& route_idx, const int& i, const int& j) {
    Solution new_solution = solution;
    //    std::reverse(new_solution[route_idx].begin() + i, new_solution[route_idx].begin() + j + 1);
    return new_solution;
}

Solution TabuSearch::apply_single_paired_insertion(
    const Solution& solution, const size_t& from_route_idx,
    const size_t& request_idx, const size_t& to_route_idx,
    const RelaxationParams& relaxation_params) const {
    Solution new_solution = solution;
    auto& from_route = new_solution.routes[from_route_idx];
    auto& to_route = new_solution.routes[to_route_idx];

    // put request in destination route
    to_route.requests.push_back(from_route.requests[request_idx]);
    auto& request = to_route.requests[to_route.requests.size() - 1];

    // remove request from original route
    if (from_route.requests.size() == 1) {
        from_route.requests.clear();
    }
    else {
        from_route.requests.erase(from_route.requests.begin() + request_idx);
    }

    // remove nodes from_route
    from_route.nodes.erase(std::remove_if(from_route.nodes.begin(), from_route.nodes.end(), [request](const Node* element) {
        return element->request_id == request.id;
        }), from_route.nodes.end());

    // recompute the from route
    const auto from_route_data = route_evaluation(from_route.nodes);
    from_route.route_excess = get<RouteExcess>(from_route_data);
    from_route.node_attributes = get<std::vector<NodeAttributes>>(from_route_data);

    const auto& pickup_node = darproblem_.nodes[request.pickup_node_idx];

    // identify critical node - e_i != 0 or l_i != T
    const bool critical_pickup = is_critical(pickup_node, darproblem_.planning_horizon);

    // if the pickup node isn't critical, the dropoff is critical
    auto new_route = critical_pickup
        ? spi_critical_pickup(to_route, request, relaxation_params)
        : spi_critical_dropoff(to_route, request, relaxation_params);

    new_solution.routes[to_route_idx] = new_route;

    return new_solution;
}

std::pair<RouteExcess, std::vector<NodeAttributes>> TabuSearch::route_evaluation(const NodeVector& nodes) const {
    std::vector<NodeAttributes> node_costs;
    const auto nodes_size = nodes.size();
    node_costs.reserve(nodes_size);
    for (const auto node : nodes) {
        NodeAttributes n{ node->request_id };
        node_costs.push_back(n);
    }
    // 1: D_0 = e_0
    double D_0 = nodes[0]->time_window_start;
    // 2: compute all costs
    compute_node_costs(node_costs, nodes, D_0);

    //3: Compute F_0
    //for first leg, we can delay leaving the depot to avoid waiting at first vertex
    const auto first_leg_cost = distance(nodes[0]->id, nodes[1]->id);
    double forward_time_slack_0 = std::max(
        (double)nodes[0]->time_window_start,
        (nodes[1]->time_window_start - first_leg_cost)
    );

    // 4: Set D_0
    double waiting_times = 0;
    for (const NodeAttributes& n : node_costs) {
        waiting_times += n.waiting_W;
    }
    // departure time from vertex 0
    D_0 = nodes[0]->time_window_start + std::min(forward_time_slack_0, waiting_times);

    // 5: update all costs
    compute_node_costs(node_costs, nodes, D_0);

    // 6: compute L_i for every request
    robin_hood::unordered_map<RequestId, std::pair<double, ptrdiff_t>> journey_times{};
    calculate_journey_times(node_costs, journey_times, 1);

    RouteExcess route_excess{};

    int current_load = 0;
    for (size_t i = 0; i < nodes_size; ++i) {
        const auto& node = nodes[i];
        const auto& nc = node_costs[i];
        const auto& current_duration = nc.arrival_A;
        
        // LOAD EXCESS
        current_load += node->load;
        const auto current_excess = current_load - darproblem_.vehicle_capacity;
        if (current_excess > route_excess.load_excess) {
            route_excess.load_excess = current_load - darproblem_.vehicle_capacity;
        }

        if (current_duration > node->time_window_end) {
            // TIME WINDOW EXCESS
            route_excess.time_window_excess += (current_duration - node->time_window_end);
        }

        // RIDE TIME EXCESS
        if (node->load < 0)
        {
            const auto& time = journey_times[node->request_id].first;
            route_excess.ride_time_excess_L += time > darproblem_.maximum_ride_time
                ? time - darproblem_.maximum_ride_time
                : 0;
        }
    }
    route_excess.duration = node_costs[node_costs.size() -1].arrival_A - node_costs[0].departure_D;
    route_excess.duration_excess = route_excess.duration > darproblem_.planning_horizon
        ? route_excess.duration - darproblem_.planning_horizon
        : 0;

    //// 7: for every vertex v_j corresponding to origin of request j
    //const auto nodes_size = nodes.size();
    //for (uint64_t i = 0; i < nodes_size; ++i) {
    //    const auto& v_j = nodes[i];
    //    // we only want pickup nodes
    //    if (v_j->load < 1) continue;

    //    // a: compute F_j
    //    //      F_j = min_i<=j<=q {sum_i<p<=j (W_p) + (min { l_j - B_j, L - P_j })^+ }
    //    double fwd_time_slack_j = std::numeric_limits<double>::max();
    //    for (uint64_t j = i; j < nodes_size; ++j) {
    //        const auto& node_j = nodes[j];
    //        double waiting_sum = 0;
    //        for (uint64_t p = i; p <= j; ++p) {
    //            waiting_sum += node_costs[p].waiting_W;
    //        }
    //        const double l_j_minus_B_j = nodes[j]->time_window_end - node_costs[j].begin_service_B;

    //        // P_j is journey time for passenger whose destination is node j.
    //        const double P_j = (node_j->load < 0) && (j != nodes_size - 1)
    //            ? get<double>(journey_times[node_j->request_id])
    //            : 0;
    //        const double L_minus_P_j = darproblem_.maximum_ride_time - P_j;

    //        const auto min_l_j_minus_B_j_L_minus_P_j = std::min(l_j_minus_B_j, L_minus_P_j);

    //        waiting_sum += min_l_j_minus_B_j_L_minus_P_j > 0
    //            ? min_l_j_minus_B_j_L_minus_P_j
    //            : 0;

    //        if (waiting_sum < fwd_time_slack_j) {
    //            fwd_time_slack_j = waiting_sum;
    //        }
    //    }

    //    // b: B_j = B_j + min{ F_j, sum_j<p<q(W_p) }
    //    //    D_j = B_j + d_j
    //    double waiting_sum = 0;
    //    for (uint64_t p = i; p < nodes_size - 1; ++p) {
    //        const auto& n = nodes[p];
    //        waiting_sum += node_costs[p].waiting_W;
    //    }
    //    node_costs[i].begin_service_B += std::min(fwd_time_slack_j, waiting_sum);
    //    node_costs[i].departure_D = node_costs[i].begin_service_B + v_j->service_duration;

    //    // c: compute_node_costs for each vertex AFTER j
    //    if (i < nodes_size - 1) {
    //        compute_node_costs(node_costs, nodes, node_costs[0].departure_D, i + 1);
    //        // d: update journey times for each request whose DESTINATION is AFTER vertex j
    //        calculate_journey_times(node_costs, journey_times, i);
    //    }
    //}
    // 8: Compute violations in load, route duration, time window, and ride time constraints.
    /*RouteExcess route_cost{};
    int32_t current_load = 0;

    for (uint64_t i = 0; i < nodes_size; ++i) {
        const Node& current_node = *nodes[i];

        const int32_t load_excess = current_load - darproblem_.vehicle_capacity;
        if (load_excess > route_cost.load_excess) {
            route_cost.load_excess = load_excess;
        }

        if (node_costs[i].arrival_A > current_node.time_window_end) {
            route_cost.time_window_excess += node_costs[i].arrival_A - current_node.time_window_end;
        }
    }
    route_cost.duration = node_costs[node_costs.size() - 1].arrival_A - node_costs[0].departure_D;
    double ride_time_violation = 0;
    for (const auto& t : journey_times) {
        const auto ride_time = get<double>(t.second);
        if (ride_time > darproblem_.maximum_ride_time) {
            route_cost.ride_time_excess_L += ride_time - darproblem_.maximum_ride_time;
        }
    }
    route_cost.duration_excess = route_cost.duration > MAX_SOLUTION_COST_
        ? route_cost.duration - MAX_SOLUTION_COST_
        : 0;*/

    return { route_excess, node_costs };
}

void TabuSearch::calculate_journey_times(std::vector<NodeAttributes>& node_costs,
    robin_hood::unordered_flat_map<RequestId, std::pair<double, ptrdiff_t>>& journey_times,
    const ptrdiff_t& start_pos) const
{
    const ptrdiff_t nodes_size = node_costs.size();
    for (ptrdiff_t i = start_pos; i < nodes_size; ++i) {
        const auto& n = node_costs[i];
        if (journey_times.contains(n.request_id))
        {
            const auto& journey_start_pos = get<ptrdiff_t>(journey_times[n.request_id]);
            if (journey_start_pos < i)
            {
                const auto duration = n.arrival_A - get<double>(journey_times[n.request_id]);
                journey_times[n.request_id].first = duration;
            }
            else {
                // should not happen?
                std::cout << "LOL" << std::endl;
            }
        }
        else {
            journey_times[n.request_id] = std::make_pair(n.departure_D, i);
        }
    }
}

void
TabuSearch::compute_node_costs(std::vector<NodeAttributes>& node_costs,
    const NodeVector& nodes) const {
    compute_node_costs(node_costs, nodes, nodes[0]->time_window_start);
}

void
TabuSearch::compute_node_costs(std::vector<NodeAttributes>& node_costs,
    const NodeVector& nodes, const double& D_0) const {
    compute_node_costs(node_costs, nodes, D_0, 0);
}

void
TabuSearch::compute_node_costs(std::vector<NodeAttributes>& node_costs, const NodeVector& nodes, const double& D_0,
    const uint64_t& start_node) const {
    compute_node_costs(node_costs, nodes, D_0, start_node, nodes.size() - 1);
}

void
TabuSearch::compute_node_costs(std::vector<NodeAttributes>& node_costs, const NodeVector& nodes, const double& D_0,
    const uint64_t& start_node, const uint64_t& end_node) const {
    if (start_node == 0) {
        // first node is a special snowflake
        node_costs[0].waiting_W = 0;
        node_costs[0].arrival_A = 0;
        node_costs[0].begin_service_B = 0;
        node_costs[0].departure_D = D_0;
    }

    double journey_time = 0;
    const uint64_t loop_start = 1 > start_node
        ? 1
        : start_node;

    const uint64_t finish_idx = end_node;

    for (size_t i = loop_start; i <= finish_idx; ++i) {
        const auto& current_node = nodes[i];
        auto& current_cost = node_costs[i];

        journey_time += distance(current_node->id, nodes[i - 1]->id);

        current_cost.arrival_A = journey_time + node_costs[i - 1].departure_D;

        current_cost.begin_service_B = current_cost.arrival_A > current_node->time_window_start
            ? current_cost.arrival_A
            : current_node->time_window_start;

        current_cost.departure_D = current_cost.begin_service_B + current_node->service_duration;

        current_cost.waiting_W = current_cost.arrival_A > current_node->time_window_start
            ? 0
            : current_node->time_window_start - current_cost.arrival_A;
    }
}

/*
 * V = vertex set
 * A = arc set
 * q = load (q_i)
 * d = service duration (d_i)
 * [e_i, l_i] = time window (non-negative)
 * T = end of planning horizon
 * Q_k = load limit for vehicle k
 * L = maximum ride time
 * A_i = arrival time at vertex i v_i
 * B_i = beginning of service at vertex v_i
 *          B_i >= max(e_i, A_i)
 * D_i = departure time from vertex v_i
 *          B_i + d_i
 * W_i = waiting time at vertex v_i
 *          W_i = B_i - A_i
 * L_i = ride time for request i
 *          L_i = B_(i+n) - D_i
 *  F_i = forward time slack of vertex v_i
 *  P_j = ride time of user who's destination is vertex v_j
 */


Neighbourhood TabuSearch::generate_neighborhood(
    const Solution& solution,
    TabuList& tabu_list, const uint64_t& current_iteration,
    const RelaxationParams& relaxation_params,
    const AspirationCriteria& aspiriation_criteria, double current_cost)
{
    Neighbourhood neighborhood;
    neighborhood.reserve(150);

    // GO OVER SECTION 4.7 - IMPLEMENT NEIGHBOURHOOD EVALUATION FROM THERE!
    // SPI
    for (size_t from_route = 0; from_route < solution.routes.size(); ++from_route) {
        for (size_t to_route = 0; to_route < solution.routes.size(); ++to_route) {
            if (from_route == to_route)
            {
                continue;
            }
            for (size_t request_idx = 0; request_idx < solution.routes[from_route].requests.size(); ++request_idx) {
                const auto& request = solution.routes[from_route].requests[request_idx];
                // Don't try to add to same route
                const TabuKey to_key{ to_route, request.id };
                const bool is_aspiration = aspiriation_criteria.contains(to_key);

                // if it's tabu
                if (is_tabu(to_key, tabu_list, current_iteration, tabu_duration_theta_))
                {
                    // if it's not in the aspiration list but in the tabu list, skip it
                    if (!is_aspiration) {
                        continue;
                    }

                    // if it's in the aspiration list, and that cost is higher, skip
                    if (aspiriation_criteria.at(to_key) > current_cost) {
                        continue;
                    }
                }


                auto candidate_solution = apply_single_paired_insertion(solution, from_route, request_idx,
                    to_route, relaxation_params);

                try
                {
                    satisfies_initial_constraints(candidate_solution);
                    // attribute is now tabu FROM route we just removed it from
                    const TabuKey from_key{ from_route, request.id };
                    tabu_list[from_key] = current_iteration;

                    // Attribute added to create this solution. Will be penalised later
                    candidate_solution.penalty_map[to_key] += 1;

                    candidate_solution.attribute_added = to_key;
                    candidate_solution.attribute_removed = from_key;

                    neighborhood.push_back(candidate_solution);
                }
                catch (const InvalidSolutionException& e)
                {
                    std::cout << e.what() << std::endl;
                }

            }
        }
    }

    return neighborhood;
}

Solution TabuSearch::construct_initial_solution(const int num_vehicles) {
    Solution initial_solution{};
    auto& nodes = darproblem_.nodes;
    number_of_attributes_ = darproblem_.number_of_vehicles * (darproblem_.nodes.size() - 1);
    // we use this later for the diversification strategy.
    update_lambda_x_attributes(penalty_lambda_);

    for (int i = 0; i < num_vehicles; ++i) {
        std::vector<Request> req;
        NodeVector no;
        std::vector<NodeAttributes> na;
        req.reserve((nodes.size() / 2) - 1);
        no.reserve(nodes.size());
        na.reserve(nodes.size());
        // put depot at start of every route
        no.push_back(&nodes[0]);
        initial_solution.routes.push_back({ req, no, na, {} });
    }

    // construct initial solution
    std::random_device r;
    std::default_random_engine e1(r());
    std::uniform_int_distribution<> random_index(0, num_vehicles - 1);
    const auto ns = nodes.size();
    for (int i = 1; i < ns / 2; ++i) {
        // pick random route to put request on
        const auto idx = random_index(e1);

        const Node* pickup_node = &nodes[i];
        const Node* dropoff_node = &nodes[i + (nodes.size() / 2) - 1];
        auto& current_route = initial_solution.routes[idx];

        const auto node_size = current_route.nodes.size();
        current_route.nodes.push_back(pickup_node);
        current_route.nodes.push_back(dropoff_node);
        current_route.requests.push_back({ pickup_node->request_id, pickup_node->id, dropoff_node->id });
    }

    // put depot at the end of every route
    for (auto& route : initial_solution.routes) {
        route.nodes.push_back(&nodes[nodes.size() - 1]);
    }

    // calculate node attributes for each route
    for (auto& route : initial_solution.routes) {
        const auto route_data = route_evaluation(route.nodes);
        route.node_attributes = get<std::vector<NodeAttributes>>(route_data);
        route.route_excess = get<RouteExcess>(route_data);
    }

    return initial_solution;
}

std::pair<Solution, SolutionCost>
TabuSearch::find_best_solution(const Neighbourhood& neighbourhood, const RelaxationParams& relaxation_params,
    const SolutionResult& previous_result) const {
    SolutionCost best_cost{};
    best_cost.total_cost = std::numeric_limits<double>::max();
    int best_solution_idx = 0;
    auto best_solution_itr = neighbourhood.cbegin();
    for (Neighbourhood::const_iterator current_solution = best_solution_itr; current_solution != neighbourhood.cend(); current_solution++) {
        SolutionCost cost = solution_cost(*current_solution, relaxation_params);
        // 4.3 diversification strategy
        if (cost.total_cost >= previous_result.cost) {
            const auto& key = current_solution->attribute_added;
            const auto& attribute_count = current_solution->penalty_map.at(key);
            const auto penalty = lambda_x_attributes_ * attribute_count * cost.total_cost;
            cost.total_cost += penalty;
        }
        if (cost.total_cost < best_cost.total_cost) {
            best_cost = cost;
            best_solution_itr = current_solution;
        }
    }

    return { *best_solution_itr, best_cost};
}

bool TabuSearch::satisfies_initial_constraints(const Solution& solution) {
    struct RequestMetaData {
        unsigned int route_idx;
        unsigned int count;
    };
    std::unordered_map<RequestId, RequestMetaData> encountered_requests{};
    for (unsigned int i = 0; i < solution.routes.size(); ++i) {
        const auto& r = solution.routes[i];
        if ((r.nodes[0]->request_id | r.nodes[r.nodes.size() - 1]->request_id) != 0) {
            throw InvalidSolutionException("Route not correctly terminated by depots", solution);
        }
        for (auto n : r.nodes) {
            // pickup node
            if (n->load > 0) {
                if (encountered_requests.contains(n->request_id)) {
                    throw InvalidSolutionException("Solution already contains request pickup", solution);
                }
                encountered_requests[n->request_id] = { i,1 };
            } // dropoff node
            else if (n->load < 0) {
                if (!encountered_requests.contains(n->request_id)) {
                    throw InvalidSolutionException("Solution does not contain request pickup", solution);
                }
                if (encountered_requests[n->request_id].route_idx != i) {
                    throw InvalidSolutionException("Request is present in multiple routes", solution);
                }
                if (encountered_requests[n->request_id].count != 1) {
                    throw InvalidSolutionException("Request count is not 1", solution);
                }
                encountered_requests[n->request_id].count = 2;
            }
        }
    }
    for (const auto& pair : encountered_requests) {
        if (pair.second.count != 2) {
            throw InvalidSolutionException("Request count is not 2", solution);
        }
    }
    return true;
}

Route
TabuSearch::spi_critical_pickup(const Route& route, const Request& request,
    const RelaxationParams& relaxation_params) const
{
    const auto& nodes_in = route.nodes;
    const auto& critical_node = darproblem_.nodes[request.pickup_node_idx];
    const auto& non_critical_node = darproblem_.nodes[request.dropoff_node_idx];

    // create list of to_route nodes
    std::list<const Node*> nodes_list(nodes_in.begin(), nodes_in.end());

    // find best position for critical node
    auto best_position = nodes_list.begin();
    double best_cost = std::numeric_limits<double>::max();

    // we insert *before* current position, so increment one to start
    best_position++;
    std::list<const Node *>::iterator current_position = best_position;

    // going FORWARD through the list
    for (std::list<const Node*>::iterator current_pos = best_position; current_pos != nodes_list.end(); current_pos++) {
        // insert critical node at position
        auto inserted_pos = nodes_list.insert(current_pos, &critical_node);

        // make a vector so  we can calculate costs
        NodeVector new_nodes(nodes_list.begin(), nodes_list.end());
        auto route_costs = route_evaluation(new_nodes);

        // save cost if it's best
        auto cost = cost_function_f_s(get<RouteExcess>(route_costs), relaxation_params);
        if (cost < best_cost) {
            best_cost = cost;
            best_position = current_pos;
        }

        // remove critical node
        nodes_list.erase(inserted_pos);
    }
    nodes_list.insert(best_position, &critical_node);

    // create new solutions where the dropoff node is inserted after every other node, without violating 1 & 2

    // reset cost
    best_cost = std::numeric_limits<double>::max();

    // put current position iterator where we just inserted
    current_position = best_position;
   
    best_position = current_position;

    // save the route costs so we can return them
    std::pair<RouteExcess, std::vector<NodeAttributes>> best_route_costs{};

    // going FORWARD through the list
    for (std::list<const Node*>::iterator current_pos = best_position; current_pos != nodes_list.end(); ++current_pos) {
        // insert node at position
        auto inserted_pos = nodes_list.insert(current_pos, &non_critical_node);

        // make a vector so  we can calculate costs
        NodeVector new_nodes(nodes_list.begin(), nodes_list.end());
        auto route_costs = route_evaluation(new_nodes);

        // save cost if it's best
        auto cost = cost_function_f_s(get<RouteExcess>(route_costs), relaxation_params);
        if (cost < best_cost) {
            best_cost = cost;
            best_route_costs = route_costs;
            best_position = current_pos;
        }

        // remove critical node
        nodes_list.erase(inserted_pos);
        //if (current_position != nodes_list.cend()) ++current_position;
    }
    nodes_list.insert(best_position, &non_critical_node);

    Route new_route{
            route.requests,
            {nodes_list.begin(), nodes_list.end()},
            get<std::vector<NodeAttributes>>(best_route_costs),
            get<RouteExcess>(best_route_costs),
    };

    return new_route;
}

Route
TabuSearch::spi_critical_dropoff(const Route& route, const Request& request,
    const RelaxationParams& relaxation_params) const {

    const auto& nodes_in = route.nodes;
    const auto& critical_node = darproblem_.nodes[request.dropoff_node_idx];
    const auto& non_critical_node = darproblem_.nodes[request.pickup_node_idx];


    // create list of to_route nodes
    std::list<const Node*> nodes_list(nodes_in.begin(), nodes_in.end());

    // find best position for critical node
    auto best_position = nodes_list.begin();
    double best_cost = std::numeric_limits<double>::max();

    // we insert *before* current position, so increment one to start
    ++best_position;

    // going FORWARD through the list
    for (std::list<const Node*>::iterator current_pos = best_position; current_pos != nodes_list.end(); current_pos++) {
        // insert critical node at position
        auto inserted_pos = nodes_list.insert(current_pos, &critical_node);

        // make a vector so  we can calculate costs
        NodeVector new_nodes(nodes_list.begin(), nodes_list.end());
        auto route_costs = route_evaluation(new_nodes);

        // save cost if it's best
        auto cost = cost_function_f_s(get<RouteExcess>(route_costs), relaxation_params);
        if (cost <= best_cost) {
            best_cost = cost;
            best_position = current_pos;
        }

        // remove critical node
        nodes_list.erase(inserted_pos);
    }
    nodes_list.insert(best_position, &critical_node);

    // create new solutions where the dropoff node is inserted after every other node, without violating 1 & 2

    // reset cost
    best_cost = std::numeric_limits<double>::max();

    // save the route costs so we can return them
    std::pair<RouteExcess, std::vector<NodeAttributes>> best_route_costs{};

    // go back one again
    --best_position;

    // going BACKWARD through the list
    for (std::list<const Node*>::iterator current_pos = best_position; current_pos != nodes_list.begin(); current_pos--) {
        // insert node at position
        auto inserted_pos = nodes_list.insert(current_pos, &non_critical_node);

        // make a vector so  we can calculate costs
        NodeVector new_nodes(nodes_list.begin(), nodes_list.end());
        auto route_costs = route_evaluation(new_nodes);

        // save cost if it's best
        auto cost = cost_function_f_s(get<RouteExcess>(route_costs), relaxation_params);
        if (cost < best_cost) {
            best_cost = cost;
            best_route_costs = route_costs;
            best_position = current_pos;
        }

        // remove critical node
        nodes_list.erase(inserted_pos);
    }
    nodes_list.insert(best_position, &non_critical_node);

    Route new_route{
        route.requests,
        {nodes_list.begin(), nodes_list.end()},
        get<std::vector<NodeAttributes>>(best_route_costs),
        get<RouteExcess>(best_route_costs),
    };

    return new_route;
}

std::pair<Solution, SolutionCost>
TabuSearch::intra_route_exchanges(const Solution& solution,
    const RelaxationParams& relaxation_params)
{
    Routes new_routes;
    new_routes.reserve(solution.routes.size());

    int improvement = 0;

    for (const auto& route : solution.routes) {
        Route route_wip = route;
        const auto original_cost = cost_function_f_s(route.route_excess, relaxation_params);
        for (const auto& request : route.requests) {
            // remove nodes from_route
            route_wip.nodes.erase(std::remove_if(
                route_wip.nodes.begin(), route_wip.nodes.end(),
                [request](const Node* element) {
                    return element->request_id == request.id;
                }), route_wip.nodes.end());
            const auto& pickup_node = darproblem_.nodes[request.pickup_node_idx];

            // if the pickup node isn't critical, the dropoff is critical
            auto new_wip_route = is_critical(pickup_node, darproblem_.planning_horizon)
                ? spi_critical_pickup(route_wip, request, relaxation_params)
                : spi_critical_dropoff(route_wip, request, relaxation_params);
            route_wip = new_wip_route;
        }
        const auto new_cost = cost_function_f_s(route_wip.route_excess, relaxation_params);
        improvement += original_cost > new_cost
            ? 1
            : -1;
        if (new_cost < original_cost) {
            new_routes.push_back(route_wip);
        }
        else {
            new_routes.push_back(route);
        }

    }

    Solution new_solution{ new_routes, solution.penalty_map };
    new_solution.attribute_added = solution.attribute_added;
    new_solution.attribute_removed = solution.attribute_removed;
    const auto old_solution_cost = solution_cost(solution, relaxation_params);
    const auto new_solution_cost = solution_cost(new_solution, relaxation_params);

    //const auto fbs_old_cost = find_best_solution({ solution }, relaxation_params, {solution, old_solution_cost.total_cost});

    //const auto fbs_new_cost = find_best_solution({ new_solution }, relaxation_params, {new_solution, new_solution_cost.total_cost});
    return { new_solution, new_solution_cost };
}


//std::pair<Solution, SolutionCost>
//TabuSearch::intra_route_exchanges(const Solution& solution,
//    const RelaxationParams& relaxation_params)
//{
//    Routes new_routes;
//    new_routes.reserve(solution.routes.size());
//
//    for (const auto& route : solution.routes) {
//        std::list route_list(route.nodes.cbegin(), route.nodes.cend());
//        auto current_node = route_list.begin();
//        RouteExcess best_route_excess{};
//        double best_route_cost = std::numeric_limits<double>::max();
//
//        //new_routes.push_back());
//    }
//    Solution new_solution{ new_routes, solution.penalty_map };
//    return { new_solution, solution_cost(new_solution, relaxation_params) };
//}
