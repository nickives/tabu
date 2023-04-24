//
// Created by Nicholas Ives on 13/04/2023.
//

#include <iostream>
#include <vector>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <limits>
#include <list>
#include <random>
#include <tuple>

#include "TabuSearch.h"
#include "InvalidSolutionException.h"
#include "SolutionPrinter.h"
#include "tabutypes.h"

bool TabuSearch::is_tabu(const TabuKey& key, const TabuList& tabu_list) {
    if (tabu_list.contains(key)) {
        return tabu_list.at(key);
    }
    return false;
}

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
        + (relaxation_params.ride_time_tau * route_excess.ride_time_excess);
}

double TabuSearch::distance(const NodeId& node_idx1, const NodeId& node_idx2) const {
    return darproblem_.getDistances()[node_idx1][node_idx2];
}

RouteExcess TabuSearch::route_cost(const Route& route) const {
    const auto& nodes = route.nodes;
    RouteExcess route_cost{ {}, 0, 0, 0, 0 };
    if (route.nodes.size() < 2) return route_cost;

    uint16_t current_load = 0;

    std::map<RequestId, double> journey_times;
    for (size_t i = 1; i <= route.nodes.size() - 1; ++i) {
        route_cost.duration += distance(nodes[i - 1]->id, nodes[i]->id);
        const auto& current_node = nodes[i];

        // LOAD EXCESS
        current_load += current_node->load;
        if ((current_load - MAX_VEHICLE_LOAD_) > route_cost.load_excess) {
            route_cost.load_excess = current_load - MAX_VEHICLE_LOAD_;
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
        if (current_load > 0) {
            journey_times[current_node->request_id] = route_cost.duration;
        }
        else {
            const auto& pickup_time = journey_times[current_node->request_id];
            route_cost.ride_time_excess += route_cost.duration - pickup_time;
        }
        route_cost.duration += current_node->service_duration;
    }
    route_cost.duration_excess = std::max(route_cost.duration - PLANNING_HORIZON_, 0.0);
    return route_cost;
}

SolutionCost TabuSearch::solution_cost(const Solution& solution, const RelaxationParams& relaxation_params) const {
    SolutionCost solution_cost{};
    for (const auto& route : solution.routes) {
        const auto route_excess = route_cost(route);
        solution_cost.duration += route_excess.duration;
        solution_cost.total_ride_time_excess += route_excess.ride_time_excess;
        solution_cost.total_time_window_excess += route_excess.time_window_excess;
        solution_cost.total_load_excess += route_excess.load_excess;
        solution_cost.total_duration_excess += route_excess.duration_excess;
        solution_cost.route_costs.push_back(route_excess);
    }
    solution_cost.total_cost = cost_function_f_s(solution_cost, relaxation_params);
    return solution_cost;
}

bool TabuSearch::is_feasible(const SolutionCost& solution_cost) const {
    return (solution_cost.total_cost < MAX_SOLUTION_COST_)
        && (solution_cost.total_time_window_excess == 0)
        && (solution_cost.total_ride_time_excess == 0)
        && (solution_cost.total_duration_excess <= 0)
        && (solution_cost.total_load_excess <= 0);
}

SolutionResult TabuSearch::search(int num_vehicles, int max_iterations) {
    RelaxationParams relaxation_params{};
    Solution best_valid_solution = construct_initial_solution(num_vehicles);
    SolutionCost best_valid_cost{};
    best_valid_cost.total_cost = std::numeric_limits<double>::max();
    Solution current_solution = best_valid_solution;
    SolutionCost current_solution_cost{};

    const auto relaxation_factor = 1 + relaxation_params.delta;

    std::cout << "Initial Solution: " << std::endl;
    SolutionPrinter::print_solution(current_solution);
    std::cout << "Total Cost: " << best_valid_cost.total_cost << std::endl;
    std::cout << "Iterations: " << max_iterations << std::endl;
    //Solution previous_solution = current_solution;

    for (uint64_t iteration = 0; iteration < max_iterations; ++iteration) {
        std::cout << "Current Iteration: " << iteration << '\r';
        //previous_solution = current_solution;
        const auto td = TABU_DURATION_;
        // clean tabu list
        std::erase_if(tabu_list_, [iteration, td](const auto& pair) {
            auto const& [key, value] = pair;
            return value + td < iteration;
            });

        //std::cout << "Iteration: " << iteration << std::endl;
        //SolutionPrinter::print_solution(current_solution);
        //std::cout << "Total Cost: " << current_solution_cost.total_cost << std::endl;
        //std::cout << "Best Cost: " << best_valid_cost.total_cost << std::endl;

        Neighbourhood neighbourhood = generate_neighborhood(current_solution, tabu_list_,
            iteration, relaxation_params);


        auto current_solution_tuple = find_best_solution(neighbourhood, relaxation_params,
            { current_solution, current_solution_cost.total_cost });


        current_solution = get<Solution>(current_solution_tuple);
        current_solution_cost = get<SolutionCost>(current_solution_tuple);

        if ((current_solution_cost.total_cost < best_valid_cost.total_cost) && is_feasible(current_solution_cost)) {
            best_valid_solution = current_solution;
            best_valid_cost = current_solution_cost;
            relaxation_params.load_alpha /= relaxation_factor;
            relaxation_params.duration_beta /= relaxation_factor;
            relaxation_params.time_window_gamma /= relaxation_factor;
            relaxation_params.ride_time_tau /= relaxation_factor;
        }
        else {
            relaxation_params.load_alpha *= relaxation_factor;
            relaxation_params.duration_beta *= relaxation_factor;
            relaxation_params.time_window_gamma *= relaxation_factor;
            relaxation_params.ride_time_tau *= relaxation_factor;
        }

    }

    Solution return_solution = best_valid_solution;

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

std::unique_ptr<Solution> TabuSearch::apply_single_paired_insertion(const Solution& solution, const unsigned long& from_route_idx,
    const int& request_idx, const unsigned long& to_route_idx,
    const RelaxationParams& relaxation_params) const {
    auto new_solution = std::make_unique<Solution>(solution);
    auto& from_route = new_solution->routes[from_route_idx];
    auto& to_route = new_solution->routes[to_route_idx];

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

    const auto pickup_node = darproblem_.getNodeVector()[request.pickup_node_idx];

    // identify critical node - e_i != 0 or l_i != T
    const bool critical_pickup = (pickup_node->time_window_start != 0) || (pickup_node->time_window_end != PLANNING_HORIZON_);

    // if the pickup node isn't critical, the dropoff is critical
    auto new_route = critical_pickup
        ? spi_critical_pickup(to_route.nodes, to_route, request, relaxation_params)
        : spi_critical_dropoff(to_route.nodes, to_route, request, relaxation_params);

    new_route.requests = new_solution->routes[to_route_idx].requests;

    new_solution->routes[to_route_idx] = new_route;

    return new_solution;
}

std::pair<RouteExcess, std::vector<NodeAttributes>> TabuSearch::route_evaluation(const NodeVector& nodes) const {
    std::vector<NodeAttributes> node_costs;
    for (const auto node : nodes) {
        NodeAttributes n{ node->request_id };
        node_costs.push_back(n);
    }
    // 1: D_0 = e_0
    // 2: compute all costs
    compute_node_costs(node_costs, nodes);

    // 3: Compute F_0
    // for first leg, we can delay leaving the depot to avoid waiting at first vertex
    const auto first_leg_cost = distance(nodes[0]->id, nodes[1]->id);
    double forward_time_slack_0 = std::max((double)nodes[0]->time_window_start, (nodes[1]->time_window_start - first_leg_cost));

    // 4: Set D_0
    double waiting_times = 0;
    for (const NodeAttributes& n : node_costs) {
        waiting_times += n.waiting_W;
    }
    // departure time from vertex 0
    const auto D_0 = nodes[0]->time_window_start + std::min(forward_time_slack_0, waiting_times);
    // 5: update all costs
    compute_node_costs(node_costs, nodes, D_0);

    // 6: compute L_i for every request
    std::map<RequestId, double> journey_times{};
    calculate_journey_times(node_costs, journey_times, 1);

    // 7: for every vertex v_j corresponding to origin of request j
    for (int i = 0; i < nodes.size(); ++i) {
        const auto& v_j = nodes[i];
        // we only want pickup nodes
        if (v_j->load < 1) continue;

        // a: compute F_j
        //      F_j = min_i<=j<=q {sum_i<p<=j (W_p) + (min { l_j - B_j, L - P_j })^+ }
        double fwd_time_slack_j = std::numeric_limits<double>::max();
        for (int j = i; j < nodes.size(); ++j) {
            const auto& node_j = nodes[j];
            double waiting_sum = 0;
            for (int p = i; p <= j; ++p) {
                waiting_sum += node_costs[p].waiting_W;
            }
            const double l_j_minus_B_j = nodes[j]->time_window_end - node_costs[j].begin_service_B;

            // P_j is journey time for passenger whose destination is node j.
            const double P_j = (node_j->load < 0) && (j != nodes.size() - 1)
                ? journey_times[node_j->request_id]
                : 0;
            const double L_minus_P_j = MAX_RIDE_TIME - P_j;

            const auto min_l_j_minus_B_j_L_minus_P_j = std::min(l_j_minus_B_j, L_minus_P_j);

            waiting_sum += min_l_j_minus_B_j_L_minus_P_j > 0
                ? min_l_j_minus_B_j_L_minus_P_j
                : 0;

            if (waiting_sum < fwd_time_slack_j) {
                fwd_time_slack_j = waiting_sum;
            }
        }

        // b: B_j = B_j + min{ F_j, sum_j<p<q(W_p) }
        //    D_j = B_j + d_j
        double waiting_sum = 0;
        for (int p = i; p < nodes.size() - 1; ++p) {
            const auto& n = nodes[p];
            waiting_sum += node_costs[p].waiting_W;
        }
        node_costs[i].begin_service_B += std::min(fwd_time_slack_j, waiting_sum);
        node_costs[i].departure_D = node_costs[i].begin_service_B + v_j->service_duration;

        // c: compute_node_costs for each vertex AFTER j
        if (i < nodes.size() - 1) {
            compute_node_costs(node_costs, nodes, node_costs[0].departure_D, i + 1);
            // d: update journey times for each request whose DESTINATION is AFTER vertex j
            calculate_journey_times(node_costs, journey_times, i);
        }
    }
    // 8: Compute violations in load, route duration, time window, and ride time constraints.
    RouteExcess route_cost{
        0,0,0,0 }
    ;
    int32_t current_load = 0;

    for (int i = 0; i < nodes.size(); ++i) {
        const Node& current_node = *nodes[i];
        current_load += nodes[i]->load;

        const int32_t load_excess = current_load - MAX_VEHICLE_LOAD_;
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
        if (t.second > MAX_RIDE_TIME) route_cost.ride_time_excess += t.second - MAX_RIDE_TIME;
    }
    route_cost.duration_excess = route_cost.duration > MAX_SOLUTION_COST_
        ? route_cost.duration - MAX_SOLUTION_COST_
        : 0;

    return { route_cost, node_costs };
}

void TabuSearch::calculate_journey_times(std::vector<NodeAttributes>& node_costs,
    std::map<RequestId, double>& journey_times, const int& start_pos) const {
    for (int i = start_pos; i < node_costs.size(); ++i) {
        const auto& n = node_costs[i];
        if (journey_times.contains(n.id)) {
            const auto duration = n.arrival_A - journey_times[n.id];
            journey_times[n.id] = duration;
        }
        else {
            journey_times[n.id] = n.departure_D;
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
    const long& start_node) const {
    compute_node_costs(node_costs, nodes, D_0, start_node, nodes.size() - 1);
}

void
TabuSearch::compute_node_costs(std::vector<NodeAttributes>& node_costs, const NodeVector& nodes, const double& D_0,
    const long& start_node, const long& end_node) const {
    if (start_node == 0) {
        // first node is a special snowflake
        node_costs[0].waiting_W = 0;
        node_costs[0].arrival_A = 0;
        node_costs[0].begin_service_B = 0;
        node_costs[0].departure_D = D_0;
    }

    double journey_time = 0;
    const long loop_start = 1 > start_node
        ? 1
        : start_node;

    const auto finish_idx = end_node;

    for (long i = loop_start; i <= finish_idx; ++i) {
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


Neighbourhood TabuSearch::generate_neighborhood(const Solution& solution, TabuList& tabu_list, const uint64_t& current_iteration,
    const RelaxationParams& relaxation_params) {
    Neighbourhood neighborhood;
    neighborhood.reserve(5000);

    // GO OVER SECTION 4.7 - IMPLEMENT NEIGHBOURHOOD EVALUATION FROM THERE!
    // SPI
    for (int from_route = 0; from_route < solution.routes.size(); ++from_route) {
        for (int request_idx = 0; request_idx < solution.routes[from_route].requests.size(); ++request_idx) {
            const auto& request = solution.routes[from_route].requests[request_idx];
            for (int to_route = 0; to_route < solution.routes.size(); ++to_route) {
                if ((from_route == to_route) || is_tabu({ to_route, request.id }, tabu_list)) {
                    continue; // Skip the same route or if it's tabu
                }

                auto candidate_solution = apply_single_paired_insertion(solution, from_route, request_idx,
                    to_route, relaxation_params);

                try {
                    satisfies_initial_constraints(*candidate_solution);
                    TabuKey key = { from_route, request.id };
                    if (tabu_list.size() < TABU_LIST_SIZE_) tabu_list[key] = current_iteration;
                    candidate_solution->penalty_list[key] += 1;
                    neighborhood.push_back(std::move(candidate_solution));
                }
                catch (const InvalidSolutionException& e) {
                    std::cout << e.what() << std::endl;
                }


            }
        }
    }

    //    // Swap operator
    //    for (size_t route1 = 0; route1 < solution.size(); ++route1) {
    //        for (size_t pos1 = 0; pos1 < solution[route1].size(); ++pos1) {
    //            for (size_t route2 = 0; route2 < solution.size(); ++route2) {
    //                for (size_t pos2 = 0; pos2 < solution[route2].size(); ++pos2) {
    //                    if (route1 == route2 && pos1 == pos2) {
    //                        continue; // Skip the same position
    //                    }
    //
    //                    Solution candidate_solution = apply_swap(solution, route1,
    //                                                             route2);
    //                    if (is_feasible(candidate_solution)) {
    //                        neighborhood.push_back(candidate_solution);
    //                    }
    //                }
    //            }
    //        }
    //    }
    //
    //    // Two-opt operator
    //    for (size_t route_idx = 0; route_idx < solution.size(); ++route_idx) {
    //        for (size_t i = 0; i < solution[route_idx].size() - 1; ++i) {
    //            for (size_t j = i + 1; j < solution[route_idx].size(); ++j) {
    //                Solution candidate_solution = apply_two_opt(solution, route_idx, i, j);
    //                if (is_feasible(candidate_solution)) {
    //                    neighborhood.push_back(candidate_solution);
    //                }
    //            }
    //        }
    //    }
    return neighborhood;
}

Solution TabuSearch::construct_initial_solution(const int num_vehicles) {
    Solution initial_solution{};
    auto nodes = darproblem_.getNodeVector();
    // we use this later for the diversification strategy.
    lambda_x_attributes_ = PENALTY_LAMBDA_ * sqrt((num_vehicles * (nodes.size() - 1)));

    for (int i = 0; i < num_vehicles; ++i) {
        std::vector<Request> req;
        NodeVector no;
        std::vector<NodeAttributes> na;
        // put depot at start of every route
        no.push_back(nodes[0]);
        initial_solution.routes.push_back({ req, no, na, {} });
    }

    // construct initial solution
    std::random_device r;
    std::default_random_engine e1(r());
    std::uniform_int_distribution<> random_index(0, num_vehicles - 1);
    for (int i = 1; i < nodes.size() / 2; ++i) {
        // pick random route to put request on
        const auto idx = random_index(e1);

        const auto pickup_node = nodes[i];
        const auto dropoff_node = nodes[i + (nodes.size() / 2) - 1];
        auto& current_route = initial_solution.routes[idx];

        const auto node_size = current_route.nodes.size();
        current_route.nodes.push_back(pickup_node);
        current_route.nodes.push_back(dropoff_node);
        current_route.requests.push_back({ pickup_node->request_id, pickup_node->id, dropoff_node->id });
    }

    // put depot at the end of every route
    for (auto& route : initial_solution.routes) {
        route.nodes.push_back(nodes[nodes.size() - 1]);
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
    best_cost.total_cost = std::numeric_limits<double>::max();
    for (int i = 0; i < neighbourhood.size(); ++i) {
        const auto& solution = neighbourhood[i];
        SolutionCost cost = solution_cost(*solution, relaxation_params);
        // 4.3 diversification strategy
        if (cost.total_cost >= previous_result.cost) {
            for (const auto& penalty : solution->penalty_list) {
                cost.total_cost += lambda_x_attributes_ * penalty.second * cost.total_cost;
            }
        }
        if (cost.total_cost < best_cost.total_cost) {
            best_cost = cost;
            best_solution_idx = i;
        }
    }

    Solution retval = *neighbourhood[best_solution_idx];

    return {retval, best_cost};
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
TabuSearch::spi_critical_pickup(const NodeVector& nodes_in, const Route& route_in, const Request& request,
    const RelaxationParams& relaxation_params) const {


    const auto& critical_node = darproblem_.getNodeVector()[request.pickup_node_idx];
    const auto& non_critical_node = darproblem_.getNodeVector()[request.dropoff_node_idx];


    // create list of to_route nodes
    std::list<const Node*> nodes_list(nodes_in.begin(), nodes_in.end());

    // find best position for critical node
    auto best_position = nodes_list.begin();
    double best_cost = std::numeric_limits<double>::max();

    // we insert *before* current position, so increment one to start
    best_position++;
    auto current_position = best_position;

    int critical_idx = 0;
    // going FORWARD through the list
    for (int i = 0; i < nodes_list.size() - 1; ++i) {
        // insert critical node at position
        auto inserted_pos = nodes_list.insert(current_position, critical_node);

        // make a vector so  we can calculate costs
        NodeVector new_nodes(nodes_list.begin(), nodes_list.end());
        auto route_costs = route_evaluation(new_nodes);

        // save cost if it's best
        auto cost = cost_function_f_s(get<RouteExcess>(route_costs), relaxation_params);
        if (cost < best_cost) {
            best_cost = cost;
            best_position = current_position;
            critical_idx = i;
        }

        // remove critical node
        nodes_list.erase(inserted_pos);
        current_position++;
    }
    nodes_list.insert(best_position, critical_node);

    // create new solutions where the dropoff node is inserted after every other node, without violating 1 & 2

    // reset cost
    best_cost = std::numeric_limits<double>::max();

    // put current position iterator where we just inserted
    current_position = best_position;
    ++current_position;
    best_position = current_position;

    // save the route costs so we can return them
    std::pair<RouteExcess, std::vector<NodeAttributes>> best_route_costs{};

    // going FORWARD through the list
    for (int i = critical_idx - 1; i < nodes_list.size() - 1; ++i) {
        // insert node at position
        auto inserted_pos = nodes_list.insert(current_position, non_critical_node);

        // make a vector so  we can calculate costs
        NodeVector new_nodes(nodes_list.begin(), nodes_list.end());
        auto route_costs = route_evaluation(new_nodes);

        // save cost if it's best
        auto cost = cost_function_f_s(get<RouteExcess>(route_costs), relaxation_params);
        if (cost < best_cost) {
            best_cost = cost;
            best_route_costs = route_costs;
            best_position = current_position;
        }

        // remove critical node
        nodes_list.erase(inserted_pos);
        ++current_position;
    }
    nodes_list.insert(best_position, non_critical_node);

    Route new_route{
            {},
            {nodes_list.begin(), nodes_list.end()},
            get<std::vector<NodeAttributes>>(best_route_costs),
            get<RouteExcess>(best_route_costs),
    };
    new_route.requests = route_in.requests;

    return new_route;
}

Route
TabuSearch::spi_critical_dropoff(const NodeVector& nodes_in, const Route& route_in, const Request& request,
    const RelaxationParams& relaxation_params) const {

    const auto& critical_node = darproblem_.getNodeVector()[request.dropoff_node_idx];
    const auto& non_critical_node = darproblem_.getNodeVector()[request.pickup_node_idx];


    // create list of to_route nodes
    std::list<const Node*> nodes_list(nodes_in.begin(), nodes_in.end());

    // find best position for critical node
    auto best_position = nodes_list.begin();
    double best_cost = std::numeric_limits<double>::max();

    // we insert *before* current position, so increment one to start
    ++best_position;

    // going FORWARD through the list
    for (auto i = best_position; i != nodes_list.end(); i++) {
        // insert critical node at position
        auto inserted_pos = nodes_list.insert(i, critical_node);

        // make a vector so  we can calculate costs
        NodeVector new_nodes(nodes_list.begin(), nodes_list.end());
        auto route_costs = route_evaluation(new_nodes);

        // save cost if it's best
        auto cost = cost_function_f_s(get<RouteExcess>(route_costs), relaxation_params);
        if (cost < best_cost) {
            best_cost = cost;
            best_position = i;
        }

        // remove critical node
        nodes_list.erase(inserted_pos);
    }
    nodes_list.insert(best_position, critical_node);

    // create new solutions where the dropoff node is inserted after every other node, without violating 1 & 2

    // reset cost
    best_cost = std::numeric_limits<double>::max();

    // save the route costs so we can return them
    std::pair<RouteExcess, std::vector<NodeAttributes>> best_route_costs{};

    // go back one again
    --best_position;

    // going BACKWARD through the list
    for (auto i = best_position; i != nodes_list.begin(); i--) {
        // insert node at position
        auto inserted_pos = nodes_list.insert(i, non_critical_node);

        // make a vector so  we can calculate costs
        NodeVector new_nodes(nodes_list.begin(), nodes_list.end());
        auto route_costs = route_evaluation(new_nodes);

        // save cost if it's best
        auto cost = cost_function_f_s(get<RouteExcess>(route_costs), relaxation_params);
        if (cost < best_cost) {
            best_cost = cost;
            best_route_costs = route_costs;
            best_position = i;
        }

        // remove critical node
        nodes_list.erase(inserted_pos);
    }
    nodes_list.insert(best_position, non_critical_node);

    Route new_route{
        {},
        {nodes_list.begin(), nodes_list.end()},
        get<std::vector<NodeAttributes>>(best_route_costs),
        get<RouteExcess>(best_route_costs),
    };
    new_route.requests = route_in.requests;

    return new_route;
}


