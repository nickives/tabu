#pragma once
//
// Created by Nicholas Ives on 13/04/2023.
//

#ifndef TABU_TABUSEARCH_H
#define TABU_TABUSEARCH_H

#include <utility>
#include <forward_list>
#include <random>

#include "tabutypes.h"
#include "DARProblem.h"

class TabuSearch {
public:
    explicit TabuSearch(const uint16_t max_vehicle_load,
        const double planning_horizon, const double tabu_duration,
        const double penalty_lambda, DARProblem darproblem)
        : tabu_duration_theta_(tabu_duration),
        penalty_lambda_(penalty_lambda), darproblem_(std::move(darproblem)),
        tabu_list_(TabuList()) {};

    SolutionResult search(int max_iterations);

private:
    double tabu_duration_theta_;
    double penalty_lambda_;
    TabuList tabu_list_;
    const DARProblem darproblem_;

    uint32_t number_of_attributes_{ 0 };
    double lambda_x_attributes_{ 0 };

private:
    static inline bool
        is_tabu(const TabuKey& key, const TabuList& tabu_list,
            const uint64_t current_iteration, const double tabu_duration)
    {
        if (tabu_list.contains(key)) {
            const auto value = tabu_list.at(key);
            return (current_iteration - value) <=  tabu_duration;
        }
        return false;
    }

    /**
    * identify critical node - e_i != 0 or l_i != T
    */
    [[nodiscard]] inline static bool
        is_critical(const Node& node, const double planning_horizon)
    {
        return (node.time_window_start != 0)
            || (node.time_window_end < planning_horizon);
    }

    [[nodiscard]] double
        distance(const NodeId& node_idx1, const NodeId& node_idx2) const;

    [[nodiscard]] RouteExcess
        route_cost(const NodeVector& nodes) const;

    [[nodiscard]] SolutionCost
        solution_cost(const Solution& solution, const RelaxationParams& relaxation_params) const;

    static Solution
        apply_swap(const Solution& solution, const RequestId& request_id1, const RequestId& request_id2, const int& route1,
            const int& route2);

    static Solution
        apply_two_opt(const Solution& solution, const int& route_idx, const int& i, const int& j);

    [[nodiscard]] Solution
        apply_single_paired_insertion(const Solution& solution, const size_t& from_route_idx, const size_t& request_idx,
            const size_t& to_route_idx, const RelaxationParams& relaxation_params) const;

    Neighbourhood
        generate_neighborhood(const Solution& solution, TabuList& tabu_list, const uint64_t& current_iteration,
            const RelaxationParams& relaxation_params, const AspirationCriteria& aspiration_criteria, double current_cost);

    [[nodiscard]] std::pair<RouteExcess, std::vector<NodeAttributes>>
        route_evaluation(const NodeVector& nodes) const;

    [[nodiscard]] bool
        is_feasible(const SolutionCost& solution_cost) const;


    /**
     * Compute waiting, arrival, service begin, and departure for each node. Provide start / end node positions to only
     * partially update costs
     *
     * @param node_costs Node costs object to update
     * @param nodes nodes to calculate costs from
     * @param D_0 Departure time from depot
     * @param start_node  Node to start calculating from
     * @param end_node Node to finish calculating from
     */
    void
        compute_node_costs(std::vector<NodeAttributes>& node_costs,
            const NodeVector& nodes, const double& D_0, const uint64_t& start_node, const uint64_t& end_node) const;

    void
        compute_node_costs(std::vector<NodeAttributes>& node_costs,
            const NodeVector& nodes, const double& D_0, const uint64_t& start_node) const;

    void
        compute_node_costs(std::vector<NodeAttributes>& node_costs,
            const NodeVector& nodes, const double& D_0) const;

    void
        compute_node_costs(std::vector<NodeAttributes>& node_costs,
            const NodeVector& nodes) const;

    void
        calculate_journey_times(std::vector<NodeAttributes>& node_costs,
            robin_hood::unordered_flat_map<RequestId, std::pair<double, ptrdiff_t>>& journey_times,
            const ptrdiff_t& start_pos) const;

    [[nodiscard]] std::pair<Solution, SolutionCost>
        find_best_solution(const Neighbourhood& neighbourhood, const RelaxationParams& relaxation_params,
            const SolutionResult& previous_result) const;

    Solution
        construct_initial_solution(int num_vehicles);

    /**
     * Test that each route:
     * 1: Starts and ends at the depot
     * 2: Each request, both vertices belong to the same route and destination vertex is visited after
     *    origin vertex
     * @param solution Solution to test
     * @return True if solution satisfies constraints
     */
    static bool
        satisfies_initial_constraints(const Solution& solution);

    static double
        cost_function_f_s(const SolutionCost& solution_cost, const RelaxationParams& relaxation_params);

    static double
        cost_function_f_s(const RouteExcess& route_excess, const RelaxationParams& relaxation_params);

    Route
        spi_critical_pickup(const Route& route, const Request& request,
            const RelaxationParams& relaxation_params) const;

    Route
        spi_critical_dropoff(const Route& route, const Request& request,
            const RelaxationParams& relaxation_params) const;

    /**
    * Update searc params
    * This updates the provided relaxation paramaters delta, and also the tabu duration
    * and penalty factor.
    * 
    * @param relaxation_params RelaxationParams relaxation parameters to update
    */
    inline void
        update_search_params(RelaxationParams& relaxation_params)
    {
        std::random_device r;
        std::mt19937_64 e1(r());

        // update relaxation delta
        std::uniform_real_distribution<> random_delta(0, 0.5);
        relaxation_params.delta = random_delta(e1);

        // update penalty lambda
        std::uniform_real_distribution<> random_lambda(0, 0.015);
        penalty_lambda_ = random_lambda(e1);
        update_lambda_x_attributes(penalty_lambda_);

        // update tabu duration theta
        const double theta_max = 7.5 * std::log10(number_of_attributes_);
        std::uniform_real_distribution<> random_theta(0, theta_max);
        tabu_duration_theta_ = std::ceil(random_theta(e1));
    }

    inline void
        update_lambda_x_attributes(const double lambda)
    {
        lambda_x_attributes_ = lambda * sqrt(number_of_attributes_);
    }

    [[nodiscard]] std::pair<Solution, SolutionCost>
        intra_route_exchanges(const Solution& solution,
            const RelaxationParams& relaxation_params);

};


#endif //TABU_TABUSEARCH_H
