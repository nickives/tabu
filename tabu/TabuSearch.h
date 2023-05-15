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
#include "BlockingMoveQueue.h"
#include "BlockingSolutionQueue.h"

class TabuSearch {
public:
    explicit TabuSearch(const uint16_t max_vehicle_load,
        const double planning_horizon, const double tabu_duration,
        const double penalty_lambda, DARProblem darproblem)
        : tabu_duration_theta_(tabu_duration),
        penalty_lambda_(penalty_lambda), darproblem_(move(darproblem)),
        tabu_list_(TabuList()) {};

    SolutionResult search(int max_iterations);

    const DARProblem darproblem_;

private:
    double tabu_duration_theta_;
    double penalty_lambda_;
    TabuList tabu_list_;
    PenaltyMap penalty_map_;
    uint64_t number_of_attributes_{ 0 };
    double lambda_x_attributes_{ 0 };


public:

    [[nodiscard]] Solution
        apply_single_paired_insertion(const Solution& solution, const size_t& from_route_idx, const size_t& request_idx,
            const size_t& to_route_idx, const RelaxationParams& relaxation_params) const;

    [[nodiscard]] Solution
        apply_swap(const Solution& solution, const size_t& route_idx_1,
            const size_t& request_idx_1, const size_t& route_idx_2,
            const size_t& request_idx_2, const RelaxationParams& relaxation_params) const;

private:
    static inline bool
        is_tabu(const TabuKey& key, const TabuList& tabu_list,
            const uint64_t current_iteration, const double tabu_duration,
            AspirationCriteria aspiration_criteria, double current_cost)
    {
        if (tabu_list.contains(key)) {
            const auto tabu_iteration = tabu_list.at(key);

            // if it's in the aspiration criteria
            if (aspiration_criteria.contains(key)) {
                // if not cheaper, tabu
                const bool is_not_cheaper = current_cost > aspiration_criteria[key];
                // is recent, then tabu
                const bool is_recent = (current_iteration - tabu_iteration) < 2;
                const auto retval = is_not_cheaper && is_recent;
                if (retval) {
                    const int thing = 0;
                }
                return retval;
            }
            // if it's expired
            const auto lifetime = current_iteration - tabu_iteration;
            // so we can generate single and swaps in same neighbourhood
            return (lifetime <= tabu_duration && lifetime != 0);
        }
        // definitely not tabu
        return false;
    }

    [[nodiscard]] double
        distance(const NodeId& node_idx1, const NodeId& node_idx2) const;


    [[nodiscard]] RouteExcess
        route_cost(const NodePtrVector& nodes) const;

    [[nodiscard]] SolutionCost
        solution_cost(const Solution& solution, const RelaxationParams& relaxation_params,
            const double previous_cost) const;

    static Solution
        apply_two_opt(const Solution& solution, const int& route_idx, const int& i, const int& j);

    Neighbourhood
        generate_neighborhood(const Solution& solution, TabuList& tabu_list,
            const uint64_t& current_iteration, const RelaxationParams& relaxation_params,
            const AspirationCriteria& aspiration_criteria, double current_cost);

    [[nodiscard]] pair<Solution, SolutionCost>
        spi_neighbourhood(const Solution& solution, TabuList& tabu_list,
            const uint64_t& current_iteration, const AspirationCriteria& aspiriation_criteria,
            double current_cost, const RelaxationParams& relaxation_params);

    [[nodiscard]] pair<Solution, SolutionCost>
        send_moves_to_workers(const Solution& solution, TabuList& tabu_list,
            const uint64_t& current_iteration, const AspirationCriteria& aspiriation_criteria,
            double current_cost, const RelaxationParams& relaxation_params,
            BlockingMoveQueue& move_q, BlockingSolutionQueue& solution_q);

    uint64_t send_spi_to_workers(const size_t& routes_size, const Solution& solution,
        TabuList& tabu_list, const uint64_t& current_iteration,
        const AspirationCriteria& aspiriation_criteria, double current_cost,
        BlockingMoveQueue& move_q);

    uint64_t send_swap_to_workers(const size_t& routes_size, const Solution& solution,
        TabuList& tabu_list, const uint64_t& current_iteration,
        const AspirationCriteria& aspiriation_criteria, double current_cost,
        BlockingMoveQueue& move_q);

    void
        add_swap_to_neighbourhood(const Solution& solution, TabuList& tabu_list,
            const uint64_t& current_iteration, const AspirationCriteria& aspiriation_criteria,
            double current_cost, const RelaxationParams& relaxation_params,
            Neighbourhood& neighborhood);

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
        compute_node_costs(NodeAttributeVector& node_costs,
            const NodePtrVector& nodes, const double& D_0, const uint64_t& start_node, const uint64_t& end_node) const;

    void
        compute_node_costs(NodeAttributeVector& node_costs,
            const NodePtrVector& nodes, const double& D_0, const uint64_t& start_node) const;

    void
        compute_node_costs(NodeAttributeVector& node_costs,
            const NodePtrVector& nodes, const double& D_0) const;


    void
        calculate_journey_times(NodeAttributeVector& node_costs,
            robin_hood::unordered_flat_map<RequestId, pair<double, ptrdiff_t>>& journey_times,
            const ptrdiff_t& start_pos) const;

    [[nodiscard]] pair<Solution, SolutionCost>
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
        random_device r;
        mt19937_64 e1(r());

        // update relaxation delta
        uniform_real_distribution<> random_delta(0, 0.5);
        relaxation_params.delta = random_delta(e1);

        // update penalty lambda
        uniform_real_distribution<> random_lambda(0, 0.015);
        penalty_lambda_ = random_lambda(e1);
        update_lambda_x_attributes(penalty_lambda_);

        // update tabu duration theta
        const double theta_max = 7.5 * log10(number_of_attributes_);
        uniform_real_distribution<double> random_theta(0, theta_max);
        tabu_duration_theta_ = ceil(random_theta(e1));
    }

    inline void
        update_lambda_x_attributes(const double lambda)
    {
        lambda_x_attributes_ = lambda * sqrt(number_of_attributes_);
    }

    [[nodiscard]] pair<Solution, SolutionCost>
        intra_route_exchanges(const Solution& solution,
            const RelaxationParams& relaxation_params);

    void
        adjust_relaxation_params(RelaxationParams& relaxation_params,
            const SolutionCost& solution_cost);

    /**
    * identify critical node - e_i != 0 or l_i != T
    */
    [[nodiscard]] inline static bool
        is_critical(const Node& node, const double planning_horizon)
    {
        return (node.time_window_start != 0)
            || (node.time_window_end < planning_horizon);
    }

    [[nodiscard]] pair<RouteExcess, NodeAttributeVector>
        route_evaluation(const NodePtrVector& nodes) const;

    Route
        spi_critical_pickup(const Route& route, const size_t& request_idx,
            const RelaxationParams& relaxation_params) const;

    Route
        spi_critical_dropoff(const Route& route, const size_t& request_idx,
            const RelaxationParams& relaxation_params) const;
};


#endif //TABU_TABUSEARCH_H
