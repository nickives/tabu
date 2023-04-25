#pragma once
//
// Created by Nicholas Ives on 13/04/2023.
//

#ifndef TABU_TABUSEARCH_H
#define TABU_TABUSEARCH_H

#include <utility>
#include <forward_list>

#include "tabutypes.h"
#include "DARProblem.h"

class TabuSearch {
public:
    explicit TabuSearch(const double max_solution_cost, uint16_t max_vehicle_load, double planning_horizon,
        const uint64_t tabu_list_size, const uint64_t tabu_duration, const uint16_t penalty_lambda,
        DARProblem darproblem)
        : MAX_SOLUTION_COST_(max_solution_cost), MAX_VEHICLE_LOAD_(max_vehicle_load),
        PLANNING_HORIZON_(planning_horizon), TABU_DURATION_(tabu_duration), TABU_LIST_SIZE_(tabu_list_size),
        PENALTY_LAMBDA_(penalty_lambda), darproblem_(std::move(darproblem)), tabu_list_(TabuList()) {};

    SolutionResult search(int num_vehicles, int max_iterations);

private:
    const double MAX_SOLUTION_COST_;
    const uint16_t MAX_VEHICLE_LOAD_;
    const double PLANNING_HORIZON_;
    const uint64_t TABU_DURATION_;
    const uint64_t TABU_LIST_SIZE_;
    const uint16_t PENALTY_LAMBDA_;
    TabuList tabu_list_;
    const DARProblem darproblem_;
    double lambda_x_attributes_{ 0 };

private:
    static bool
        is_tabu(const TabuKey& key, const TabuList& tabu_list);

    [[nodiscard]] double
        distance(const NodeId& node_idx1, const NodeId& node_idx2) const;

    [[nodiscard]] RouteExcess
        route_cost(const Route& route) const;

    [[nodiscard]] SolutionCost
        solution_cost(const Solution& solution, const RelaxationParams& relaxation_params) const;

    static Solution
        apply_swap(const Solution& solution, const RequestId& request_id1, const RequestId& request_id2, const int& route1,
            const int& route2);

    static Solution
        apply_two_opt(const Solution& solution, const int& route_idx, const int& i, const int& j);

    [[nodiscard]] std::unique_ptr<Solution>
        apply_single_paired_insertion(const Solution& solution, const unsigned long& from_route_idx, const int& request_idx,
            const unsigned long& to_route_idx, const RelaxationParams& relaxation_params) const;

    Neighbourhood
        generate_neighborhood(const Solution& solution, TabuList& tabu_list, const uint64_t& current_iteration,
            const RelaxationParams& relaxation_params);

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
            const NodeVector& nodes, const double& D_0, const long& start_node, const long& end_node) const;

    void
        compute_node_costs(std::vector<NodeAttributes>& node_costs,
            const NodeVector& nodes, const double& D_0, const long& start_node) const;

    void
        compute_node_costs(std::vector<NodeAttributes>& node_costs,
            const NodeVector& nodes, const double& D_0) const;

    void
        compute_node_costs(std::vector<NodeAttributes>& node_costs,
            const NodeVector& nodes) const;

    void
        calculate_journey_times(std::vector<NodeAttributes>& node_costs, std::map<RequestId, double>& journey_times,
            const int& start_pos) const;

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
        spi_critical_pickup(const NodeVector& nodes, const Route& route_in,
            const Request& request, const RelaxationParams& relaxation_params) const;

    Route
        spi_critical_dropoff(const NodeVector& nodes_in, const Route& route_in,
            const Request& request, const RelaxationParams& relaxation_params) const;
};


#endif //TABU_TABUSEARCH_H
