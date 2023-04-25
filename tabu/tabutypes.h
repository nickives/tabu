#pragma once

//
// Created by Nicholas Ives on 13/04/2023.
//

#ifndef TABU_TABUTYPES_H
#define TABU_TABUTYPES_H

#include <boost/unordered/unordered_flat_map.hpp>
#include <list>
#include <vector>
#include <memory>

enum MoveType { SPI, SWAP, MOVE };
typedef uint64_t RequestId;
typedef uint64_t RouteIndex;
typedef std::vector<std::vector<double>> Distances;
typedef std::tuple<int, RequestId> TabuKey;
typedef TabuKey PenaltyKey;
typedef unsigned int TabuIteration;
typedef unsigned int TimesAdded;
typedef boost::unordered_flat_map<PenaltyKey, TimesAdded> PenaltyList;
typedef boost::unordered_flat_map<TabuKey, TabuIteration> TabuList;
typedef uint64_t NodeId;

struct Node {
    NodeId id;
    uint64_t request_id;
    int16_t load;
    uint32_t service_duration;
    uint32_t time_window_start;
    uint32_t time_window_end;
    double x;
    double y;
};
typedef std::vector<const Node*> NodeVector;

struct Request {
    RequestId id;
    NodeId pickup_node_idx;
    NodeId dropoff_node_idx;

    bool operator==(const Request& rhs) const {
        return id == rhs.id;
    }

    bool operator!=(const Request& rhs) const {
        return !(rhs == *this);
    }
};

struct RouteExcess {
    int load_excess = 0;
    double ride_time_excess = 0;
    double time_window_excess = 0;
    double duration = 0;
    double duration_excess = 0;
};

struct NodeAttributes {
    RequestId id{};
    double arrival_A = 0;
    double waiting_W = 0;
    double begin_service_B = 0;
    double departure_D = 0;
};

struct JourneyMoveCost {
    std::vector<NodeAttributes> node_costs;
};

struct SolutionCost {
    std::vector<RouteExcess> route_costs;
    double total_cost = 0;
    double duration = 0;
    unsigned int total_load_excess = 0;
    double total_ride_time_excess = 0;
    double total_time_window_excess = 0;
    double total_duration_excess = 0;
};

struct Route {
    std::vector<Request> requests;
    NodeVector nodes;
    std::vector<NodeAttributes> node_attributes;
    RouteExcess route_excess;
};

typedef std::vector<Route> Routes;
struct Solution {
    Routes routes;
    PenaltyList penalty_list;
};

typedef std::vector<Solution> Neighbourhood;


struct RelaxationParams {
    double load_alpha = 1;
    double duration_beta = 1;
    double time_window_gamma = 1;
    double ride_time_tau = 1;
    uint8_t delta = 1;
};

struct SolutionResult {
    Solution solution;
    double cost;
};

#endif //TABU_TABUTYPES_H
