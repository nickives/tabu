#pragma once

//
// Created by Nicholas Ives on 13/04/2023.
//

#ifndef TABU_TABUTYPES_H
#define TABU_TABUTYPES_H

#include <list>
#include <vector>
#include <memory>

#include "robin_hood.h"

enum MoveType { SPI, SWAP, MOVE };
typedef uint64_t RequestId;
typedef uint64_t RouteIndex;

class TabuKey  {
public:
    TabuKey(const RouteIndex& route, const RequestId& request)
        : key(init_key(route, request)) {};

    std::string key;

    bool operator==(const TabuKey& rhs) {
        return key == rhs.key;
    }
private:

    inline static std::string init_key(const RouteIndex& route, const RequestId& request)
    {
        std::string k;
        k += 'V' + std::to_string(route) + 'R' + std::to_string(request);
        return k;
    }
};

struct TabuKeyHash {
    size_t operator()(const TabuKey& key) const
    {
        return robin_hood::hash<std::string>()(key.key);
    }
};

struct TabuKeyEquals {
    bool operator()(const TabuKey& lhs, const TabuKey& rhs) const
    {
        return lhs.key == rhs.key;
    }
};
typedef TabuKey PenaltyKey;
typedef TabuKeyHash PenaltyKeyHash;
typedef TabuKeyEquals PenaltyKeyEquals;

typedef std::vector<std::vector<double>> Distances;
//typedef std::tuple<i nt, RequestId> TabuKey;

typedef uint64_t TabuIteration;
typedef unsigned int TimesAdded;

//typedef robin_hood::pair<int, int> TabuKey;
//typedef robin_hood::pair<int, int> PenaltyKey;

typedef robin_hood::unordered_flat_map<PenaltyKey, TimesAdded, PenaltyKeyHash, PenaltyKeyEquals> PenaltyMap;
typedef robin_hood::unordered_flat_map<TabuKey, TabuIteration, TabuKeyHash, TabuKeyEquals> TabuList;
typedef uint64_t NodeId;
typedef double RouteCost;
typedef robin_hood::unordered_flat_map<TabuKey, RouteCost, TabuKeyHash, TabuKeyEquals> AspirationCriteria;

struct Node {
    NodeId id;
    RequestId request_id;
    int16_t load;
    double service_duration;
    double time_window_start;
    double time_window_end;
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
    double ride_time_excess_L = 0;
    double time_window_excess = 0;
    double duration = 0;
    double duration_excess = 0;
};

struct NodeAttributes {
    RequestId request_id{};
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
    RouteCost total_cost = 0;
    double raw_cost = 0;
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
    PenaltyMap penalty_map;
    PenaltyKey attribute_added{0,0};
    PenaltyKey attribute_removed{ 0,0 };
};

typedef std::vector<Solution> Neighbourhood;


struct RelaxationParams {
    double load_alpha = 10;
    double duration_beta = 1;
    double time_window_gamma = 1;
    double ride_time_tau = 1;
    double delta = 0.5;
};

struct SolutionResult {
    Solution solution;
    RouteCost cost;
};

#endif //TABU_TABUTYPES_H
