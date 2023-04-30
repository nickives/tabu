#pragma once

//
// Created by Nicholas Ives on 13/04/2023.
//

#ifndef TABU_TABUTYPES_H
#define TABU_TABUTYPES_H

#include <vector>
#include "robin_hood.h"

using namespace std;

enum MoveType { SPI, SWAP, MOVE };
typedef uint64_t RequestId;
typedef uint64_t RouteIndex;

class TabuKey  {
public:
    TabuKey(const RouteIndex& route, const RequestId& request)
        : key(init_key(route, request)) {};

    string key;

    bool operator==(const TabuKey& rhs) {
        return key == rhs.key;
    }
private:

    inline static string init_key(const RouteIndex& route, const RequestId& request)
    {
        string k;
        k += 'V' + to_string(route) + 'R' + to_string(request);
        return k;
    }
};

struct TabuKeyHash {
    size_t operator()(const TabuKey& key) const
    {
        return robin_hood::hash<string>()(key.key);
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

typedef vector<vector<double>> Distances;
//typedef tuple<i nt, RequestId> TabuKey;

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
typedef vector<const Node*> NodeVector;

struct Request {
    const RequestId id;

    const Node* pickup_node;
    const Node* dropoff_node;

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
    vector<NodeAttributes> node_costs;
};

struct SolutionCost {
    vector<RouteExcess> route_costs;
    RouteCost total_cost = 0;
    double raw_cost = 0;
    double duration = 0;
    unsigned int total_load_excess = 0;
    double total_ride_time_excess = 0;
    double total_time_window_excess = 0;
    double total_duration_excess = 0;
};

struct Route {
    vector<const Request*> requests;
    NodeVector nodes;
    vector<NodeAttributes> node_attributes;
    RouteExcess route_excess;
};

typedef vector<Route> Routes;
struct HistoryItem {
    PenaltyKey attribute_added{ 0,0 };
    PenaltyKey attribute_removed{ 0,0 };
};
struct Solution {
    Routes routes;
    PenaltyKey attribute_added{0,0};
    PenaltyKey attribute_removed{ 0,0 };
    bool swap = false;
};

typedef vector<Solution> Neighbourhood;


struct RelaxationParams {
    double load_alpha = 1;
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
