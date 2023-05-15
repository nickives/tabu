#pragma once

//
// Created by Nicholas Ives on 19/04/2023.
//

#ifndef TABU_DARPROBLEM_H
#define TABU_DARPROBLEM_H

#include <filesystem>
#include "tabutypes.h"

class DARProblem final {

public:
    /**
    * DARProblem
    * 
    * @
    */
    explicit DARProblem(vector<Node> nodes_in, const uint16_t vehicles_count,
        const uint16_t requests_count, const uint16_t vehicle_cap,
        const uint16_t max_ride_time, const double plan_horizon, const filesystem::path f_path)
        : nodes(nodes_in), distances(init_distance(nodes)),
        number_of_vehicles(vehicles_count), number_of_requests(requests_count),
        vehicle_capacity(vehicle_cap), maximum_ride_time(max_ride_time),
        planning_horizon(plan_horizon), requests(init_requests(nodes)), path(f_path) {};

public:
    const vector<Node> nodes;
    const Distances distances;
    const uint16_t number_of_vehicles;
    const uint16_t number_of_requests;
    const uint16_t vehicle_capacity;
    const uint16_t maximum_ride_time;
    const double planning_horizon;
    const vector<Request> requests;
    const filesystem::path path;

private:
    static double
        distance(const double& x1, const double& y1, const double& x2, const double& y2);

    static Distances
        init_distance(const vector<Node>& nodes);

    static vector<Request>
        init_requests(const vector<Node>& nodes);

};


#endif //TABU_DARPROBLEM_H
