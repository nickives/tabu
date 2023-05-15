//
// Created by Nicholas Ives on 19/04/2023.
//

#include <complex>

#include "DARProblem.h"
#include "tabutypes.h"

//DARProblem::DARProblem(DARProblem&& o) noexcept
//    : nodes(o.nodes), distances(o.distances),
//    number_of_vehicles(o.number_of_vehicles), number_of_requests(o.number_of_requests),
//    vehicle_capacity(o.vehicle_capacity), maximum_ride_time(o.maximum_ride_time),
//    planning_horizon(o.planning_horizon), path(o.path)
//{
//
//}

double DARProblem::distance(const double& x1, const double& y1, const double& x2, const double& y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

Distances DARProblem::init_distance(const vector<Node>& nodes) {
    const auto nodes_size = nodes.size();
    Distances distances = Distances(nodes_size);
    for (int i = 0; i < nodes_size; ++i) {
        distances[i] = std::vector<double>(nodes_size);
        for (int j = 0; j < nodes_size; ++j) {
            const auto& node1 = nodes[i];
            const auto& node2 = nodes[j];
            distances[i][j] = distance(node1.x, node1.y, node2.x, node2.y);
        }
    }
    return distances;
}

vector<Request> DARProblem::init_requests(const vector<Node>& nodes)
{
    const size_t midpoint = (nodes.size() / 2) - 1;
    vector<Request> requests;
    requests.reserve(midpoint);
    
    vector<Node> thang;
    thang.emplace_back(Node{5, 5, 5, 5, 5, 5, 5, 5});

    const unique_ptr<vector<Node>> thing = make_unique<vector<Node>>(thang);

    const auto result = thing.get()[0][0];

    // 0 is depot
    for (int i = 1; i <= midpoint; ++i) {
        const Node* pickup_node = &nodes[i];
        const Node* dropoff_node = &nodes[i + midpoint];
        requests.emplace_back(pickup_node->request_id, pickup_node, dropoff_node);
    }
    return requests;
}
