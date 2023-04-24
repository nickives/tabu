#include "DARProblem.h"

#include <complex>

//
// Created by Nicholas Ives on 19/04/2023.
//

#include "DARProblem.h"
#include "tabutypes.h"

double DARProblem::distance(const double& x1, const double& y1, const double& x2, const double& y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

double DARProblem::distance(const int& node_idx1, const int& node_idx2) {
    return distances_[node_idx1][node_idx2];
}

const std::vector<Node>& DARProblem::getNodes() const {
    return nodes_;
}

const NodeVector& DARProblem::getNodeVector() const {
    return node_vector_;
}

const Distances& DARProblem::getDistances() const {
    return distances_;
}

Distances DARProblem::init_distance(const std::vector<Node>& nodes) {
    Distances distances = Distances(nodes.size());
    for (int i = 0; i < nodes.size(); ++i) {
        distances[i] = std::vector<double>(nodes.size());
        for (int j = 0; j < nodes.size(); ++j) {
            const auto& node1 = nodes[i];
            const auto& node2 = nodes[j];
            distances[i][j] = distance(node1.x, node1.y, node2.x, node2.y);
        }
    }
    return distances;
}

NodeVector DARProblem::init_node_vector(const std::vector<Node>& nodes) {
    NodeVector nv{};
    nv.reserve(nodes.size());
    for (auto& node : nodes) {
        nv.push_back(&node);
    }
    return nv;
}
