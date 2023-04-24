#pragma once

//
// Created by Nicholas Ives on 19/04/2023.
//

#ifndef TABU_DARPROBLEM_H
#define TABU_DARPROBLEM_H

#include "tabutypes.h"

class DARProblem final {

public:
    explicit DARProblem(const std::vector<Node>& nodes) : nodes_(nodes), distances_(init_distance(nodes)),
        node_vector_(init_node_vector(nodes)) {};

    double
        distance(const int& node_idx1, const int& node_idx2);

    [[nodiscard]] const std::vector<Node>&
        getNodes() const;

    [[nodiscard]] const Distances&
        getDistances() const;

    [[nodiscard]] const NodeVector&
        getNodeVector() const;

private:
    const std::vector<Node> nodes_;
    const NodeVector node_vector_;
    const Distances distances_;

private:
    static double
        distance(const double& x1, const double& y1, const double& x2, const double& y2);

    static Distances
        init_distance(const std::vector<Node>& nodes);

    static NodeVector
        init_node_vector(const std::vector<Node>& nodes);
};


#endif //TABU_DARPROBLEM_H
