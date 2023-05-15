#include "SolutionPrinter.h"
//
// Created by Nicholas Ives on 23/04/2023.
//

#include <string>

#include "SolutionPrinter.h"

void SolutionPrinter::print_solution(const Solution& solution) {
    {
        for (size_t r = 0; r < solution.routes.size(); ++r) {
            std::cout << "Vehicle " << r + 1 << ":";
            for (const auto& node : solution.routes[r].nodes) {
                std::cout << " " << std::to_string(node->id);
            }
            std::cout << " : Duration : " << std::to_string(solution.routes[r].route_excess.duration) << std::endl;
        }
    }
}

