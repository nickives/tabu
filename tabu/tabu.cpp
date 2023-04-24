// tabu.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

#include <iostream>
#include <vector>
#include "SolutionPrinter.h"
#include "tabutypes.h"
#include "TabuSearch.h"

int main() {
    std::vector<Node> nodes = {
            {0, 0, 0, 0, 0, 1400, 0, 0},
            {1, 1, 1, 10, 0, 1400, 9, 8},
            {2, 2, 1, 5, 0, 1400, 6, 7},
            {3, 3, 1, 10, 0, 1400, 7, 9},
            {4, 4, 1, 5, 0, 1400, 3, 5},
            {5, 5, 1, 5, 0, 1400, 2, 5},
            {6, 6, 1, 5, 0, 1400, 5, 4.8},
            {7, 7, 1, 5, 0, 1400, 6, 5.9},
            {8, 8, 1, 5, 0, 1400, 8.4, 9.6},
            {9, 9, 1, 5, 0, 1400, 5, 5},
            {10, 10, 1, 5, 0, 1400, 7.5, 9.1},
            {11, 1, -1, 5, 40, 100, -1, -6},
            {12, 2, -1, 5, 20, 160, -3, -5.5},
            {13, 3, -1, 5, 40, 300, -8, -7.32},
            {14, 4, -1, 10, 40, 160, -8.5, -2.9},
            {15, 5, -1, 10, 45, 200, -6.5, -2.5},
            {16, 6, -1, 10, 100, 160, -6.3, -3.45},
            {17, 7, -1, 10, 200, 500, -2.5, -3},
            {18, 8, -1, 10, 40, 160, -9.5, -7.45},
            {19, 9, -1, 10, 40, 160, -4.5, -4.45},
            {20, 10, -1, 10, 40, 160, -6.25, -3.45},
            {21, 0, 0, 0, 0, 1400, 0, 0}
    };

    const int max_iterations = 5000;
    const int max_solution_cost = 1000;
    const int max_vehicle_load = 3;
    const int planning_horizon = 1400;
    const int tabu_list_size = 12;
    const uint64_t penalty_lambda = 5;
    const uint64_t tabu_duration = 5;
    const int num_vehicles = 4;

    DARProblem problem{ nodes };

    TabuSearch ts(max_solution_cost, max_vehicle_load, planning_horizon, tabu_list_size, tabu_duration, penalty_lambda, problem);

    try {
        auto best_solution = ts.search(num_vehicles, max_iterations);
        std::cout << "Best solution cost: " << best_solution.cost << std::endl;
        SolutionPrinter::print_solution(best_solution.solution);
    }
    catch (const std::bad_alloc& e) {
        std::cout << e.what() << std::endl;
    }
    return 0;
}